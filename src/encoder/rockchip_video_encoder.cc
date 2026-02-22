// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/video_coding/codecs/mpp/rockchip_video_encoder.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "api/scoped_refptr.h"
#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
#include "rtc_base/ref_counted_object.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "modules/video_coding/codecs/h264/include/h264_globals.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "third_party/libyuv/include/libyuv.h"

// Include DMA-BUF frame buffer for zero-copy encoding
#ifdef __linux__
#include "common_video/dmabuf_video_frame_buffer.h"
#endif

namespace webrtc {

namespace {

// Constants for encoder configuration
constexpr int kDefaultGOP = 30;
constexpr int kDefaultFPS = 30;
constexpr int kDefaultBitrate = 2000000;  // 2 Mbps

// LTR interval: mark every Nth frame as long-term reference
constexpr uint64_t kLTRInterval = 120;

// Scene change detection threshold (histogram L1 distance)
constexpr uint32_t kSceneChangeThreshold = 30000;

// GDR: number of GOPs over which to cycle intra-refresh
constexpr int kGDRCycleDivisor = 30;

// ROI: center region gets lower QP
constexpr int kROICenterQpDelta = -6;
constexpr int kROIBorderQpDelta = 4;

// Bitstream dump: write first N frames to /tmp/mpp_dump.h264 for diagnosis
constexpr uint64_t kDumpFrameCount = 600;
static int g_dump_fd = -1;
static uint64_t g_dump_written = 0;

// Helper to convert WebRTC frame type to MPP
bool IsKeyFrame(const std::vector<VideoFrameType>* frame_types) {
  if (!frame_types || frame_types->empty()) {
    return false;
  }
  return (*frame_types)[0] == VideoFrameType::kVideoFrameKey;
}

// Scan an Annex-B byte stream for NAL units and detect keyframe.
// Handles both 3-byte (00 00 01) and 4-byte (00 00 00 01) start codes.
bool ScanAnnexBForKeyframe(const uint8_t* data, size_t size,
                           MppCodingType codec_type) {
  if (!data || size < 4)
    return false;

  size_t i = 0;
  while (i < size - 3) {
    // Find start code
    if (data[i] == 0 && data[i + 1] == 0) {
      int start_code_len = 0;
      if (i + 3 < size && data[i + 2] == 0 && data[i + 3] == 1) {
        start_code_len = 4;
      } else if (data[i + 2] == 1) {
        start_code_len = 3;
      }

      if (start_code_len > 0 && i + start_code_len < size) {
        if (codec_type == MPP_VIDEO_CodingAVC) {
          // H.264: NAL type is lower 5 bits of first byte after start code
          uint8_t nal_type = data[i + start_code_len] & 0x1F;
          // 5 = IDR slice, 7 = SPS
          if (nal_type == 5 || nal_type == 7) {
            return true;
          }
        } else if (codec_type == MPP_VIDEO_CodingHEVC) {
          // H.265: NAL type is bits 1-6 of first byte after start code
          uint8_t nal_type = (data[i + start_code_len] >> 1) & 0x3F;
          // 19 = IDR_W_RADL, 20 = IDR_N_LP, 32 = VPS, 33 = SPS
          if (nal_type == 19 || nal_type == 20 ||
              nal_type == 32 || nal_type == 33) {
            return true;
          }
        }
        i += start_code_len;
        continue;
      }
    }
    i++;
  }
  return false;
}

}  // namespace

RockchipVideoEncoder::RockchipVideoEncoder(MppCodingType codec_type)
    : codec_type_(codec_type),
      mpp_ctx_(nullptr),
      mpp_mpi_(nullptr),
      buffer_group_(nullptr),
      buffer_pool_idx_(0),
      mpp_cfg_(nullptr),
      encoded_callback_(nullptr),
      bitrate_bps_(kDefaultBitrate),
      framerate_(kDefaultFPS),
      frame_count_(0),
      last_timestamp_ms_(0),
      has_prev_histogram_(false),
      gdr_requested_(false),
      total_mbs_(0),
      ltr_interval_(kLTRInterval),
      ltr_index_(0),
      hor_stride_(0),
      ver_stride_(0) {
  memset(prev_histogram_, 0, sizeof(prev_histogram_));
  memset(buffer_pool_, 0, sizeof(buffer_pool_));
  RTC_LOG(LS_INFO) << "RockchipVideoEncoder: Constructor (codec="
                    << (codec_type == MPP_VIDEO_CodingHEVC ? "HEVC" : "AVC")
                    << ")";
}

RockchipVideoEncoder::~RockchipVideoEncoder() {
  Release();
}

void RockchipVideoEncoder::SetFecControllerOverride(
    FecControllerOverride* fec_controller_override) {
  // Not implemented for hardware encoder
}

int RockchipVideoEncoder::InitEncode(
    const VideoCodec* codec_settings,
    const VideoEncoder::Settings& settings) {
  RTC_LOG(LS_INFO) << "RockchipVideoEncoder::InitEncode";

  if (!codec_settings) {
    RTC_LOG(LS_ERROR) << "InitEncode: codec_settings is null";
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  webrtc::MutexLock lock(&mutex_);

  codec_settings_ = *codec_settings;
  bitrate_bps_ = codec_settings->startBitrate * 1000;
  framerate_ = codec_settings->maxFramerate > 0 ? codec_settings->maxFramerate
                                                 : kDefaultFPS;

  // MPP reference: hor_stride = MPP_ALIGN(width, 16) for NV12.
  // ver_stride = MPP_ALIGN(height, 16) for macroblock alignment.
  // Buffer allocation uses MPP_ALIGN(..., 64) for DMA safety.
  hor_stride_ = MPP_ALIGN(codec_settings->width, 16);
  ver_stride_ = MPP_ALIGN(codec_settings->height, 16);

  // Compute total macroblocks for GDR intra-refresh
  int mb_w = (codec_settings->width + 15) / 16;
  int mb_h = (codec_settings->height + 15) / 16;
  total_mbs_ = mb_w * mb_h;

  fprintf(stderr, "[MPP] InitEncode: %dx%d stride=%dx%d @%d fps bitrate=%d mbs=%d\n",
          codec_settings->width, codec_settings->height,
          hor_stride_, ver_stride_, framerate_, bitrate_bps_, total_mbs_);
  RTC_LOG(LS_INFO) << "InitEncode: " << codec_settings->width << "x"
                   << codec_settings->height
                   << " stride=" << hor_stride_ << "x" << ver_stride_
                   << " @ " << framerate_
                   << " fps, bitrate: " << bitrate_bps_
                   << " mbs=" << total_mbs_;

  // Step 1: Create MPP context
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_mpi_);
  if (ret != MPP_OK || !mpp_ctx_ || !mpp_mpi_) {
    RTC_LOG(LS_ERROR) << "mpp_create failed: " << ret;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Step 2: Initialize MPP for encoding (AVC or HEVC)
  ret = mpp_init(mpp_ctx_, MPP_CTX_ENC, codec_type_);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "mpp_init failed: " << ret;
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Step 3: Create buffer group (DRM for hardware DMA access).
  ret = mpp_buffer_group_get_internal(&buffer_group_, MPP_BUFFER_TYPE_DRM);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "DRM buffer group failed, trying ION";
    ret = mpp_buffer_group_get_internal(&buffer_group_, MPP_BUFFER_TYPE_ION);
    if (ret != MPP_OK) {
      RTC_LOG(LS_ERROR) << "mpp_buffer_group_get_internal failed: " << ret;
      Release();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  // Step 4: Configure encoder
  ret = mpp_enc_cfg_init(&mpp_cfg_);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "mpp_enc_cfg_init failed: " << ret;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_GET_CFG, mpp_cfg_);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "MPP_ENC_GET_CFG failed: " << ret;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (!ConfigureEncoder() || !ConfigureRateControl()) {
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Step 5: Apply configuration
  ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_CFG, mpp_cfg_);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "MPP_ENC_SET_CFG failed: " << ret;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Step 6: Prepend SPS/PPS (or VPS/SPS/PPS for HEVC) to each IDR/IRAP frame
  MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
  ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_HEADER_MODE, &header_mode);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "MPP_ENC_SET_HEADER_MODE failed: " << ret;
  }

  frame_count_ = 0;
  last_timestamp_ms_ = 0;
  has_prev_histogram_ = false;
  gdr_requested_ = false;
  ltr_index_ = 0;

  // Open bitstream dump file for diagnosis (first N frames)
  if (g_dump_fd < 0) {
    const char* dump_path = (codec_type_ == MPP_VIDEO_CodingHEVC)
                                ? "/tmp/mpp_dump.h265"
                                : "/tmp/mpp_dump.h264";
    g_dump_fd = open(dump_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    g_dump_written = 0;
    if (g_dump_fd >= 0) {
      RTC_LOG(LS_INFO) << "Bitstream dump: writing first " << kDumpFrameCount
                       << " frames to " << dump_path;
    }
  }

  const char* codec_name = (codec_type_ == MPP_VIDEO_CodingHEVC)
                               ? "HEVC Main" : "H.264 High";
  RTC_LOG(LS_INFO) << "RockchipVideoEncoder initialized successfully"
                   << " (" << codec_name << ", stride="
                   << hor_stride_ << "x" << ver_stride_ << ")";
  return WEBRTC_VIDEO_CODEC_OK;
}

bool RockchipVideoEncoder::ConfigureEncoder() {
  // Prepare configuration with correct stride alignment
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:width", codec_settings_.width);
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:height", codec_settings_.height);
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:hor_stride", hor_stride_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:ver_stride", ver_stride_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:format", MPP_FMT_YUV420SP);  // NV12

  mpp_enc_cfg_set_s32(mpp_cfg_, "codec:type", codec_type_);

  // Disable slice splitting — one slice per frame to prevent
  // depacketization issues that can cause visible tearing.
  mpp_enc_cfg_set_s32(mpp_cfg_, "split:mode", 0);
  mpp_enc_cfg_set_s32(mpp_cfg_, "split:arg", 0);

  if (codec_type_ == MPP_VIDEO_CodingHEVC) {
    // H.265 Main Profile
    mpp_enc_cfg_set_s32(mpp_cfg_, "h265:profile", 1);  // Main Profile
    mpp_enc_cfg_set_s32(mpp_cfg_, "h265:level", 120);   // Level 4.0
    RTC_LOG(LS_INFO) << "Encoder: HEVC Main Profile, Level 4.0";
  } else {
    // H.264 High Profile — matching GStreamer rockchip-mpp plugins.
    // CABAC gives ~15% better compression, 8x8 transform improves detail.
    mpp_enc_cfg_set_s32(mpp_cfg_, "h264:profile", 100);  // High
    mpp_enc_cfg_set_s32(mpp_cfg_, "h264:level", 40);     // Level 4.0
    mpp_enc_cfg_set_s32(mpp_cfg_, "h264:cabac_en", 1);   // CABAC
    mpp_enc_cfg_set_s32(mpp_cfg_, "h264:cabac_idc", 0);
    mpp_enc_cfg_set_s32(mpp_cfg_, "h264:trans8x8", 1);   // 8x8 transform

    RTC_LOG(LS_INFO) << "Encoder: H.264 High Profile, Level 4.0, "
                        "CABAC, 8x8, single-slice";
  }

  return true;
}

bool RockchipVideoEncoder::ConfigureRateControl() {
  // VBR with tight band — GStreamer-style VBR bounds.
  // +6.25% max prevents pacer overflow, wide min floor allows low-motion
  // savings.
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:mode", MPP_ENC_RC_MODE_VBR);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_target", bitrate_bps_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_max", bitrate_bps_ * 17 / 16);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_min", bitrate_bps_ * 1 / 16);

  // QP: full range matching GStreamer defaults.  The encoder gracefully
  // degrades quality under bandwidth pressure rather than dropping frames.
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_init", 26);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_min", 10);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_max", 51);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_min_i", 10);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_max_i", 51);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_ip", 2);   // I-P QP delta

  // Allow 1 re-encode attempt per frame for better quality
  mpp_enc_cfg_set_u32(mpp_cfg_, "rc:max_reenc_times", 1);

  // FPS configuration
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_flex", 0);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_num", framerate_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_denorm", 1);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_flex", 0);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_num", framerate_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_denorm", 1);

  // GOP: shorter GOP = more frequent error recovery points
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:gop", kDefaultGOP);

  RTC_LOG(LS_INFO) << "Rate control: VBR target=" << bitrate_bps_
                   << " max=" << bitrate_bps_ * 17 / 16
                   << " @ " << framerate_ << " fps, GOP=" << kDefaultGOP;
  return true;
}

bool RockchipVideoEncoder::InitBufferPool() {
  // Disabled — allocating fresh buffers per frame for debugging.
  return false;
}

void RockchipVideoEncoder::DestroyBufferPool() {
  for (int i = 0; i < kBufferPoolSize; i++) {
    if (buffer_pool_[i]) {
      mpp_buffer_put(buffer_pool_[i]);
      buffer_pool_[i] = nullptr;
    }
  }
  buffer_pool_idx_ = 0;
}

MppBuffer RockchipVideoEncoder::GetPoolBuffer() {
  // Disabled — return nullptr so caller allocates fresh.
  return nullptr;
}

int32_t RockchipVideoEncoder::RegisterEncodeCompleteCallback(
    EncodedImageCallback* callback) {
  webrtc::MutexLock lock(&mutex_);
  encoded_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipVideoEncoder::Release() {
  fprintf(stderr, "[MPP] Release (frame_count=%lu)\n", (unsigned long)frame_count_);
  RTC_LOG(LS_INFO) << "RockchipVideoEncoder::Release";

  webrtc::MutexLock lock(&mutex_);

  DestroyBufferPool();

  if (mpp_cfg_) {
    mpp_enc_cfg_deinit(mpp_cfg_);
    mpp_cfg_ = nullptr;
  }

  if (buffer_group_) {
    mpp_buffer_group_put(buffer_group_);
    buffer_group_ = nullptr;
  }

  if (mpp_ctx_) {
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    mpp_mpi_ = nullptr;
  }

  // Close dump file
  if (g_dump_fd >= 0) {
    close(g_dump_fd);
    g_dump_fd = -1;
    RTC_LOG(LS_INFO) << "Bitstream dump closed (" << g_dump_written
                     << " frames written)";
  }

  encoded_callback_ = nullptr;
  frame_count_ = 0;

  return WEBRTC_VIDEO_CODEC_OK;
}

MppBuffer RockchipVideoEncoder::ImportDMABuffer(int fd,
                                                  size_t size,
                                                  int width,
                                                  int height) {
  RTC_DCHECK(buffer_group_);

  MppBuffer mpp_buffer = nullptr;
  MppBufferInfo info;
  memset(&info, 0, sizeof(info));

  // Use DRM buffer type for zero-copy DMA-buf import
  info.type = MPP_BUFFER_TYPE_DRM;
  info.size = size;
  info.fd = fd;

  MPP_RET ret = mpp_buffer_import_with_tag(buffer_group_, &info, &mpp_buffer,
                                           "webrtc_dmabuf", __FUNCTION__);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "mpp_buffer_import_with_tag failed: " << ret;
    return nullptr;
  }

  RTC_LOG(LS_VERBOSE) << "Imported DMA-BUF fd=" << fd << " size=" << size
                      << " as MppBuffer";
  return mpp_buffer;
}

void RockchipVideoEncoder::ComputeHistogram(const uint8_t* y_plane,
                                             int width, int height,
                                             int stride,
                                             uint32_t histogram[16]) {
  memset(histogram, 0, sizeof(uint32_t) * 16);
  for (int y = 0; y < height; y += 8) {
    const uint8_t* row = y_plane + y * stride;
    for (int x = 0; x < width; x += 8) {
      histogram[row[x] >> 4]++;
    }
  }
}

bool RockchipVideoEncoder::DetectSceneChange(const uint8_t* y_plane,
                                              int width, int height,
                                              int stride) {
  uint32_t curr_histogram[16];
  ComputeHistogram(y_plane, width, height, stride, curr_histogram);

  bool scene_change = false;
  if (has_prev_histogram_) {
    uint32_t distance = 0;
    for (int i = 0; i < 16; i++) {
      distance += std::abs(static_cast<int>(curr_histogram[i]) -
                           static_cast<int>(prev_histogram_[i]));
    }
    scene_change = (distance > kSceneChangeThreshold);
  }

  memcpy(prev_histogram_, curr_histogram, sizeof(prev_histogram_));
  has_prev_histogram_ = true;
  return scene_change;
}

bool RockchipVideoEncoder::ConfigureROI(MppFrame mpp_frame) {
  // Disabled for debugging
  return false;
}

void RockchipVideoEncoder::RequestGradualRefresh() {
  webrtc::MutexLock lock(&mutex_);
  gdr_requested_ = true;
}

int32_t RockchipVideoEncoder::Encode(
    const VideoFrame& frame,
    const std::vector<VideoFrameType>* frame_types) {
  webrtc::MutexLock lock(&mutex_);

  if (!mpp_ctx_ || !mpp_mpi_) {
    RTC_LOG(LS_ERROR) << "Encode called before InitEncode";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  if (!encoded_callback_) {
    RTC_LOG(LS_WARNING) << "Encode callback not set";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  // Only request IDR when WebRTC explicitly asks for one, or on first frame.
  // Do NOT force periodic IDR — let the GOP handle it.  The old code forced
  // IDR every 60 frames and then OVERRODE the keyframe flag, which labelled
  // P-frames as keyframes when MPP_ENC_SET_IDR_FRAME didn't take effect
  // immediately.  This broke the receiver's reference-frame chain during
  // motion, causing the tearing artifact.
  bool request_idr = IsKeyFrame(frame_types) || (frame_count_ == 0);

  // Extract video frame buffer
  rtc::scoped_refptr<VideoFrameBuffer> buffer = frame.video_frame_buffer();

  MppFrame mpp_frame = nullptr;
  MPP_RET ret = mpp_frame_init(&mpp_frame);
  if (ret != MPP_OK || !mpp_frame) {
    RTC_LOG(LS_ERROR) << "mpp_frame_init failed: " << ret;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Per-frame stride: may differ from hor_stride_ for DMA-BUF frames
  int frame_hor_stride = hor_stride_;

  // Check if this is a native buffer (DMA-BUF backed) for zero-copy path
  if (buffer->type() == VideoFrameBuffer::Type::kNative) {
#ifdef __linux__
    auto* dmabuf_buffer = static_cast<DMABufVideoFrameBuffer*>(buffer.get());
    int dmabuf_fd = dmabuf_buffer->GetDmaBufFd();
    size_t buffer_size = dmabuf_buffer->GetSize();
    int buffer_stride = dmabuf_buffer->GetStride();

    if (dmabuf_fd >= 0 && buffer_size > 0) {
      MppBuffer mpp_buffer = ImportDMABuffer(dmabuf_fd, buffer_size,
                                             frame.width(), frame.height());
      if (mpp_buffer) {
        mpp_frame_set_buffer(mpp_frame, mpp_buffer);
        if (buffer_stride > 0) {
          frame_hor_stride = buffer_stride;
        }
        mpp_buffer_put(mpp_buffer);
      } else {
        goto cpu_copy;
      }
    } else {
      goto cpu_copy;
    }
#else
    goto cpu_copy;
#endif
  } else {
cpu_copy:
    // CPU copy path — allocate a FRESH MppBuffer per frame.
    // Buffer size follows MPP reference: MPP_ALIGN(stride, 64) for DMA safety.
    size_t frame_size = MPP_ALIGN(hor_stride_, 64) *
                        MPP_ALIGN(ver_stride_, 64) * 3 / 2;

    MppBuffer mpp_buffer = nullptr;
    ret = mpp_buffer_get(buffer_group_, &mpp_buffer, frame_size);
    if (ret != MPP_OK || !mpp_buffer) {
      RTC_LOG(LS_ERROR) << "mpp_buffer_get failed: " << ret;
      mpp_frame_deinit(&mpp_frame);
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    uint8_t* dst_y = static_cast<uint8_t*>(mpp_buffer_get_ptr(mpp_buffer));
    // UV plane offset = hor_stride * ver_stride (per MPP spec).
    uint8_t* dst_uv = dst_y + hor_stride_ * ver_stride_;

    // Check buffer type BEFORE calling GetNV12() — WebRTC's GetNV12()
    // has RTC_CHECK(type()==kNV12) which aborts on non-NV12 buffers.
    if (buffer->type() == VideoFrameBuffer::Type::kNV12) {
      const NV12BufferInterface* nv12 = buffer->GetNV12();
      // Direct NV12 -> NV12 plane copy (no I420 round-trip)
      libyuv::CopyPlane(nv12->DataY(), nv12->StrideY(),
                        dst_y, hor_stride_,
                        frame.width(), frame.height());
      libyuv::CopyPlane(nv12->DataUV(), nv12->StrideUV(),
                        dst_uv, hor_stride_,
                        frame.width(), (frame.height() + 1) / 2);
    } else {
      // Convert I420 (or other) -> NV12
      rtc::scoped_refptr<I420BufferInterface> i420_buffer = buffer->ToI420();
      libyuv::I420ToNV12(
          i420_buffer->DataY(), i420_buffer->StrideY(),
          i420_buffer->DataU(), i420_buffer->StrideU(),
          i420_buffer->DataV(), i420_buffer->StrideV(),
          dst_y, hor_stride_,
          dst_uv, hor_stride_,
          frame.width(), frame.height());
    }

    mpp_frame_set_buffer(mpp_frame, mpp_buffer);
    mpp_buffer_put(mpp_buffer);
  }

  // Configure frame properties
  mpp_frame_set_width(mpp_frame, frame.width());
  mpp_frame_set_height(mpp_frame, frame.height());
  mpp_frame_set_hor_stride(mpp_frame, frame_hor_stride);
  mpp_frame_set_ver_stride(mpp_frame, ver_stride_);
  mpp_frame_set_fmt(mpp_frame, MPP_FMT_YUV420SP);
  mpp_frame_set_eos(mpp_frame, 0);
  mpp_frame_set_pts(mpp_frame, frame.timestamp_us());

  // Request IDR only when explicitly needed
  if (request_idr) {
    mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_IDR_FRAME, nullptr);
  }

  // Encode: put frame then get packet
  if (!SendFrame(mpp_frame)) {
    mpp_frame_deinit(&mpp_frame);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  VideoFrameType webrtc_frame_type = request_idr
      ? VideoFrameType::kVideoFrameKey
      : VideoFrameType::kVideoFrameDelta;
  bool packet_ok = RetrievePacket(frame.rtp_timestamp(), frame.timestamp_us(),
                                   webrtc_frame_type, request_idr);

  mpp_frame_deinit(&mpp_frame);

  if (!packet_ok) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  frame_count_++;
  return WEBRTC_VIDEO_CODEC_OK;
}

bool RockchipVideoEncoder::SendFrame(MppFrame mpp_frame) {
  // Full memory barrier: ensure all CPU writes to the MppBuffer are
  // globally visible before the VEPU DMA engine starts reading.
  __sync_synchronize();

  MPP_RET ret = mpp_mpi_->encode_put_frame(mpp_ctx_, mpp_frame);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "encode_put_frame failed: " << ret
                        << " (dropping frame)";
    return false;
  }
  return true;
}

bool RockchipVideoEncoder::RetrievePacket(uint32_t rtp_timestamp,
                                           int64_t timestamp_us,
                                           VideoFrameType frame_type,
                                           bool force_keyframe) {
  MppPacket mpp_packet = nullptr;

  MPP_RET ret = mpp_mpi_->encode_get_packet(mpp_ctx_, &mpp_packet);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "encode_get_packet failed: " << ret;
    return false;
  }

  if (!mpp_packet) {
    RTC_LOG(LS_WARNING) << "No packet retrieved";
    return false;
  }

  // Extract packet data
  void* packet_data = mpp_packet_get_pos(mpp_packet);
  size_t packet_size = mpp_packet_get_length(mpp_packet);
  RK_U32 packet_flag = mpp_packet_get_flag(mpp_packet);

  // *** CRITICAL FIX: detect keyframe ONLY from encoder output ***
  // NEVER override with our request flag — if MPP didn't produce an IDR,
  // lying about it breaks the receiver's reference chain and causes tearing.
  bool is_keyframe = (packet_flag & MPP_PACKET_FLAG_INTRA) != 0;

  // Backup: scan Annex-B NAL units in case MPP flag is missing
  if (!is_keyframe) {
    is_keyframe = ScanAnnexBForKeyframe(
        static_cast<const uint8_t*>(packet_data), packet_size, codec_type_);
  }

  // Log if we requested IDR but didn't get one
  if (force_keyframe && !is_keyframe) {
    fprintf(stderr, "[MPP] WARNING: Requested IDR but got P-frame (frame=%lu size=%zu)\n",
            (unsigned long)frame_count_, packet_size);
  }

  VideoFrameType actual_frame_type =
      is_keyframe ? VideoFrameType::kVideoFrameKey
                  : VideoFrameType::kVideoFrameDelta;

  // Bitstream dump for offline analysis
  if (g_dump_fd >= 0 && g_dump_written < kDumpFrameCount) {
    ssize_t written = write(g_dump_fd, packet_data, packet_size);
    if (written > 0) {
      g_dump_written++;
    }
    if (g_dump_written >= kDumpFrameCount) {
      RTC_LOG(LS_INFO) << "Bitstream dump complete: " << g_dump_written
                       << " frames to /tmp/mpp_dump."
                       << (codec_type_ == MPP_VIDEO_CodingHEVC ? "h265" : "h264");
    }
  }

  // Log frame stats every 60 frames
  if (frame_count_ % 60 == 0) {
    fprintf(stderr, "[MPP] Frame %lu %s size=%zu w=%d h=%d\n",
            (unsigned long)frame_count_,
            is_keyframe ? "[IDR]" : "[P]",
            packet_size,
            codec_settings_.width, codec_settings_.height);
  }

  // Create EncodedImage
  EncodedImage encoded_image;
  encoded_image.SetEncodedData(
      EncodedImageBuffer::Create(static_cast<const uint8_t*>(packet_data),
                                 packet_size));
  encoded_image._encodedWidth = codec_settings_.width;
  encoded_image._encodedHeight = codec_settings_.height;
  encoded_image._frameType = actual_frame_type;
  encoded_image.SetRtpTimestamp(rtp_timestamp);
  encoded_image.capture_time_ms_ = timestamp_us / 1000;

  // Release packet before callback (data was copied into EncodedImageBuffer)
  mpp_packet_deinit(&mpp_packet);

  // Codec-specific info
  CodecSpecificInfo codec_info;

  if (codec_type_ == MPP_VIDEO_CodingHEVC) {
    codec_info.codecType = kVideoCodecH265;
  } else {
    codec_info.codecType = kVideoCodecH264;
    codec_info.codecSpecific.H264.packetization_mode =
        H264PacketizationMode::NonInterleaved;
    codec_info.codecSpecific.H264.temporal_idx = kNoTemporalIdx;
    codec_info.codecSpecific.H264.idr_frame = is_keyframe;
    codec_info.codecSpecific.H264.base_layer_sync = false;
  }

  // Deliver to callback
  EncodedImageCallback::Result result =
      encoded_callback_->OnEncodedImage(encoded_image, &codec_info);

  if (result.error != EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_ERROR) << "Encode callback failed: " << result.error;
    return false;
  }

  return true;
}

void RockchipVideoEncoder::SetRates(const RateControlParameters& parameters) {
  webrtc::MutexLock lock(&mutex_);

  if (!mpp_ctx_ || !mpp_mpi_) {
    return;
  }

  bitrate_bps_ = parameters.bitrate.get_sum_bps();

  if (parameters.framerate_fps > 0) {
    framerate_ = static_cast<int32_t>(parameters.framerate_fps);
  }

  fprintf(stderr, "[MPP] SetRates: bitrate=%d fps=%d\n", bitrate_bps_, framerate_);
  RTC_LOG(LS_INFO) << "SetRates: bitrate=" << bitrate_bps_
                   << " fps=" << framerate_;

  MPP_RET ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_GET_CFG, mpp_cfg_);
  if (ret == MPP_OK) {
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_target", bitrate_bps_);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_max", bitrate_bps_ * 17 / 16);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_min", bitrate_bps_ * 1 / 16);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_num", framerate_);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_num", framerate_);

    ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_CFG, mpp_cfg_);
    if (ret != MPP_OK) {
      RTC_LOG(LS_ERROR) << "Failed to update rate control: " << ret;
    }
  }
}

VideoEncoder::EncoderInfo RockchipVideoEncoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = (codec_type_ == MPP_VIDEO_CodingHEVC)
                                 ? "RockchipMPP_HEVC" : "RockchipMPP";
  info.has_trusted_rate_controller = true;
  info.is_hardware_accelerated = true;
  info.supports_simulcast = false;
  info.resolution_bitrate_limits = {};
  info.requested_resolution_alignment = 2;

  // Tell WebRTC we accept both NV12 (preferred) and I420.
  info.preferred_pixel_formats = {VideoFrameBuffer::Type::kNV12,
                                  VideoFrameBuffer::Type::kI420};

  // Lock resolution — no downscaling.  The hardware encoder handles low
  // bitrate at full resolution fine; quality on a 4K TV matters more
  // than framerate.
  info.scaling_settings = VideoEncoder::ScalingSettings::kOff;
  info.resolution_bitrate_limits = {
      VideoEncoder::ResolutionBitrateLimits(320 * 180, 30000, 30000, 500000),
      VideoEncoder::ResolutionBitrateLimits(640 * 360, 30000, 30000, 1500000),
      VideoEncoder::ResolutionBitrateLimits(1280 * 720, 30000, 30000, 4000000),
      VideoEncoder::ResolutionBitrateLimits(1920 * 1080, 30000, 30000, 8000000),
  };

  return info;
}

}  // namespace webrtc
