// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/video_coding/codecs/mpp/rockchip_video_encoder.h"

#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "api/scoped_refptr.h"
#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
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
constexpr int kDefaultGOP = 60;
constexpr int kDefaultFPS = 30;
constexpr int kDefaultBitrate = 2000000;  // 2 Mbps
constexpr int kIDRFrameInterval = 60;     // Force IDR every 60 frames

// Helper to convert WebRTC frame type to MPP
bool IsKeyFrame(const std::vector<VideoFrameType>* frame_types) {
  if (!frame_types || frame_types->empty()) {
    return false;
  }
  return (*frame_types)[0] == VideoFrameType::kVideoFrameKey;
}

}  // namespace

RockchipVideoEncoder::RockchipVideoEncoder()
    : mpp_ctx_(nullptr),
      mpp_mpi_(nullptr),
      buffer_group_(nullptr),
      mpp_cfg_(nullptr),
      encoded_callback_(nullptr),
      bitrate_bps_(kDefaultBitrate),
      framerate_(kDefaultFPS),
      frame_count_(0),
      last_timestamp_ms_(0) {
  RTC_LOG(LS_INFO) << "RockchipVideoEncoder: Constructor";
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

  RTC_LOG(LS_INFO) << "InitEncode: " << codec_settings->width << "x"
                   << codec_settings->height << " @ " << framerate_
                   << " fps, bitrate: " << bitrate_bps_;

  // Step 1: Create MPP context
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_mpi_);
  if (ret != MPP_OK || !mpp_ctx_ || !mpp_mpi_) {
    RTC_LOG(LS_ERROR) << "mpp_create failed: " << ret;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Step 2: Initialize MPP for H.264 encoding
  ret = mpp_init(mpp_ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingAVC);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "mpp_init failed: " << ret;
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Step 3: Create internal buffer group for frame allocation
  ret = mpp_buffer_group_get_internal(&buffer_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "mpp_buffer_group_get_internal failed: " << ret;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
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

  // Step 6: Configure SEI
  MppEncSeiMode sei_mode = MPP_ENC_SEI_MODE_ONE_FRAME;
  ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_SEI_CFG, &sei_mode);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "MPP_ENC_SET_SEI_CFG failed: " << ret;
  }

  // Step 7: Prepend SPS/PPS to each IDR frame
  MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
  ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_HEADER_MODE, &header_mode);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "MPP_ENC_SET_HEADER_MODE failed: " << ret;
  }

  frame_count_ = 0;
  last_timestamp_ms_ = 0;

  RTC_LOG(LS_INFO) << "RockchipVideoEncoder initialized successfully"
                   << " (Constrained Baseline Profile)";
  return WEBRTC_VIDEO_CODEC_OK;
}

bool RockchipVideoEncoder::ConfigureEncoder() {
  // Prepare configuration
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:width", codec_settings_.width);
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:height", codec_settings_.height);
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:hor_stride",
                      MPP_ALIGN(codec_settings_.width, 16));
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:ver_stride",
                      MPP_ALIGN(codec_settings_.height, 16));
  mpp_enc_cfg_set_s32(mpp_cfg_, "prep:format", MPP_FMT_YUV420SP);  // NV12

  // H.264 Constrained Baseline Profile — required for WebRTC browser compat.
  // The factory advertises CB in SDP; the encoder MUST match.
  mpp_enc_cfg_set_s32(mpp_cfg_, "codec:type", MPP_VIDEO_CodingAVC);
  mpp_enc_cfg_set_s32(mpp_cfg_, "h264:profile", 66);   // Baseline Profile
  mpp_enc_cfg_set_s32(mpp_cfg_, "h264:level", 31);     // Level 3.1
  mpp_enc_cfg_set_s32(mpp_cfg_, "h264:cabac_en", 0);   // CAVLC only (Baseline)
  mpp_enc_cfg_set_s32(mpp_cfg_, "h264:cabac_idc", 0);
  mpp_enc_cfg_set_s32(mpp_cfg_, "h264:trans8x8", 0);   // No 8x8 (Baseline)

  RTC_LOG(LS_INFO) << "Encoder: Constrained Baseline, Level 3.1";
  return true;
}

bool RockchipVideoEncoder::ConfigureRateControl() {
  // Rate control - VBR for better visual quality in WebRTC
  // VBR allows the encoder to allocate more bits to complex frames,
  // preventing the heavy pixelation that CBR causes under tight bounds.
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:mode", MPP_ENC_RC_MODE_VBR);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_target", bitrate_bps_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_max", bitrate_bps_ * 2);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_min", bitrate_bps_ / 4);

  // QP bounds — prevent extreme quantization that causes pixelation.
  // Lower QP = higher quality. Range: 0 (best) to 51 (worst).
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_init", 26);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_min", 10);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_max", 38);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_min_i", 10);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:qp_max_i", 30);

  // FPS configuration
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_flex", 0);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_num", framerate_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_in_denorm", 1);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_flex", 0);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_num", framerate_);
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_denorm", 1);

  // GOP configuration
  mpp_enc_cfg_set_s32(mpp_cfg_, "rc:gop", kDefaultGOP);

  RTC_LOG(LS_INFO) << "Rate control configured: VBR " << bitrate_bps_
                   << " bps @ " << framerate_ << " fps"
                   << " (QP: init=26, I=[10,30], P=[10,38])";
  return true;
}

int32_t RockchipVideoEncoder::RegisterEncodeCompleteCallback(
    EncodedImageCallback* callback) {
  webrtc::MutexLock lock(&mutex_);
  encoded_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipVideoEncoder::Release() {
  RTC_LOG(LS_INFO) << "RockchipVideoEncoder::Release";

  webrtc::MutexLock lock(&mutex_);

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

  info.type = MPP_BUFFER_TYPE_DMA_HEAP;
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

  bool force_keyframe = IsKeyFrame(frame_types) ||
                        (frame_count_ == 0) ||
                        (frame_count_ % kIDRFrameInterval == 0);
  VideoFrameType webrtc_frame_type = force_keyframe
                                          ? VideoFrameType::kVideoFrameKey
                                          : VideoFrameType::kVideoFrameDelta;

  // Extract video frame buffer
  rtc::scoped_refptr<VideoFrameBuffer> buffer = frame.video_frame_buffer();

  MppFrame mpp_frame = nullptr;
  MPP_RET ret = mpp_frame_init(&mpp_frame);
  if (ret != MPP_OK || !mpp_frame) {
    RTC_LOG(LS_ERROR) << "mpp_frame_init failed: " << ret;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Check if this is a native buffer (DMA-BUF backed) for zero-copy path
  if (buffer->type() == VideoFrameBuffer::Type::kNative) {
#ifdef __linux__
    auto* dmabuf_buffer = static_cast<DMABufVideoFrameBuffer*>(buffer.get());
    int dmabuf_fd = dmabuf_buffer->GetDmaBufFd();
    size_t buffer_size = dmabuf_buffer->GetSize();

    if (dmabuf_fd >= 0 && buffer_size > 0) {
      RTC_LOG(LS_VERBOSE) << "Zero-copy encode path: DMA-BUF fd=" << dmabuf_fd;

      MppBuffer mpp_buffer = ImportDMABuffer(dmabuf_fd, buffer_size,
                                             frame.width(), frame.height());
      if (mpp_buffer) {
        mpp_frame_set_buffer(mpp_frame, mpp_buffer);
        mpp_buffer_put(mpp_buffer);
      } else {
        RTC_LOG(LS_WARNING) << "DMA-BUF import failed, falling back to CPU copy";
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
    // CPU copy path: convert I420 to NV12 into MPP buffer
    int hor_stride = MPP_ALIGN(frame.width(), 16);
    int ver_stride = MPP_ALIGN(frame.height(), 16);
    size_t frame_size = hor_stride * ver_stride * 3 / 2;  // NV12

    MppBuffer mpp_buffer = nullptr;
    ret = mpp_buffer_get(buffer_group_, &mpp_buffer, frame_size);
    if (ret != MPP_OK) {
      RTC_LOG(LS_ERROR) << "mpp_buffer_get failed: " << ret;
      mpp_frame_deinit(&mpp_frame);
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    void* mpp_ptr = mpp_buffer_get_ptr(mpp_buffer);
    rtc::scoped_refptr<I420BufferInterface> i420_buffer = buffer->ToI420();

    // Convert I420 to NV12 (Y plane + interleaved UV)
    uint8_t* dst_y = static_cast<uint8_t*>(mpp_ptr);
    uint8_t* dst_uv = dst_y + hor_stride * ver_stride;

    libyuv::I420ToNV12(
        i420_buffer->DataY(), i420_buffer->StrideY(),
        i420_buffer->DataU(), i420_buffer->StrideU(),
        i420_buffer->DataV(), i420_buffer->StrideV(),
        dst_y, hor_stride,
        dst_uv, hor_stride,
        frame.width(), frame.height());

    mpp_frame_set_buffer(mpp_frame, mpp_buffer);
    mpp_buffer_put(mpp_buffer);
  }

  // Configure frame properties
  mpp_frame_set_width(mpp_frame, frame.width());
  mpp_frame_set_height(mpp_frame, frame.height());
  mpp_frame_set_hor_stride(mpp_frame, MPP_ALIGN(frame.width(), 16));
  mpp_frame_set_ver_stride(mpp_frame, MPP_ALIGN(frame.height(), 16));
  mpp_frame_set_fmt(mpp_frame, MPP_FMT_YUV420SP);
  mpp_frame_set_eos(mpp_frame, 0);

  // Set PTS (presentation timestamp)
  mpp_frame_set_pts(mpp_frame, frame.timestamp_us());

  // Force keyframe if needed
  if (force_keyframe) {
    mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_IDR_FRAME, nullptr);
    RTC_LOG(LS_VERBOSE) << "Forcing keyframe";
  }

  // Send frame to encoder
  if (!SendFrame(mpp_frame)) {
    mpp_frame_deinit(&mpp_frame);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  mpp_frame_deinit(&mpp_frame);

  // Retrieve encoded packet
  if (!RetrievePacket(frame.timestamp_us(), webrtc_frame_type,
                      force_keyframe)) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  frame_count_++;
  return WEBRTC_VIDEO_CODEC_OK;
}

bool RockchipVideoEncoder::SendFrame(MppFrame mpp_frame) {
  MPP_RET ret = mpp_mpi_->encode_put_frame(mpp_ctx_, mpp_frame);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "encode_put_frame failed: " << ret;
    return false;
  }
  return true;
}

bool RockchipVideoEncoder::RetrievePacket(int64_t timestamp_us,
                                           VideoFrameType frame_type,
                                           bool force_keyframe) {
  MppPacket mpp_packet = nullptr;

  // Retrieve encoded packet (blocking)
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

  // Detect keyframes via MPP flag
  bool is_keyframe = (packet_flag & MPP_PACKET_FLAG_INTRA) != 0;

  // Also check H.264 NAL unit type for keyframe detection
  // (MPP doesn't always set the INTRA flag reliably)
  if (!is_keyframe && packet_size >= 5) {
    const uint8_t* p = static_cast<const uint8_t*>(packet_data);
    // Skip Annex B start code (00 00 00 01 or 00 00 01)
    int offset = 0;
    if (p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) offset = 4;
    else if (p[0] == 0 && p[1] == 0 && p[2] == 1) offset = 3;
    if (offset > 0 && (int)packet_size > offset) {
      uint8_t nal_type = p[offset] & 0x1F;
      // NAL type 5 = IDR slice, NAL type 7 = SPS (precedes IDR)
      if (nal_type == 7 || nal_type == 5) {
        is_keyframe = true;
      }
    }
  }

  // If we requested a keyframe, trust our request
  if (force_keyframe) {
    is_keyframe = true;
  }

  VideoFrameType actual_frame_type =
      is_keyframe ? VideoFrameType::kVideoFrameKey
                  : VideoFrameType::kVideoFrameDelta;

  RTC_LOG(LS_VERBOSE) << "Retrieved packet: size=" << packet_size
                      << " keyframe=" << is_keyframe;

  // Create EncodedImage
  EncodedImage encoded_image;
  encoded_image.SetEncodedData(
      EncodedImageBuffer::Create(static_cast<const uint8_t*>(packet_data),
                                 packet_size));
  encoded_image._encodedWidth = codec_settings_.width;
  encoded_image._encodedHeight = codec_settings_.height;
  encoded_image._frameType = actual_frame_type;
  encoded_image.SetRtpTimestamp(frame_count_);
  encoded_image.capture_time_ms_ = timestamp_us / 1000;

  // Release packet
  mpp_packet_deinit(&mpp_packet);

  // H.264 codec-specific info required for proper RTP packetization
  CodecSpecificInfo codec_info;
  codec_info.codecType = kVideoCodecH264;
  codec_info.codecSpecific.H264.packetization_mode =
      H264PacketizationMode::NonInterleaved;
  codec_info.codecSpecific.H264.temporal_idx = kNoTemporalIdx;
  codec_info.codecSpecific.H264.idr_frame = is_keyframe;
  codec_info.codecSpecific.H264.base_layer_sync = false;

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

  RTC_LOG(LS_INFO) << "SetRates: bitrate=" << bitrate_bps_
                   << " fps=" << framerate_;

  // Update rate control dynamically
  MPP_RET ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_GET_CFG, mpp_cfg_);
  if (ret == MPP_OK) {
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_target", bitrate_bps_);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_max", bitrate_bps_ * 2);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:bps_min", bitrate_bps_ / 4);
    mpp_enc_cfg_set_s32(mpp_cfg_, "rc:fps_out_num", framerate_);

    ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_CFG, mpp_cfg_);
    if (ret != MPP_OK) {
      RTC_LOG(LS_ERROR) << "Failed to update rate control: " << ret;
    }
  }
}

VideoEncoder::EncoderInfo RockchipVideoEncoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = true;  // DMA-BUF support
  info.implementation_name = "RockchipMPP";
  info.has_trusted_rate_controller = true;
  info.is_hardware_accelerated = true;
  info.supports_simulcast = false;

  // Resolution alignment requirements for MPP
  info.resolution_bitrate_limits = {};
  info.requested_resolution_alignment = 16;  // RK3588 requires 16-byte alignment

  return info;
}

}  // namespace webrtc
