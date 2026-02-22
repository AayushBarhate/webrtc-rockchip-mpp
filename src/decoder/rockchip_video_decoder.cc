// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/video_coding/codecs/mpp/rockchip_video_decoder.h"

#include <string.h>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "third_party/libyuv/include/libyuv.h"

namespace webrtc {

namespace {

constexpr int kMaxDecodeTimeoutMs = 100;  // 100ms timeout for decode

}  // namespace

RockchipVideoDecoder::RockchipVideoDecoder(MppCodingType codec_type)
    : codec_type_(codec_type),
      mpp_ctx_(nullptr),
      mpp_mpi_(nullptr),
      frame_group_(nullptr),
      mpp_cfg_(nullptr),
      decoded_callback_(nullptr),
      width_(0),
      height_(0),
      frame_count_(0),
      last_decode_time_ms_(0) {
  RTC_LOG(LS_INFO) << "RockchipVideoDecoder: Constructor (codec="
                    << (codec_type == MPP_VIDEO_CodingHEVC ? "HEVC" : "AVC")
                    << ")";
}

RockchipVideoDecoder::~RockchipVideoDecoder() {
  Release();
}

bool RockchipVideoDecoder::Configure(const Settings& settings) {
  RTC_LOG(LS_INFO) << "RockchipVideoDecoder::Configure";

  webrtc::MutexLock lock(&mutex_);

  // Step 1: Create MPP context
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_mpi_);
  if (ret != MPP_OK || !mpp_ctx_ || !mpp_mpi_) {
    RTC_LOG(LS_ERROR) << "mpp_create failed: " << ret;
    return false;
  }

  // Step 2: Initialize MPP for decoding (AVC or HEVC)
  ret = mpp_init(mpp_ctx_, MPP_CTX_DEC, codec_type_);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "mpp_init failed: " << ret;
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    return false;
  }

  // Step 3: Create buffer group for decoded frames
  ret = mpp_buffer_group_get_internal(&frame_group_, MPP_BUFFER_TYPE_DMA_HEAP);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "mpp_buffer_group_get_internal failed: " << ret
                        << ", trying ION";
    ret = mpp_buffer_group_get_internal(&frame_group_, MPP_BUFFER_TYPE_ION);
    if (ret != MPP_OK) {
      RTC_LOG(LS_ERROR) << "Failed to create buffer group: " << ret;
      Release();
      return false;
    }
  }

  // Step 4: Configure decoder (optional)
  if (!ConfigureDecoder()) {
    Release();
    return false;
  }

  // Step 5: Enable parser for stream parsing
  RK_U32 need_split = 1;
  ret = mpp_mpi_->control(mpp_ctx_, MPP_DEC_SET_PARSER_SPLIT_MODE,
                          &need_split);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "MPP_DEC_SET_PARSER_SPLIT_MODE failed: " << ret;
  }

  // Step 6: Set output frame format (NV12 preferred)
  MppFrameFormat fmt = MPP_FMT_YUV420SP;  // NV12
  ret = mpp_mpi_->control(mpp_ctx_, MPP_DEC_SET_OUTPUT_FORMAT, &fmt);
  if (ret != MPP_OK) {
    RTC_LOG(LS_WARNING) << "MPP_DEC_SET_OUTPUT_FORMAT failed: " << ret;
  }

  frame_count_ = 0;
  last_decode_time_ms_ = 0;

  const char* codec_name = (codec_type_ == MPP_VIDEO_CodingHEVC)
                               ? "HEVC" : "AVC";
  RTC_LOG(LS_INFO) << "RockchipVideoDecoder configured successfully ("
                   << codec_name << ")";
  return true;
}

bool RockchipVideoDecoder::ConfigureDecoder() {
  // Decoder configuration is mostly automatic
  // MPP handles stream parsing and resolution detection
  RTC_LOG(LS_INFO) << "Decoder will auto-configure from stream";
  return true;
}

int32_t RockchipVideoDecoder::RegisterDecodeCompleteCallback(
    DecodedImageCallback* callback) {
  webrtc::MutexLock lock(&mutex_);
  decoded_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipVideoDecoder::Release() {
  RTC_LOG(LS_INFO) << "RockchipVideoDecoder::Release";

  webrtc::MutexLock lock(&mutex_);

  if (mpp_cfg_) {
    mpp_dec_cfg_deinit(mpp_cfg_);
    mpp_cfg_ = nullptr;
  }

  if (frame_group_) {
    mpp_buffer_group_put(frame_group_);
    frame_group_ = nullptr;
  }

  if (mpp_ctx_) {
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    mpp_mpi_ = nullptr;
  }

  decoded_callback_ = nullptr;
  frame_count_ = 0;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipVideoDecoder::Decode(const EncodedImage& input_image,
                                      bool missing_frames,
                                      int64_t render_time_ms) {
  webrtc::MutexLock lock(&mutex_);

  if (!mpp_ctx_ || !mpp_mpi_) {
    RTC_LOG(LS_ERROR) << "Decode called before Configure";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  if (!decoded_callback_) {
    RTC_LOG(LS_WARNING) << "Decode callback not set";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  if (!input_image.data() || input_image.size() == 0) {
    RTC_LOG(LS_ERROR) << "Invalid encoded image";
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  int64_t start_time_ms = rtc::TimeMillis();

  RTC_LOG(LS_VERBOSE) << "Decode frame " << frame_count_ << " size="
                      << input_image.size() << " timestamp="
                      << input_image.RtpTimestamp();

  // Send packet to decoder
  if (!SendPacket(input_image.data(), input_image.size(),
                  input_image.RtpTimestamp())) {
    RTC_LOG(LS_ERROR) << "SendPacket failed";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Retrieve decoded frame
  if (!RetrieveFrame(input_image.RtpTimestamp())) {
    RTC_LOG(LS_ERROR) << "RetrieveFrame failed";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  int64_t decode_time_ms = rtc::TimeMillis() - start_time_ms;
  last_decode_time_ms_ = decode_time_ms;
  frame_count_++;

  RTC_LOG(LS_VERBOSE) << "Decode completed in " << decode_time_ms << "ms";

  return WEBRTC_VIDEO_CODEC_OK;
}

bool RockchipVideoDecoder::SendPacket(const uint8_t* data,
                                       size_t size,
                                       int64_t timestamp) {
  // Create MPP packet
  MppPacket mpp_packet = nullptr;
  MPP_RET ret = mpp_packet_init(&mpp_packet, (void*)data, size);
  if (ret != MPP_OK || !mpp_packet) {
    RTC_LOG(LS_ERROR) << "mpp_packet_init failed: " << ret;
    return false;
  }

  // Set packet timestamp
  mpp_packet_set_pts(mpp_packet, timestamp);
  mpp_packet_set_pos(mpp_packet, (void*)data);
  mpp_packet_set_length(mpp_packet, size);

  // Send packet to decoder
  ret = mpp_mpi_->decode_put_packet(mpp_ctx_, mpp_packet);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "decode_put_packet failed: " << ret;
    mpp_packet_deinit(&mpp_packet);
    return false;
  }

  mpp_packet_deinit(&mpp_packet);

  RTC_LOG(LS_VERBOSE) << "Packet sent to decoder: size=" << size;
  return true;
}

bool RockchipVideoDecoder::RetrieveFrame(int64_t timestamp) {
  MppFrame mpp_frame = nullptr;

  // Retrieve decoded frame (blocking with timeout)
  MPP_RET ret = mpp_mpi_->decode_get_frame(mpp_ctx_, &mpp_frame);
  if (ret != MPP_OK) {
    RTC_LOG(LS_ERROR) << "decode_get_frame failed: " << ret;
    return false;
  }

  if (!mpp_frame) {
    RTC_LOG(LS_WARNING) << "No frame retrieved (need more packets)";
    return true;  // Not an error, decoder may need more data
  }
  // Check for info-change frame (resolution change notification).
  // MPP sends these with dimensions but no data buffer.  We must
  // acknowledge and optionally configure the external buffer group,
  // then retrieve the real decoded frame.
  RK_U32 info_change = mpp_frame_get_info_change(mpp_frame);
  if (info_change) {
    RK_U32 w = mpp_frame_get_width(mpp_frame);
    RK_U32 h = mpp_frame_get_height(mpp_frame);
    RK_U32 hs = mpp_frame_get_hor_stride(mpp_frame);
    RK_U32 vs = mpp_frame_get_ver_stride(mpp_frame);
    RTC_LOG(LS_INFO) << "MPP info-change: " << w << "x" << h
                     << " stride=" << hs << "x" << vs;
    width_ = w;
    height_ = h;

    // If we have an external buffer group, attach it now.
    if (frame_group_) {
      MPP_RET r = mpp_mpi_->control(mpp_ctx_, MPP_DEC_SET_EXT_BUF_GROUP,
                                     frame_group_);
      if (r != MPP_OK) {
        RTC_LOG(LS_WARNING) << "MPP_DEC_SET_EXT_BUF_GROUP failed: " << r;
      }
    }

    // Tell MPP we're ready for the new resolution.
    MPP_RET r = mpp_mpi_->control(mpp_ctx_, MPP_DEC_SET_INFO_CHANGE_READY,
                                   nullptr);
    if (r != MPP_OK) {
      RTC_LOG(LS_WARNING) << "MPP_DEC_SET_INFO_CHANGE_READY failed: " << r;
    }

    mpp_frame_deinit(&mpp_frame);

    // Try to get the actual decoded frame now.
    mpp_frame = nullptr;
    ret = mpp_mpi_->decode_get_frame(mpp_ctx_, &mpp_frame);
    if (ret != MPP_OK || !mpp_frame) {
      // No frame yet — that's OK, it'll come on a subsequent Decode call.
      RTC_LOG(LS_INFO) << "No frame after info-change ack (will arrive later)";
      return true;
    }
  }

  // Check if frame has error
  RK_U32 err_info = mpp_frame_get_errinfo(mpp_frame);
  if (err_info) {
    RTC_LOG(LS_WARNING) << "Frame has error info: " << err_info;
    mpp_frame_deinit(&mpp_frame);
    return false;
  }

  // Check if frame is EOS
  RK_U32 eos = mpp_frame_get_eos(mpp_frame);
  if (eos) {
    RTC_LOG(LS_INFO) << "Received EOS frame";
    mpp_frame_deinit(&mpp_frame);
    return true;
  }

  // Check for buffer
  MppBuffer frame_buf = mpp_frame_get_buffer(mpp_frame);
  if (!frame_buf) {
    RTC_LOG(LS_WARNING) << "Frame has no buffer (skipping)";
    mpp_frame_deinit(&mpp_frame);
    return true;  // Not fatal — decoder may produce usable frames later.
  }

  // Get frame dimensions
  RK_U32 width = mpp_frame_get_width(mpp_frame);
  RK_U32 height = mpp_frame_get_height(mpp_frame);
  RK_S64 pts = mpp_frame_get_pts(mpp_frame);

  RTC_LOG(LS_VERBOSE) << "Decoded frame: " << width << "x" << height
                      << " pts=" << pts;

  // Update dimensions if changed
  if (width != static_cast<RK_U32>(width_) ||
      height != static_cast<RK_U32>(height_)) {
    width_ = width;
    height_ = height;
    RTC_LOG(LS_INFO) << "Decoder resolution changed to " << width_ << "x"
                     << height_;
  }

  // Convert MPP frame to VideoFrame
  rtc::scoped_refptr<VideoFrameBuffer> buffer =
      ConvertMppFrameToVideoFrame(mpp_frame);

  mpp_frame_deinit(&mpp_frame);

  if (!buffer) {
    RTC_LOG(LS_ERROR) << "Failed to convert MPP frame to VideoFrame";
    return false;
  }

  // Create VideoFrame
  VideoFrame decoded_frame = VideoFrame::Builder()
                                  .set_video_frame_buffer(buffer)
                                  .set_timestamp_rtp(static_cast<uint32_t>(pts))
                                  .build();

  // Deliver to callback
  decoded_callback_->Decoded(decoded_frame);

  RTC_LOG(LS_VERBOSE) << "Frame delivered to callback: " << width << "x"
                      << height;
  return true;
}

rtc::scoped_refptr<VideoFrameBuffer>
RockchipVideoDecoder::ConvertMppFrameToVideoFrame(MppFrame mpp_frame) {
  if (!mpp_frame) {
    return nullptr;
  }

  MppBuffer mpp_buffer = mpp_frame_get_buffer(mpp_frame);
  if (!mpp_buffer) {
    RTC_LOG(LS_ERROR) << "MppFrame has no buffer";
    return nullptr;
  }

  RK_U32 width = mpp_frame_get_width(mpp_frame);
  RK_U32 height = mpp_frame_get_height(mpp_frame);
  RK_U32 hor_stride = mpp_frame_get_hor_stride(mpp_frame);
  RK_U32 ver_stride = mpp_frame_get_ver_stride(mpp_frame);
  MppFrameFormat fmt = mpp_frame_get_fmt(mpp_frame);

  RTC_LOG(LS_VERBOSE) << "Converting frame: " << width << "x" << height
                      << " stride=" << hor_stride << "x" << ver_stride
                      << " format=" << fmt;

  // CPU copy: convert NV12/NV21 to I420 via libyuv.
  // DMA-BUF zero-copy is not used because mpp_frame_deinit() recycles
  // the underlying buffer, invalidating the fd before the frame is consumed.

  void* ptr = mpp_buffer_get_ptr(mpp_buffer);
  if (!ptr) {
    RTC_LOG(LS_ERROR) << "Cannot get buffer pointer";
    return nullptr;
  }

  rtc::scoped_refptr<I420Buffer> i420_buffer =
      I420Buffer::Create(width, height);

  // UV plane offset MUST use hor_stride * ver_stride (not width * height).
  // When stride > width (due to alignment), using width * height causes
  // the UV pointer to land inside the Y plane data, corrupting colors.
  const uint8_t* src_y = static_cast<const uint8_t*>(ptr);
  const uint8_t* src_uv = src_y + (hor_stride * ver_stride);

  // Convert based on format.
  // RK3588 RKVDEC reports MPP_FMT_YUV420SP (NV12) but the chroma plane
  // is actually VU-interleaved (NV21).  Use NV21ToI420 for both cases
  // on this SoC to get correct colors.
  if (fmt == MPP_FMT_YUV420SP || fmt == MPP_FMT_YUV420SP_VU) {
    libyuv::NV21ToI420(
        src_y, hor_stride,
        src_uv, hor_stride,
        i420_buffer->MutableDataY(), i420_buffer->StrideY(),
        i420_buffer->MutableDataU(), i420_buffer->StrideU(),
        i420_buffer->MutableDataV(), i420_buffer->StrideV(),
        width, height);
  } else {
    RTC_LOG(LS_ERROR) << "Unsupported MPP frame format: " << fmt;
    return nullptr;
  }

  return i420_buffer;
}

const char* RockchipVideoDecoder::ImplementationName() const {
  return (codec_type_ == MPP_VIDEO_CodingHEVC)
             ? "RockchipMPP_HEVC" : "RockchipMPP";
}

}  // namespace webrtc
