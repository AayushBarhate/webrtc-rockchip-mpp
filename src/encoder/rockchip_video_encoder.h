// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_H_
#define MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_H_

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "api/video_codecs/video_encoder.h"
#include "common_video/include/video_frame_buffer.h"
#include "rtc_base/synchronization/mutex.h"

// Rockchip MPP headers
extern "C" {
#include "rk_mpi.h"
#include "rk_type.h"
#include "mpp_buffer.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "mpp_meta.h"
#include "mpp_err.h"
#include "rk_venc_cmd.h"
#include "rk_venc_cfg.h"
#include "mpp_common.h"
#include "mpp_packet_impl.h"
}

namespace webrtc {

// Stride alignment for RK3588 — camera HAL and VPU both expect 64-byte
// aligned horizontal stride for NV12/NV21 buffers.
constexpr int kRk3588StrideAlign = 64;

// Buffer pool size for non-blocking encode pipeline.
constexpr int kBufferPoolSize = 3;

class RockchipVideoEncoder : public VideoEncoder {
 public:
  // codec_type: MPP_VIDEO_CodingAVC or MPP_VIDEO_CodingHEVC
  explicit RockchipVideoEncoder(
      MppCodingType codec_type = MPP_VIDEO_CodingAVC);
  ~RockchipVideoEncoder() override;

  // VideoEncoder implementation.
  void SetFecControllerOverride(
      FecControllerOverride* fec_controller_override) override;

  int InitEncode(const VideoCodec* codec_settings,
                 const VideoEncoder::Settings& settings) override;

  int32_t RegisterEncodeCompleteCallback(
      EncodedImageCallback* callback) override;

  int32_t Release() override;

  int32_t Encode(const VideoFrame& frame,
                 const std::vector<VideoFrameType>* frame_types) override;

  void SetRates(const RateControlParameters& parameters) override;

  EncoderInfo GetEncoderInfo() const override;

  // Request GDR (gradual decoder refresh) instead of full IDR on next PLI.
  void RequestGradualRefresh();

 private:
  // Codec type (AVC or HEVC)
  MppCodingType codec_type_;

  // MPP context and API
  MppCtx mpp_ctx_;
  MppApi* mpp_mpi_;
  MppBufferGroup buffer_group_;

  // Buffer pool: pre-allocated MppBuffers for non-blocking encode
  MppBuffer buffer_pool_[kBufferPoolSize];
  int buffer_pool_idx_;

  // Encoder configuration
  MppEncCfg mpp_cfg_;

  // WebRTC state
  EncodedImageCallback* encoded_callback_;
  VideoCodec codec_settings_;
  int32_t bitrate_bps_;
  int32_t framerate_;

  // Frame tracking
  uint64_t frame_count_;
  int64_t last_timestamp_ms_;

  // Scene change detection state
  uint32_t prev_histogram_[16];
  bool has_prev_histogram_;

  // GDR (gradual decoder refresh) state
  bool gdr_requested_;
  int total_mbs_;

  // LTR (long-term reference) state
  uint64_t ltr_interval_;
  int32_t ltr_index_;

  // Thread safety
  webrtc::Mutex mutex_;

  // Cached stride values
  int hor_stride_;
  int ver_stride_;

  // Helper functions
  bool ConfigureEncoder();
  bool ConfigureRateControl();
  bool ConfigureROI(MppFrame mpp_frame);
  bool InitBufferPool();
  void DestroyBufferPool();
  MppBuffer GetPoolBuffer();
  MppBuffer ImportDMABuffer(int fd, size_t size, int width, int height);
  bool SendFrame(MppFrame mpp_frame);
  bool RetrievePacket(uint32_t rtp_timestamp, int64_t timestamp_us,
                      VideoFrameType frame_type,
                      bool force_keyframe = false);

  // Scene change detection
  bool DetectSceneChange(const uint8_t* y_plane, int width, int height,
                         int stride);

  // Lightweight histogram computation on downscaled frame
  void ComputeHistogram(const uint8_t* y_plane, int width, int height,
                        int stride, uint32_t histogram[16]);

  // Disallow copy and assign
  RockchipVideoEncoder(const RockchipVideoEncoder&) = delete;
  RockchipVideoEncoder& operator=(const RockchipVideoEncoder&) = delete;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_H_
