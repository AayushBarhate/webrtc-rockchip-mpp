// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_H_
#define MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_H_

#include <memory>
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

class RockchipVideoEncoder : public VideoEncoder {
 public:
  RockchipVideoEncoder();
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

 private:
  // MPP context and API
  MppCtx mpp_ctx_;
  MppApi* mpp_mpi_;
  MppBufferGroup buffer_group_;

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

  // Thread safety
  webrtc::Mutex mutex_;

  // Helper functions
  bool ConfigureEncoder();
  bool ConfigureRateControl();
  MppBuffer ImportDMABuffer(int fd, size_t size, int width, int height);
  bool SendFrame(MppFrame mpp_frame);
  bool RetrievePacket(int64_t timestamp_us, VideoFrameType frame_type);

  // Disallow copy and assign
  RockchipVideoEncoder(const RockchipVideoEncoder&) = delete;
  RockchipVideoEncoder& operator=(const RockchipVideoEncoder&) = delete;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_H_
