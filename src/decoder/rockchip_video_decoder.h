// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_DECODER_H_
#define MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_DECODER_H_

#include <memory>
#include <vector>

#include "api/video_codecs/video_decoder.h"
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
#include "rk_vdec_cmd.h"
#include "rk_vdec_cfg.h"
#include "mpp_common.h"
}

namespace webrtc {

class RockchipVideoDecoder : public VideoDecoder {
 public:
  // codec_type: MPP_VIDEO_CodingAVC or MPP_VIDEO_CodingHEVC
  explicit RockchipVideoDecoder(
      MppCodingType codec_type = MPP_VIDEO_CodingAVC);
  ~RockchipVideoDecoder() override;

  // VideoDecoder implementation.
  bool Configure(const Settings& settings) override;

  int32_t Decode(const EncodedImage& input_image,
                 bool missing_frames,
                 int64_t render_time_ms) override;

  int32_t RegisterDecodeCompleteCallback(
      DecodedImageCallback* callback) override;

  int32_t Release() override;

  const char* ImplementationName() const override;

 private:
  // Codec type (AVC or HEVC)
  MppCodingType codec_type_;

  // MPP context and API
  MppCtx mpp_ctx_;
  MppApi* mpp_mpi_;
  MppBufferGroup frame_group_;

  // Decoder configuration
  MppDecCfg mpp_cfg_;

  // WebRTC state
  DecodedImageCallback* decoded_callback_;
  int32_t width_;
  int32_t height_;

  // Frame tracking
  uint64_t frame_count_;
  int64_t last_decode_time_ms_;

  // Thread safety
  webrtc::Mutex mutex_;

  // Helper functions
  bool ConfigureDecoder();
  bool SendPacket(const uint8_t* data, size_t size, int64_t timestamp);
  bool RetrieveFrame(int64_t timestamp);
  rtc::scoped_refptr<VideoFrameBuffer> ConvertMppFrameToVideoFrame(
      MppFrame mpp_frame);

  // Disallow copy and assign
  RockchipVideoDecoder(const RockchipVideoDecoder&) = delete;
  RockchipVideoDecoder& operator=(const RockchipVideoDecoder&) = delete;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_DECODER_H_
