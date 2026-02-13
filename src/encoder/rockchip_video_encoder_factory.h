// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_FACTORY_H_
#define MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_FACTORY_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "api/environment/environment.h"
#include "api/video_codecs/video_encoder.h"
#include "api/video_codecs/video_encoder_factory.h"

namespace webrtc {

class RockchipVideoEncoderFactory : public VideoEncoderFactory {
 public:
  RockchipVideoEncoderFactory();
  ~RockchipVideoEncoderFactory() override;

  // VideoEncoderFactory implementation.
  std::vector<SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<VideoEncoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) override;

  CodecSupport QueryCodecSupport(
      const SdpVideoFormat& format,
      std::optional<std::string> scalability_mode) const override;

 private:
  bool IsFormatSupported(const SdpVideoFormat& format) const;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_ENCODER_FACTORY_H_
