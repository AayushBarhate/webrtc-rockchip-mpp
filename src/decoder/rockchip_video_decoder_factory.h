// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_DECODER_FACTORY_H_
#define MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_DECODER_FACTORY_H_

#include <memory>
#include <vector>

#include "api/environment/environment.h"
#include "api/video_codecs/video_decoder.h"
#include "api/video_codecs/video_decoder_factory.h"

namespace webrtc {

class RockchipVideoDecoderFactory : public VideoDecoderFactory {
 public:
  RockchipVideoDecoderFactory();
  ~RockchipVideoDecoderFactory() override;

  // VideoDecoderFactory implementation.
  std::vector<SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<VideoDecoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) override;

 private:
  bool IsFormatSupported(const SdpVideoFormat& format) const;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_MPP_ROCKCHIP_VIDEO_DECODER_FACTORY_H_
