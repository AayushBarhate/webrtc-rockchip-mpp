// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/video_coding/codecs/mpp/rockchip_video_encoder_factory.h"

#include <memory>
#include <optional>
#include <string>

#include "absl/strings/match.h"
#include "api/environment/environment.h"
#include "api/video_codecs/h264_profile_level_id.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder.h"
#include "media/base/codec.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/mpp/rockchip_video_encoder.h"
#include "rtc_base/logging.h"

namespace webrtc {

namespace {

// Check if hardware encoder is available
bool IsRockchipEncoderAvailable() {
  // Check for RK3588 or similar SoC
  // In production, add runtime detection (e.g., check /dev/mpp_service)
  return true;  // Assume available for now
}

}  // namespace

RockchipVideoEncoderFactory::RockchipVideoEncoderFactory() {
  RTC_LOG(LS_INFO) << "RockchipVideoEncoderFactory created";
}

RockchipVideoEncoderFactory::~RockchipVideoEncoderFactory() {
  RTC_LOG(LS_INFO) << "RockchipVideoEncoderFactory destroyed";
}

std::vector<SdpVideoFormat> RockchipVideoEncoderFactory::GetSupportedFormats()
    const {
  std::vector<SdpVideoFormat> supported_formats;

  if (!IsRockchipEncoderAvailable()) {
    RTC_LOG(LS_WARNING) << "Rockchip hardware encoder not available";
    return supported_formats;
  }

  // H.265/HEVC — offered first as preferred codec.
  // Chrome 136+ and Safari support HEVC in WebRTC.
  // Firefox does not — H.264 fallback handles that.
  supported_formats.push_back(SdpVideoFormat(
      kH265CodecName,
      {{"profile-id", "1"}}));  // Main Profile

  // H.264 Constrained High Profile — CABAC enabled for ~15% better
  // compression.  Modern WebRTC endpoints handle this fine.
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, kH264ProfileLevelConstrainedHigh},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  // H.264 Constrained Baseline Profile (Level 3.1)
  // Widest compatibility fallback
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, kH264ProfileLevelConstrainedBaseline},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  RTC_LOG(LS_INFO) << "Rockchip encoder supports " << supported_formats.size()
                   << " codec profiles (H.265 + H.264)";

  return supported_formats;
}

bool RockchipVideoEncoderFactory::IsFormatSupported(
    const SdpVideoFormat& format) const {
  if (!IsRockchipEncoderAvailable()) {
    return false;
  }

  // H.265/HEVC
  if (absl::EqualsIgnoreCase(format.name, kH265CodecName)) {
    return true;
  }

  // H.264
  if (!absl::EqualsIgnoreCase(format.name, kH264CodecName)) {
    return false;
  }

  // Validate H.264 profile
  const std::optional<H264ProfileLevelId> profile_level_id =
      ParseSdpForH264ProfileLevelId(format.parameters);

  if (!profile_level_id) {
    RTC_LOG(LS_WARNING) << "Invalid H.264 profile-level-id";
    return false;
  }

  // MPP supports Baseline, Main, and High profiles
  if (profile_level_id->profile != H264Profile::kProfileBaseline &&
      profile_level_id->profile != H264Profile::kProfileConstrainedBaseline &&
      profile_level_id->profile != H264Profile::kProfileMain &&
      profile_level_id->profile != H264Profile::kProfileConstrainedHigh &&
      profile_level_id->profile != H264Profile::kProfileHigh) {
    RTC_LOG(LS_WARNING) << "Unsupported H.264 profile: "
                        << static_cast<int>(profile_level_id->profile);
    return false;
  }

  return true;
}

std::unique_ptr<VideoEncoder>
RockchipVideoEncoderFactory::Create(const Environment& env,
                                    const SdpVideoFormat& format) {
  if (!IsFormatSupported(format)) {
    RTC_LOG(LS_ERROR) << "Unsupported video format: " << format.name;
    return nullptr;
  }

  // Select codec type based on negotiated format
  if (absl::EqualsIgnoreCase(format.name, kH265CodecName)) {
    RTC_LOG(LS_INFO) << "Creating RockchipVideoEncoder for H.265/HEVC";
    return std::make_unique<RockchipVideoEncoder>(MPP_VIDEO_CodingHEVC);
  }

  RTC_LOG(LS_INFO) << "Creating RockchipVideoEncoder for H.264";
  return std::make_unique<RockchipVideoEncoder>(MPP_VIDEO_CodingAVC);
}

VideoEncoderFactory::CodecSupport
RockchipVideoEncoderFactory::QueryCodecSupport(
    const SdpVideoFormat& format,
    std::optional<std::string> scalability_mode) const {
  CodecSupport codec_support;
  codec_support.is_supported = IsFormatSupported(format);

  if (codec_support.is_supported) {
    codec_support.is_power_efficient = true;  // Hardware encoder
  }

  return codec_support;
}

}  // namespace webrtc
