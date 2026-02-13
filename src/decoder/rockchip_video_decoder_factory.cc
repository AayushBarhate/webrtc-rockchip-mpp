// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/video_coding/codecs/mpp/rockchip_video_decoder_factory.h"

#include <memory>
#include <optional>
#include <string>

#include "absl/strings/match.h"
#include "api/environment/environment.h"
#include "api/video_codecs/h264_profile_level_id.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_decoder.h"
#include "media/base/codec.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/mpp/rockchip_video_decoder.h"
#include "rtc_base/logging.h"

namespace webrtc {

namespace {

// Check if hardware decoder is available
bool IsRockchipDecoderAvailable() {
  // Check for RK3588 or similar SoC
  // In production, add runtime detection (e.g., check /dev/mpp_service)
  return true;  // Assume available for now
}

}  // namespace

RockchipVideoDecoderFactory::RockchipVideoDecoderFactory() {
  RTC_LOG(LS_INFO) << "RockchipVideoDecoderFactory created";
}

RockchipVideoDecoderFactory::~RockchipVideoDecoderFactory() {
  RTC_LOG(LS_INFO) << "RockchipVideoDecoderFactory destroyed";
}

std::vector<SdpVideoFormat>
RockchipVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> supported_formats;

  if (!IsRockchipDecoderAvailable()) {
    RTC_LOG(LS_WARNING) << "Rockchip hardware decoder not available";
    return supported_formats;
  }

  // MPP hardware decoder handles all H.264 profiles.
  // Advertise all of them so the wrapper's IsSameCodec check matches
  // regardless of what profile the remote peer negotiates.

  // Constrained Baseline (42e01f) — most common in WebRTC
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, kH264ProfileLevelConstrainedBaseline},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  // Baseline (42001f)
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, "42001f"},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  // Main (4d001f)
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, "4d001f"},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  // High (64001f)
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, "64001f"},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  // Constrained High (640c1f)
  supported_formats.push_back(SdpVideoFormat(
      kH264CodecName,
      {{kH264FmtpProfileLevelId, kH264ProfileLevelConstrainedHigh},
       {kH264FmtpLevelAsymmetryAllowed, "1"},
       {kH264FmtpPacketizationMode, "1"}}));

  RTC_LOG(LS_INFO) << "Rockchip decoder supports " << supported_formats.size()
                   << " H.264 profiles";

  return supported_formats;
}

bool RockchipVideoDecoderFactory::IsFormatSupported(
    const SdpVideoFormat& format) const {
  if (!IsRockchipDecoderAvailable()) {
    return false;
  }

  // Only H.264 is supported by RK3588 MPP hardware decoder
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

  // MPP decoder supports Baseline, Main, and High profiles
  if (profile_level_id->profile != H264Profile::kProfileConstrainedBaseline &&
      profile_level_id->profile != H264Profile::kProfileBaseline &&
      profile_level_id->profile != H264Profile::kProfileMain &&
      profile_level_id->profile != H264Profile::kProfileHigh &&
      profile_level_id->profile != H264Profile::kProfileConstrainedHigh) {
    RTC_LOG(LS_WARNING) << "Unsupported H.264 profile: "
                        << static_cast<int>(profile_level_id->profile);
    return false;
  }

  return true;
}

std::unique_ptr<VideoDecoder>
RockchipVideoDecoderFactory::Create(
    const Environment& env,
    const SdpVideoFormat& format) {
  if (!IsFormatSupported(format)) {
    RTC_LOG(LS_ERROR) << "Unsupported video format: " << format.name;
    return nullptr;
  }

  RTC_LOG(LS_INFO) << "Creating RockchipVideoDecoder for " << format.name;
  return std::make_unique<RockchipVideoDecoder>();
}

}  // namespace webrtc
