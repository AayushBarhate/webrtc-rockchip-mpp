# Beta Branch Setup Guide

This guide explains how to fetch and use the `beta` branches from both repositories to get the latest hardware-accelerated video encoding/decoding on RK3588.

## Repositories

| Repository | Description |
|---|---|
| [webrtc-rockchip-mpp](https://github.com/AayushBarhate/webrtc-rockchip-mpp) | Rockchip MPP encoder/decoder for WebRTC (builds `libwebrtc.a`) |
| [livekit-rockchip-sdk](https://github.com/AayushBarhate/livekit-rockchip-sdk) | LiveKit Rust SDK fork with MPP integration + video_call app |

## 1. Fetch the beta branches

```bash
# Clone webrtc-rockchip-mpp (encoder/decoder source)
git clone -b beta https://github.com/AayushBarhate/webrtc-rockchip-mpp.git
cd webrtc-rockchip-mpp

# If you already have the repo, just switch to beta:
git fetch origin
git checkout beta
git pull origin beta
```

```bash
# Clone livekit-rockchip-sdk (LiveKit Rust SDK + video_call app)
git clone -b beta https://github.com/AayushBarhate/livekit-rockchip-sdk.git
cd livekit-rockchip-sdk

# If you already have the repo, just switch to beta:
git fetch origin
git checkout beta
git pull origin beta
```

## 2. Build libwebrtc.a with MPP codecs

On the RK3588 device (or a cross-compile host):

```bash
cd webrtc-rockchip-mpp
./build_libwebrtc.sh --livekit-sdk-dir /path/to/livekit-rockchip-sdk
```

This builds `libwebrtc.a` with Rockchip MPP H.264 and H.265 hardware encoder/decoder baked in. Output lands in `linux-arm64-release/`.

## 3. Build the video_call app

```bash
cd livekit-rockchip-sdk

# Point the build at the custom libwebrtc
export LK_CUSTOM_WEBRTC=/path/to/webrtc-rockchip-mpp/linux-arm64-release

# Build
cargo build --release -p video_call
```

## 4. Configure

Create a `config.toml` in the `livekit-rockchip-sdk/` root directory (this file is gitignored):

```toml
[livekit]
url = "wss://YOUR-PROJECT.livekit.cloud"
api_key = "YOUR_API_KEY"
api_secret = "YOUR_API_SECRET"
room = "my-room"
identity = "rk3588-cam"

[video]
camera_index = 0
width = 1280
height = 720
fps = 30
codec = "h264"
max_bitrate = 2500000
simulcast = false
```

### Force a specific LiveKit Cloud region

By default, LiveKit Cloud routes you to the nearest server. To force a region (e.g., India), query the available regions and use the direct URL:

```bash
# Generate a JWT token, then:
curl -s 'https://YOUR-PROJECT.livekit.cloud/settings/regions' \
  -H "Authorization: Bearer <YOUR_JWT>" | python3 -m json.tool
```

Example response:
```json
{
  "regions": [
    {"region": "ohyderabad1a", "url": "https://YOUR-PROJECT.ohyderabad1a.production.livekit.cloud", "distance": "596622"},
    {"region": "omumbai1a", "url": "https://YOUR-PROJECT.omumbai1a.production.livekit.cloud", "distance": "669985"},
    {"region": "osingapore1b", "url": "https://YOUR-PROJECT.osingapore1b.production.livekit.cloud", "distance": "3467088"}
  ]
}
```

Use the closest region URL in your `config.toml`:
```toml
url = "wss://YOUR-PROJECT.ohyderabad1a.production.livekit.cloud"
```

## 5. Run

```bash
cd livekit-rockchip-sdk

# CSI camera (e.g., IMX219 on rkisp_mainpath = /dev/video11)
./target/release/video_call --camera-device /dev/video11 --codec h264 --no-display

# USB camera (auto-detects MJPEG)
./target/release/video_call --codec h264 --no-display
```

## What's in the beta branches

### webrtc-rockchip-mpp (beta)
- H.264 High Profile (100) with CABAC and 8x8 transform
- VBR rate control with tight +6.25% ceiling (bps_max = target * 17/16)
- Full QP range 10-51, qp_ip=2, max_reenc_times=1
- Resolution scaling disabled (ScalingSettings::kOff) for stable 720p
- H.265/HEVC encoder and decoder support
- Zero-copy DMA-BUF path for CSI cameras

### livekit-rockchip-sdk (beta)
- H.265/HEVC routing to Rockchip MPP hardware encoder and decoder
- H.265 bitrate scaling (0.7x) for efficient bandwidth usage
- QueryCodecSupport checks hardware factories first
- CSI camera NV12 zero-copy capture
- Side-by-side local/remote display, HDMI audio output

## Patching the encoder without a full rebuild

If you only change `rockchip_video_encoder.cc`, you can hot-patch `libwebrtc.a` without rebuilding all of WebRTC:

```bash
# On the device
WEBRTC_BASE="/path/to/livekit-rockchip-sdk/target/release/build/scratch-*/out/livekit_webrtc/livekit/linux-arm64-release-webrtc-*/linux-arm64-release"
WEBRTC_INC="$WEBRTC_BASE/include"
DEFINES=$(head -1 "$WEBRTC_BASE/webrtc.ninja" | sed 's/^defines = //')

# Compile just the encoder
g++ -c -std=gnu++20 $DEFINES \
  -I"$WEBRTC_INC" \
  -I"$WEBRTC_INC/third_party/abseil-cpp" \
  -I"$WEBRTC_INC/third_party/libyuv/include" \
  -I"$WEBRTC_INC/third_party/mpp/inc" \
  -I"$WEBRTC_INC/third_party/mpp/osal/inc" \
  -I"$WEBRTC_INC/third_party/mpp/mpp/base/inc" \
  -O2 -fPIC -o /tmp/rockchip_video_encoder.o /tmp/rockchip_video_encoder.cc

# Replace in libwebrtc.a
ar r "$WEBRTC_BASE/lib/libwebrtc.a" /tmp/rockchip_video_encoder.o

# Rebuild video_call (only links, very fast)
cd /path/to/livekit-rockchip-sdk
touch webrtc-sys/build.rs
cargo build --release -p video_call
```

## Troubleshooting

| Issue | Fix |
|---|---|
| `Device or resource busy` on camera | `pkill -9 video_call` — a stale process is holding the device |
| Encoder `frame_count=0`, resolution cascades to 240p | Do NOT use non-blocking MPP timeouts or CBR mode — they break WebRTC's encode pipeline |
| No video on receiver with H.265 | LiveKit Cloud doesn't enable H.265 by default — use `h264` codec |
| High latency / wrong region | Check your `config.toml` URL — use a region-specific URL (see section 4) |
