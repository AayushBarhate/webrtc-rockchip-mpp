# WebRTC Rockchip MPP Hardware Video Codec

<!-- Badges -->
![Build Status](https://img.shields.io/github/actions/workflow/status/AayushBarhate/webrtc-rockchip-mpp/build.yml?branch=main&label=build)
![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)
![Platform](https://img.shields.io/badge/platform-RK3588%20%7C%20aarch64-green)
![WebRTC](https://img.shields.io/badge/WebRTC-M137-orange)

Hardware-accelerated H.264 encoder and decoder for WebRTC M137, targeting the
Rockchip RK3588 SoC via its Media Process Platform (MPP) library. Designed for
integration with [LiveKit](https://livekit.io/) via the
[rust-sdks](https://github.com/livekit/rust-sdks) `webrtc-sys` crate.

## Features

- **Hardware H.264 Encode/Decode** -- Offloads video coding to RK3588's
  dedicated VPU, freeing CPU for application logic.
- **Zero-Copy DMA-BUF Path** -- When the video source provides DMA-BUF file
  descriptors (e.g., V4L2 camera, ISP pipeline), frames pass directly to MPP
  without any CPU memcpy.
- **CPU-Copy Fallback** -- For sources that provide CPU-accessible buffers
  (USB cameras, screen capture), I420-to-NV12 conversion is performed
  automatically via libyuv before handing frames to MPP.
- **CBR Rate Control** -- Constant bitrate mode with configurable target, max
  (target * 17/16), and min (target * 15/16) bitrate bounds, suitable for
  real-time WebRTC bandwidth adaptation.
- **M137 API Compatibility** -- Factory classes implement the current M137
  `Create(const Environment&, const SdpVideoFormat&)` interface, with
  `std::optional` replacing the deprecated `absl::optional`.

## Prerequisites

| Requirement | Details |
|---|---|
| **Hardware** | RK3588-based board (Orange Pi 5, Rock 5B, etc.) running Linux |
| **MPP Runtime** | `librockchip_mpp.so` installed at `/usr/lib/` (typically from the BSP or `mpp` package) |
| **depot_tools** | [Chromium depot_tools](https://chromium.googlesource.com/chromium/tools/depot_tools.git) (will be cloned automatically if missing) |
| **Build tools** | `git`, `python3`, `pkg-config`, `ninja-build` |
| **LiveKit rust-sdks** | Source checkout of [livekit/rust-sdks](https://github.com/livekit/rust-sdks) (provides `.gclient`, patches, and `boringssl_prefix_symbols.txt`) |

## Quick Start

```bash
# 1. Clone this repository
git clone https://github.com/<user>/webrtc-rockchip-mpp.git
cd webrtc-rockchip-mpp

# 2. Run the build script (point it at your rust-sdks checkout)
./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks

# 3. Artifacts land in linux-arm64-release/
ls linux-arm64-release/lib/libwebrtc.a
```

The build takes 30-90 minutes depending on your hardware and network speed
(the initial `gclient sync` downloads ~15 GB of Chromium/WebRTC source).

## Directory Structure

```
webrtc-rockchip-mpp/
  README.md                  # This file
  LICENSE                    # BSD-3-Clause
  build_libwebrtc.sh         # Main build script
  .gitignore
  .github/
    workflows/
      build.yml              # CI workflow
  src/
    BUILD.gn                 # GN build rules for MPP targets
    encoder/
      rockchip_video_encoder.h
      rockchip_video_encoder.cc
      rockchip_video_encoder_factory.h
      rockchip_video_encoder_factory.cc
    decoder/
      rockchip_video_decoder.h
      rockchip_video_decoder.cc
      rockchip_video_decoder_factory.h
      rockchip_video_decoder_factory.cc
    common/
      dmabuf_video_frame_buffer.h
      dmabuf_video_frame_buffer.cc
  third_party/
    mpp/                     # Vendored MPP headers (inc/, osal/, mpp/)
  patches/
    001-disable-crel.patch   # Disables CREL assembler flag
  docs/
    ARCHITECTURE.md          # Encoding paths and MPP flow
    UPGRADING.md             # Guide for WebRTC version bumps
    API_MIGRATION.md         # M114 to M137 API changes
```

## How the Build Works

The `build_libwebrtc.sh` script automates six steps:

| Step | Description |
|---|---|
| **A1** | Fetch the WebRTC M137 source tree using LiveKit's `.gclient` configuration and `gclient sync`. |
| **A2** | Apply LiveKit's standard patches from `rust-sdks/webrtc-sys/libwebrtc/patches/` (license additions, GCC fixes, SME disable for libyuv, etc.). |
| **A3** | Copy the MPP encoder, decoder, DMA-BUF frame buffer, and vendored MPP headers from this repository's `src/` and `third_party/mpp/` into the M137 source tree. |
| **A4** | Apply workaround patches: disable CREL assembler flags (`patches/001-disable-crel.patch`) and add `libyuv_use_sme=false` to GN args. |
| **A5** | Inject GN build targets -- `webrtc_rockchip_mpp` in `modules/video_coding/BUILD.gn`, `dmabuf_video_frame_buffer` in `common_video/BUILD.gn`, and wire both as dependencies of the top-level `:webrtc` target. |
| **A6** | Run `gn gen` with LiveKit-compatible args, build with `ninja`, then package `libwebrtc.a` (with MPP objects), headers, and metadata into `linux-arm64-{release|debug}/`. |

## Verification

After a successful build, verify that Rockchip MPP symbols are present in the
static library:

```bash
nm linux-arm64-release/lib/libwebrtc.a | grep -i Rockchip
```

You should see symbols such as:

```
T _ZN6webrtc20RockchipVideoEncoderC1Ev
T _ZN6webrtc20RockchipVideoDecoder9ConfigureERKNS_12VideoDecoder8SettingsE
T _ZN6webrtc27RockchipVideoEncoderFactory6CreateERKNS_11EnvironmentERKNS_14SdpVideoFormatE
T _ZN6webrtc27RockchipVideoDecoderFactory6CreateERKNS_11EnvironmentERKNS_14SdpVideoFormatE
```

## Supported Codecs

| Codec | Profile | SDP profile-level-id | Direction |
|---|---|---|---|
| H.264 | Constrained Baseline | `42e01f` | Encode + Decode |
| H.264 | Constrained High | `640032` | Encode + Decode |

The RK3588 VPU also supports H.265/HEVC and VP9 in hardware, but this
implementation currently targets H.264 only, matching the most common WebRTC
negotiation profiles.

## Architecture

### Zero-Copy DMA-BUF Path

When the video source provides frames as DMA-BUF file descriptors (e.g., a
V4L2 camera that exports buffers via `VIDIOC_EXPBUF`), frames flow through the
system without any CPU memory copy:

```
V4L2 Camera --> DMA-BUF fd --> DMABufVideoFrameBuffer --> MPP encode_put_frame
                                                              |
                                                        (no memcpy)
                                                              |
                                                       MppBuffer (DMA-BUF import)
```

The `DMABufVideoFrameBuffer` class wraps the file descriptor as a
`VideoFrameBuffer::Type::kNative` buffer. The encoder detects the native type
and imports the fd directly into MPP via `mpp_buffer_import_with_tag`.

### CPU Copy Fallback

For sources that deliver I420 frames in CPU memory (USB cameras, software
rendering, screen capture), the encoder converts I420 to NV12 using libyuv and
copies the result into an MPP-managed buffer:

```
USB Camera --> I420 buffer --> libyuv::I420ToNV12 --> MppBuffer (heap) --> MPP encode
```

This path adds one memcpy but works with any video source.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for full details.

## Integration with LiveKit

After building, point the LiveKit Rust SDK at the custom `libwebrtc.a`:

```bash
export LK_CUSTOM_WEBRTC=$(pwd)/linux-arm64-release
cd /path/to/rust-sdks
cargo build --release
```

The LiveKit `webrtc-sys` build system will pick up the custom library, which
includes the Rockchip MPP codec objects alongside the standard WebRTC codecs.

## License

This project is licensed under the [BSD-3-Clause License](LICENSE).

Copyright (c) The WebRTC Project Authors and RK3588 MPP Contributors.
