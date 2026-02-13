# Architecture

This document describes the internal architecture of the Rockchip MPP WebRTC
codec integration, covering the two encoding paths, the MPP initialization
flow, rate control, and frame format handling.

## Overview

The integration adds two hardware codec classes to WebRTC's video coding module:

- `RockchipVideoEncoder` -- H.264 hardware encoder using MPP
- `RockchipVideoDecoder` -- H.264 hardware decoder using MPP

Both are registered via their respective factory classes
(`RockchipVideoEncoderFactory`, `RockchipVideoDecoderFactory`), which
implement the M137 `VideoEncoderFactory`/`VideoDecoderFactory` interfaces.

A supporting class, `DMABufVideoFrameBuffer`, provides a
`VideoFrameBuffer::Type::kNative` wrapper around Linux DMA-BUF file
descriptors for zero-copy frame passing.

## Two Encoding Paths

### Path 1: Zero-Copy DMA-BUF (Preferred)

This path is used when the video source provides frames as DMA-BUF file
descriptors. This is typical for:

- V4L2 camera drivers that support `VIDIOC_EXPBUF`
- ISP (Image Signal Processor) pipelines on RK3588
- GPU rendering pipelines that export DMA-BUFs
- Other hardware producers on the same SoC

**Data flow:**

```
Video Source (V4L2/ISP)
    |
    v
DMA-BUF file descriptor (kernel-managed memory)
    |
    v
DMABufVideoFrameBuffer (wraps fd as VideoFrameBuffer::Type::kNative)
    |
    v
WebRTC VideoFrame
    |
    v
RockchipVideoEncoder::Encode()
    |
    |-- Detects kNative type
    |-- Casts to DMABufVideoFrameBuffer
    |-- Gets fd via GetDmaBufFd()
    |
    v
mpp_buffer_import_with_tag(fd)  --> MppBuffer (DMA-BUF backed)
    |
    v
mpp_frame_set_buffer(mpp_frame, mpp_buffer)
    |
    v
MPP Hardware Encoder (VPU)
    |
    v
H.264 bitstream (MppPacket)
```

**No CPU memcpy occurs in this path.** The DMA-BUF fd is imported directly
into MPP, and the VPU reads from the same physical memory that the camera/ISP
wrote to.

### Path 2: CPU Copy Fallback

This path is used when the video source provides frames in CPU-accessible
memory (standard `I420Buffer`). This is typical for:

- USB cameras (UVC devices) that deliver frames to userspace buffers
- Screen capture sources
- Software-generated video (test patterns, overlays)
- Any source that delivers `VideoFrameBuffer::Type::kI420`

**Data flow:**

```
Video Source (USB Camera / Software)
    |
    v
I420Buffer (CPU memory: Y + U + V planes)
    |
    v
RockchipVideoEncoder::Encode()
    |
    |-- Detects non-kNative type (or kNative without valid fd)
    |-- Calls buffer->ToI420()
    |
    v
libyuv::I420ToNV12()  (CPU conversion: I420 -> NV12)
    |
    v
memcpy into MppBuffer (heap-allocated by MPP)
    |
    v
mpp_frame_set_buffer(mpp_frame, mpp_buffer)
    |
    v
MPP Hardware Encoder (VPU)
    |
    v
H.264 bitstream (MppPacket)
```

This path involves one format conversion (I420 to NV12) and one memory copy
into MPP-managed memory. The conversion is performed by libyuv, which uses
NEON SIMD on aarch64 for good throughput.

### Path Selection Logic

The encoder selects the path automatically in `Encode()`:

```cpp
if (buffer->type() == VideoFrameBuffer::Type::kNative) {
    auto* dmabuf = static_cast<DMABufVideoFrameBuffer*>(buffer.get());
    int fd = dmabuf->GetDmaBufFd();
    if (fd >= 0) {
        // --> Zero-copy DMA-BUF path
        MppBuffer mpp_buf = ImportDMABuffer(fd, ...);
        mpp_frame_set_buffer(frame, mpp_buf);
    } else {
        // --> CPU copy fallback (invalid fd)
    }
} else {
    // --> CPU copy fallback (I420 or other CPU buffer)
}
```

## Decoding Path

The decoder similarly supports two output paths:

1. **Zero-copy output**: If the decoded MppBuffer has a valid DMA-BUF fd
   (via `mpp_buffer_get_fd`), the decoder wraps it in a
   `DMABufVideoFrameBuffer` for downstream consumers that can accept native
   buffers.

2. **CPU copy output**: If no fd is available, the decoder converts the
   NV12/NV21 decoded frame to I420 via libyuv and returns a standard
   `I420Buffer`.

## MPP Initialization Flow

### Encoder Initialization (`InitEncode`)

```
1. mpp_create(&ctx, &mpi)        -- Create MPP context and API handle
2. mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingAVC)  -- Init for H.264 encode
3. mpp_buffer_group_get_external(&group, MPP_BUFFER_TYPE_DMA_HEAP)
                                   -- Create buffer group for DMA-BUF import
4. mpp_enc_cfg_init(&cfg)         -- Create encoder config object
5. mpi->control(ctx, MPP_ENC_GET_CFG, cfg)  -- Load current config
6. ConfigureEncoder()              -- Set resolution, stride, format, profile
7. ConfigureRateControl()          -- Set CBR params, bitrate, FPS, GOP
8. mpi->control(ctx, MPP_ENC_SET_CFG, cfg)  -- Apply configuration
9. mpi->control(ctx, MPP_ENC_SET_SEI_CFG, &sei_mode)  -- Enable SEI
```

### Decoder Initialization (`Configure`)

```
1. mpp_create(&ctx, &mpi)        -- Create MPP context and API handle
2. mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingAVC)  -- Init for H.264 decode
3. mpp_buffer_group_get_internal(&group, MPP_BUFFER_TYPE_DMA_HEAP)
                                   -- Create internal buffer group
   (falls back to MPP_BUFFER_TYPE_ION if DMA_HEAP unavailable)
4. mpi->control(ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &need_split)
                                   -- Enable NAL unit splitting
5. mpi->control(ctx, MPP_DEC_SET_OUTPUT_FORMAT, &fmt)
                                   -- Request NV12 output
```

### Encode Frame Flow

```
1. mpp_frame_init(&mpp_frame)
2. [Import DMA-BUF or copy I420->NV12 into MppBuffer]
3. mpp_frame_set_width/height/hor_stride/ver_stride/fmt(mpp_frame, ...)
4. mpp_frame_set_pts(mpp_frame, timestamp_us)
5. [If keyframe needed] mpi->control(ctx, MPP_ENC_SET_IDR_FRAME, NULL)
6. mpi->encode_put_frame(ctx, mpp_frame)    -- Submit frame to encoder
7. mpi->encode_get_packet(ctx, &mpp_packet) -- Retrieve encoded bitstream
8. Extract data via mpp_packet_get_pos/length/flag
9. Construct EncodedImage and deliver to callback
10. mpp_packet_deinit(&mpp_packet)
11. mpp_frame_deinit(&mpp_frame)
```

### Decode Frame Flow

```
1. mpp_packet_init(&mpp_packet, data, size)
2. mpp_packet_set_pts(mpp_packet, timestamp)
3. mpi->decode_put_packet(ctx, mpp_packet)  -- Submit packet to decoder
4. mpi->decode_get_frame(ctx, &mpp_frame)   -- Retrieve decoded frame
5. [Check for errors and EOS]
6. [Export as DMABufVideoFrameBuffer or convert NV12->I420]
7. Construct VideoFrame and deliver to callback
8. mpp_frame_deinit(&mpp_frame)
```

## Rate Control

The encoder uses **CBR (Constant Bitrate)** mode, which is the most
appropriate mode for real-time WebRTC where bandwidth estimation drives bitrate
changes.

### Configuration Parameters

| Parameter | Value | Description |
|---|---|---|
| `rc:mode` | `MPP_ENC_RC_MODE_CBR` | Constant bitrate mode |
| `rc:bps_target` | From `SetRates()` | Target bitrate in bps |
| `rc:bps_max` | `target * 17/16` | Maximum allowed bitrate (~6% headroom) |
| `rc:bps_min` | `target * 15/16` | Minimum bitrate (~6% below target) |
| `rc:fps_in_num` | From codec settings | Input frame rate numerator |
| `rc:fps_out_num` | From `SetRates()` | Output frame rate numerator |
| `rc:gop` | 60 | Group of Pictures size (keyframe interval) |

### Dynamic Rate Adaptation

WebRTC calls `SetRates()` when bandwidth estimation changes. The encoder
responds by:

1. Reading the current configuration (`MPP_ENC_GET_CFG`)
2. Updating `rc:bps_target`, `rc:bps_max`, `rc:bps_min`, and `rc:fps_out_num`
3. Applying the updated configuration (`MPP_ENC_SET_CFG`)

This allows the encoder to adapt to network conditions in real-time without
reinitializing the hardware codec.

## Frame Format: I420 to NV12 Conversion

MPP operates on **NV12** (aka YUV420SP) frames, where:
- Y plane: width * height bytes, 16-byte aligned stride
- UV plane: interleaved U and V, width * height/2 bytes

WebRTC internally uses **I420** (aka YUV420P), where:
- Y plane: width * height bytes
- U plane: width/2 * height/2 bytes
- V plane: width/2 * height/2 bytes

The conversion is performed by `libyuv::I420ToNV12()` in the CPU copy path.
In the zero-copy DMA-BUF path, the source is assumed to already be in NV12
format (which is the standard output format of V4L2 camera drivers on RK3588).

### Stride Alignment

MPP requires 16-byte aligned horizontal and vertical strides:

```cpp
int hor_stride = MPP_ALIGN(width, 16);   // e.g., 1920 -> 1920 (already aligned)
int ver_stride = MPP_ALIGN(height, 16);  // e.g., 1080 -> 1088
```

The buffer size for NV12 is `hor_stride * ver_stride * 3 / 2`.

## H.264 Profile Configuration

The encoder is configured for **H.264 High Profile Level 4.1**:

| Parameter | Value | Description |
|---|---|---|
| `h264:profile` | 100 | High Profile |
| `h264:level` | 41 | Level 4.1 (supports up to 1080p @ 30fps) |
| `h264:cabac_en` | 1 | CABAC entropy coding enabled |
| `h264:cabac_idc` | 0 | CABAC initialization IDC |
| `h264:trans8x8` | 1 | 8x8 transform enabled (High Profile feature) |

The factory advertises both **Constrained Baseline** (profile-level-id
`42e01f`) and **Constrained High** (`640032`) profiles via SDP negotiation,
but the hardware always encodes at High Profile. Constrained Baseline is
advertised for compatibility with peers that only support Baseline.

## Thread Safety

Both encoder and decoder use `webrtc::Mutex` to protect their internal state.
All public methods acquire the mutex before accessing MPP context or
configuration objects. This is necessary because WebRTC may call `Encode()`
and `SetRates()` from different threads.

## Error Handling

- MPP API calls return `MPP_RET` (0 = success).
- All MPP errors are logged via `RTC_LOG` and translated to WebRTC error codes
  (`WEBRTC_VIDEO_CODEC_ERROR`, `WEBRTC_VIDEO_CODEC_ERR_PARAMETER`, etc.).
- The encoder returns `WEBRTC_VIDEO_CODEC_UNINITIALIZED` if `Encode()` is
  called before `InitEncode()` or without a registered callback.
- Resource cleanup in `Release()` is idempotent (safe to call multiple times).
