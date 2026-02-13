// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef COMMON_VIDEO_DMABUF_VIDEO_FRAME_BUFFER_H_
#define COMMON_VIDEO_DMABUF_VIDEO_FRAME_BUFFER_H_

#include <stdint.h>
#include <unistd.h>

#include "api/scoped_refptr.h"
#include "api/video/video_frame_buffer.h"
#include "api/video/i420_buffer.h"

namespace webrtc {

// VideoFrameBuffer backed by a DMA-BUF file descriptor from V4L2.
// This enables zero-copy video encoding by passing the file descriptor
// directly to hardware encoders (e.g., Rockchip MPP) without CPU memcpy.
//
// Usage:
//   // After V4L2 VIDIOC_EXPBUF
//   rtc::scoped_refptr<DMABufVideoFrameBuffer> buffer =
//       rtc::make_ref_counted<DMABufVideoFrameBuffer>(
//           dmabuf_fd, width, height, buffer_size, V4L2_PIX_FMT_NV12);
//
//   VideoFrame frame = VideoFrame::Builder()
//       .set_video_frame_buffer(buffer)
//       .set_timestamp_us(timestamp_us)
//       .build();
//
class DMABufVideoFrameBuffer : public VideoFrameBuffer {
 public:
  // Constructor
  // - fd: DMA-BUF file descriptor from VIDIOC_EXPBUF (will be dup'd)
  // - width/height: Video dimensions in pixels
  // - size: Total buffer size in bytes
  // - format: V4L2 pixel format (e.g., V4L2_PIX_FMT_NV12)
  // - stride: Horizontal stride in bytes (0 = auto-calculate)
  DMABufVideoFrameBuffer(int fd,
                          int width,
                          int height,
                          size_t size,
                          uint32_t format,
                          int stride = 0);

  ~DMABufVideoFrameBuffer() override;

  // Accessors for zero-copy hardware encoding
  int GetDmaBufFd() const { return fd_; }
  size_t GetSize() const { return size_; }
  uint32_t GetFormat() const { return format_; }
  int GetStride() const { return stride_; }

  // VideoFrameBuffer interface
  Type type() const override { return Type::kNative; }
  int width() const override { return width_; }
  int height() const override { return height_; }

  // Fallback conversion for software encoders (CPU copy)
  rtc::scoped_refptr<I420BufferInterface> ToI420() override;

 private:
  int fd_;           // DMA-BUF file descriptor (owned)
  int width_;
  int height_;
  size_t size_;      // Buffer size in bytes
  uint32_t format_;  // V4L2 pixel format (V4L2_PIX_FMT_*)
  int stride_;       // Horizontal stride in bytes
  bool owns_fd_;     // Whether to close fd in destructor

  // Disallow copy and assign
  DMABufVideoFrameBuffer(const DMABufVideoFrameBuffer&) = delete;
  DMABufVideoFrameBuffer& operator=(const DMABufVideoFrameBuffer&) = delete;
};

}  // namespace webrtc

#endif  // COMMON_VIDEO_DMABUF_VIDEO_FRAME_BUFFER_H_
