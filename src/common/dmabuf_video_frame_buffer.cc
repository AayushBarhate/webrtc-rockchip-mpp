// Copyright 2026 The WebRTC Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "common_video/dmabuf_video_frame_buffer.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include "api/video/i420_buffer.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv.h"

namespace webrtc {

namespace {

// Calculate stride based on width and format.
// For NV12/NV21, the stride is typically aligned to 64 bytes on RK3588.
int CalculateStride(int width, uint32_t format) {
  switch (format) {
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21:
      // RK3588 camera HAL aligns stride to 64 bytes for DMA performance.
      // Fall back to width if no explicit stride is provided.
      return width;
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
      return width * 2;
    default:
      RTC_LOG(LS_WARNING) << "Unknown format " << format << ", assuming width";
      return width;
  }
}

}  // namespace

DMABufVideoFrameBuffer::DMABufVideoFrameBuffer(int fd,
                                                int width,
                                                int height,
                                                size_t size,
                                                uint32_t format,
                                                int stride)
    : fd_(dup(fd)),  // Duplicate fd to maintain ownership
      width_(width),
      height_(height),
      size_(size),
      format_(format),
      stride_(stride > 0 ? stride : CalculateStride(width, format)),
      owns_fd_(true) {
  RTC_DCHECK_GE(fd, 0) << "Invalid file descriptor: " << fd;
  RTC_DCHECK_GT(width_, 0) << "Invalid width: " << width_;
  RTC_DCHECK_GT(height_, 0) << "Invalid height: " << height_;
  RTC_DCHECK_GT(size_, 0) << "Invalid size: " << size_;

  if (fd_ < 0) {
    RTC_LOG(LS_ERROR) << "Failed to dup DMA-BUF fd " << fd << ": "
                      << strerror(errno);
    owns_fd_ = false;
    return;
  }

  // Verify fd is valid
  if (fcntl(fd_, F_GETFD) == -1) {
    RTC_LOG(LS_ERROR) << "Invalid DMA-BUF fd " << fd_ << ": "
                      << strerror(errno);
    close(fd_);
    fd_ = -1;
    owns_fd_ = false;
  }

  RTC_LOG(LS_VERBOSE) << "DMABufVideoFrameBuffer created: fd=" << fd_
                      << " size=" << size_ << " " << width_ << "x" << height_
                      << " stride=" << stride_ << " format=" << format_;
}

DMABufVideoFrameBuffer::~DMABufVideoFrameBuffer() {
  if (owns_fd_ && fd_ >= 0) {
    close(fd_);
    RTC_LOG(LS_VERBOSE) << "Closed DMA-BUF fd " << fd_;
  }
}

rtc::scoped_refptr<I420BufferInterface> DMABufVideoFrameBuffer::ToI420() {
  // This is called as a fallback if hardware encoder is not used
  // or for preview/recording paths that require CPU access
  RTC_LOG(LS_WARNING) << "DMABufVideoFrameBuffer::ToI420() - CPU copy fallback";

  if (fd_ < 0) {
    RTC_LOG(LS_ERROR) << "Cannot convert: invalid fd";
    return nullptr;
  }

  // Map DMA-BUF to CPU memory
  void* mapped = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd_, 0);
  if (mapped == MAP_FAILED) {
    RTC_LOG(LS_ERROR) << "Failed to mmap DMA-BUF fd=" << fd_ << ": "
                      << strerror(errno);
    return nullptr;
  }

  rtc::scoped_refptr<I420Buffer> i420_buffer =
      I420Buffer::Create(width_, height_);

  // Convert based on V4L2 pixel format
  int result = -1;

  switch (format_) {
    case V4L2_PIX_FMT_NV12: {
      // NV12: Y plane followed by interleaved UV.
      // CRITICAL: UV plane offset is stride_ * height_, NOT width_ * height_.
      // When stride > width (e.g. 1920 → 1984 due to 64-byte alignment),
      // using width * height puts the UV pointer inside the Y data.
      const uint8_t* src_y = static_cast<const uint8_t*>(mapped);
      const uint8_t* src_uv = src_y + (stride_ * height_);

      result = libyuv::NV12ToI420(
          src_y, stride_,
          src_uv, stride_,
          i420_buffer->MutableDataY(), i420_buffer->StrideY(),
          i420_buffer->MutableDataU(), i420_buffer->StrideU(),
          i420_buffer->MutableDataV(), i420_buffer->StrideV(),
          width_, height_);
      break;
    }

    case V4L2_PIX_FMT_NV21: {
      // NV21: Y plane followed by interleaved VU.
      // Same stride-based offset as NV12.
      const uint8_t* src_y = static_cast<const uint8_t*>(mapped);
      const uint8_t* src_vu = src_y + (stride_ * height_);

      result = libyuv::NV21ToI420(
          src_y, stride_,
          src_vu, stride_,
          i420_buffer->MutableDataY(), i420_buffer->StrideY(),
          i420_buffer->MutableDataU(), i420_buffer->StrideU(),
          i420_buffer->MutableDataV(), i420_buffer->StrideV(),
          width_, height_);
      break;
    }

    case V4L2_PIX_FMT_YUYV: {
      // YUYV: Packed YUV 4:2:2
      const uint8_t* src_yuyv = static_cast<const uint8_t*>(mapped);

      result = libyuv::YUY2ToI420(
          src_yuyv, stride_,
          i420_buffer->MutableDataY(), i420_buffer->StrideY(),
          i420_buffer->MutableDataU(), i420_buffer->StrideU(),
          i420_buffer->MutableDataV(), i420_buffer->StrideV(),
          width_, height_);
      break;
    }

    case V4L2_PIX_FMT_UYVY: {
      // UYVY: Packed YUV 4:2:2
      const uint8_t* src_uyvy = static_cast<const uint8_t*>(mapped);

      result = libyuv::UYVYToI420(
          src_uyvy, stride_,
          i420_buffer->MutableDataY(), i420_buffer->StrideY(),
          i420_buffer->MutableDataU(), i420_buffer->StrideU(),
          i420_buffer->MutableDataV(), i420_buffer->StrideV(),
          width_, height_);
      break;
    }

    default:
      RTC_LOG(LS_ERROR) << "Unsupported V4L2 format for ToI420 conversion: 0x"
                        << format_;
      munmap(mapped, size_);
      return nullptr;
  }

  munmap(mapped, size_);

  if (result != 0) {
    RTC_LOG(LS_ERROR) << "YUV conversion failed: " << result;
    return nullptr;
  }

  RTC_LOG(LS_VERBOSE) << "Converted DMA-BUF to I420: " << width_ << "x"
                      << height_;
  return i420_buffer;
}

}  // namespace webrtc
