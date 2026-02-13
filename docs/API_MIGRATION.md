# API Migration: M114 to M137

This document records the specific API changes that were required when porting
the Rockchip MPP codec from WebRTC M114 to M137. It serves as a reference for
future version upgrades.

## Summary of Changes

| Area | M114 | M137 |
|---|---|---|
| Factory Create method | `CreateVideoEncoder(SdpVideoFormat)` | `Create(Environment&, SdpVideoFormat&)` |
| Factory Create method | `CreateVideoDecoder(SdpVideoFormat)` | `Create(Environment&, SdpVideoFormat&)` |
| Optional type | `absl::optional<T>` | `std::optional<T>` |
| Environment header | Not needed | `#include "api/environment/environment.h"` |
| Optional header | `#include "absl/types/optional.h"` | `#include <optional>` |

## Detailed Changes

### 1. VideoEncoderFactory::Create

**M114 (old):**

```cpp
// video_encoder_factory.h
class VideoEncoderFactory {
 public:
  virtual std::unique_ptr<VideoEncoder> CreateVideoEncoder(
      const SdpVideoFormat& format) = 0;
};
```

```cpp
// rockchip_video_encoder_factory.h (old)
class RockchipVideoEncoderFactory : public VideoEncoderFactory {
 public:
  std::unique_ptr<VideoEncoder> CreateVideoEncoder(
      const SdpVideoFormat& format) override;
};
```

**M137 (new):**

```cpp
// video_encoder_factory.h
class VideoEncoderFactory {
 public:
  virtual std::unique_ptr<VideoEncoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) = 0;
};
```

```cpp
// rockchip_video_encoder_factory.h (new)
#include "api/environment/environment.h"

class RockchipVideoEncoderFactory : public VideoEncoderFactory {
 public:
  std::unique_ptr<VideoEncoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) override;
};
```

### 2. VideoDecoderFactory::Create

**M114 (old):**

```cpp
class VideoDecoderFactory {
 public:
  virtual std::unique_ptr<VideoDecoder> CreateVideoDecoder(
      const SdpVideoFormat& format) = 0;
};
```

**M137 (new):**

```cpp
class VideoDecoderFactory {
 public:
  virtual std::unique_ptr<VideoDecoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) = 0;
};
```

The same pattern applies: method renamed from `CreateVideoDecoder` to
`Create`, and an `Environment` parameter was added.

### 3. absl::optional to std::optional

**M114 (old):**

```cpp
#include "absl/types/optional.h"

VideoEncoderFactory::CodecSupport QueryCodecSupport(
    const SdpVideoFormat& format,
    absl::optional<std::string> scalability_mode) const override;
```

**M137 (new):**

```cpp
#include <optional>

VideoEncoderFactory::CodecSupport QueryCodecSupport(
    const SdpVideoFormat& format,
    std::optional<std::string> scalability_mode) const override;
```

This change affects:
- `QueryCodecSupport` in both encoder and decoder factories
- Any internal use of `absl::optional` (e.g., `ParseSdpForH264ProfileLevelId`
  returns `std::optional<H264ProfileLevelId>` in M137)
- The `absl/types/optional.h` include can be removed if no other Abseil
  optional usage remains

### 4. Environment Parameter

The `Environment` class (from `api/environment/environment.h`) is a new M125+
concept that bundles runtime dependencies (task queues, clock, event log,
field trials) into a single object. Factory methods receive it so they can
pass it to the codecs they create.

In our implementation, the `Environment` parameter is accepted but not used,
since the MPP encoder/decoder manage their own resources:

```cpp
std::unique_ptr<VideoEncoder>
RockchipVideoEncoderFactory::Create(const Environment& env,
                                    const SdpVideoFormat& format) {
  // env is not used -- MPP manages its own context
  if (!IsFormatSupported(format)) {
    return nullptr;
  }
  return std::make_unique<RockchipVideoEncoder>();
}
```

If a future version of the encoder needs access to WebRTC's clock or task
queue infrastructure, the `env` parameter provides it.

### 5. Include Changes

**Removed includes:**

```cpp
// No longer needed in M137
#include "absl/types/optional.h"
```

**Added includes:**

```cpp
// Required for factory Create method
#include "api/environment/environment.h"

// Required for std::optional
#include <optional>
```

**Changed includes (none in this migration, but common in other upgrades):**

Some header paths may change between versions. If you get "file not found"
errors during compilation, search the M137 source tree for the header:

```bash
find lk-webrtc-build/src -name "the_missing_header.h"
```

### 6. BUILD.gn Dependency Changes

The `api/environment` GN target must be added as a dependency for any target
that includes `environment.h`:

```gn
deps = [
  # ... existing deps ...
  "../../api/environment",  # New in M137
]
```

Some old deps may no longer exist. In particular:

- `../../rtc_base:rtc_base_approved` -- may be removed or merged into
  `../../rtc_base` in newer versions
- `../../api/video:video_frame_i420` -- may be merged into
  `../../api/video:video_frame`

If `gn gen` fails with "no such target" errors, check the relevant BUILD.gn
in the WebRTC tree to find the current target name.

## Migration Steps (Recipe)

If you need to perform a similar migration for a future version bump, follow
these steps:

1. **Identify the base class changes.** Look at
   `api/video_codecs/video_encoder_factory.h` and
   `api/video_codecs/video_decoder_factory.h` in the new version.

2. **Update the factory headers.** Match the new pure virtual method
   signatures exactly (method name, parameters, const qualifiers, return type).

3. **Update the factory implementations.** Change the method definition to
   match the new signature.

4. **Search for `absl::` usage.** Replace `absl::optional` with
   `std::optional`, `absl::string_view` with `std::string_view`, etc., as
   needed.

5. **Update includes.** Add any new required headers (like
   `environment.h`), remove deprecated Abseil headers.

6. **Update BUILD.gn deps.** Add new targets, remove targets that no longer
   exist.

7. **Build and fix.** Run the build script and address any remaining
   compilation errors. Most will be straightforward include or type fixes.
