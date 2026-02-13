# Upgrading to a New WebRTC Version

This guide covers what to do when LiveKit bumps the WebRTC version in
`rust-sdks` (e.g., from M137 to M138 or later).

## Step-by-Step Process

### 1. Check the new version

Look at the LiveKit `rust-sdks` changelog or `.gclient` file to determine the
new WebRTC milestone version:

```bash
cd /path/to/rust-sdks
cat webrtc-sys/libwebrtc/.gclient
# Look for the branch, e.g., m138_release
```

### 2. Update your rust-sdks checkout

```bash
cd /path/to/rust-sdks
git pull origin main
```

### 3. Re-run gclient sync

Delete the old build directory (or let the script handle it):

```bash
cd /path/to/webrtc-rockchip-mpp
rm -rf lk-webrtc-build/src  # Force a fresh sync
./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks
```

The build script will fetch the new WebRTC source and re-apply all patches.

### 4. Check for MPP factory API changes

The most common breakage between WebRTC versions is in the factory class
interfaces. After the build fails (or before running it), check if the
following signatures have changed:

```cpp
// In api/video_codecs/video_encoder_factory.h
class VideoEncoderFactory {
  virtual std::unique_ptr<VideoEncoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) = 0;
};

// In api/video_codecs/video_decoder_factory.h
class VideoDecoderFactory {
  virtual std::unique_ptr<VideoDecoder> Create(
      const Environment& env,
      const SdpVideoFormat& format) = 0;
};
```

If the method signatures changed, update the corresponding files in this
repository:

- `src/encoder/rockchip_video_encoder_factory.h`
- `src/encoder/rockchip_video_encoder_factory.cc`
- `src/decoder/rockchip_video_decoder_factory.h`
- `src/decoder/rockchip_video_decoder_factory.cc`

See [API_MIGRATION.md](API_MIGRATION.md) for a detailed example of the M114
to M137 migration that was already performed.

### 5. Check for changed includes

WebRTC periodically reorganizes headers. Common changes include:

- Headers moving to new directories
- Headers being renamed
- Headers being split into multiple files
- New required includes for types that were previously transitively included

Run the build and look for `#include` errors. The fix is usually
straightforward: find the new header path and update the `#include` directive.

### 6. Check for changed GN targets

If the build fails at the `gn gen` or `ninja` stage with dependency errors,
the GN target names or paths may have changed. Check:

- `src/BUILD.gn` in this repo (the target definitions injected by the build
  script)
- The deps listed in `modules/video_coding/BUILD.gn` and
  `common_video/BUILD.gn`

### 7. Check for new compiler flags

New WebRTC versions may introduce compiler flags that cause issues on
aarch64. The CREL flag fix in `patches/001-disable-crel.patch` is one
example. If the build fails with assembler or compiler errors:

1. Check `build/config/compiler/BUILD.gn` for new flags
2. Create a new patch in `patches/` if needed
3. Update the build script to apply it

### 8. Re-run the build

```bash
./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks
```

### 9. Verify

```bash
nm linux-arm64-release/lib/libwebrtc.a | grep -i Rockchip
```

## Common Issues

### Changed type aliases

WebRTC has been migrating from Abseil types to C++ standard library types:

| Old (pre-M137) | New (M137+) |
|---|---|
| `absl::optional<T>` | `std::optional<T>` |
| `absl::string_view` | `std::string_view` |
| `absl::make_unique<T>` | `std::make_unique<T>` |

If you see errors about `absl::optional` not being found, replace with
`std::optional` and add `#include <optional>`.

### New `Environment` parameter

Starting in M125+, factory `Create` methods require an `Environment`
parameter. If upgrading from a very old version, see
[API_MIGRATION.md](API_MIGRATION.md).

### Removed or renamed GN targets

GN targets like `:rtc_base_approved` or `:video_frame_i420` may be removed or
renamed. Check the upstream BUILD.gn files to find the current target names.

### New sysroot requirements

New Chrome/WebRTC versions may require a newer sysroot. The build script runs
`install-sysroot.py` automatically, but if it fails, you may need to update
depot_tools or manually install the sysroot.

### GLIBC version conflicts

On some systems, the cross-compilation sysroot may conflict with the host
GLIBC version. The build script includes a workaround for `__GLIBC_USE_ISOC2X`
but new GLIBC incompatibilities may appear. Check the error messages and apply
similar `sed` fixes if needed.

## Checklist

Use this checklist when upgrading:

- [ ] Updated `rust-sdks` to the new version
- [ ] Deleted old build directory (`lk-webrtc-build/src`)
- [ ] Checked for factory API signature changes
- [ ] Checked for renamed/moved includes
- [ ] Checked for new problematic compiler flags
- [ ] Checked for changed GN target names
- [ ] Build completes without errors
- [ ] `nm | grep Rockchip` shows symbols in libwebrtc.a
- [ ] LiveKit SDK builds with the new libwebrtc.a
- [ ] Basic encode/decode smoke test passes on RK3588 hardware
