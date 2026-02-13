#!/bin/bash
# =============================================================================
# build_libwebrtc.sh
# Build WebRTC M137 with Rockchip RK3588 MPP hardware encoder/decoder
#
# This script automates Phase A: building a libwebrtc.a that includes
# Rockchip MPP codec objects alongside LiveKit's M137 WebRTC.
#
# Usage:
#   ./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks
#   ./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks --profile debug
#   ./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks --arch arm64
#   ./build_libwebrtc.sh --livekit-sdk-dir /path/to/rust-sdks --cross-compile
#
# Prerequisites:
#   - depot_tools in PATH (or will be cloned automatically)
#   - librockchip_mpp.so installed on the target (/usr/lib/)
#   - LiveKit rust-sdks source checkout
#   - git, python3, pkg-config, ninja-build
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/lk-webrtc-build"

# Defaults
profile="release"
arch="arm64"
livekit_sdk_dir=""
cross_compile="false"

# ─────────────────────────────────────────────────────────────────────────────
# Argument parsing
# ─────────────────────────────────────────────────────────────────────────────
usage() {
  cat <<EOF
Usage: $0 [OPTIONS]

Options:
  --profile <debug|release>      Build profile (default: release)
  --arch <arm64>                 Target architecture (default: arm64)
  --livekit-sdk-dir <path>       Path to LiveKit rust-sdks checkout (required)
  --cross-compile                Cross-compile from x86_64 host (uses Chromium clang)
  -h, --help                     Show this help message

Example:
  $0 --livekit-sdk-dir /home/user/rust-sdks
  $0 --livekit-sdk-dir /home/user/rust-sdks --profile debug
EOF
  exit 1
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --profile)
      profile="$2"
      if [ "$profile" != "debug" ] && [ "$profile" != "release" ]; then
        echo "Error: --profile must be 'debug' or 'release', got '$profile'"
        exit 1
      fi
      shift 2
      ;;
    --arch)
      arch="$2"
      shift 2
      ;;
    --livekit-sdk-dir)
      livekit_sdk_dir="$2"
      shift 2
      ;;
    --cross-compile)
      cross_compile="true"
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Error: Unknown argument '$1'"
      usage
      ;;
  esac
done

# Validate required arguments
if [ -z "$livekit_sdk_dir" ]; then
  echo "Error: --livekit-sdk-dir is required"
  echo ""
  usage
fi

LIVEKIT_LIBWEBRTC_DIR="$livekit_sdk_dir/webrtc-sys/libwebrtc"

if [ ! -d "$LIVEKIT_LIBWEBRTC_DIR" ]; then
  echo "Error: LiveKit libwebrtc directory not found at: $LIVEKIT_LIBWEBRTC_DIR"
  echo "Make sure --livekit-sdk-dir points to the rust-sdks repository root."
  exit 1
fi

echo "========================================"
echo "Building WebRTC M137 with RK3588 MPP"
echo "  Arch:          $arch"
echo "  Profile:       $profile"
echo "  Cross-compile: $cross_compile"
echo "  Build dir:     $BUILD_DIR"
echo "  LiveKit SDK:   $livekit_sdk_dir"
echo "  MPP source:    $SCRIPT_DIR/src/"
echo "========================================"

# ─────────────────────────────────────────────────────────────────────────────
# Step A1: Fetch M137 source using LiveKit's gclient config
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Step A1: Fetch M137 source ==="

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Get depot_tools if needed
if [ ! -e "$BUILD_DIR/depot_tools" ]; then
  echo "  Cloning depot_tools..."
  git clone --depth 1 https://chromium.googlesource.com/chromium/tools/depot_tools.git
fi
export PATH="$BUILD_DIR/depot_tools:$PATH"

# Copy LiveKit's .gclient config (points to webrtc-sdk/webrtc.git@m137_release)
cp "$LIVEKIT_LIBWEBRTC_DIR/.gclient" "$BUILD_DIR/.gclient"

# Sync the source (skip if already done)
if [ ! -e "$BUILD_DIR/src" ]; then
  echo "  Running gclient sync (this takes a while -- ~15 GB download)..."
  gclient sync -D --no-history
else
  echo "  Source already present, skipping gclient sync"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Step A2: Apply LiveKit's patches
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Step A2: Apply LiveKit patches ==="

PATCHES_DIR="$LIVEKIT_LIBWEBRTC_DIR/patches"
cd "$BUILD_DIR/src"

# Apply patches with --check first to avoid re-applying
apply_patch() {
  local patch_file="$1"
  local target_dir="${2:-.}"
  local patch_name
  patch_name="$(basename "$patch_file")"

  cd "$BUILD_DIR/src/$target_dir"
  if git apply --check "$patch_file" 2>/dev/null; then
    echo "  Applying: $patch_name (in $target_dir)"
    git apply "$patch_file" -v --ignore-space-change --ignore-whitespace --whitespace=nowarn
  else
    echo "  Skipping (already applied or conflict): $patch_name"
  fi
  cd "$BUILD_DIR/src"
}

# Apply the same patches as LiveKit's build_linux.sh
apply_patch "$PATCHES_DIR/add_licenses.patch" "."
apply_patch "$PATCHES_DIR/ssl_verify_callback_with_native_handle.patch" "."
apply_patch "$PATCHES_DIR/add_deps.patch" "."
apply_patch "$PATCHES_DIR/force_gcc.patch" "build"
apply_patch "$PATCHES_DIR/david_disable_gun_source_macro.patch" "third_party"
apply_patch "$PATCHES_DIR/disable_sme_for_libyuv.patch" "third_party/libyuv"

# ─────────────────────────────────────────────────────────────────────────────
# Step A3: Copy MPP codec files into M137 tree
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Step A3: Copy MPP codec files ==="

cd "$BUILD_DIR/src"

# Create target directories
mkdir -p modules/video_coding/codecs/mpp
mkdir -p common_video
mkdir -p third_party/mpp

# Copy encoder files
echo "  Copying encoder files..."
cp "$SCRIPT_DIR/src/encoder/rockchip_video_encoder.h" \
   modules/video_coding/codecs/mpp/
cp "$SCRIPT_DIR/src/encoder/rockchip_video_encoder.cc" \
   modules/video_coding/codecs/mpp/
cp "$SCRIPT_DIR/src/encoder/rockchip_video_encoder_factory.h" \
   modules/video_coding/codecs/mpp/
cp "$SCRIPT_DIR/src/encoder/rockchip_video_encoder_factory.cc" \
   modules/video_coding/codecs/mpp/

# Copy decoder files
echo "  Copying decoder files..."
cp "$SCRIPT_DIR/src/decoder/rockchip_video_decoder.h" \
   modules/video_coding/codecs/mpp/
cp "$SCRIPT_DIR/src/decoder/rockchip_video_decoder.cc" \
   modules/video_coding/codecs/mpp/
cp "$SCRIPT_DIR/src/decoder/rockchip_video_decoder_factory.h" \
   modules/video_coding/codecs/mpp/
cp "$SCRIPT_DIR/src/decoder/rockchip_video_decoder_factory.cc" \
   modules/video_coding/codecs/mpp/

# Copy BUILD.gn for the mpp codecs directory
cp "$SCRIPT_DIR/src/BUILD.gn" \
   modules/video_coding/codecs/mpp/BUILD.gn

# Copy DMA-BUF frame buffer
echo "  Copying common_video/dmabuf_video_frame_buffer..."
cp "$SCRIPT_DIR/src/common/dmabuf_video_frame_buffer.h" common_video/
cp "$SCRIPT_DIR/src/common/dmabuf_video_frame_buffer.cc" common_video/

# Copy third_party/mpp (vendored headers)
echo "  Copying third_party/mpp/..."
cp -r "$SCRIPT_DIR/third_party/mpp/"* third_party/mpp/

echo "  MPP files copied successfully"

# ─────────────────────────────────────────────────────────────────────────────
# Step A4: Apply workaround patches (CREL, SME, etc.)
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Step A4: Apply workaround patches ==="

cd "$BUILD_DIR/src"

# Apply CREL disable patch
# The CREL assembler flag is not supported by all toolchains and causes build
# failures. This patch comments out the offending lines in build/config/compiler/BUILD.gn.
CREL_PATCH="$SCRIPT_DIR/patches/001-disable-crel.patch"
if [ -f "$CREL_PATCH" ]; then
  cd "$BUILD_DIR/src"
  if git apply --check "$CREL_PATCH" 2>/dev/null; then
    echo "  Applying CREL disable patch..."
    git apply "$CREL_PATCH" -v --ignore-space-change --ignore-whitespace --whitespace=nowarn
  else
    echo "  CREL patch already applied or not needed"
    # Fallback: manually comment out the CREL line if patch fails
    if grep -q '"-Wa,--crel,--allow-experimental-crel"' build/config/compiler/BUILD.gn 2>/dev/null; then
      echo "  Manually disabling CREL flag..."
      sed -i 's|cflags += \[ "-Wa,--crel,--allow-experimental-crel" \]|# cflags += [ "-Wa,--crel,--allow-experimental-crel" ]  # Disabled for RK3588 build|' \
        build/config/compiler/BUILD.gn
    fi
  fi
else
  echo "  Warning: CREL patch not found at $CREL_PATCH"
  # Fallback: manually comment out the CREL line
  if grep -q '"-Wa,--crel,--allow-experimental-crel"' build/config/compiler/BUILD.gn 2>/dev/null; then
    echo "  Manually disabling CREL flag..."
    sed -i 's|cflags += \[ "-Wa,--crel,--allow-experimental-crel" \]|# cflags += [ "-Wa,--crel,--allow-experimental-crel" ]  # Disabled for RK3588 build|' \
      build/config/compiler/BUILD.gn
  fi
fi

# ─────────────────────────────────────────────────────────────────────────────
# Step A5: Add MPP build targets to BUILD.gn files
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Step A5: Add MPP build targets ==="

cd "$BUILD_DIR/src"

# --- Add webrtc_rockchip_mpp target to modules/video_coding/BUILD.gn ---
if ! grep -q "webrtc_rockchip_mpp" modules/video_coding/BUILD.gn; then
  echo "  Adding webrtc_rockchip_mpp target to modules/video_coding/BUILD.gn..."

  cat >> modules/video_coding/BUILD.gn << 'BUILDGN_EOF'

# RK3588 MPP Hardware Codec
rtc_library("webrtc_rockchip_mpp") {
  visibility = [ "*" ]
  sources = [
    "codecs/mpp/rockchip_video_decoder.cc",
    "codecs/mpp/rockchip_video_decoder.h",
    "codecs/mpp/rockchip_video_decoder_factory.cc",
    "codecs/mpp/rockchip_video_decoder_factory.h",
    "codecs/mpp/rockchip_video_encoder.cc",
    "codecs/mpp/rockchip_video_encoder.h",
    "codecs/mpp/rockchip_video_encoder_factory.cc",
    "codecs/mpp/rockchip_video_encoder_factory.h",
  ]

  deps = [
    ":video_codec_interface",
    ":video_coding_utility",
    "../../api:scoped_refptr",
    "../../api/environment",
    "../../api/video:encoded_image",
    "../../api/video:video_frame",
    "../../api/video:video_rtp_headers",
    "../../api/video_codecs:video_codecs_api",
    "../../common_video",
    "../../common_video:dmabuf_video_frame_buffer",
    "../../media:rtc_media_base",
    "../../media:media_constants",
    "../../rtc_base:checks",
    "../../rtc_base:logging",
    "../../rtc_base:timeutils",
    "../../rtc_base/synchronization:mutex",
    "../../system_wrappers",
    "//third_party/abseil-cpp/absl/strings",
    "//third_party/libyuv",
  ]

  include_dirs = [
    "//third_party/mpp/inc",
    "//third_party/mpp/osal/inc",
    "//third_party/mpp/mpp/base/inc",
  ]

  libs = [ "rockchip_mpp" ]

  cflags = [
    "-Wno-unused-parameter",
    "-Wno-sign-compare",
  ]

  if (!is_linux) {
    sources = []
    deps = []
  }
}
BUILDGN_EOF
  echo "  Target added"
else
  echo "  webrtc_rockchip_mpp target already exists"
fi

# --- Add dmabuf_video_frame_buffer target to common_video/BUILD.gn ---
if ! grep -q "dmabuf_video_frame_buffer" common_video/BUILD.gn; then
  echo "  Adding dmabuf_video_frame_buffer target to common_video/BUILD.gn..."

  cat >> common_video/BUILD.gn << 'BUILDGN_EOF'

# RK3588 DMA-BUF VideoFrameBuffer
rtc_library("dmabuf_video_frame_buffer") {
  visibility = [ "*" ]
  sources = [
    "dmabuf_video_frame_buffer.cc",
    "dmabuf_video_frame_buffer.h",
  ]
  deps = [
    "../api:scoped_refptr",
    "../api/video:video_frame",
    "../rtc_base:checks",
    "../rtc_base:logging",
    "//third_party/libyuv",
  ]

  if (!is_linux) {
    sources = []
    deps = []
  }
}
BUILDGN_EOF
  echo "  Target added"
else
  echo "  dmabuf_video_frame_buffer target already exists"
fi

# --- Add MPP deps to the top-level :webrtc target ---
if ! grep -q "webrtc_rockchip_mpp" BUILD.gn; then
  echo "  Adding MPP deps to :webrtc target in top-level BUILD.gn..."
  sed -i 's|"api/crypto:frame_crypto_transformer",|"api/crypto:frame_crypto_transformer",\n      "modules/video_coding:webrtc_rockchip_mpp",\n      "common_video:dmabuf_video_frame_buffer",|' BUILD.gn
  echo "  MPP deps added to :webrtc target"
else
  echo "  MPP deps already in :webrtc target"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Step A6: Build
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Step A6: Generate, build, and package ==="

OUTPUT_DIR="$BUILD_DIR/src/out-${arch}-${profile}"
ARTIFACTS_DIR="$BUILD_DIR/linux-${arch}-${profile}"

# Install sysroot for cross-compilation
echo "  Installing sysroot..."
python3 "$BUILD_DIR/src/build/linux/sysroot_scripts/install-sysroot.py" --arch="$arch"

# Fix GLIBC compatibility issue on some systems
if [ "$arch" = "arm64" ]; then
  if [ -f /usr/aarch64-linux-gnu/include/features.h ]; then
    sudo sed -i 's/__GLIBC_USE_ISOC2X[[:space:]]*1/__GLIBC_USE_ISOC2X\t0/' \
      /usr/aarch64-linux-gnu/include/features.h 2>/dev/null || true
  fi
fi

debug="false"
if [ "$profile" = "debug" ]; then
  debug="true"
fi

# GN args -- based on LiveKit's build_linux.sh, with SME fix added
args="is_debug=$debug \
  target_os=\"linux\" \
  target_cpu=\"$arch\" \
  rtc_enable_protobuf=false \
  treat_warnings_as_errors=false \
  use_custom_libcxx=false \
  use_llvm_libatomic=false \
  use_libcxx_modules=false \
  use_custom_libcxx_for_host=false \
  rtc_include_tests=false \
  rtc_build_tools=false \
  rtc_build_examples=false \
  rtc_libvpx_build_vp9=true \
  enable_libaom=true \
  is_component_build=false \
  enable_stripping=true \
  ffmpeg_branding=\"Chrome\" \
  rtc_use_h264=true \
  rtc_use_h265=true \
  rtc_use_pipewire=true \
  symbol_level=0 \
  enable_iterator_debugging=false \
  use_rtti=true \
  is_clang=true \
  rtc_use_x11=true \
  libyuv_use_sme=false"

echo "  Generating ninja files..."
gn gen "$OUTPUT_DIR" --root="$BUILD_DIR/src" --args="${args}"

echo "  Building (this takes a while)..."
ninja -C "$OUTPUT_DIR" :default

# ─────────────────────────────────────────────────────────────────────────────
# Package artifacts
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Packaging artifacts ==="

mkdir -p "$ARTIFACTS_DIR/lib"

# Pick the right tools for native vs cross-compile
LLVM_BIN="$BUILD_DIR/src/third_party/llvm-build/Release+Asserts/bin"
if [ -x "$LLVM_BIN/llvm-ar" ]; then
  AR_TOOL="$LLVM_BIN/llvm-ar"
elif [ "$cross_compile" = "true" ] && command -v aarch64-linux-gnu-ar &>/dev/null; then
  AR_TOOL="aarch64-linux-gnu-ar"
else
  AR_TOOL="ar"
fi

if [ -x "$LLVM_BIN/llvm-objcopy" ]; then
  OBJCOPY_TOOL="$LLVM_BIN/llvm-objcopy"
elif [ "$cross_compile" = "true" ] && command -v aarch64-linux-gnu-objcopy &>/dev/null; then
  OBJCOPY_TOOL="aarch64-linux-gnu-objcopy"
else
  OBJCOPY_TOOL="objcopy"
fi

echo "  Using ar: $AR_TOOL"
echo "  Using objcopy: $OBJCOPY_TOOL"

echo "  Creating libwebrtc.a (including MPP objects)..."
"$AR_TOOL" -rc "$ARTIFACTS_DIR/lib/libwebrtc.a" \
  $(find "$OUTPUT_DIR/obj" -name '*.o' -not -path "*/third_party/nasm/*")

# Apply boringssl symbol prefixing
if [ -f "$LIVEKIT_LIBWEBRTC_DIR/boringssl_prefix_symbols.txt" ]; then
  echo "  Applying boringssl symbol prefixing..."
  "$OBJCOPY_TOOL" --redefine-syms="$LIVEKIT_LIBWEBRTC_DIR/boringssl_prefix_symbols.txt" \
    "$ARTIFACTS_DIR/lib/libwebrtc.a"
fi

# Generate licenses
echo "  Generating licenses..."
python3 "$BUILD_DIR/src/tools_webrtc/libs/generate_licenses.py" \
  --target :default "$OUTPUT_DIR" "$OUTPUT_DIR" 2>/dev/null || true

# Copy metadata
cp "$OUTPUT_DIR/obj/webrtc.ninja" "$ARTIFACTS_DIR" 2>/dev/null || true
cp "$OUTPUT_DIR/obj/modules/desktop_capture/desktop_capture.ninja" "$ARTIFACTS_DIR" 2>/dev/null || true
cp "$OUTPUT_DIR/args.gn" "$ARTIFACTS_DIR"
[ -f "$OUTPUT_DIR/LICENSE.md" ] && cp "$OUTPUT_DIR/LICENSE.md" "$ARTIFACTS_DIR"

# Copy headers
echo "  Copying headers..."
cd "$BUILD_DIR/src"
find . -name "*.h" -print | cpio -pd "$ARTIFACTS_DIR/include" 2>/dev/null
find . -name "*.inc" -print | cpio -pd "$ARTIFACTS_DIR/include" 2>/dev/null

# Also create a symlink in the repo root for convenience
cd "$SCRIPT_DIR"
if [ ! -e "linux-${arch}-${profile}" ]; then
  ln -sf "$ARTIFACTS_DIR" "linux-${arch}-${profile}"
fi

echo ""
echo "========================================"
echo "Build complete!"
echo ""
echo "Artifacts: $ARTIFACTS_DIR"
echo "  lib/libwebrtc.a   -- static library with MPP objects"
echo "  include/           -- all WebRTC + MPP headers"
echo "  args.gn            -- GN arguments used"
echo ""
echo "Verify MPP symbols:"
echo "  nm $ARTIFACTS_DIR/lib/libwebrtc.a | grep -i Rockchip"
echo ""
echo "To use with LiveKit SDK:"
echo "  export LK_CUSTOM_WEBRTC=$ARTIFACTS_DIR"
echo "  cd $livekit_sdk_dir && cargo build --release"
echo "========================================"
