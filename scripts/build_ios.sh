#!/usr/bin/env bash
# Cross-compile the GDExtension + from-source MuJoCo for iOS (iphoneos, arm64).
# Requires macOS + Xcode command-line tools. Exits 2 on Linux/Windows.
#
# Usage:
#   bash scripts/build_ios.sh
#   GODOTCPP_TARGET=template_release bash scripts/build_ios.sh
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "iOS builds require macOS + Xcode (this host is $(uname -s))." >&2
  echo "CI covers this on macos-latest; locally: run this script on your Mac." >&2
  exit 2
fi

if ! xcrun --sdk iphoneos --show-sdk-path >/dev/null 2>&1; then
  echo "Xcode iPhoneOS SDK not found. Install Xcode and accept the license." >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${GODOTCPP_TARGET:-template_debug}"
BUILD_DIR="${GMJ_IOS_BUILD_DIR:-$ROOT/build-ios}"
SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEPLOY="${GMJ_IOS_DEPLOYMENT_TARGET:-13.0}"

echo "Using iPhoneOS SDK: $SDK"
echo "deployment=$DEPLOY  target=$TARGET"

cmake -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOY" \
  -DCMAKE_OSX_SYSROOT="$SDK" \
  -DCMAKE_MACOSX_BUNDLE=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DGODOTCPP_TARGET="$TARGET" \
  -DGMJ_BUILD_MUJOCO_FROM_SOURCE=ON \
  -DGMJ_BUILD_CORE_SMOKE=ON \
  -DBUILD_APPLICATIONS=OFF

JOBS="$(sysctl -n hw.ncpu)"
cmake --build "$BUILD_DIR" -j "$JOBS"

echo "iOS build finished. Staged under demo/addons/godot_mujoco/bin/"
