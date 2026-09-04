#!/usr/bin/env bash
# Cross-compile the GDExtension + from-source MuJoCo for Android arm64-v8a.
#
# Requires an NDK. Resolution order:
#   1. $ANDROID_NDK or $ANDROID_NDK_HOME
#   2. $ANDROID_HOME/ndk/<newest>
#   3. On Linux: download pinned NDK r27c into $GMJ_NDK_CACHE (default ~/.cache)
#
# Usage:
#   bash scripts/build_android.sh
#   GODOTCPP_TARGET=template_release bash scripts/build_android.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NDK_VERSION="${GMJ_NDK_VERSION:-r27c}"
# Official SHA-1 from https://github.com/android/ndk/releases/tag/r27c
NDK_SHA1_LINUX="${GMJ_NDK_SHA1_LINUX:-090e8083a715fdb1a3e402d0763c388abb03fb4e}"
CACHE="${GMJ_NDK_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/godot-mujoco}"
ABI="${ANDROID_ABI:-arm64-v8a}"
API="${ANDROID_PLATFORM:-android-24}"
TARGET="${GODOTCPP_TARGET:-template_debug}"
BUILD_DIR="${GMJ_ANDROID_BUILD_DIR:-$ROOT/build-android}"

find_ndk() {
  if [[ -n "${ANDROID_NDK:-}" && -f "${ANDROID_NDK}/build/cmake/android.toolchain.cmake" ]]; then
    echo "$ANDROID_NDK"; return
  fi
  if [[ -n "${ANDROID_NDK_HOME:-}" && -f "${ANDROID_NDK_HOME}/build/cmake/android.toolchain.cmake" ]]; then
    echo "$ANDROID_NDK_HOME"; return
  fi
  if [[ -n "${ANDROID_HOME:-}" ]]; then
    local newest
    newest="$(ls -1d "${ANDROID_HOME}/ndk"/* 2>/dev/null | tail -1 || true)"
    if [[ -n "$newest" && -f "$newest/build/cmake/android.toolchain.cmake" ]]; then
      echo "$newest"; return
    fi
  fi
  echo ""
}

download_ndk_linux() {
  local dest="$CACHE/android-ndk-${NDK_VERSION}"
  local zip="$CACHE/android-ndk-${NDK_VERSION}-linux.zip"
  local url="https://dl.google.com/android/repository/android-ndk-${NDK_VERSION}-linux.zip"
  if [[ -f "$dest/build/cmake/android.toolchain.cmake" ]]; then
    printf '%s\n' "$dest"
    return
  fi
  mkdir -p "$CACHE"
  if [[ ! -f "$zip" ]]; then
    echo "Downloading Android NDK ${NDK_VERSION}..." >&2
    curl -fL --retry 3 -o "$zip.partial" "$url"
    mv "$zip.partial" "$zip"
  fi
  echo "${NDK_SHA1_LINUX}  ${zip}" | sha1sum -c - >&2
  rm -rf "$dest"
  unzip -q "$zip" -d "$CACHE"
  # Archive extracts to android-ndk-<ver>/
  if [[ ! -f "$dest/build/cmake/android.toolchain.cmake" ]]; then
    echo "NDK extract missing toolchain at $dest" >&2
    exit 1
  fi
  printf '%s\n' "$dest"
}

NDK="$(find_ndk)"
if [[ -z "$NDK" ]]; then
  if [[ "$(uname -s)" == "Linux" ]]; then
    NDK="$(download_ndk_linux)"
  else
    echo "No Android NDK found. On macOS install it via Android Studio / sdkmanager" >&2
    echo "and export ANDROID_NDK=\$HOME/Library/Android/sdk/ndk/<version>" >&2
    echo "(the official macOS NDK ships as a .dmg this script will not mount)." >&2
    exit 2
  fi
fi

echo "Using NDK: $NDK"
echo "ABI=$ABI  platform=$API  target=$TARGET"

cmake -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI="$ABI" \
  -DANDROID_PLATFORM="$API" \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE=Release \
  -DGODOTCPP_TARGET="$TARGET" \
  -DGMJ_BUILD_MUJOCO_FROM_SOURCE=ON \
  -DGMJ_BUILD_CORE_SMOKE=ON \
  -DCMAKE_C_COMPILER_LAUNCHER="${CMAKE_C_COMPILER_LAUNCHER:-}" \
  -DCMAKE_CXX_COMPILER_LAUNCHER="${CMAKE_CXX_COMPILER_LAUNCHER:-}"

cmake --build "$BUILD_DIR" -j "$(nproc 2>/dev/null || sysctl -n hw.ncpu)"

echo "Android build finished. Staged under demo/addons/godot_mujoco/bin/"
