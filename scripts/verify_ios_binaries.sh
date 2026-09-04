#!/usr/bin/env bash
# Assert the iOS arm64 GDExtension + MuJoCo runtime were built against the
# iPhoneOS SDK (not macOS). Run on macOS after scripts/build_ios.sh.
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "verify_ios_binaries.sh must run on macOS (otool / lipo)." >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${GMJ_ADDON_BIN:-$ROOT/demo/addons/godot_mujoco/bin}"

echo "== staged files =="
ls -la "$BIN"

shopt -s nullglob
# godot-cpp may emit .dylib or .a depending on version / library type.
ext=( "$BIN"/libgodot_mujoco.ios.*.arm64.dylib "$BIN"/libgodot_mujoco.ios.*.arm64.a )
if [[ ${#ext[@]} -eq 0 ]]; then
  echo "FAIL: no libgodot_mujoco.ios.*.arm64.dylib/.a in $BIN" >&2
  ls -la "$BIN" >&2 || true
  exit 1
fi
EXT="${ext[0]}"
if [[ "$EXT" == *..* ]]; then
  echo "FAIL: iOS library name has an empty arch component: $EXT" >&2
  exit 1
fi

mj=( "$BIN"/libmujoco*.dylib "$BIN"/libmujoco*.so )
if [[ ${#mj[@]} -eq 0 ]]; then
  echo "FAIL: no libmujoco dylib staged in $BIN" >&2
  exit 1
fi

for f in "$EXT" "${mj[0]}"; do
  echo
  echo "== $f =="
  file "$f"
  lipo -info "$f" || true
  otool -hv "$f" | head -20
  echo "-- LC_LOAD_DYLIB --"
  otool -L "$f"
  if lipo -info "$f" | grep -qi x86_64; then
    echo "FAIL: $f contains x86_64 (expected iphoneos arm64)" >&2
    exit 1
  fi
  if ! lipo -info "$f" | grep -Eqi 'arm64|aarch64'; then
    echo "FAIL: $f is not arm64" >&2
    exit 1
  fi
  # Reject macOS-only binaries: they typically list /usr/lib/libSystem.B.dylib
  # *and* a macosx platform load command. iOS uses /usr/lib/libSystem.B.dylib
  # too, so we check LC_BUILD_VERSION / LC_VERSION_MIN_IPHONEOS.
  if otool -l "$f" | grep -q LC_VERSION_MIN_MACOSX; then
    echo "FAIL: $f is a macOS binary (LC_VERSION_MIN_MACOSX)" >&2
    exit 1
  fi
  if otool -l "$f" | grep -q 'platform macos'; then
    echo "FAIL: $f platform is macos, not ios" >&2
    exit 1
  fi
done

echo
echo "IOS BINARY CHECK: PASS"
echo "  extension: $EXT"
echo "  mujoco:    ${mj[0]}"
