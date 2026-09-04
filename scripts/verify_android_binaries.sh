#!/usr/bin/env bash
# Assert the Android arm64 GDExtension + MuJoCo runtime look like a real
# Android NDK build (not a Linux aarch64 binary). Used locally and in CI.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${GMJ_ADDON_BIN:-$ROOT/demo/addons/godot_mujoco/bin}"

echo "== staged files =="
ls -la "$BIN"

shopt -s nullglob
ext=( "$BIN"/libgodot_mujoco.android.*.arm64.so )
if [[ ${#ext[@]} -eq 0 ]]; then
  echo "FAIL: no libgodot_mujoco.android.*.arm64.so in $BIN" >&2
  exit 1
fi
EXT="${ext[0]}"

mj=( "$BIN"/libmujoco.so* )
if [[ ${#mj[@]} -eq 0 ]]; then
  echo "FAIL: no libmujoco.so* in $BIN" >&2
  exit 1
fi

smoke="$BIN/gmj_core_smoke"
if [[ ! -f "$smoke" ]]; then
  echo "FAIL: gmj_core_smoke not staged" >&2
  exit 1
fi

dump() {
  echo
  echo "== $1 =="
  file "$1"
  readelf -h "$1" | sed -n '1,20p'
  echo "-- NEEDED --"
  readelf -d "$1" | awk '/NEEDED/ {print}'
  echo "-- Android note --"
  readelf -p .note.android.ident "$1" 2>/dev/null || echo "(no .note.android.ident)"
}

dump "$EXT"
dump "${mj[0]}"
dump "$smoke"

# Machine must be AArch64 and OS ABI must NOT be GNU/Linux SYSV with libc.so.6.
for f in "$EXT" "${mj[0]}" "$smoke"; do
  readelf -h "$f" | tee /tmp/gmj-hdr.txt >/dev/null
  grep -q 'Machine:.*AArch64' /tmp/gmj-hdr.txt
  # Bionic binaries typically report OS/ABI UNIX - System V, so we distinguish
  # them by the Android note and by NEEDED libc.so (not libc.so.6).
  if readelf -d "$f" | grep -q 'NEEDED.*libc.so.6'; then
    echo "FAIL: $f links glibc (libc.so.6) — this is a Linux binary, not Android" >&2
    exit 1
  fi
done

if ! readelf -p .note.android.ident "$EXT" 2>/dev/null | grep -qi android; then
  echo "FAIL: $EXT missing Android identification note" >&2
  exit 1
fi

if ! readelf -d "$EXT" | grep -q 'NEEDED.*libmujoco'; then
  echo "FAIL: extension does not NEEDED libmujoco" >&2
  exit 1
fi

# readelf -s truncates long names; -W keeps the full export.
# Process substitution avoids `set -o pipefail` + `grep -q` SIGPIPE false fails.
if ! grep -q 'godot_mujoco_library_init' < <(readelf -Ws "$EXT"); then
  echo "FAIL: missing GDExtension entry symbol godot_mujoco_library_init" >&2
  exit 1
fi

if readelf -d "$EXT" | grep -q 'RUNPATH\|RPATH'; then
  readelf -d "$EXT" | awk '/RPATH|RUNPATH/ {print}'
  if ! readelf -d "$EXT" | grep -E 'RPATH|RUNPATH' | grep -q '\$ORIGIN'; then
    echo "WARN: RUNPATH is set but does not contain \$ORIGIN"
  fi
fi

echo
echo "ANDROID BINARY CHECK: PASS"
echo "  extension: $EXT"
echo "  mujoco:    ${mj[0]}"
echo "  smoke:     $smoke"
