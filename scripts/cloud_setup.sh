#!/usr/bin/env bash
# Idempotent Cloud Agent setup for the godot-mujoco GDExtension.
#
# - Fetches a standard Godot 4.6 binary (no .NET/Mono needed).
# - Builds the GDExtension, which auto-fetches + bundles the MuJoCo runtime and
#   godot-cpp (no manual MuJoCo install, no LD_LIBRARY_PATH).
# - Registers the extension and validates it end-to-end with the headless
#   smoke test.
#
# Safe to run repeatedly.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

GODOT_VERSION="4.7.2-stable"
GODOT_DIR="$HOME/godot"
GODOT_BIN="$GODOT_DIR/Godot_v${GODOT_VERSION}_linux.x86_64"
GODOT_URL="https://github.com/godotengine/godot-builds/releases/download/${GODOT_VERSION}/Godot_v${GODOT_VERSION}_linux.x86_64.zip"

# --- Godot binary ---------------------------------------------------------
if [ ! -x "$GODOT_BIN" ]; then
  echo "[cloud_setup] Downloading Godot ${GODOT_VERSION}..."
  mkdir -p "$GODOT_DIR"
  curl -fsSL -o /tmp/godot.zip "$GODOT_URL"
  unzip -o /tmp/godot.zip -d "$GODOT_DIR" >/dev/null
  rm -f /tmp/godot.zip
fi
"$GODOT_BIN" --headless --version || true

# --- GDExtension build (auto-fetches MuJoCo + godot-cpp) -------------------
if [ -f "$REPO_ROOT/CMakeLists.txt" ] && [ -d "$REPO_ROOT/demo" ]; then
  echo "[cloud_setup] Building godot-mujoco GDExtension..."
  # The default cc/c++ alternatives on this base image point at clang, which
  # cannot locate libstdc++ here; use gcc/g++ explicitly.
  cmake -S "$REPO_ROOT" -B "$REPO_ROOT/build" \
    -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
  cmake --build "$REPO_ROOT/build" -j "$(nproc)"

  # Import/scan pass. This registers the GDExtension (writes
  # .godot/extension_list.cfg) so the native MjWorld class is available to
  # GDScript at runtime. Godot's headless editor can segfault on teardown
  # *after* the registration is written, so tolerate a non-zero exit here.
  echo "[cloud_setup] Importing demo project (registers the GDExtension)..."
  timeout 180 "$GODOT_BIN" --headless --path "$REPO_ROOT/demo" --import || true

  # Validate end-to-end: the headless smoke test loads the model and steps
  # MuJoCo entirely in-engine. Match the PASS marker rather than the exit code,
  # since the headless renderer can segfault on teardown after a clean quit.
  echo "[cloud_setup] Running headless MuJoCo smoke test..."
  if timeout 120 "$GODOT_BIN" --headless --path "$REPO_ROOT/demo" res://HeadlessTest.tscn 2>&1 \
      | tee /tmp/gmj_smoke.log | grep -q "SMOKE TEST: PASS"; then
    echo "[cloud_setup] Smoke test PASSED."
  else
    echo "[cloud_setup] Smoke test FAILED:"
    cat /tmp/gmj_smoke.log
    exit 1
  fi
else
  echo "[cloud_setup] Project not present; skipping build."
fi

echo "[cloud_setup] Done."
