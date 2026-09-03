#!/usr/bin/env bash
# Idempotent Cloud Agent setup for the godot-mujoco GDExtension workflow.
#
# - Fetches a standard Godot 4.6 binary (no .NET/Mono needed).
# - Builds the GDExtension, which auto-fetches + bundles the MuJoCo runtime and
#   godot-cpp (no manual MuJoCo install, no LD_LIBRARY_PATH).
# - Imports the demo project so it is ready to run.
#
# Safe to run repeatedly and safe on branches that do not contain gdextension/.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

GODOT_VERSION="4.6-stable"
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
if [ -d "$REPO_ROOT/gdextension" ]; then
  echo "[cloud_setup] Building godot-mujoco GDExtension..."
  # The default cc/c++ alternatives on this base image point at clang, which
  # cannot locate libstdc++ here; use gcc/g++ explicitly.
  cmake -S "$REPO_ROOT/gdextension" -B "$REPO_ROOT/gdextension/build" \
    -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
  cmake --build "$REPO_ROOT/gdextension/build" -j "$(nproc)"

  echo "[cloud_setup] Importing demo project..."
  "$GODOT_BIN" --headless --path "$REPO_ROOT/gdextension/demo" --import || true
else
  echo "[cloud_setup] gdextension/ not present on this branch; skipping build."
fi

echo "[cloud_setup] Done."
