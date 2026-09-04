#!/usr/bin/env bash
# Sourceable pin reader. Sets MUJOCO_VERSION, GODOT_CPP_TAG, GODOT_VERSION
# from the single sources of truth (CMakeLists.txt + cloud_setup.sh).
#
#   source scripts/pinned_versions.sh
#   echo "$MUJOCO_VERSION"
_gmj_pin_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MUJOCO_VERSION="$(sed -n 's/.*set(MUJOCO_VERSION "\([^"]*\)".*/\1/p' "$_gmj_pin_root/CMakeLists.txt" | head -1)"
GODOT_CPP_TAG="$(sed -n 's/.*set(GODOT_CPP_TAG "\([^"]*\)".*/\1/p' "$_gmj_pin_root/CMakeLists.txt" | head -1)"
GODOT_VERSION="$(sed -n 's/.*GODOT_VERSION="\([^"]*\)".*/\1/p' "$_gmj_pin_root/scripts/cloud_setup.sh" | head -1)"
export MUJOCO_VERSION GODOT_CPP_TAG GODOT_VERSION
