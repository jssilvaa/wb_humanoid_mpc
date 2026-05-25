#!/usr/bin/env bash
# Configure + build the STANDALONE (non-ROS 2) disturbance-rejection workspace.
#
# This is the standalone CMake build used for the active-disturbance-rejection
# research contribution (see DISTURBANCE_REJECTION_PLAN.md). It is SEPARATE from
# the ROS 2 / colcon build driven by the Makefile — both coexist; this one does
# not touch the per-package ament CMakeLists.
#
# Usage:
#   ./build.sh                  # configure (if needed) and build
#   ./build.sh --clean          # wipe build_standalone/ and reconfigure
#   ./build.sh --reconfigure    # delete CMakeCache only, keep _deps (fast reset)
#   ./build.sh -t mujocoSimPDStand   # build a single target
#   ./build.sh --debug          # configure as Debug instead of Release
#
# Env overrides:
#   MUJOCO_DIR=/path/to/mujoco-3.8.0  use a prebuilt MuJoCo (fast)
#                                     unset => FETCH_MUJOCO=ON (build from source)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT/build_standalone"
BUILD_TYPE="Release"
TARGET=""
CLEAN=0
RECONFIGURE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)        CLEAN=1; shift ;;
    --reconfigure)  RECONFIGURE=1; shift ;;
    --debug)        BUILD_TYPE="Debug"; shift ;;
    --release)      BUILD_TYPE="Release"; shift ;;
    -t|--target)    TARGET="$2"; shift 2 ;;
    -h|--help)      sed -n '2,21p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ $CLEAN -eq 1 ]]; then
  echo "[build.sh] wiping $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

if [[ $RECONFIGURE -eq 1 ]]; then
  echo "[build.sh] removing CMakeCache to force reconfigure"
  rm -f "$BUILD_DIR/CMakeCache.txt"
  rm -rf "$BUILD_DIR/CMakeFiles"
fi

CMAKE_ARGS=(
  -S "$ROOT"
  -B "$BUILD_DIR"
  -G Ninja
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

if [[ -n "${MUJOCO_DIR:-}" ]]; then
  echo "[build.sh] using prebuilt MuJoCo at $MUJOCO_DIR"
  CMAKE_ARGS+=(-DMUJOCO_DIR="$MUJOCO_DIR" -DFETCH_MUJOCO=OFF)
else
  echo "[build.sh] MUJOCO_DIR unset — falling back to FETCH_MUJOCO=ON"
  CMAKE_ARGS+=(-DFETCH_MUJOCO=ON)
fi

# If a previous configure used a different generator, wipe CMake state to switch
# to Ninja cleanly while keeping _deps/ (avoid re-downloading MuJoCo sources).
if [[ -f "$BUILD_DIR/CMakeCache.txt" && ! -f "$BUILD_DIR/build.ninja" ]]; then
  echo "[build.sh] generator changed — clearing CMake cache (keeping _deps sources)"
  rm -f "$BUILD_DIR/CMakeCache.txt" "$BUILD_DIR/Makefile" "$BUILD_DIR/cmake_install.cmake"
  rm -rf "$BUILD_DIR/CMakeFiles"
  find "$BUILD_DIR/_deps" -maxdepth 1 -type d -name '*-subbuild' -exec rm -rf {} + 2>/dev/null || true
fi

if [[ ! -f "$BUILD_DIR/build.ninja" ]]; then
  cmake "${CMAKE_ARGS[@]}"
fi

if [[ -n "$TARGET" ]]; then
  cmake --build "$BUILD_DIR" --target "$TARGET"
else
  cmake --build "$BUILD_DIR"
fi

echo "[build.sh] done — binaries in $BUILD_DIR/bin/"
