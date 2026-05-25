#!/usr/bin/env bash
# Build BLASFEO + HPIPM from source at the tags OCS2's hpipm_catkin/blasfeo_catkin
# pin, and install them (static) to external/install/. One-time; portable across
# Ubuntu + macOS. The standalone OCS2 build (cmake/Ocs2Standalone.cmake) imports
# from external/install via HPIPM_INSTALL_DIR.
#
# Why from source (not a prebuilt): the HPIPM C API (e.g. d_ocp_qp_dim_set_all)
# changed across tags, and wb's HpipmInterface.cpp is written against these exact
# pinned tags.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXT="$ROOT/external"
PREFIX="$EXT/install"
mkdir -p "$EXT" "$PREFIX"

# Tags must match lib/ocs2_ros2/ocs2_sqp/{blasfeo_catkin,hpipm_catkin}/CMakeLists.txt
BLASFEO_TAG=ae6e2d1dea015862a09990b95905038a756ffc7d
HPIPM_TAG=255ffdf38d3a5e2c3285b29568ce65ae286e5faf

COMMON_FLAGS=(-G Ninja -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  -DBUILD_SHARED_LIBS=OFF
  -DTARGET=GENERIC   # GENERIC: portable C kernels (safe arm64 + x86 baseline)
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5)  # blasfeo/hpipm pin cmake_minimum_required<3.5, removed in CMake 4

echo "[build_hpipm] === BLASFEO ($BLASFEO_TAG) ==="
[ -d "$EXT/blasfeo/.git" ] || git clone https://github.com/giaf/blasfeo "$EXT/blasfeo"
git -C "$EXT/blasfeo" checkout --quiet "$BLASFEO_TAG"
cmake -S "$EXT/blasfeo" -B "$EXT/blasfeo/build" "${COMMON_FLAGS[@]}" \
  -DBLASFEO_EXAMPLES=OFF -DBLASFEO_TESTING=OFF
cmake --build "$EXT/blasfeo/build" --target install

echo "[build_hpipm] === HPIPM ($HPIPM_TAG) ==="
[ -d "$EXT/hpipm/.git" ] || git clone https://github.com/giaf/hpipm "$EXT/hpipm"
git -C "$EXT/hpipm" checkout --quiet "$HPIPM_TAG"
cmake -S "$EXT/hpipm" -B "$EXT/hpipm/build" "${COMMON_FLAGS[@]}" \
  -DHPIPM_TESTING=OFF -DBLASFEO_PATH="$PREFIX"
cmake --build "$EXT/hpipm/build" --target install

echo "[build_hpipm] done -> $PREFIX (libblasfeo.a, libhpipm.a)"
ls -la "$PREFIX/lib" | grep -E "blasfeo|hpipm" || true
