#!/bin/bash
# Build dddmr_navigation from source without Docker and without touching the
# system PCL (1.12).  The four packages that require PCL 1.15 (lego_loam_bor,
# mcl_3dl, dddmr_pg_map_server, dddmr_semantic_segmentation) are built against an
# isolated PCL 1.15 prefix; everything else keeps using the system PCL.
#
# Isolated prefixes (all under third_party/, none installed into /usr):
#   dddmr_pcl15    PCL 1.15.0 built from source (BUILD_visualization=OFF)
#   dddmr_install  small_gicp
#   dddmr_apt      gtsam 4.2.0 + ackermann-msgs (extracted .deb)
#
# Usage: bash third_party/build_dddmr_native.sh [extra colcon args...]
set -euo pipefail

ROOT=/home/rainple/nav_test
TP=$ROOT/third_party

# --- one-time: build PCL 1.15 into an isolated prefix (skip if present) --------
if [ ! -f "$TP/dddmr_pcl15/share/pcl-1.15/PCLConfig.cmake" ]; then
  echo "PCL 1.15 not found; building it into $TP/dddmr_pcl15 ..."
  [ -d "$TP/dddmr_sources/pcl" ] || \
    git clone --depth 1 --branch pcl-1.15.0 \
      https://github.com/PointCloudLibrary/pcl.git "$TP/dddmr_sources/pcl"
  cmake -S "$TP/dddmr_sources/pcl" -B "$TP/dddmr_build/pcl15" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$TP/dddmr_pcl15" \
    -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF \
    -DBUILD_visualization=OFF -DBUILD_simulation=OFF \
    -DWITH_QT=OFF -DWITH_VTK=OFF \
    -DPCL_ENABLE_MARCHNATIVE=OFF -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_AVX=OFF
  cmake --build "$TP/dddmr_build/pcl15" -j"$(nproc)"
  cmake --install "$TP/dddmr_build/pcl15"
fi

# --- build dddmr_navigation ---------------------------------------------------
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/meshnav_demo_ws/install/setup.bash" 2>/dev/null || true

export CMAKE_PREFIX_PATH="$TP/dddmr_pcl15:$TP/dddmr_install:$TP/dddmr_apt/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$TP/dddmr_apt/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$TP/dddmr_pcl15/lib:${LD_LIBRARY_PATH:-}"

cd "$ROOT/dddmr_navigation"

# --- tracked upstream fixes (idempotent) --------------------------------------
# dddmr-06d50cc-actuator-type.patch: upstream never calls
# configurateActuatorType(), so TrajectoryGeneratorTheory::actuator_type_ stays
# indeterminate and P2PMoveBase::publishVelocity() discards every trajectory
# ("Actuator type is not defined in Commanding Trajectory!").
for patch in "$TP"/patches/dddmr-*.patch; do
  [ -e "$patch" ] || continue
  if git apply --reverse --check "$patch" 2>/dev/null; then
    echo "patch already applied: $(basename "$patch")"
  else
    git apply "$patch"
    echo "applied patch: $(basename "$patch")"
  fi
done

# The gtsam .deb only defines GTSAM_INCLUDE_DIR, while the upstream CMakeLists
# uses GTSAM_INCLUDE_DIRS; pass it explicitly instead of patching upstream.
colcon build --symlink-install --parallel-workers "${JOBS:-12}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_INCLUDE_DIRS="$TP/dddmr_apt/opt/ros/humble/include" \
  "$@"

echo "dddmr_navigation built. Source the runtime environment (the install"
echo "setup.bash alone does not cover the transitive libmetis-gtsam.so) with:"
echo "  source $TP/setup_dddmr_env.sh"
