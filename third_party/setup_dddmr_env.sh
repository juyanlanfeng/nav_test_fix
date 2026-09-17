#!/bin/bash
# Runtime environment for the natively-built dddmr_navigation stack.
#
#   source /home/rainple/nav_test/third_party/setup_dddmr_env.sh
#
# Source this in every terminal that runs DDDMR nodes.  It is intentionally a
# separate, opt-in script: do NOT add it to ~/.bashrc or any global profile, so
# the rest of the machine keeps the system PCL 1.12 and no extra search paths.
#
# Why LD_LIBRARY_PATH is required:
#   The isolated gtsam .deb's `libmetis-gtsam.so` is a *transitive* dependency
#   of `libgtsam.so`.  The binaries carry a DT_RUNPATH entry that covers their
#   direct dependencies, but DT_RUNPATH is not consulted for transitive ones, so
#   the gtsam library directory must be on LD_LIBRARY_PATH.  The PCL 1.15 and
#   small_gicp directories are added as well for the same reason.
#   The prefix's plain `lib` directory is required too: DDDMR's local planner
#   creates an ackermann_msgs subscription, and the ROS type-support libraries
#   are loaded with dlopen at run time, so they never show up in `ldd`.
#
# Isolated prefixes (all under third_party/, nothing installed into /usr):
#   dddmr_apt/opt/ros/humble  gtsam 4.2.0 + ackermann-msgs (extracted .deb)
#   dddmr_pcl15               PCL 1.15.0 (source build)
#   dddmr_install             small_gicp

_dddmr_root=/home/rainple/nav_test
_dddmr_tp=$_dddmr_root/third_party
_dddmr_apt=$_dddmr_tp/dddmr_apt/opt/ros/humble
_dddmr_gtsam_lib=$_dddmr_apt/lib/x86_64-linux-gnu

if [ ! -f "$_dddmr_tp/dddmr_apt/opt/ros/humble/lib/libgtsam.so" ] && \
   [ ! -d "$_dddmr_tp/dddmr_apt/opt/ros/humble/lib" ]; then
  echo "setup_dddmr_env.sh: gtsam prefix not found under $_dddmr_tp/dddmr_apt" >&2
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$_dddmr_root/dddmr_navigation/install/setup.bash"

export LD_LIBRARY_PATH="$_dddmr_apt/lib/x86_64-linux-gnu:$_dddmr_apt/lib:$_dddmr_tp/dddmr_pcl15/lib:$_dddmr_tp/dddmr_install/lib:${LD_LIBRARY_PATH:-}"
export AMENT_PREFIX_PATH="$_dddmr_apt:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$_dddmr_tp/dddmr_pcl15:$_dddmr_tp/dddmr_install:$_dddmr_apt:${CMAKE_PREFIX_PATH:-}"
export PYTHONPATH="$_dddmr_apt/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"

unset _dddmr_root _dddmr_tp _dddmr_apt _dddmr_gtsam_lib
