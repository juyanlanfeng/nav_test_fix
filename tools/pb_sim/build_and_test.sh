#!/usr/bin/env bash
# One-click build + test for the PB simulation stack
# (doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md sections 9 and 10.3).
#
# Builds the packages this project owns, then runs every test suite that guards the
# repaired behaviour: the ground-truth adapter and goal client, the terminal-control
# logic, and the tracked simulation tools.  Non-zero exit if any stage fails.
#
#   tools/pb_sim/build_and_test.sh
set -o pipefail

ROOT="/home/rainple/nav_test"
cd "$ROOT" || exit 2

export ROS_HOME="${ROS_HOME:-$ROOT/log/ros_accept_home}"
mkdir -p "$ROS_HOME/log"
# Keep the test run out of whatever domain a simulation may be using.
export ROS_DOMAIN_ID="${ROOT_TEST_DOMAIN:-209}"
export ROS_LOCALHOST_ONLY=1

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/meshnav_demo_ws/install/setup.bash" 2>/dev/null

PACKAGES=(pb_gazebo_sim_support pb_terminal_controller pb_vehicle_adapter
          mesh_controller mesh_navigation_tutorials)
failures=0

echo "=== build: ${PACKAGES[*]} ==="
(cd "$ROOT/meshnav_demo_ws" && \
  colcon build --symlink-install --parallel-workers 2 --packages-select "${PACKAGES[@]}")
BUILD_STATUS=$?
echo "build exit code: $BUILD_STATUS"
[ "$BUILD_STATUS" -eq 0 ] || failures=$((failures + 1))

echo "=== gtest: terminal control logic ==="
timeout 300 "$ROOT/meshnav_demo_ws/build/pb_terminal_controller/test_terminal_logic" \
  || failures=$((failures + 1))

echo "=== gtest: diagnostics switches and message layout ==="
timeout 300 "$ROOT/meshnav_demo_ws/build/pb_gazebo_sim_support/test_diagnostics_logic" \
  || failures=$((failures + 1))

echo "=== pytest: pb_vehicle_adapter ==="
(cd "$ROOT/meshnav_demo_ws/src/pb_vehicle_adapter" && timeout 300 python3 -m pytest test -q) \
  || failures=$((failures + 1))

echo "=== pytest: tools/pb_sim ==="
(cd "$ROOT/tools" && timeout 300 python3 -m pytest pb_sim/test_pb_sim_tools.py -q) \
  || failures=$((failures + 1))

echo
if [ "$failures" -eq 0 ]; then
  echo "build_and_test: ALL PASS"
else
  echo "build_and_test: $failures stage(s) FAILED"
fi
exit "$failures"
