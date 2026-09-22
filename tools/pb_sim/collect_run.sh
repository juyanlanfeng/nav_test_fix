#!/usr/bin/env bash
# Collect one closed-loop evidence run for the PB simulation (R5, plan section 8).
#
# Fixes over the historical log/run_slope_reproduction.sh:
#   * every terminal loads one explicit, immutable session file (no global
#     "last experiment" domain/partition file);
#   * the isolation check gates the run and treats timeouts as failures;
#   * readiness and preflight return codes gate the goal; nothing is sent on a
#     failed prerequisite;
#   * the recorder must show real data on /clock and /odom before the goal is sent
#     (no fixed "sleep is ready" assumption);
#   * the goal is cancelled through R2's --cancel-after, so the cancellation
#     targets the UUID this run created;
#   * EXIT/INT/TERM traps tear down only this run's process trees, stopping the
#     recorder first so the bag is flushed (no global pkill);
#   * environment/assets are captured after the domain and partition are final;
#   * times are written with nine nanosecond digits.
#
# Usage:
#   tools/pb_sim/collect_run.sh --session log/pb_sim_sessions/<id>.json \
#       [--framework meshnav] [--controller-plugin pb_terminal_controller] \
#       [--x 1.35 --y 5.95 --z 0.203 --yaw 0] \
#       [--spawn-x -1.90 --spawn-y 5.95] [--cancel-after 30] [--label ramp]
set -o pipefail

ROOT="/home/rainple/nav_test"
cd "$ROOT" || exit 2

SESSION=""
FRAMEWORK="meshnav"
CONTROLLER_PLUGIN="mesh_controller"
SPAWN_X="-1.90"; SPAWN_Y="5.95"; SPAWN_Z="0.25"; SPAWN_YAW="0"
GOAL_X=""; GOAL_Y=""; GOAL_Z="0.0"; GOAL_YAW="0"
CANCEL_AFTER="30"
LABEL="run"
COLLECT_SECONDS="150"
CONTACT_DIAGNOSTICS="False"
POST_GOAL_SECONDS="5"
DRIVE_MODEL="legacy"
HEIGHT_DIFF_THRESHOLD="0.2"
MAP_CACHE="$ROOT/meshnav_demo_ws/rmuc2026_pb_navigation.h5"

while [ $# -gt 0 ]; do
  case "$1" in
    --session) SESSION="$2"; shift 2 ;;
    --framework) FRAMEWORK="$2"; shift 2 ;;
    --controller-plugin) CONTROLLER_PLUGIN="$2"; shift 2 ;;
    --spawn-x) SPAWN_X="$2"; shift 2 ;;
    --spawn-y) SPAWN_Y="$2"; shift 2 ;;
    --spawn-z) SPAWN_Z="$2"; shift 2 ;;
    --spawn-yaw) SPAWN_YAW="$2"; shift 2 ;;
    --x) GOAL_X="$2"; shift 2 ;;
    --y) GOAL_Y="$2"; shift 2 ;;
    --z) GOAL_Z="$2"; shift 2 ;;
    --yaw) GOAL_YAW="$2"; shift 2 ;;
    --cancel-after) CANCEL_AFTER="$2"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    --collect-seconds) COLLECT_SECONDS="$2"; shift 2 ;;
    --contact-diagnostics) CONTACT_DIAGNOSTICS="True"; shift ;;
    --post-goal-seconds) POST_GOAL_SECONDS="$2"; shift 2 ;;
    --drive-model) DRIVE_MODEL="$2"; shift 2 ;;
    --height-diff-threshold) HEIGHT_DIFF_THRESHOLD="$2"; shift 2 ;;
    --map-cache) MAP_CACHE="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$SESSION" ] || [ ! -f "$SESSION" ]; then
  echo "FAIL --session must point at an existing session file (see tools/pb_sim/session.py create)" >&2
  exit 2
fi

# Sim time with nine nanosecond digits.
sim_time() {
  timeout 10 ros2 topic echo --once --field clock /clock 2>/dev/null \
    | python3 -c 'import sys,re
text=sys.stdin.read()
sec=re.search(r"^sec:\s*(\d+)", text, re.M)
nano=re.search(r"^nanosec:\s*(\d+)", text, re.M)
print("%s.%09d" % (sec.group(1), int(nano.group(1))) if sec and nano else "")'
}

wall_time() { python3 -c 'import time; print("%d.%09d" % (int(time.time()), time.time_ns() % 1000000000))'; }

# Kill one process tree.  Only trees this run started are ever passed in.
kill_tree() {
  local pid=$1 sig=${2:-INT} child
  [ -n "$pid" ] || return 0
  for child in $(cat "/proc/$pid/task/$pid/children" 2>/dev/null); do
    kill_tree "$child" "$sig"
  done
  kill -"$sig" "$pid" 2>/dev/null
}

source /opt/ros/humble/setup.bash
source "$ROOT/meshnav_demo_ws/install/setup.bash"
# shellcheck disable=SC1090
source <(python3 "$ROOT/tools/pb_sim/session.py" env "$SESSION")
export ROS_HOME
mkdir -p "$ROS_HOME/log"

REPORT_DIR="$ROOT/log/${LABEL}_report_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$REPORT_DIR"
python3 - "$REPORT_DIR/run_config.json" "$SESSION" "$CONTROLLER_PLUGIN" "$DRIVE_MODEL" \
  "$SPAWN_X" "$SPAWN_Y" "$SPAWN_Z" "$SPAWN_YAW" \
  "$GOAL_X" "$GOAL_Y" "$GOAL_Z" "$GOAL_YAW" "$MAP_CACHE" \
  "$HEIGHT_DIFF_THRESHOLD" "$CONTACT_DIAGNOSTICS" "$CANCEL_AFTER" "$POST_GOAL_SECONDS" <<'PY'
import json, sys
keys = ['session', 'controller', 'drive_model', 'spawn_x', 'spawn_y', 'spawn_z',
        'spawn_yaw_deg', 'goal_x', 'goal_y', 'goal_z', 'goal_yaw', 'map_cache',
        'height_diff_threshold', 'contact_diagnostics', 'cancel_after', 'post_goal_seconds']
with open(sys.argv[1], 'w') as stream:
    json.dump(dict(zip(keys, sys.argv[2:])), stream, indent=2)
PY
SIM_LOG="$REPORT_DIR/sim.log"; NAV_LOG="$REPORT_DIR/nav.log"
GOAL_LOG="$REPORT_DIR/goal.log"; BAG_LOG="$REPORT_DIR/bag_record.log"
: > "$SIM_LOG"; : > "$NAV_LOG"; : > "$GOAL_LOG"; : > "$BAG_LOG"

SIM_PID=""; NAV_PID=""; BAG_PID=""; GOAL_PID=""
GZ_PIDS=()
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  echo "cleaning up (status $status)" | tee -a "$REPORT_DIR/summary.txt"
  # Recorder first: SIGINT lets rosbag2 flush and close the bag.
  if [ -n "$BAG_PID" ]; then
    kill_tree "$BAG_PID" INT
    for _ in $(seq 1 20); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.5; done
  fi
  kill_tree "$GOAL_PID" INT
  for pid in "${GZ_PIDS[@]}"; do kill_tree "$pid" INT; done
  kill_tree "$NAV_PID" INT
  kill_tree "$SIM_PID" INT
  sleep 8
  kill_tree "$NAV_PID" KILL
  kill_tree "$SIM_PID" KILL
  kill_tree "$GOAL_PID" KILL
  for pid in "${GZ_PIDS[@]}"; do kill_tree "$pid" KILL; done
  exit "$status"
}
trap cleanup EXIT INT TERM

echo "report dir: $REPORT_DIR"
echo "session   : $(python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print(r["session_id"], r["domain"], r["partition"])' "$SESSION")"

# ---- the domain must be free before anything is started -------------------
# `--mode free`: no publisher may own the domain yet, so a leftover session (or a
# second terminal that picked the same domain) is caught before it can pollute the
# evidence.  A failed query counts as "cannot prove it is free".
python3 "$ROOT/tools/pb_sim/check_isolation.py" --mode free \
  --report "$REPORT_DIR/isolation_before.json" > "$REPORT_DIR/isolation_before.txt" 2>&1
ISOLATION_STATUS=$?
cat "$REPORT_DIR/isolation_before.txt"
if [ "$ISOLATION_STATUS" -ne 0 ]; then
  echo "FAIL domain is not free (pick another ACCEPT domain when creating the session)" \
    | tee -a "$REPORT_DIR/summary.txt"
  exit 4
fi

# ---- environment and assets, captured with the final domain/partition -----
python3 "$ROOT/tools/pb_sim/session.py" env "$SESSION" > "$REPORT_DIR/environment.txt"
{
  echo "assets_taken_wall=$(wall_time)"
  sha256sum "$ROOT/meshnav_demo_ws/src/pb_vehicle_adapter/models/pb_navigation_robot.sdf" \
            "$ROOT/meshnav_demo_ws/src/pb_vehicle_adapter/urdf/pb_navigation_robot.urdf" \
            "$ROOT/meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials/maps/rmuc2026_field.ply" 2>/dev/null
  ls -l --time-style=full-iso "$MAP_CACHE" 2>/dev/null
  sha256sum "$MAP_CACHE" 2>/dev/null
  find "$ROOT/third_party" -name 'libMecanumDrive2*' -exec sha256sum {} \; 2>/dev/null
  sha256sum "$ROOT/meshnav_demo_ws/install/pb_gazebo_sim_support/lib/libpb_chassis_truth_system.so" 2>/dev/null
} > "$REPORT_DIR/assets.txt" 2>&1
git rev-parse HEAD > "$REPORT_DIR/git_head.txt" 2>&1
git status --short >> "$REPORT_DIR/git_head.txt" 2>&1

# ---- simulation ----------------------------------------------------------
ros2 launch pb_vehicle_adapter pb_vehicle_sim.launch.py \
  world_name:=rmuc2026_field control_source:=$FRAMEWORK \
  start_gazebo_gui:=False start_rviz:=False spawn_rendering_sensors:=False \
  contact_diagnostics:="$CONTACT_DIAGNOSTICS" \
  drive_model:="$DRIVE_MODEL" \
  spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_z:=$SPAWN_Z spawn_yaw_deg:=$SPAWN_YAW \
  > "$SIM_LOG" 2>&1 &
SIM_PID=$!

spawned=0
for _ in $(seq 1 60); do
  grep -q "creation of entity" "$SIM_LOG" 2>/dev/null && { spawned=1; break; }
  sleep 2
done
if [ "$spawned" != "1" ]; then
  echo "FAIL simulation did not spawn; see $SIM_LOG" | tee -a "$REPORT_DIR/summary.txt"
  exit 3
fi
echo "sim spawned"

# ---- navigation and the readiness gate -----------------------------------
if [ "$FRAMEWORK" = "meshnav" ]; then
  # The MeshNav entry selects its terminal-control layer explicitly.
  ros2 launch pb_vehicle_adapter "pb_${FRAMEWORK}.launch.py" \
    start_sim:=False start_rviz:=False startup_timeout_s:=180 \
    controller_plugin:="$CONTROLLER_PLUGIN" \
    mesh_map_working_path:="$MAP_CACHE" height_diff_threshold:="$HEIGHT_DIFF_THRESHOLD" \
    > "$NAV_LOG" 2>&1 &
else
  ros2 launch pb_vehicle_adapter "pb_${FRAMEWORK}.launch.py" \
    start_sim:=False start_rviz:=False startup_timeout_s:=180 > "$NAV_LOG" 2>&1 &
fi
NAV_PID=$!

timeout 240 ros2 run pb_vehicle_adapter pb_preflight --framework "$FRAMEWORK" --wait-timeout 120 \
  > "$REPORT_DIR/preflight.txt" 2>&1
PREFLIGHT_STATUS=$?
cat "$REPORT_DIR/preflight.txt"
if [ "$PREFLIGHT_STATUS" -ne 0 ]; then
  echo "FAIL preflight did not pass (rc=$PREFLIGHT_STATUS); not sending any goal" \
    | tee -a "$REPORT_DIR/summary.txt"
  exit 5
fi

# A second isolation check now that this run owns the domain: it must show this
# run's single owners and no leftovers from an earlier experiment.
python3 "$ROOT/tools/pb_sim/check_isolation.py" --mode running \
  --report "$REPORT_DIR/isolation_run.json" > "$REPORT_DIR/isolation_run.txt" 2>&1
if [ $? -ne 0 ]; then
  cat "$REPORT_DIR/isolation_run.txt"
  echo "FAIL isolation check failed after startup; refusing to send a goal" \
    | tee -a "$REPORT_DIR/summary.txt"
  exit 4
fi

# ---- recorder, with an actual-data readiness gate -------------------------
ros2 bag record --include-hidden-topics -o "$REPORT_DIR/closed_loop" \
  /clock /cmd_vel /pb/cmd_vel_safe /odom /pb/odom_raw /pb_sim/chassis_truth \
  /pb/truth_health /tf /tf_static /joint_states /rosout \
  /move_base_flex/current_goal /move_base_flex/path \
  /move_base_flex/exe_path/_action/status /move_base_flex/exe_path/_action/feedback \
  /mesh_controller/terminal_state /pb_terminal_controller/terminal_state \
  /pb_terminal_controller/terminal_debug \
  > "$BAG_LOG" 2>&1 &
BAG_PID=$!

subscribed=0
for _ in $(seq 1 60); do
  grep -q "Subscribed to topic" "$BAG_LOG" 2>/dev/null && { subscribed=1; break; }
  sleep 1
done
if [ "$subscribed" != "1" ]; then
  echo "FAIL recorder did not report any subscription" | tee -a "$REPORT_DIR/summary.txt"
  exit 6
fi
clock_ok=0; odom_ok=0
for _ in $(seq 1 30); do
  [ "$clock_ok" = "0" ] && timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 && clock_ok=1
  [ "$odom_ok" = "0" ] && timeout 5 ros2 topic echo --once /odom > /dev/null 2>&1 && odom_ok=1
  [ "$clock_ok" = "1" ] && [ "$odom_ok" = "1" ] && break
  sleep 1
done
if [ "$clock_ok" != "1" ] || [ "$odom_ok" != "1" ]; then
  echo "FAIL recorder readiness: clock=$clock_ok odom=$odom_ok" | tee -a "$REPORT_DIR/summary.txt"
  exit 6
fi
echo "recorder ready (subscriptions and live data confirmed)"

# ---- goal, cancelled through R2 with the same UUID -----------------------
timeout -s INT "$COLLECT_SECONDS" ign topic -e -t /world/rmuc2026_field/dynamic_pose/info \
  > "$REPORT_DIR/gz_dynamic_pose.txt" 2>&1 &
GZ_PIDS+=("$!")
timeout -s INT "$COLLECT_SECONDS" ign topic -e -t /robot/cmd_vel \
  > "$REPORT_DIR/gz_cmd_vel.txt" 2>&1 &
GZ_PIDS+=("$!")
# The stats stream is what carries the real-time factor; without it the analysis
# has no RTF evidence and reports the capture as missing.
timeout -s INT "$COLLECT_SECONDS" ign topic -e -t /world/rmuc2026_field/stats \
  > "$REPORT_DIR/gz_stats.txt" 2>&1 &
GZ_PIDS+=("$!")
# Read-only contact/actuator diagnostics (P1).  These topics exist only when the
# model opts in with <enable>true</enable>; with the default switch off the capture
# simply stays empty, which is what "default off" means in the evidence.
timeout -s INT "$COLLECT_SECONDS" ign topic -e -t /pb_sim/diagnostics/requested_wrench \
  > "$REPORT_DIR/gz_requested_wrench.txt" 2>&1 &
GZ_PIDS+=("$!")
timeout -s INT "$COLLECT_SECONDS" ign topic -e -t /pb_sim/diagnostics/joint_state \
  > "$REPORT_DIR/gz_joint_state.txt" 2>&1 &
GZ_PIDS+=("$!")
timeout -s INT "$COLLECT_SECONDS" ign topic -e -t /pb_sim/diagnostics/contact \
  > "$REPORT_DIR/gz_contact.txt" 2>&1 &
GZ_PIDS+=("$!")

{
  echo "goal_wall_sent=$(wall_time)"
  echo "goal_sim_sent=$(sim_time)"
} > "$REPORT_DIR/timeline.txt"

GOAL_CONTROLLER_ARGS=()
[ "$FRAMEWORK" = "meshnav" ] && GOAL_CONTROLLER_ARGS=(--controller "$CONTROLLER_PLUGIN")
timeout $((COLLECT_SECONDS + 180)) ros2 run pb_vehicle_adapter pb_nav_goal \
  --framework "$FRAMEWORK" "${GOAL_CONTROLLER_ARGS[@]}" \
  --x "$GOAL_X" --y "$GOAL_Y" --z "$GOAL_Z" --yaw "$GOAL_YAW" \
  --cancel-after "$CANCEL_AFTER" --timeout 120 > "$GOAL_LOG" 2>&1 &
GOAL_PID=$!

wait "$GOAL_PID"
GOAL_STATUS=$?
{
  echo "goal_wall_done=$(wall_time)"
  echo "goal_sim_done=$(sim_time)"
  echo "goal_exit_status=$GOAL_STATUS"
} >> "$REPORT_DIR/timeline.txt"
grep -v "selected interface" "$GOAL_LOG" | tail -15

# Keep recording the actual stop/hold after the action has returned.
sleep "$POST_GOAL_SECONDS"

# ---- stop the recorder cleanly so the bag is complete --------------------
kill_tree "$BAG_PID" INT
for _ in $(seq 1 30); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.5; done
echo "recorder stopped"
ros2 bag info "$REPORT_DIR/closed_loop" > "$REPORT_DIR/bag_info.txt" 2>&1

{
  echo "# goal"
  grep -v "selected interface" "$GOAL_LOG" | tail -20
  echo "# nav events"
  grep -iE "outcome|arrived|ignored|fail|error" "$NAV_LOG" | grep -v "selected interface" | tail -20
} > "$REPORT_DIR/summary.txt" 2>&1
cat "$REPORT_DIR/summary.txt"
echo "report dir: $REPORT_DIR"
echo "goal exit status: $GOAL_STATUS"
exit "$GOAL_STATUS"
