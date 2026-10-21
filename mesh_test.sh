#!/usr/bin/env bash
# MeshNav 实车一键启动：传感器/LIO/重定位/TF/点云 + PCL + MeshNav
#
# 用法：
#   ./mesh_test.sh                    # 不启动 RViz
#   START_RVIZ=true ./mesh_test.sh    # 在机器人上启动 RViz
#
# 顺序：
#   1 Livox 驱动
#   2 FAST-LIVO2
#   3 registration（mesh_nav_ws，新地图）
#   4 hnurm_uart
#   5 tf_transformer
#   6 hnurm_bringup test.launch.py（地面分割/pointcloud_to_laserscan/滤波）
#   7 PCL 背景差分
#   8 MeshNav（等 registration 进入 TRACKING 后再启动）
#
# 注意：
# - 传感器/LIO/UART/TF 来自 /home/rm/26_Sentry。
# - registration/PCL/MeshNav 来自 /home/rm/nav_test_fix/mesh_nav_ws。
# - PCL 首次加载 1,216,145 点背景并建树需要一点时间；
#   以日志 "Loaded background point cloud" 为准，之后再看 /dynamic_obstacles。
# - 每个终端都 source 各自工作空间的 install/setup.bash，不能混用。

set -u

SENSOR_WS=/home/rm/26_Sentry
NAV_WS=/home/rm/nav_test_fix/mesh_nav_ws
ROS_SETUP=/opt/ros/humble/setup.bash
START_RVIZ="${START_RVIZ:-false}"
export START_RVIZ

for f in "$SENSOR_WS/install/setup.bash" "$NAV_WS/install/setup.bash"; do
  if [ ! -f "$f" ]; then
    echo "缺少 $f，请先完成对应工作空间的构建。"
    exit 1
  fi
done

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "未找到 gnome-terminal；请在图形终端运行，或按本文件顺序手动启动。"
  exit 1
fi

# 26_Sentry 工作空间终端
sensor_term() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -c "
    cd '$SENSOR_WS'
    source '$ROS_SETUP'
    source '$SENSOR_WS/install/setup.bash'
    $cmd
    exec bash"
}

# mesh_nav_ws 工作空间终端
nav_term() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -c "
    cd '$NAV_WS'
    source '$ROS_SETUP'
    source '$NAV_WS/install/setup.bash'
    $cmd
    exec bash"
}

echo "== 1/8 Livox 驱动"
sensor_term "livox" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 4

echo "== 2/8 FAST-LIVO2"
sensor_term "fast-livo" "ros2 launch fast_livo mapping_mid360.launch.py"
sleep 4

echo "== 3/8 registration（新地图）"
nav_term "registration" "ros2 launch registration registration.launch.py"
sleep 4

echo "== 4/8 hnurm_uart"
sensor_term "uart" "ros2 launch hnurm_uart hnurm_uart.launch.py"
sleep 4

echo "== 5/8 tf_transformer"
sensor_term "tf" "ros2 launch hnurm_bringup tf_transformer.launch.py"
sleep 4

echo "== 6/8 点云流"
sensor_term "pointcloud" "ros2 launch hnurm_bringup test.launch.py"
sleep 4

echo "== 7/8 等待 registration TRACKING"
# ROS 2 的 setup.bash 不是 set -u 安全的，source 期间临时关闭 nounset。
set +u
source "$ROS_SETUP"
source "$NAV_WS/install/setup.bash"
set -u
if ! python3 - <<'PY'
import subprocess
import sys
import time

for _ in range(90):
    try:
        out = subprocess.run(
            ['ros2', 'topic', 'echo', '/registration_status', '--once', '--field', 'data'],
            capture_output=True, text=True, timeout=3).stdout
    except Exception:
        out = ''
    if 'TRACKING' in out:
        print('registration status: TRACKING')
        sys.exit(0)
    time.sleep(1)

print('registration status: not TRACKING (timeout)')
sys.exit(1)
PY
then
    echo "registration 未达到 TRACKING，PCL / MeshNav 未启动。"
    echo "确认 registration 正常后，手动执行："
    echo "  cd $NAV_WS"
    echo "  source $ROS_SETUP"
    echo "  source install/setup.bash"
    echo "  ros2 launch pcl_obstacle_point_subtraction pcl_obstacle_point_subtraction.launch.py"
    echo "  ros2 launch pb_vehicle_adapter meshnav_pb_sim.launch.py start_rviz:=$START_RVIZ"
    exit 1
fi

echo "== 8/8 PCL + MeshNav（map->odom 已就绪）"
nav_term "pcl" "ros2 launch pcl_obstacle_point_subtraction pcl_obstacle_point_subtraction.launch.py"
sleep 3
nav_term "MeshNav" "ros2 launch pb_vehicle_adapter meshnav_pb_sim.launch.py"

echo "启动顺序已执行完成。"
echo "PCL 加载 1.2M 背景后才会出 /dynamic_obstacles；以 'Loaded background' 为准。"
echo "RViz：START_RVIZ=true ./mesh_test.sh，或在操作电脑单独启动。"
