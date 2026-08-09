# SCAN-Planner 全向舵轮真机快速验证手册

本文只用于尽快验证 SCAN-Planner 在舵轮真机上的建图、膨胀、规划和 `/cmd_vel` 输出效果，不是最终生产部署方案。

本次临时假设如下：

- 把机器狗与全向舵轮底盘的运动能力视为相同；
- 不修改 B 样条规划器的速度、加速度和 jerk 限制；
- 底盘驱动已经封装好，直接接收 `geometry_msgs/msg/Twist` 类型的 `/cmd_vel`；
- SCAN 继续使用现有闭环控制器，输出 `linear.x`、`linear.y`、`angular.z`；
- 不加载 Go2 URDF，不启动仿真器和 `robot_state_publisher`；
- 把原来的前后双圆柱碰撞体临时改成一个覆盖底盘的 XY 外接圆；
- 先在断开底盘输出的情况下验证规划，再把输出接到真实 `/cmd_vel`。

> 这套流程仅用于低速、空旷场地验证。底盘虽然会自行处理物理限制，但它不能代替急停、人员隔离和碰撞测试流程。

## 1. 最终接线关系

```text
LIO/定位 ── /Odometry ───────────────┐
                                             ├─> scan_planner_node
LIO/定位 ── /LIVO2/imu_propagate ──────────────────┤
                                             │
LiDAR/LIO ─ /cloud_registered ──────────────┘
                                                   │
                                                   ├─> /planning/bspline
                                                   │
                                                   ▼
                                       closed_loop_controller
                                                   │
                                      /scan_quick/cmd_vel_test
                                                   │ 验证正确后改接
                                                   ▼
                                               /cmd_vel
                                                   │
                                                   ▼
                                           已封装的舵轮底盘
```

下面命令默认使用仓库当前真机话题：

| 用途 | 默认话题 | 类型 |
|---|---|---|
| 车体位姿 | `/Odometry` | `nav_msgs/msg/Odometry` |
| IMU/传感器位姿 | `/LIVO2/imu_propagate` | `nav_msgs/msg/Odometry` |
| LiDAR 点云 | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` |
| 平地目标 | `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` |
| 带高度参考路径 | `/initial_path` | `nav_msgs/msg/Path` |
| 规划轨迹 | `/planning/bspline` | `scan_planner_msgs/msg/Bspline` |
| 底盘速度 | `/cmd_vel` | `geometry_msgs/msg/Twist` |

如果真机话题名称不同，只替换命令中对应的话题，不要修改消息类型。

## 2. 计算单外接圆半径

先测量底盘最大外形长度 `L` 和最大外形宽度 `W`，单位为米。把保险杠、轮子转向扫过的范围和不可碰撞的传感器支架包含进去。

外接圆半径：

```text
R = sqrt((L / 2)^2 + (W / 2)^2) + 安全余量
```

快速验证可先使用 `0.05 m` 安全余量。例如底盘长 `0.80 m`、宽 `0.60 m`：

```text
R = sqrt(0.40^2 + 0.30^2) + 0.05 = 0.55 m
```

后文示例使用 `R=0.55`。必须把它换成你们自己的计算结果。

本项目现有碰撞体是半径相同、前后错开的双圆柱：

```text
grid_map.double_cylinder_radius
grid_map.double_cylinder_offset
```

快速验证时设置：

```text
grid_map.double_cylinder_radius = R
grid_map.double_cylinder_offset = 0.0
```

偏移为零后，前后两个圆柱完全重合，碰撞检测和膨胀层等效为一个 XY 圆形碰撞体，不需要修改源码。

## 3. 编译并准备环境

打开终端 1：

```bash
cd /home/rainple/nav_test/SCAN-Planner
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix scan_planner
```

最后一条应输出类似：

```text
/home/rainple/nav_test/SCAN-Planner/install/scan_planner
```

如果提示找不到包，说明编译失败或忘记执行 `source install/setup.bash`，先不要继续。

后面每打开一个新终端，都先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash
```

开始真机验证前，先关闭之前启动的 SCAN 仿真或旧 `run.launch.py`。检查：

```bash
ros2 node list | sort
```

如果列表中已经存在 `/scan_planner_node`、`/closed_loop_controller`、`/open_loop_controller`、`/go2_kinematic_sim` 或 `/go2_robot_state_publisher`，回到启动它们的原终端按 `Ctrl+C`。不要让旧控制器和本手册的新控制器同时运行，也不要用不清楚目标范围的批量杀进程命令。

## 4. 启动并检查真机基础节点

打开终端 2，启动你们已有的底盘、LiDAR 和 LIO/定位程序。这里不启动 SCAN-Planner。

先确认三个输入话题确实存在：

```bash
ros2 topic list -t | sort
```

分别检查消息类型：

```bash
ros2 topic type /Odometry
ros2 topic type /LIVO2/imu_propagate
ros2 topic type /cloud_registered
```

预期结果依次是：

```text
nav_msgs/msg/Odometry
nav_msgs/msg/Odometry
sensor_msgs/msg/PointCloud2
```

检查是否持续发布。每条命令观察几秒后按 `Ctrl+C`：

```bash
ros2 topic hz /Odometry
ros2 topic hz /LIVO2/imu_propagate
ros2 topic hz /cloud_registered
```

检查一帧数据：

```bash
ros2 topic echo /Odometry --once
ros2 topic echo /LIVO2/imu_propagate --once
ros2 topic echo /cloud_registered --once --field header
```

必须确认：

- 两个 Odometry 的 `pose.pose.orientation.w/x/y/z` 不是全零；
- 机器人移动时 `/Odometry` 的位置会变化；
- 点云持续发布且时间戳在更新；
- 车体位姿、传感器位姿和点云描述的是同一个定位世界中的数据；
- 所有程序都使用系统时间，本流程不使用 `/clock` 和 `use_sim_time`。

## 5. 确认点云坐标模式

这一步不能猜错，否则会出现地图飞走、障碍重影或 RViz 什么都没有。

查看点云 `frame_id`：

```bash
ros2 topic echo /cloud_registered --once --field header
```

### 模式 A：点云仍在 LiDAR 局部坐标系

如果点云每一帧都以雷达为原点，机器人移动后静态障碍在该帧中的坐标会变化，使用：

```text
grid_map.cloud_is_world = false
```

本文默认采用这个模式，并沿用当前仓库的接法：

```text
sensor_pose  -> /LIVO2/imu_propagate
cloud        -> /cloud_registered
need_extrinsic = true
```

`need_extrinsic=true` 会继续使用源码中现有的 IMU 到 LiDAR 小外参。这只是快速验证用法。如果传感器安装关系已经明显改变，必须先修正外参，否则地图会有系统性偏移。

### 模式 B：点云坐标已经是 world/map 全局坐标

如果 LIO 已经把当前点云转换到全局坐标，启动规划器时把下面命令中的：

```bash
-p grid_map.cloud_is_world:=false \
-p grid_map.need_extrinsic:=true \
-r sensor_pose:=/LIVO2/imu_propagate \
```

替换为：

```bash
-p grid_map.cloud_is_world:=true \
-p grid_map.need_extrinsic:=false \
-r sensor_pose:=/Odometry \
```

即使点云已经在全局坐标，当前 GridMap 实现仍要求收到 `sensor_pose` 才处理点云，所以这里临时把车体 Odometry 同时接给 `sensor_pose`。这会让射线原点近似位于车体中心，足够做第一轮效果验证，但不是最终标定方案。

## 6. 第一阶段：只启动规划器，不让底盘运动

保持底盘急停按下，或者保证没有节点向真实 `/cmd_vel` 发布命令。

打开终端 3，启动规划器。下面的 `0.55` 换成第 2 节算出的外接圆半径：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash

ros2 run scan_planner scan_planner_node --ros-args \
  --params-file /home/rainple/nav_test/SCAN-Planner/src/planner/plan_manage/config/planner.yaml \
  -p use_sim_time:=false \
  -p fsm.navi_mode:=1 \
  -p grid_map.sensor_type:=lidar \
  -p grid_map.frame_id:=world \
  -p grid_map.double_cylinder_radius:=0.55 \
  -p grid_map.double_cylinder_offset:=0.0 \
  -p grid_map.cloud_is_world:=false \
  -p grid_map.need_extrinsic:=true \
  -r body_pose:=/Odometry \
  -r sensor_pose:=/LIVO2/imu_propagate \
  -r cloud:=/cloud_registered \
  -r move_base_simple/goal:=/move_base_simple/goal
```

这条命令不会启动：

- Go2 URDF；
- `robot_state_publisher`；
- Gazebo 或其他仿真器；
- 运动控制器；
- `/cmd_vel` 发布者。

另开终端检查节点参数：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash

ros2 node info /scan_planner_node
ros2 param get /scan_planner_node grid_map.double_cylinder_radius
ros2 param get /scan_planner_node grid_map.double_cylinder_offset
ros2 param get /scan_planner_node grid_map.cloud_is_world
```

预期半径是你输入的数值，偏移必须是 `0.0`。

## 7. RViz 正确配置

打开终端 4：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash
ros2 launch scan_planner rviz.launch.py use_sim_time:=false
```

当前默认 RViz 文件含有仿真时期的 Go2、`/quad_0/cloud` 和 `/quad_0/path` 显示项。真机快速验证时按下面操作：

1. `Global Options -> Fixed Frame` 设置为 `world`；
2. 取消勾选或删除 `Go2`，因为本流程没有 URDF；
3. 取消勾选 `Robot Path`，它订阅的是仿真话题 `/quad_0/path`；
4. 取消勾选 `Global Map`，本流程不启动仿真地图发布器；
5. 把 `Sensor Cloud` 的 Topic 改为 `/cloud_registered`；
6. 保留 `Occupancy`，Topic 为 `/grid_map/occupancy`；
7. 保留 `Inflated Occupancy`，Topic 为 `/grid_map/occupancy_inflate`；
8. 保留 `Sliding Map Bounds`，Topic 为 `/grid_map/sliding_map_bbox`；
9. 保留 `Goal`，Topic 为 `/goal_point`；
10. 保留或添加 `Marker`，Topic 为 `/optimal_list`；
11. 再添加一个 `Marker`，Topic 设置为 `/self_inflation`，它就是单外接圆的可视化；
12. 如需看中间规划结果，再分别添加 Marker：`/global_list`、`/init_list`、`/a_star_list`。

本阶段正常时至少应看到：

- `/grid_map/occupancy` 原始占用点；
- `/grid_map/occupancy_inflate` 圆形膨胀后的占用点；
- `/grid_map/sliding_map_bbox` 滑动地图边框；
- `/self_inflation` 位于车体位置、直径约为 `2R` 的半透明圆柱；
- 发目标后出现 `/goal_point` 和 `/optimal_list`。

不显示三维机器人模型是正常现象，不影响规划。默认配置中的 Go2 报错也不代表 SCAN 规划器报错，禁用 Go2 显示项即可。

如果原始 Sensor Cloud 因缺少 TF 无法显示，但 `/grid_map/occupancy` 能显示，可以先继续验证；SCAN 当前按数值位姿转换点云，RViz 显示原始点云才依赖其 `frame_id` 对应的 TF。

## 8. 发一个平地目标，只验证是否出轨迹

必须先等 `/self_inflation` 和占用点出现，再发目标。机器人先保持静止，这样可以暂时避开 Odometry 速度坐标语义问题。

在 RViz 顶部选择 `2D Goal Pose`，在机器人附近约 1～2 m 的空旷平地区域点击并拖出方向。

也可以不用 RViz，直接用命令发目标。把 `x`、`y` 换成 `world` 坐标系中的安全位置：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

检查是否输出轨迹：

```bash
ros2 topic echo /planning/bspline --once
```

成功标准：

- 规划器终端出现 `Triggered!`；
- 出现生成新轨迹或执行轨迹相关日志，而不是持续报 `no sensor_pose`；
- `/planning/bspline` 能收到 `pos_pts` 和 `knots` 非空的消息；
- RViz 能看到目标和规划轨迹；
- 轨迹绕开膨胀占用点，没有从障碍中穿过。

此时还没有启动控制器，所以底盘不应运动。

## 9. 第二阶段：检查速度指令，但先不接真实底盘

打开终端 5，启动闭环控制器，但把输出接到测试话题 `/scan_quick/cmd_vel_test`：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash

ros2 run scan_planner closed_loop_controller --ros-args \
  --params-file /home/rainple/nav_test/SCAN-Planner/src/planner/plan_manage/config/controllers.yaml \
  -p use_sim_time:=false \
  -r body_pose:=/Odometry \
  -r planning/bspline:=/planning/bspline \
  -r cmd_vel:=/scan_quick/cmd_vel_test
```

再次发送附近目标，然后另开终端观察：

```bash
ros2 topic hz /scan_quick/cmd_vel_test
```

以及：

```bash
ros2 topic echo /scan_quick/cmd_vel_test
```

预期约 100 Hz 输出 `geometry_msgs/msg/Twist`：

```text
linear.x   前后速度
linear.y   横向速度
angular.z  角速度
```

`linear.z`、`angular.x`、`angular.y` 应保持为零。轨迹包含 Z 变化时，当前控制器也不会给底盘发送 Z 方向速度；Z 仅用于三维路径和碰撞检测。

确认完成后，在控制器终端按 `Ctrl+C` 停止。必须先停掉测试控制器，不能同时启动两个控制器实例。

## 10. 第三阶段：真正接入舵轮底盘

开始前满足以下条件：

- 场地空旷并完成清场；
- 操作员手持可用的硬件急停；
- 底盘自身 `/cmd_vel` 超时停机已经验证；
- SCAN 地图、膨胀圆和轨迹显示正确；
- `/scan_quick/cmd_vel_test` 的方向与底盘约定一致；
- 已停止上一节的测试控制器。

先确认 `/cmd_vel` 的订阅者确实是底盘驱动：

```bash
ros2 topic info /cmd_vel -v
```

然后重新启动控制器，只把最后一条 remap 改为真实 `/cmd_vel`：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash

ros2 run scan_planner closed_loop_controller --ros-args \
  --params-file /home/rainple/nav_test/SCAN-Planner/src/planner/plan_manage/config/controllers.yaml \
  -p use_sim_time:=false \
  -r body_pose:=/Odometry \
  -r planning/bspline:=/planning/bspline \
  -r cmd_vel:=/cmd_vel
```

第一次真机目标建议：

- 距离 0.5～1.0 m；
- 平地；
- 前方无障碍；
- 不测试斜坡和窄通道；
- 随时准备按急停。

到达或终止测试后，先在控制器终端按 `Ctrl+C`，再主动发一次零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 11. 斜坡验证不能使用普通 2D Goal

当前 `navi_mode=1` 的 RViz 目标会强制使用收到第一帧 `body_pose` 时的 Z 高度。也就是说，机器人在坡底时，即使鼠标点在坡顶，普通 `2D Goal Pose` 仍会把目标 Z 设置成坡底高度。

因此：

- 平地快速验证使用 `navi_mode=1`；
- 坡底到坡顶验证必须使用 `navi_mode=3`，向 `/initial_path` 提供包含地面高度 Z 的 `nav_msgs/msg/Path`。

先停止规划器和控制器。重新启动规划器，把第 6 节命令中的：

```bash
-p fsm.navi_mode:=1 \
-r move_base_simple/goal:=/move_base_simple/goal
```

替换为：

```bash
-p fsm.navi_mode:=3 \
-r initial_path:=/initial_path
```

路径中每个点的 Z 应是该位置的**地面表面高度**。代码会自动给每个 Z 加上：

```text
grid_map.body_height
```

默认值是 `0.4 m`。如果你填写的 Z 已经是 `base_link` 高度而不是地面高度，启动规划器时必须额外覆盖：

```bash
-p grid_map.body_height:=0.0
```

下面是一个格式示例，坐标必须换成你们 PCD/LIO 世界坐标中的真实坡道点：

```bash
ros2 topic pub --once /initial_path nav_msgs/msg/Path \
  "{header: {frame_id: world}, poses: [
    {header: {frame_id: world}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: world}, pose: {position: {x: 1.5, y: 0.0, z: 0.2}, orientation: {w: 1.0}}},
    {header: {frame_id: world}, pose: {position: {x: 2.5, y: 0.0, z: 0.6}, orientation: {w: 1.0}}}
  ]}"
```

先只观察 `/planning/bspline` 和 RViz 轨迹，确认轨迹的 Z 沿坡面上升，再按第 9、10 节顺序接入控制器和底盘。

注意：真机模式的 GridMap 使用实时 `/cloud_registered` 建立局部占用，不会因为仓库目录里放了 `map.pcd` 就自动读取该文件。PCD 中有坡道不等于真机规划节点已经看到了坡道；必须确认实时点云和 `/grid_map/occupancy` 中确实出现坡面。

## 12. 常见故障快速定位

### 12.1 RViz 只有网格，Global Status Error

依次执行：

```bash
ros2 topic hz /Odometry
ros2 topic hz /LIVO2/imu_propagate
ros2 topic hz /cloud_registered
ros2 topic hz /grid_map/occupancy
```

然后检查：

- Fixed Frame 是否为 `world`；
- 是否禁用了 Go2 RobotModel；
- Occupancy Topic 是否为 `/grid_map/occupancy`；
- 规划器终端是否持续提示 `no sensor_pose received`；
- 是否错误选择了 `cloud_is_world`。

### 12.2 有点云，但没有 Occupancy

检查订阅接线：

```bash
ros2 node info /scan_planner_node
```

应能看到 `body_pose`、`sensor_pose` 和 `cloud` 已 remap 到真机话题。再检查点云是否为空、传感器四元数是否有效。

### 12.3 地图跟着机器人一起移动或出现双影

通常是点云坐标模式设置反了：

- 全局点云被再次乘传感器位姿：应改为 `cloud_is_world=true`；
- LiDAR 局部点云没有做位姿转换：应改为 `cloud_is_world=false`；
- IMU 到 LiDAR 外参不适用于当前安装位置。

### 12.4 整个机器人周围都是障碍，无法起步

LiDAR 可能看到了底盘、轮子或支架。先在上游点云中过滤机器人自身区域，再送入 SCAN。不要通过把外接圆半径调小来掩盖自点问题。

### 12.5 发目标后没有 `/planning/bspline`

检查：

```bash
ros2 topic echo /move_base_simple/goal --once
ros2 topic echo /planning/bspline --once
```

并确认：

- 第一帧 `body_pose` 已经到达后才发目标；
- 目标位于滑动地图和规划范围内；
- 目标没有落在膨胀占用点中；
- 平地使用 `navi_mode=1`，参考路径使用 `navi_mode=3`；
- 两种模式切换后已经重启规划节点。

### 12.6 有轨迹但底盘不动

检查：

```bash
ros2 topic hz /planning/bspline
ros2 topic hz /cmd_vel
ros2 topic info /cmd_vel -v
```

同时查看控制器是否收到 Odometry。若 `/cmd_vel` 有数据但底盘不动，问题在底盘驱动、急停、使能或底盘接口，不在 SCAN 规划器。

### 12.7 底盘方向相反或横移方向错误

立即急停。`Twist` 约定应为：

```text
linear.x > 0：车体向前
linear.y > 0：车体向左
angular.z > 0：逆时针旋转
```

如果底盘厂商接口约定不同，应在底盘适配层修正符号，不能继续带错误方向测试规划器。

## 13. 本轮快速验证的通过标准

- [ ] 三个真机输入话题类型和频率正确；
- [ ] 规划器不启动 Go2 URDF、仿真器或 `robot_state_publisher`；
- [ ] RViz Fixed Frame 为 `world`；
- [ ] 能看到原始占用点、膨胀占用点和滑动地图边框；
- [ ] `/self_inflation` 是位于车体中心、半径为 `R` 的单圆；
- [ ] 平地目标可以生成 `/planning/bspline`；
- [ ] 轨迹不会穿过膨胀占用点；
- [ ] 测试话题能收到 `linear.x`、`linear.y`、`angular.z`；
- [ ] 改接 `/cmd_vel` 后底盘运动方向正确；
- [ ] 停止控制器或按急停后底盘可靠停止；
- [ ] 若测斜坡，使用带 Z 的 `/initial_path`，而不是普通 2D Goal；
- [ ] 实时 Occupancy 中确实存在坡面，而不只是磁盘上的 PCD 文件中存在。

完成以上检查，只能说明“当前 SCAN-Planner 已经接通舵轮真机并能做第一轮效果验证”。它不代表执行器、安全看门狗、全向姿态碰撞检查、传感器时间同步和最终参数已经达到正式部署要求。
