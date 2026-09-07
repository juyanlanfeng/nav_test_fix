# 两套导航的明确启动步骤

适用目录：`/home/rainple/nav_test`；ROS 2 Humble、Gazebo Fortress。
本文使用当前已经编译安装的版本。日常启动不需要重新转换 STEP、不需要重新编译。

**先选择一条路线完整执行，不要同时让两个控制器驾驶机器人。**

| 路线 | 需要保持运行的终端 | 操作窗口 | 如何开始运动 |
|---|---|---|---|
| A：MeshNav | A1 一个 | Gazebo + Mesh RViz | 发送 Mesh Goal，规划成功后自动执行 |
| B：JIE | B1、B2、B3 三个 | Gazebo + JIE 地图导入 GUI | 导入并转换地图、设置起终点，再用 B4 发送开始命令 |

## 0. 每个新终端都先执行这段环境命令

以下环境设置只影响执行它的终端，**新开终端必须再执行一次**：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

前三条加载 ROS 和两套工作区中的程序。`ROS_DOMAIN_ID` 统一通信域；`ROS_LOCALHOST_ONLY` 限定本机通信；后两条统一 DDS 实现并扩大自动参与者编号范围，避免节点较多时 DDS 初始化失败。已经运行的节点不会随新终端的 `export` 改变环境。

如果上一轮仿真还在运行，请先按第 3 节停止它，再按下面的一条路线启动。不要重复启动 Gazebo。

## A. 启动 MeshNav：一个终端

### A1：启动仿真、定位、规划器和可视化

新开终端，执行第 0 节，然后执行：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
ros2 launch mesh_navigation_tutorials mesh_navigation_tutorials_launch.py \
  world_name:=rmuc2026_field \
  map_name:=rmuc2026_field \
  localization:=ground_truth \
  obstacle_segmentation:=none \
  start_gazebo_gui:=True \
  start_rviz:=True
```

保持终端运行，等待 Gazebo 和 RViz 出现。

| 参数/命令 | 含义 |
|---|---|
| `cd .../meshnav_demo_ws` | 固定工作目录；Mesh 的相对路径 H5 缓存也落在这里 |
| `world_name` | 加载 RMUC2026 Gazebo 场景，包括物理碰撞模型 |
| `map_name` | 加载供 MeshNav 规划的 RMUC2026 PLY 网格，不是 PCD |
| `localization:=ground_truth` | 用仿真真值定位，配合里程计提供机器人到地图的 TF |
| `obstacle_segmentation:=none` | 不启动在线障碍物分割；先测试静态全局图 |
| `start_gazebo_gui` / `start_rviz` | 分别打开仿真窗口和 Mesh 导航窗口，注意这里使用 `True`/`False` |

### A2：在 RViz 发送目标

1. 确认 Gazebo 没有暂停，RViz 中能看到网格和机器人。
2. 查看 `MbfGoalActions` 面板，`Get Path` 和 `Execute Path` 的 Server 应为 `ready`。
3. 如名称为空，选择 Planner Name 为 `mesh_planner`，Controller Name 为 `mesh_controller`。
4. 使用工具栏 **Mesh Goal** 在机器人附近的平坦可通行区域设置目标。不是 `2D Goal Pose`，也不需要用 `2D Pose Estimate` 设置起点；起点来自当前机器人 TF。
5. 当前面板实现会在 Get Path 成功后**自动发送 Execute Path**，机器人应开始运动。不要寻找额外的“开始”按钮。

第一次先测附近短路径，再测隧道或斜坡。隧道上下表面重叠时，Mesh Goal 的 `Intersection Layer=0` 选择射线最近交点，可能是顶面；选择下层可尝试 `1`，并检查目标箭头实际落在哪一层。`Switch Bottom/Top` 不是切换隧道楼层。

停止当前运动：点击面板的 **Stop Controller Action**；规划仍进行时也点击 **Stop Planner Action**。

## B. 启动 JIE：三个常驻终端 + 一个发命令终端

### B1：只打开 Gazebo 仿真窗口

新开终端，执行第 0 节，然后执行：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
ros2 launch mesh_navigation_tutorials mesh_navigation_tutorials_launch.py \
  world_name:=rmuc2026_field \
  map_name:=rmuc2026_field \
  localization:=ground_truth \
  obstacle_segmentation:=none \
  start_gazebo_gui:=True \
  start_rviz:=False
```

保持运行。这里复用同一仿真和定位，所以与 A1 只有 RViz 开关不同。
**这个公共 launch 仍会启动 Mesh MBF 后端，但本路线不向它发送目标；驾驶由 B3 的 JIE 控制器负责。**

### B2：启动 JIE 地图窗口和规划节点

新开终端，执行第 0 节，然后执行：

```bash
ros2 launch jie_octomap import_pcd_map.launch.py \
  use_sim_time:=true \
  rmuc2026_profile:=true \
  start_import_gui:=true
```

保持运行。这条命令启动 JIE 后端及可视化导入窗口；`use_sim_time` 使用 Gazebo 时钟，`rmuc2026_profile` 加载本场地参数。

在窗口中依次操作：

1. 点击选择 PCD，打开下面这个文件：

   ```text
   /home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd
   ```

2. 确认 OctoMap 分辨率为 **0.040 m**，下采样为 **0**。不要使用旧的 0.1 m 地图设置。
3. 点击“转换为 Octomap”。**左边出现 PCD 预览不代表后端已经拿到 OctoMap。**
4. 等待右侧显示栅格，以及终端完成可通行性/代价图重建（例如 `Derived traversability rebuilt`、`Preblocked costmap rebuilt`）。当前地图是 631,564 点；本机此前实测后端重建约 22 秒，不保证每次耗时相同。
5. 点击“起始点”，在**右侧栅格窗口**选择机器人当前所在位置附近的地面；再点击“目标点”选择附近可通行地面，等待路径出现。

注意：GUI 选起点不会把 Gazebo 机器人移动过去。只做规划可以任意选起点；需要实车仿真跟踪时，起点必须与当前机器人位置一致。默认出生位置的 XY 约为 `(-11.9, -4.4)`，机器人移动后不能继续把这里当作当前位置。选点高度应落在对应地面，不是车体中心或隧道顶面。

### B3：启动路径跟踪控制器

新开终端，执行第 0 节，然后执行：

```bash
ros2 launch octo_planner meshnav_ceres_controller.launch.py \
  require_start_command:=true
```

保持运行。这会启动路径跟踪和速度格式转换。`require_start_command:=true` 表示有路径也先不动，等待明确的开始命令。

### B4：确认路径后开始运动 / 停止

新开终端，执行第 0 节。确认 B1 未暂停、B2 已经出现有效路径、B3 正常运行后，执行：

```bash
ros2 topic pub --once --qos-durability volatile /start_navigation std_msgs/msg/Bool "{data: true}"
```

这条命令只发送一次布尔开始信号，然后退出；B1/B2/B3 不能关闭。

需要停止时，在同一终端执行：

```bash
ros2 topic pub --once --qos-durability volatile /stop_navigation std_msgs/msg/Bool "{data: true}"
```

如果需要重新导入地图，先停止运动；转换可能清除起终点及旧路径，需要重新选点。

## 3. 结束运行和切换导航

最清晰的切换方法是结束当前路线，再启动另一条路线：

- Mesh → JIE：先在 RViz 点击 Stop Controller Action / Stop Planner Action，再在 A1 按 `Ctrl+C`，等退出后执行 B1～B4。
- JIE → Mesh：先用 B4 发送停止，再在 B3、B2、B1 分别按 `Ctrl+C`，等退出后执行 A1～A2。

不要只关闭窗口而保留另一套控制器后台运行；也不要用 `killall` 清理其他项目。
若明确要保留同一个 Gazebo 进程做对比，可复用 A1/B1，不再启动第二份仿真；但必须先停止旧路线的运动。JIE 切回 Mesh 时还要结束 B3，避免速度转换节点继续与 Mesh 争用 `/cmd_vel`。

## 4. 启动失败时，只先查这几项

以下诊断另开终端并执行第 0 节，不要中断常驻 launch。

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_link
```

前两条分别读取一条仿真时钟和里程计。第三条持续输出机器人在地图中的位置，可用来核对 JIE 起点；看完按 `Ctrl+C` 结束**诊断命令**。若前两条一直没有输出，也按 `Ctrl+C`，检查仿真是否启动、是否暂停、各终端环境是否一致。

| 现象 | 优先检查 |
|---|---|
| Mesh `Could not get the current robot pose`，RViz 不显示车 | `map → base_link` TF 是否存在；仿真、定位及环境是否一致。重新点击目标不能补齐 TF |
| Mesh 有图但 Server 不 ready | 等后端初始化；检查 A1 日志，不要重复启动第二份 A1 |
| JIE 只有左侧点云，没有右侧地图 | 是否点击转换、是否完成重建 |
| JIE 有路径但不动 | B3 是否运行、B4 是否发送、Gazebo 是否暂停、起点是否在当前车附近 |
| 车只转或速度异常 | 是否有另一套控制器仍在执行；不要同时发送 Mesh Goal 和 JIE 开始命令 |
| DDS 初始化失败或节点互相看不到 | 所有终端是否执行完整第 0 节；旧节点需要按相同环境重新启动 |
| GUI 没出现 | 查看对应终端报错，在本机桌面终端启动；不要用 sudo 启动 GUI，也不要随意升级 NumPy |

## 5. 两条路线的数据流

```text
共同仿真：Gazebo → /clock、里程计 → 定位/TF → 机器人在 map 中的位置

Mesh：PLY 网格 → mesh_planner → GetPath 返回 nav_msgs/Path
      → 面板自动调用 ExePath → mesh_controller → 仿真速度接口 → Gazebo

JIE：PCD → 导入转换 → octomap_msgs/Octomap → jie_path_node
     起终点 geometry_msgs/PointStamped → /planned_path (nav_msgs/Path)
     → JIE 控制器 → /cmd_vel_jie (geometry_msgs/Twist)
     → 时间戳转换 → /cmd_vel (geometry_msgs/TwistStamped) → Gazebo
```

Gazebo 场景、Mesh PLY 和 JIE PCD 是不同用途的文件，不能互相替代。
详细地图生成、历史排查记录见 [完整说明](CONVERSION_AND_USAGE.md)；本轮 PCD 检查结果见 [地图审计](MAP_AUDIT_20260906.md)。日常启动以本文为入口。
