# RMUC2026：同一 Gazebo 场景测试 MeshNav 与 JIE 全局规划器

最后审计：2026-08-09，ROS 2 Humble + Gazebo Fortress（Ignition Gazebo 6）。

这份文档对应当前工作区 `/home/rainple/nav_test` 的实际文件和已验证参数，覆盖：

- STEP 地图应转换成什么格式；
- 为什么斜坡和隧道曾经断开，以及当前如何保留多层拓扑；
- 两套规划器在同一个 Gazebo 场景中的完整启动方法；
- 每条关键命令的作用；
- 文件、ROS 消息、TF、QoS 和速度指令的数据流；
- “有路径但车不动”“只转一下”“QoS 不兼容”“地图纯白”等问题的排查顺序；
- 当前版本的实测结果和已知限制。

## 1. 当前结论

当前 canonical 地图不是旧的 0.1 m、35°、最高层单层地图。它已经改成从高精度视觉 STL 向下进行多重射线采样得到的 0.05 m 多层曲面图，因此能够同时表达：

- 正 Y 方向斜坡；
- 负 Y 方向的对称斜坡；
- 隧道下层地面；
- 隧道上方与斜坡相连的上层地面。

当前资产状态如下。

| 用途 | canonical 文件 | 当前内容 | SHA-256 |
|---|---|---|---|
| 原始 CAD | `field/RMUC2026_V2.0.0.stp` | STEP B-Rep，源单位 mm，约 1.25 GB | `8dfe9ebd…ffae33` |
| Gazebo 视觉 | `rmuc2026_field_visual.stl` | binary STL，2,234,919 面，111,746,034 bytes | `af971f46…00af26` |
| Gazebo 碰撞 | `rmuc2026_field_collision.stl` | binary STL，499,999 面，25,000,034 bytes | `0724d137…6704c` |
| MeshNav 全局图 | `rmuc2026_field.ply` | binary little-endian，多层三角面；180,417 顶点、348,297 面 | `2dd79f2d…ad065` |
| JIE 全局图源 | `rmuc2026_field.pcd` | PCD v0.7，binary XYZ float32；碰撞表面确定性栅格化，391,226 点 | `ba548bd9…5de44` |
| MeshNav 工作缓存 | `meshnav_demo_ws/rmuc2026_field.h5` | 首次启动按新 PLY 重建的运行缓存；不是原始输入图 | 会随重建变化 |

完整路径分别是：

```text
/home/rainple/nav_test/field/converted_rmuc2026/gazebo/models/rmuc2026_field/
/home/rainple/nav_test/field/converted_rmuc2026/gazebo/worlds/rmuc2026_field.sdf
/home/rainple/nav_test/field/converted_rmuc2026/mesh_planner/rmuc2026_field.ply
/home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd
```

一键核对哈希、文件格式、顶点/面数、SDF 引用以及 ROS 工作区副本：

```bash
cd /home/rainple/nav_test
python3 field/verify_rmuc_project.py
```

`cd` 把后续相对路径固定到项目根目录；审计脚本只读文件，不会重建或覆盖地图。结尾应显示“审计通过”。

## 2. 为什么同一个场景需要三套派生数据

Gazebo、MeshNav 和 JIE 消费的不是同一种“地图”。正确关系是：

```text
RMUC2026 STEP（CAD B-Rep，mm）
│
├─ 三角化、缩放 0.001、中心/地面对齐
│  ├─ visual.stl ───────────────> Gazebo 渲染
│  ├─ component collision.stl ──> Gazebo 刚体接触/碰撞
│  └─ collision.stl 确定性表面栅格化 ─> XYZ PCD ─> OctoMap ─> JIE A* ─> nav_msgs/Path
│
└─ visual.stl 上的多重垂直射线
   └─ 多层可通行 PLY ─> MeshMap/代价层 ─> CVP Mesh Planner ─> nav_msgs/Path
```

### 2.1 Gazebo 所需格式

Gazebo 使用 `.sdf` 描述 world/model，并从 STL 读取三角网格。当前 model 中：

- `<visual>` 指向高精度 `rmuc2026_field_visual.stl`；
- `<collision>` 指向 500k 面预算的 `rmuc2026_field_collision.stl`；
- STL 不保存 CAD 材质，所以 `model.sdf` 显式设置了蓝灰色 ambient/diffuse/specular，避免 Gazebo 默认纯白；
- 所有几何单位都已经是米，SDF 中 `<scale>` 为 `1 1 1`。

不能让 Gazebo 直接读取 `.stp`。STEP 是 CAD 拓扑/曲面格式，而 Gazebo 物理引擎需要三角碰撞面。

### 2.2 MeshNav 所需格式

MeshNav 需要一个有顶点和三角形邻接关系的可通行曲面，当前使用 binary PLY：

```text
property float x
property float y
property float z
property list uchar int vertex_indices
```

它不是点云，也不是 Gazebo 的完整实体表面。当前 PLY 只保留满足以下条件的可导航表面：

- 网格分辨率 0.05 m；
- 最大几何坡度 55°；
- RMUC 仿真车碰撞高度约 0.215 m，构图净空至少 0.225 m；
- 全局仍要求上向法线，仅在两处已审计的 CAD 断带内允许反向法线；
- 近竖直 CAD 接缝不作为“顶板”参与净空计算；
- 同一 XY 位置允许多个 Z 层；
- 相邻射线使用互为最近高度的一对一层匹配，禁止地面与顶面交叉连边；
- 最终保留最大的边连通曲面。

`meshnav_demo_ws/rmuc2026_field.h5` 是由 PLY 和 MeshLayer 参数派生出来的运行缓存，不能把 H5 当成唯一源地图。

### 2.3 JIE 所需格式

JIE 的导入链路以 binary PCD 的 XYZ 点为输入。每一个点表示“这个位置是实体表面/障碍”，不是可通行路径顶点。canonical PCD 由 Gazebo 的 detailed collision STL 确定性栅格化而来，覆盖地面、隧道顶板和墙面；不能把 MeshNav 的可通行 PLY 当成 JIE 占据图。`pcd_to_octomap_node` 将点坐标量化为 `octomap::OcTree`，再发布：

```text
topic: /octomap
type:  octomap_msgs/msg/Octomap
```

`Octomap` 消息包含 `frame_id`、最小体素分辨率和序列化的 `int8[] data`。JIE 规划节点把它恢复为占据体素、地面支撑与预阻塞代价，再输出 `nav_msgs/msg/Path`。

RMUC 隧道专用 profile 必须使用 0.05 m OctoMap、0 m 预降采样、0.28 m 水平包络和 0.225 m 物理高度。这里的 `robot_height=0.225` 从支撑/地面量到车顶；它不是从候选自由体素中心再向上增加 0.225 m。0.225 m 来自当前 Ceres 仿真代理碰撞几何（0.215 m）加 10 mm 余量，不是真机实测值。

## 3. 斜坡和隧道为什么曾经断开

旧流程以及第一版多层流程共有五个结构性问题：

1. 隧道上方有上层结构时，“只取最高层”必然删除隧道地面；
2. 两处主坡约 17°，但 CAD 接缝处各有约 55 mm 宽、约 52.5°的短过渡带。0.1 m 采样会跨过接缝，35°或 50°阈值都会把连接三角形删掉；
3. STEP 导出的 STL 非水密且三角面绕序不一致，同一块地面会在 `normal_z=+1/-1` 间切换；只接受 `+Z` 会把地面切断；
4. 某些近竖直三角面恰好与向下射线重合，第一版把它误当作顶板，得到错误的“净空不足”。
5. 旧 JIE PCD 是在 2,234,919 面完整 visual mesh 上全局随机抽 303,144 个点。总点数看似很多，但小面积隧道地面没有逐体素覆盖保证；量化到 0.05 m 后只有 227,927 个 unique occupied voxels，正、负 Y 真隧道都无法形成路径，0.1 m 同样失败。与此同时，屋顶面积较大而被充分采样，于是形成“下层断开、屋顶还在”的现象。这不是单纯调机器人尺寸能修复的数据源缺洞。

截图所示隧道的地面到顶板下表面约 0.246 m。原教程车碰撞包络约为 `0.53 × 0.55 × 0.305 m`，物理上不能通过。RMUC 专用配置保持车宽和轴距，只把轮半径改为 0.10 m，并下移 2-D/3-D 雷达；新碰撞包络约为 `0.48 × 0.55 × 0.215 m`，构图用 0.225 m，保留约 21 mm 几何余量。其他教程 world 仍使用原车型。

当前 Mesh 流程使用 0.05 m 多层采样、55°几何阈值和 0.225 m 净空，并从高精度 `visual.stl` 生成 PLY。当前结果有 16,008 个“同 XY、不同 Z”的最终顶点位置，实际最大三角面坡度约 54.870°，最终边连通域为 1、非流形边为 0。55°只用于保留短 CAD 过渡带，不应解释成真机可持续爬 55°坡。

当前 JIE 流程改为从物理使用的 `collision.stl` 逐三角形细分，再量化到 0.05 m 表面格。输出点放在目标体素中心 `(key+0.5)×0.05`，避免负坐标和 float32 边界值落入相邻体素。PCD 保留顶板是正确的占据语义；规划器应在顶板下方有支撑的自由体素中走低层，而不是删除顶板。约 0.246 m 的净空只有在 0.05 m profile 下能保真；0.1 m 离散化会把地面和顶板压到不足以容纳 0.225 m 包络的层数，因此本地图明确不支持用 0.1 m 做隧道验收。

以下文件只是旧流程或诊断产物，不能装成正式 MeshNav 地图：

- `rmuc2026_field_slope_candidates.ply`；
- `mesh_planner_terrain_preview.png`；
- `cache_backups/rmuc2026_field_legacy_single_layer.h5`。

## 4. 最短可用流程

### 4.1 所有终端先隔离 ROS 域

每个终端都执行同样的环境命令：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
```

- `ROS_DOMAIN_ID=42` 把本次实验与默认域 0 中残留的 Go2、其他 RViz 或旧节点隔离；数字可换成 0–232 内未使用的值，但所有本次实验终端必须一致。
- `ROS_LOCALHOST_ONLY=1` 限制 DDS 只发现本机进程，避免局域网内其他 ROS 2 主机干扰；需要跨机时不要设置它。
- `source /opt/ros/humble/setup.bash` 加载 ROS 2 Humble 的命令、消息类型和基础包。

切换过 Domain ID 后若 `ros2 node list` 仍像是旧缓存，可执行：

```bash
ros2 daemon stop
ros2 daemon start
```

这只重启 ROS 2 CLI 的发现守护进程，不会终止任何机器人节点。

### 4.2 编译一次

Mesh/Gazebo 工作区：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

JIE 工作区：

```bash
cd /home/rainple/nav_test/jie_3d_nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

- `colcon build` 按包依赖顺序编译整个工作区；
- `--symlink-install` 让 Python、launch、YAML 和资源文件在 install 空间中尽量指向 source，修改后不容易继续读到旧副本；
- 修改 C++、CMake 或新增可执行程序后仍必须重新 build。

### 4.3 启动共同的 Gazebo + MeshNav 框架

终端 A：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
cd /home/rainple/nav_test/meshnav_demo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mesh_navigation_tutorials mesh_navigation_tutorials_launch.py \
  world_name:=rmuc2026_field \
  map_name:=rmuc2026_field \
  localization:=ground_truth \
  obstacle_segmentation:=none \
  start_gazebo_gui:=True \
  start_rviz:=True
```

参数含义：

| 参数 | 含义 | 为什么这样设置 |
|---|---|---|
| `world_name` | Gazebo world 名称 | 选择 `rmuc2026_field.sdf` |
| `map_name` | MeshNav PLY/H5 基名 | 选择 canonical `rmuc2026_field.ply` |
| `localization` | `map→odom` 的来源 | 仿真用 Gazebo ground truth，先排除定位误差 |
| `obstacle_segmentation` | 动态点云障碍分割 | 当前先测静态全局图，因此用 `none` |
| `start_gazebo_gui` | 是否打开 Gazebo GUI | `True` 便于观察车体和场景 |
| `start_rviz` | 是否打开 MeshNav RViz | `True` 便于下发 Mesh Goal 和看代价图 |

RMUC world 还会自动采用：

- 出生点 `(-11.9, -4.4, 0.15)`；
- `laser3d_collision=False`，即高激光雷达桅杆不参与刚体碰撞；
- 轮半径 `0.10 m`，2-D/3-D 雷达相对 `base_link` 的安装高度分别为 `0.060/0.050 m`；
- RMUC 碰撞包络约 `0.48 × 0.55 × 0.215 m`，其中宽度和轴距不靠缩小来“挤过”隧道；
- `slope_aware_drive=True`，使用支持 `linear.x`、`linear.y` 且保留车体俯仰/横滚接触动力学的 RMUC 底盘插件；
- 其他教程 world 仍默认 `(0, 0, 0.1)`、启用桅杆碰撞，并继续使用原来的轮式 `DiffDrive` 插件。

需要换出生点时可显式追加，例如：

```bash
spawn_x:=-11.9 spawn_y:=-4.4 spawn_z:=0.15
```

### 4.4 检查仿真和 TF 是否就绪

另开终端 B，source 同一域和 Mesh 工作区：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash

ros2 topic list -t
ros2 topic hz /clock
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 action list -t
```

- `topic list -t` 同时显示话题名和消息类型；
- `topic hz` 检查仿真时钟和里程计是否持续更新；
- `tf2_echo map base_link` 验证控制器能得到机器人在地图中的位姿；
- `action list -t` 应能看到 `/move_base_flex/get_path`、`/move_base_flex/exe_path` 和 `/move_base_flex/move_base`。

刚启动的前几秒，ground-truth 节点可能报告暂时找不到 `odom→base_footprint`。EKF 建立 TF 后警告应停止；如果一直存在，再按第 10 节排查。

### 4.5 测 MeshNav 规划

RViz 中应使用工具栏的 **Mesh Goal**，不要用普通的 **2D Goal Pose**。Mesh Goal 会把点击点投影到三角网格；右侧 `MbfGoalActions` 面板负责调用 MBF action。

多层位置还要在 Mesh Goal 工具属性中选择射线交点层：`Intersection Layer=0` 是相机射线遇到的最近表面，从顶视图点击隧道时通常是屋顶；隧道地面在屋顶下方时改成 `Intersection Layer=1`，再点击同一 XY 位置。`Switch Bottom/Top` 只翻转所选表面的法线/姿态方向，不能在屋顶和地面之间换层。canonical PLY 保留可达屋顶本身是正确的多层语义。

也可以用 CLI 做不依赖 GUI 的回归测试。隧道短路径：

```bash
ros2 action send_goal -f /move_base_flex/get_path mbf_msgs/action/GetPath \
"{use_start_pose: true,
  start_pose: {header: {frame_id: map}, pose: {position: {x: -1.45, y: 5.95, z: 0.0039}, orientation: {w: 1.0}}},
  target_pose: {header: {frame_id: map}, pose: {position: {x: -0.40, y: 5.95, z: 0.0039}, orientation: {w: 1.0}}},
  tolerance: 0.2, planner: mesh_planner, concurrency_slot: 0}"
```

这两个点位于截图对应的正 Y 隧道两侧。验收时不能只看 action `outcome: 0`，还应检查返回 `path.poses` 在隧道段保持 `z<0.10 m`；否则规划器可能吸附到 `z≈0.424 m` 的顶面，形成“看似成功”的假回归。负 Y 对称隧道可用 `(0.40,-5.95,0.003) → (1.45,-5.95,0.003)`。

斜坡连接回归：

```bash
ros2 action send_goal -f /move_base_flex/get_path mbf_msgs/action/GetPath \
"{use_start_pose: true,
  start_pose: {header: {frame_id: map}, pose: {position: {x: 0.5166667, y: 6.8833334, z: 0.4438785}, orientation: {w: 1.0}}},
  target_pose: {header: {frame_id: map}, pose: {position: {x: 1.2833333, y: 6.9166667, z: 0.2085318}, orientation: {w: 1.0}}},
  tolerance: 0.2, planner: mesh_planner, concurrency_slot: 0}"
```

命令中：

- `send_goal -f` 发送 action goal 并显示 feedback；
- `use_start_pose: true` 强制使用给定起点，而不是当前车位；
- `frame_id: map` 表示坐标属于全局地图坐标系；
- `tolerance: 0.2` 允许目标投影在 0.2 m 邻域内；
- `planner: mesh_planner` 明确选择 CVP Mesh Planner；
- `concurrency_slot: 0` 使用默认规划槽。

离线拓扑审计已证明正、负 Y 隧道和两处反向法线断带均在下层连通，且不经过顶面。每次重建 H5 后仍应重新运行本节 action 测试，不能沿用旧缓存的结果。

若要让 MeshNav 从机器人当前位置完整规划并执行，可在 RViz 使用 Mesh Goal，或发送 `/move_base_flex/move_base`：

```bash
ros2 action send_goal -f /move_base_flex/move_base mbf_msgs/action/MoveBase \
"{target_pose: {header: {frame_id: map}, pose: {position: {x: -10.2, y: -4.4, z: 0.1}, orientation: {w: 1.0}}},
  controller: mesh_controller, planner: mesh_planner, recovery_behaviors: []}"
```

目标必须位于 canonical PLY 上；远离曲面的任意 Z 值会导致投影或规划失败。

### 4.6 在同一仿真中运行 JIE

不要给 MeshNav 留着一个正在执行的 `move_base/exe_path` goal。MBF 节点可以继续空闲运行，但两套控制器不能同时发送非零速度。

终端 C：启动 PCD→OctoMap→JIE planner。这里必须同时 source Mesh 和 JIE 工作区，且启用仿真时钟。GUI 预览/导入使用 Open3D，新机器先安装运行依赖：

```bash
sudo apt update
sudo apt install python3-open3d
```

然后启动 RMUC 专用 profile：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash

ros2 launch jie_octomap import_pcd_map.launch.py \
  use_sim_time:=true \
  rmuc2026_profile:=true
```

- `rmuc2026_profile:=true` 一次性选择 `resolution=0.05`、`robot_radius_xy=0.28`、`robot_height=0.225`，并把 GUI 预降采样锁为 0；
- GUI 的“推荐转换参数”在此 profile 下也不会把分辨率自动推粗到 0.075/0.1 m；超过 0.05 m 的转换会被阻止；
- `robot_height` 是从实际支撑体素对应的地面到车顶的物理高度，JIE 内部不会因为候选自由体素比支撑高一格而额外多算一个 resolution；
- `min_points_per_voxel=1`、`min_cluster_voxels=1` 保留 deterministic PCD 中的全部已审计表面单元；
- `use_sim_time:=true` 让地图、路径和 RViz 使用 `/clock`，避免系统时间与仿真时间混用。

没有图形桌面或暂时未安装 Open3D 时，可让 launch 只启动后端：

```bash
ros2 launch jie_octomap import_pcd_map.launch.py \
  use_sim_time:=true \
  rmuc2026_profile:=true \
  start_import_gui:=false
```

`start_import_gui:=false` 只跳过 PyQt/Open3D 窗口，PCD 转换器、planner、marker 和地图包管理器仍会启动；随后用下面的 `/pcd_file_cmd` 命令导入。

然后把 PCD 的绝对路径作为一个 `std_msgs/msg/String` 事件发给导入节点：

```bash
ros2 topic pub --once --qos-durability volatile \
  /pcd_file_cmd std_msgs/msg/String \
"{data: '/home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd'}"
```

`--once` 只发布一次；消息内容不是点云本身，而是要读取的本地文件路径。`/pcd_file_cmd` 是一次性 volatile 事件；节点读取 binary PCD 后发布 transient-local `/octomap`，因此晚启动的 planner/RViz 也能收到最后一张地图。全图 0.05 m 派生地面支撑/预阻塞需要明显时间，等 `jie_path_node` 打印地图和派生层就绪后再发起终点。

终端 D：启动 JIE 路径跟踪器和速度类型适配器：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash

ros2 launch octo_planner meshnav_ceres_controller.launch.py \
  require_start_command:=true
```

这个 launch 启动两个节点：

1. `d1_controller`：订阅 `/planned_path`，输出 `geometry_msgs/msg/Twist` 到 `/cmd_vel_jie`；
2. `jie_twist_stamper`：添加仿真时间戳和 `base_link` frame，把它转换成 `geometry_msgs/msg/TwistStamped` 后发到统一 `/cmd_vel`。

之所以需要第二个节点，是因为当前 ros_gz_bridge 的 `/cmd_vel` 桥和 MeshNav 都使用 `TwistStamped`。不能在同一个 `parameter_bridge` 中让 `Twist` 与 `TwistStamped` 同时向同一个 Gazebo `/model/robot/cmd_vel` 发布。

终端 E：启动点击选择器和 JIE RViz：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash

ros2 run jie_octomap rviz_click_selector_node --ros-args \
  -p use_sim_time:=true
```

再开一个终端：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash

rviz2 -d /home/rainple/nav_test/jie_3d_nav/jie_octomap/rviz/octomap_test.rviz \
  --ros-args -p use_sim_time:=true
```

在这个 RViz 中使用 **Publish Point**：第一次点击发布 `/start_point`，第二次点击发布 `/goal_point`；选择器会显示绿色起点和红色终点。JIE 收到两点后自动发布 `/planned_path`。

路径出现后，由于 `require_start_command:=true`，还需要显式放行：

```bash
ros2 topic pub --once --qos-durability volatile \
  /start_navigation std_msgs/msg/Bool "{data: true}"
```

立即停止并发布零速度：

```bash
ros2 topic pub --once --qos-durability volatile \
  /stop_navigation std_msgs/msg/Bool "{data: true}"
```

这个“先规划、再确认执行”的门闩可以防止刚点击终点时机器人意外移动。

## 5. 完整数据流与消息类型

### 5.1 Gazebo 和 TF

```text
Gazebo model/robot
├─ ignition.msgs.Odometry ─bridge─> /odom  nav_msgs/msg/Odometry
├─ ignition.msgs.IMU      ─bridge─> /imu/data  sensor_msgs/msg/Imu
├─ LaserScan              ─bridge─> /scan  sensor_msgs/msg/LaserScan
├─ PointCloudPacked       ─bridge─> /cloud sensor_msgs/msg/PointCloud2
├─ Pose_V                 ─bridge─> /tf_gt tf2_msgs/msg/TFMessage
└<─ ignition.msgs.Twist   <─bridge─ /cmd_vel geometry_msgs/msg/TwistStamped

/odom + /imu/data ─> robot_localization EKF ─> odom → base_footprint
/tf_gt ─> ground_truth_localization ─────────> map → odom
robot_state_publisher ───────────────────────> base_footprint/base_link → sensors
```

JIE 和 Mesh controller 最终都依赖 `map→…→base_link` 完整 TF 链。只有路径而没有这条 TF，控制器无法计算车体相对路径的误差。

### 5.2 MeshNav

```text
rmuc2026_field.ply
  → MeshMap
  → height_diff/border/roughness/static_inflation 等代价层
  → CVPMeshPlanner
  → mbf_msgs/action/GetPath Result.path（nav_msgs/msg/Path）
  → MeshController
  → /cmd_vel（geometry_msgs/msg/TwistStamped）
  → ros_gz_bridge
  → Gazebo ignition.msgs.Twist
```

当前 RMUC 参数中：`height_diff.threshold=0.2`、静态与动态内切半径 `0.28 m`、静态外膨胀 `0.5 m`、机器人高度 `0.225 m`。`0.28 m` 来自 0.55 m 总宽的一半再加 5 mm；它能通过截图隧道约 0.797 m 的净宽，但 MeshNav 的标量半径不是带航向的矩形 footprint，窄处必须低速做 Gazebo 碰撞回归。

### 5.3 JIE

```text
/pcd_file_cmd  std_msgs/msg/String
  → 读取 binary XYZ float32 PCD（391,226 个障碍表面体素中心）
  → 按 0.05 m coordToKey 去重/占据
  → octomap::OcTree
  → /octomap  octomap_msgs/msg/Octomap
  → JIE 占据集合
  → 地面支撑深度 + preblocked/traversable/risk 派生层
  → 0.28 m XY × 0.225 m support-to-top 碰撞检查
  → 26 邻域 A*
  → /planned_path  nav_msgs/msg/Path
  → d1_controller
  → /cmd_vel_jie  geometry_msgs/msg/Twist
  → jie_twist_stamper
  → /cmd_vel  geometry_msgs/msg/TwistStamped
  → ros_gz_bridge
  → Gazebo ignition.msgs.Twist
```

文件阶段的类型变化是 `STL triangles → float32 XYZ surface centres → OctoMap occupied keys → supported free-cell graph → Path poses`。PCD 里没有 free/unknown 概率，也没有三角邻接；地面支撑和自由空间由 planner 根据占据体素重新推导。PCD converter 默认只在成功加载时发布一次 transient-local `/octomap`，不会每秒重新序列化整棵树；只有显式设置 `republish_period_s>0` 才启用兼容性周期重发。

`Twist` 只有线速度和角速度；`TwistStamped` 还增加 `Header.stamp` 和 `Header.frame_id`。适配器只增加 Header，不改变 `linear.x`、`linear.y`、`angular.z` 数值。

## 6. QoS 设计和警告解释

主要 JIE 话题约定如下。

| 话题 | 类型 | Reliability | Durability | 原因 |
|---|---|---|---|---|
| `/pcd_file_cmd` | `std_msgs/msg/String` | reliable | volatile | 一次性命令，不应让未来节点自动重放旧文件命令 |
| `/octomap` | `octomap_msgs/msg/Octomap` | reliable | transient_local | 地图状态，晚加入节点需要最后一份 |
| `/start_point`、`/goal_point` | `geometry_msgs/msg/PointStamped` | reliable | transient_local | 规划状态，需要保留最后选点 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | reliable | **volatile subscriber** | RViz 2D Goal 是 volatile 事件；当前 JIE 已修复 durability 不兼容 |
| `/planned_path` | `nav_msgs/msg/Path` | reliable | transient_local | 控制器晚启动时仍应收到路径 |
| `/start_navigation`、`/stop_navigation` | `std_msgs/msg/Bool` | reliable | volatile | 操作事件，不重放旧启动命令 |
| `/cmd_vel_jie` | `geometry_msgs/msg/Twist` | 默认 reliable/volatile | volatile | 实时控制流 |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 默认 reliable/volatile | volatile | 统一进入 Gazebo 的实时控制流 |

DDS 的核心兼容规则是“publisher 提供的服务必须不低于 subscriber 请求”：

- transient-local publisher 可以给 volatile subscriber；
- volatile publisher 不能满足要求 transient-local 的 subscriber；
- reliable publisher 可以满足 reliable/best-effort subscriber；
- best-effort publisher 不能满足要求 reliable 的 subscriber。

查看任一话题所有端点和 QoS：

```bash
ros2 topic info -v /goal_pose
ros2 topic info -v /planned_path
ros2 topic info -v /cmd_vel
```

当前 `/goal_pose` 的 JIE subscriber 应显示 `RELIABLE + VOLATILE`。如果仍出现旧的 `DURABILITY_QOS_POLICY` 警告，通常是同一 ROS 域里还有旧二进制或旧节点；先确认节点路径、重新 build，并使用新的 `ROS_DOMAIN_ID`。

传感器桥的实际 QoS 由 ros_gz_bridge 和传感器端点共同决定，不要凭经验硬改；以 `ros2 topic info -v /scan`、`/cloud` 的现场输出为准。

## 7. 从 STEP 完整重建地图

日常使用已有 canonical 文件时不要重跑这一节。只有 STEP、转换参数或机器人几何能力变化时才重建。

### 7.1 转换环境

当前 `field/.step_convert_venv/bin/python` 已验证能导入 OCP、NumPy、SciPy、trimesh、rtree 和 Pillow。直接调用绝对解释器最稳定，不依赖 `activate`。

新机器首次创建环境时：

```bash
cd /home/rainple/nav_test
python3 -m venv field/.step_convert_venv
field/.step_convert_venv/bin/python -m pip install \
  -r field/requirements-conversion.txt
```

若 Ubuntu 提示 `ensurepip is not available`，需先安装与 Python 版本对应的 `python3-venv`/`python3.10-venv` 系统包。`requirements-conversion.txt` 已固定本次转换所用 Python 库版本。

### 7.2 只检查 STEP

```bash
field/.step_convert_venv/bin/python field/step_to_nav_maps.py inspect \
  field/RMUC2026_V2.0.0.stp \
  --report field/RMUC2026_step_report.json
```

`inspect` 读取 STEP 根、面和边界并写 JSON，不输出 ROS 地图。当前脚本对这张已知 RMUC 文件按 mm 处理；它不是通用的 STEP 单位自动识别器，换 CAD 时应先在 CAD 软件中确认单位。

### 7.3 STEP 三角化、缩放和诊断采样

```bash
field/.step_convert_venv/bin/python field/step_to_nav_maps.py convert \
  field/RMUC2026_V2.0.0.stp \
  --output field/converted_rmuc2026 \
  --model-name rmuc2026_field \
  --linear-deflection-mm 50 \
  --angular-deflection-deg 20 \
  --max-slope-deg 35 \
  --sample-spacing-m 0.08 \
  --min-points 10000 \
  --max-points 3000000 \
  --ground-bin-m 0.02 \
  --origin center-ground
```

关键参数：

- `linear/angular-deflection` 控制 OpenCascade 把曲面离散为三角面时的弦高和角度精度；
- `sample-spacing-m` 只控制 `*_raw_visual_surface_sample.pcd` 诊断点云的近似点间距；随机采样不能保证隧道拓扑，不是 canonical JIE 输入；
- `origin center-ground` 把 XY 中心移到原点，并把统计地面移到 Z≈0；
- 此处 `max-slope-deg=35` **只用于输出诊断性的 slope-candidates PLY**，不是最终 MeshNav 的 55°阈值；
- 这一步输出高精度 visual STL、初始 collision STL、`*_raw_visual_surface_sample.pcd` 和诊断 PLY，但不会输出最终 tunnel-aware PLY，也不会覆盖 canonical `jie_nav/rmuc2026_field.pcd`。

本次坐标变换后边界约为：

```text
x: -14.8760 .. 14.8760 m
y:  -8.0016 ..  8.0016 m
z:  -0.1413 ..  3.6600 m
```

### 7.4 构造 Gazebo 碰撞网格

推荐用封装后的后处理命令：

```bash
field/.step_convert_venv/bin/python field/postprocess_nav_maps.py \
  field/converted_rmuc2026 \
  --model-name rmuc2026_field \
  --collision-method component-budget \
  --collision-faces 500000 \
  --collision-mandatory-max-z 0.35 \
  --collision-optional-max-z 1.0
```

该算法不是全局三角形 decimation。它合并 1 µm 内的重合顶点，完整保留所有 `min_z<=0.35 m` 的顶点连通 CAD 组件，再按“投影 XY 面积/面数”加入 `min_z<=1.0 m` 的完整组件，直到不超过 500k 面。这样不会把小斜坡或隧道层切碎。

`postprocess_nav_maps.py` 默认只重建碰撞网格和更新 metadata，不会覆盖 canonical PLY。只有显式增加 `--allow-legacy-single-layer` 才会运行已废弃的最高层算法；RMUC2026 禁止使用该开关。

需要单独调用底层碰撞工具时，等价命令是：

```bash
field/.step_convert_venv/bin/python field/build_component_collision_mesh.py \
  field/converted_rmuc2026/gazebo/models/rmuc2026_field/meshes/rmuc2026_field_visual.stl \
  field/converted_rmuc2026/gazebo/models/rmuc2026_field/meshes/rmuc2026_field_collision.stl \
  --face-budget 500000 \
  --mandatory-max-z 0.35 \
  --optional-max-z 1.0 \
  --report field/converted_rmuc2026/gazebo/collision_candidates/rmuc2026_field_collision_component_budget_500k.json \
  --force
```

`--force` 允许覆盖已存在的输出，因此运行前必须确认路径确实是本项目的 collision STL。

### 7.5 构造 JIE 占据表面 PCD

碰撞网格后处理完成后，才生成 canonical JIE PCD：

```bash
field/.step_convert_venv/bin/python field/build_jie_surface_pcd.py \
  field/converted_rmuc2026/gazebo/models/rmuc2026_field/meshes/rmuc2026_field_collision.stl \
  field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd \
  --surface-voxel-m 0.05 \
  --min-z -0.08 \
  --max-z 0.90 \
  --chunk-faces 500 \
  --edge-factor 1.1 \
  --report field/converted_rmuc2026/jie_nav/rmuc2026_field.surface.json \
  --force
```

参数和设计含义：

- 输入必须是 detailed canonical collision STL，因为它与 Gazebo 实际碰撞几何一致并完整保留所有低层组件；不要输入可通行 PLY；
- `surface-voxel-m=0.05` 是源表面格距；每个三角形被细分到最大边约 `0.05/1.1=0.04545 m` 后再并集去重；密集覆盖是否连通仍由下面的真实走廊回归判定，不能仅凭点间距作拓扑断言；
- `z=[-0.08,0.90] m` 保留地板、斜坡、墙和当前车体相关顶板，排除 `z≈-0.141 m` 的错误底壳及与地面车无关的高处装饰；
- 先用 `round(surface/0.05)` 得到目标格，再输出体素中心 `(key+0.5)×0.05`。如果直接写格边界，负坐标和 float32 序列化可能使 OctoMap `floor` 到相邻 key；
- `chunk-faces` 只限制峰值内存，不影响确定性结果；相同输入与参数应得到相同字节哈希；
- `--force` 会覆盖 canonical PCD，只能在已确认输出路径后使用。

当前生成结果为 391,226 点、4,694,886 bytes，SHA-256 为 `ba548bd9fde09f65278f9f3117e9e8cd93079eb2025769d9cd169dcf8455de44`。在当前机器上两次生成约 11.7–11.9 s，复跑峰值 RSS 约 1.71 GB；不同机器运行时间会变化，哈希不应变化。

随后运行两条真隧道的离线回归：

```bash
field/.step_convert_venv/bin/python field/verify_jie_tunnel_pcd.py \
  field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd \
  --resolution 0.05 \
  --robot-radius-xy 0.28 \
  --robot-height 0.225 \
  --report field/converted_rmuc2026/jie_nav/rmuc2026_field.tunnel.json
```

验收必须同时满足 `all_connected_on_lower_layer=true`，且正、负 Y 两条路径的 `path_z_range_m` 都是 `[0.075,0.075]`。当前离线回归耗时约 2.05 s。用 `--resolution 0.1` 时两条隧道均不通过，这是 0.246 m 净空和 0.225 m 物理包络在粗体素中的预期离散结果，不应通过缩小机器人来伪造通过。

### 7.6 构造多层 MeshNav PLY

```bash
field/.step_convert_venv/bin/python field/build_multilevel_nav_mesh.py \
  field/converted_rmuc2026/gazebo/models/rmuc2026_field/meshes/rmuc2026_field_visual.stl \
  field/converted_rmuc2026/mesh_planner/rmuc2026_field.ply \
  --rmuc2026-profile \
  --grid-m 0.05 \
  --max-slope-deg 55 \
  --hit-merge-m 0.012 \
  --ray-batch 5000 \
  --min-component-area-m2 0 \
  --report field/converted_rmuc2026/mesh_planner/rmuc2026_field.multilevel.json
```

必须把高精度 `*_visual.stl` 作为 `source_mesh`。不要使用 slope-candidates、旧单层 PLY，也不建议从已降面 collision STL 再生成导航图。

- `grid-m=0.05` 捕获 55 mm 的短坡面过渡；
- `max-slope-deg=55` 只用于保留约 52.5°的短 CAD 接缝，不代表真机持续坡度能力；
- `rmuc2026-profile` 选择 0.225 m 净空，并启用两处经过坐标审计的反向法线区域；
- `hit-merge-m=0.012` 合并同一表面的近重复射线交点，但保留真正分层；
- `ray-batch` 只影响内存/速度，不改变几何结果；
- `min-component-area-m2=0` 表示只保留最大的边连通域。

脚本会写独立 report，并自动把 PLY 路径、参数、面数和 SHA-256 回填到 `conversion_metadata.json`。RMUC profile 还会强制检查四条下层通道；任意一条只能走顶面或不连通时，生成命令会失败而不是静默交付坏图。只有生成文件哈希与旧文件相同时才继承人工验证字段，避免换参数后误用旧验证结论。

### 7.7 同步到 ROS 工作区并重建

```bash
cd /home/rainple/nav_test

cp -a field/converted_rmuc2026/gazebo/models/rmuc2026_field/. \
  meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials_sim/models/rmuc2026_field/

cp field/converted_rmuc2026/gazebo/worlds/rmuc2026_field.sdf \
  meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials_sim/worlds/rmuc2026_field.sdf

cp field/converted_rmuc2026/mesh_planner/rmuc2026_field.ply \
  meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials/maps/rmuc2026_field.ply

cd meshnav_demo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  mesh_navigation_tutorials_sim mesh_navigation_tutorials
```

JIE 当前通过 `/pcd_file_cmd` 使用 field 目录中的绝对 PCD 路径，所以无需把 PCD 复制进 ROS package。

同步后回到项目根目录运行：

```bash
python3 field/verify_rmuc_project.py
```

## 8. H5 缓存失效规则

只要发生下列任一变化，就不能继续使用旧 `rmuc2026_field.h5`：

- canonical PLY 的顶点、三角形或坐标变化；
- `height_diff`、`roughness`、`border`、inflation 等 MeshLayer 参数变化；
- MeshMap/HDF5 格式实现变化。

用可恢复方式移走旧缓存：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
mv rmuc2026_field.h5 \
  rmuc2026_field.h5.backup.$(date +%Y%m%d-%H%M%S)
```

然后从该目录重新启动 MeshNav。首次加载会从 PLY 重算。初始化完成后保存新缓存：

```bash
ros2 service call /move_base_flex/save_map std_srvs/srv/Trigger "{}"
```

如果 PLY 已修复但 RViz 仍显示旧断口，旧 H5 是最优先检查项之一。

## 9. 公平比较两个全局规划器

建议每次只执行一套控制器，并在每轮前重启 Gazebo，使出生位姿和物理状态一致。

固定以下变量：

- 同一 `world_name=rmuc2026_field`；
- 同一出生点 `(-11.9,-4.4,0.15)`；
- 同一目标的 `map` 坐标；
- 同一机器人 collision 配置；
- 同一目标容差和超时；
- 静态地图实验都使用 `obstacle_segmentation=none`。

每轮记录：

1. 是否成功；
2. 规划耗时；
3. `nav_msgs/Path` 的欧氏累计长度；
4. 执行到达时间；
5. 最终位置/航向误差；
6. 最大坡度、是否进入隧道、是否发生碰撞或卡住；
7. 失败码和对应日志。

查看当前真实起点而不是凭 Gazebo 画面估计：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

MeshNav 和 JIE 内部地图表达不同：一个沿三角曲面规划，一个在 OctoMap 体素/支撑单元中规划。因此应比较相同物理起终点和结果，不应要求两条路径逐点重合。

## 10. 全面故障排查

### 10.1 Gazebo 地图纯白或看不清

先确认实际加载的 model：

```bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 pkg prefix mesh_navigation_tutorials_sim
rg -n "visual.stl|ambient|diffuse|collision.stl" \
  /home/rainple/nav_test/meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials_sim/models/rmuc2026_field/model.sdf
```

当前 visual 必须指向 `_visual.stl`，collision 必须指向 `_collision.stl`，并存在显式 material。仍为纯白时，先用 Ctrl-C 干净结束旧 Gazebo，再确认没有遗留实例：

```bash
pgrep -af 'ign gazebo|gz sim|ruby.*ignition'
```

不要在旧 Gazebo 仍运行时反复启动同名 world；GUI 可能仍展示旧进程中的模型。

### 10.2 两处斜坡或隧道仍断开

按顺序检查：

```bash
python3 /home/rainple/nav_test/field/verify_rmuc_project.py
sha256sum /home/rainple/nav_test/meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials/maps/rmuc2026_field.ply
sha256sum /home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd
```

正确 PLY 哈希必须以 `2dd79f2d` 开头，正确 PCD 哈希必须以 `ba548bd9` 开头。

- Mesh 断开：按第 8 节移走旧 H5 并重建；不要把 `rmuc2026_field_slope_candidates.ply` 改名覆盖 canonical PLY。顶视点击隧道时还应按 4.5 节将 Mesh Goal 的 `Intersection Layer` 改成 1，`Switch Bottom/Top` 不能换层。
- JIE 断开：确认启动命令有 `rmuc2026_profile:=true`、实际 `/octomap` resolution 是 0.05 m、GUI 预降采样为 0；不要使用旧 303,144 点随机 PCD，也不要把 Mesh PLY 当 PCD。重新运行 7.5 节离线双隧道回归。
- 看到屋顶不等于地图错误：屋顶是障碍表面，Mesh 中还可能是独立可达上层。真正的验收是低层地板有支撑、路径 `z<0.15 m` 且穿过隧道。

### 10.3 JIE 看不到路径

```bash
ros2 topic info -v /octomap
ros2 topic echo /octomap --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic info -v /start_point
ros2 topic info -v /goal_point
ros2 topic echo /planned_path --once \
  --qos-reliability reliable --qos-durability transient_local
```

判断顺序：

1. `/octomap` 是否存在且 `data` 非空；
2. 起点和终点的 `header.frame_id` 是否为 `map`；
3. 两点是否落在有地面支撑的可行单元附近；
4. `jie_path_node` 是否报告 start/goal snapping 或 A* 失败；
5. `/planned_path.poses` 是否非空；
6. RViz Fixed Frame 是否为 `map`，Path display topic 是否为 `/planned_path`。

若启动时出现 `ModuleNotFoundError: No module named 'open3d'`，安装 `sudo apt install python3-open3d` 后重启；只做无界面回归时可先用 `start_import_gui:=false`，再手动发布 `/pcd_file_cmd`。如果全图导入后立即点击仍没有路径，先看 `jie_path_node` 是否仍在构造 0.05 m 派生层，完成后再发送起终点。

### 10.4 已有路径，但车完全不动

JIE 先检查是否发送了执行门闩：

```bash
ros2 topic pub --once --qos-durability volatile \
  /start_navigation std_msgs/msg/Bool "{data: true}"
```

再沿速度链逐段看：

```bash
ros2 topic echo /cmd_vel_jie
ros2 topic echo /cmd_vel
ros2 topic info -v /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

期望类型是：

```text
/cmd_vel_jie  geometry_msgs/msg/Twist
/cmd_vel      geometry_msgs/msg/TwistStamped
```

`/cmd_vel_jie` 有非零数据而 `/cmd_vel` 没有，检查 `jie_twist_stamper`；`/cmd_vel` 有非零数据但 Gazebo 不动，检查 bridge 和 Gazebo topic；两者都没有，检查 controller 是否收到路径、start 命令和 TF。

### 10.5 车先转一个角度，然后停住

短暂旋转通常说明控制器至少收到了路径并能发布角速度。继续检查：

- `/planned_path` 是否很短、只有一个点或目标被吸附到错误层；
- `map→base_link` TF 是否在旋转后停止更新；
- `/cmd_vel_jie.linear.x/linear.y` 是否一直为 0；
- 是否有另一个正在执行的 MeshNav action 向 `/cmd_vel` 写零速度；
- 车体是否被高桅杆或碰撞网格卡住；RMUC 默认应为 `laser3d_collision=False`；
- Gazebo real-time factor 是否显著下降。

当前 Ceres 使用支持 `linear.y` 的 slope-aware holonomic drive；JIE 控制器也已启用 lateral motion，EKF 接受 `vy`。旧的只取 `linear.x` 链路会表现为“朝向调好了但无法横向跟踪”。

### 10.6 QoS 警告很多

对警告中给出的具体 topic 执行：

```bash
ros2 topic info -v /出现警告的话题名
```

重点比较每个 publisher/subscriber 的 Reliability 和 Durability，而不是盲目把所有话题都改成同一种 QoS。地图/路径适合 transient-local；点击、启动和速度命令适合 volatile。

旧截图中的 `/goal_pose` durability 冲突已经在 `jie_path_node` 中修成 reliable + volatile subscriber。若仍看到完全相同警告，通常运行的是旧 install 二进制或另一个 ROS 域中的旧节点。

### 10.7 RViz 重启后什么都不显示

检查四件事：

1. RViz 与节点的 `ROS_DOMAIN_ID` 相同；
2. RViz 启动时加了 `use_sim_time:=true`；
3. Fixed Frame 为 `map`；
4. transient-local 地图端点仍存在。

然后执行：

```bash
ros2 topic list -t | rg 'octomap|planned_path|marker|clock'
ros2 topic info -v /octomap
```

### 10.8 生成了错误机器人或 robot_description 竞态

当前 Ceres 的 robot_state_publisher 使用独立话题：

```text
/meshnav/robot_description
```

`ros_gz_sim/create` 也只从该话题生成 `robot`，不会再与全局 `/robot_description` 上残留的 Go2 transient-local publisher 抢第一条 URDF。仍建议用独立 ROS Domain，避免其他节点发布相同 TF frame。

### 10.9 MeshNav 启动时出现 H5 inflation 写入问题

当前空 inflation 数据集的 H5 写入已修复。如果使用的是旧 build，重新编译相关包：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  mesh_map mesh_layers mbf_mesh_nav mesh_navigation_tutorials
```

随后按第 8 节失效旧 H5。

### 10.10 退出时 MBF 报 `exit code -11`

Humble + CycloneDDS 环境中曾复现：所有节点已经打印 clean shutdown 后，DDS `gc` 线程与进程级 RMW 动态库卸载发生竞态，最终在 `_dl_close` 后 SIGSEGV。

当前 `mbf_mesh_nav` 使用显式 `rclcpp::Context`、同 context 的 TF listener、显式销毁 executor/plugins，并在所有自有资源完成清理后用 `std::_Exit(EXIT_SUCCESS)` 跳过有竞态的进程级静态析构。带 live Gazebo 的三轮压力退出均为 0。

这是针对当前 Humble/CycloneDDS 的兼容 workaround，不是通用 ROS 2 设计：它会跳过静态析构、`atexit` 和未刷新的 C/C++ 缓冲。因此若升级 ROS/RMW，应重新验证并优先移除这一 workaround。

Gazebo 在 Ctrl-C 后由 launch 报 `-2` 通常只是 SIGINT 的正常退出表示；MBF 自身应显示 cleanly finished，而不是 `-11`。

## 11. 当前已完成的验证

本轮终审实际完成了：

- canonical visual/collision/PLY 在 `field`、ROS source 和 symlink install 中哈希一致；
- PLY：180,417 顶点、348,297 三角形、1 个边连通域、0 个非流形边；
- 正、负 Y 两条低隧道均在 `z<0.10 m` 的下层连通，顶面仍作为独立可行驶层保留；
- 两处反向法线断带的下层连通，且 `z≈-0.141 m` 的场地底壳没有进入导航图；
- RMUC xacro 生成的碰撞包络约 `0.48 × 0.55 × 0.215 m`，旧教程 profile 仍保持原尺寸；
- 碰撞网格：保留约 95.54% 表面积、98.04% XY 投影面积，低层 1 mm 内层召回约 99.452%；
- Mesh 多层生成器 4 个单元回归通过，正式 report 内 4 条区域化低层连通检查全部通过；
- Mesh H5 已按 canonical PLY 重建：2026-08-09 13:48，3,379,016 bytes；
- Mesh 在线 `GetPath` 正 Y 真隧道：`outcome=0`，长度 1.05016 m，路径 z 范围 `[-0.0044,0.004] m`；负 Y 对称隧道同样 `outcome=0`，长度 1.05007 m；
- Gazebo 正 Y 隧道入口从 `(-1.45,5.95)` 以 0.2 m/s 直驱到 `x=-0.32665`，确认当前仿真碰撞代理可物理穿过；完整 Mesh `MoveBase` 为 `outcome=0`，最终位置 `(-0.57735,5.93788,z=0.00438)`；
- 新 JIE PCD 由 collision STL 在约 11.7 s 内确定性生成 391,226 个表面单元；canonical SHA-256 为 `ba548bd9…5de44`，重复生成字节哈希相同；
- JIE 离线真隧道验收使用 0.05 m、水平半径 0.28 m、物理高度 0.225 m：正 Y 22 个路径单元/42 次 A* 迭代，负 Y 19 个路径单元/19 次迭代，两者路径 z 均固定为 0.075 m；0.1 m 对两条均按预期拒绝；
- JIE 真实 ROS/OctoMap 链在同一 RMUC profile 下也通过：正 Y 为 28 次 A* 迭代/22 poses，端点 `(-1.475,5.925)→(-0.425,5.975)`；负 Y 为 19 次迭代/19 poses，端点 `(1.375,-5.975)→(0.475,-5.975)`；两条路径的 `z_min=z_max=0.075000003 m`，没有吸附到屋顶；
- JIE 表面 PCD 新增 2 个确定性/负坐标体素中心单测；碰撞包络 9/9 gtest 通过，覆盖 0.246 m 可通边界、0.20 m 拒绝、support depth、无支撑同层障碍和负 Z 预阻塞；三包增量 build 通过；
- JIE `Path → Twist → TwistStamped → Gazebo` 的完整控制链此前已验证机器人可移动并响应 stop；本轮 canonical PCD 的在线双隧道结果以本节随后记录的独立 ROS 回归为准；
- Mesh 工作区当前结果：91 tests，0 errors，0 failures（2 skipped）；
- MBF 带 live simulation 的三轮启动/退出均返回 0。

编译时来自旧 `jsoncpp`/Ignition CMake 配置的 deprecation warning 不等同于编译失败；以 `Summary: ... finished` 和返回码为准。

## 12. 已知边界与交付注意事项

- 顶层 tutorial launch 会在 `map_name=rmuc2026_field` 时自动注入 holonomic、0.225 m 机器人高度和 0.28 m 内切半径；其他地图继续使用原教程默认值。若绕过顶层 launch、直接启动 `mbf_mesh_navigation_server_launch.py`，则必须自行传入这些 RMUC profile 参数。
- JIE 的通用 PCD 导入默认值仍保持旧地图兼容；RMUC2026 必须显式使用 `rmuc2026_profile:=true`。此 profile 的 0.05 m 分辨率和 0 降采样是 0.246 m 真隧道的组成条件，不能换成 0.1 m 后再通过缩小碰撞高度“修通”。
- 当前 0.215 m 车体碰撞高度、0.225 m 规划高度和 0.28 m 水平半径来自 CAD/xacro 与 Gazebo 仿真代理。它们说明这一个仿真模型的能力，不等于真机测量；拿到真机包络、悬挂压缩量和动态坡度能力后必须重新生成并验收两套地图。
- 保存/重载 JIE 地图包时，`robot_radius_xy`、`robot_height` 和其他派生层参数会随 metadata 传递；加载默认只重新发布 authoritative OctoMap，由 planner 按当前 profile 重算派生层，避免旧缓存 preblocked/traversable/risk 与新参数的同名双 publisher 竞态。
- 仿真 launch 同样按 `world_name` 隔离底盘：RMUC 自动使用 0.10 m 轮半径、低安装雷达和 `slope_aware_drive`；parking/tray/floor_is_lava 保留原尺寸与 `DiffDrive`，避免 RMUC 适配改变旧教程世界。
- `rmuc2026_field_visual.stl` 为 111,746,034 bytes，超过 GitHub 普通单文件 100 MiB 限制。若要推送远程仓库，应使用 Git LFS、Release 外部资产或另行分发，不能普通 `git push`。
- 当前修改已写入文件并完成 build/test，但没有代替用户创建 Git commit。关闭 VSCode 不会丢失工作树内容；正式交付前应由仓库维护者审阅并提交。
- `octo_planner` 和本次修改过的 `import_pcd_map.launch.py` 已通过各自检查；但 `jie_octomap` 整个历史包的全量 ament lint 仍有 110 个既有样式失败，集中在未参与本流程的旧 GUI/launch 文件（长行、尾随空格以及 3 个 C++ 格式项）。它们不影响当前 build 或已验证运行链路，但若要求整个仓库 CI 全绿，需要另开一次纯格式清理并审阅大范围 diff。
- 两套规划器对“可通行”的模型不同。机器人尺寸、最低净空或最大可爬坡度变化后，必须同时重新评估 collision STL、Mesh PLY、JIE 体素参数和 controller 参数，不能只改一个阈值。
