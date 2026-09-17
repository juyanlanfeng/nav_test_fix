# 北极熊车辆仿真接入现有 MeshNav / JIE：详细修改方案

日期：2026-09-08。

**本文是待实施方案，不是已完成接入的说明。此次只新增本文，不修改代码、依赖、模型或运行中的仿真。文中的新增包、参数、节点与启动命令必须在实施后才能使用。**

## 1. 目标与范围

用户目标是测试导航，而不是验证真实舵轮、电机或轮胎动力学。因此接受北极熊项目的简化全向驱动：由速度闭环直接对车身施力，不要求重新实现舵轮关节控制。

本次接入应实现：

- 保留当前 RMUC2026 Gazebo 场地，以及 MeshNav 的网格全局地图、JIE 的 PCD/OctoMap 数据。
- 复用北极熊机器人外观、碰撞体、云台、3D 雷达及模型已有的其他传感器。
- 复用 `MecanumDrive2` 简化全向驱动，将两个导航器的速度指令接到它。
- 保留 Gazebo 与 RViz 可视化；提供稳定的点云、位姿、里程计和 TF 接口。
- 默认先使用仿真真实位姿定位，排除 SLAM 误差；不引入整套北极熊 Nav2、Point-LIO、重定位或比赛裁判系统。
- 旧车辆保留，可切换。当前闭环验收实例不热替换、不同时启动第二个控制源。

不把模型能够爬坡理解为实车能够爬坡，但仍要保证导航测试基本可信：场地坐标正确、碰撞不穿模、点云与位姿一致、速度不串话题。

## 2. 已核实的上游代码及版本

导航仓库本身不包含全部车辆资源，实际依赖链为：

```text
pb2025_sentry_nav
  └─ 配套 rmu_gazebo_simulator
       ├─ pb2025_robot_description：车辆 SDF/XMacro 与传感器组合
       ├─ rmoss_gz_resources：车体、雷达、相机等模型资源
       ├─ rmoss_gazebo：MecanumDrive2 等插件
       └─ sdformat_tools：SDF/XMacro 展开和描述支持
```

来源与固定版本：

| 项目 | 本次核实依据 |
| --- | --- |
| [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | README 指向配套仿真仓库 |
| [rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator) | Ubuntu 22.04 / ROS Humble / Gazebo Fortress，与当前环境匹配 |
| [依赖清单](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/blob/main/dependencies.repos) | 实施时应另行记录该清单自身的 commit，避免 main 后续变化 |
| `pb2025_robot_description` | `a0541dddcbfe376f369a6345b532a10923ad9149` |
| `rmoss_gazebo` | `7443ff06c345752d0e23172c479d99b27207e2c7` |
| `rmoss_gz_resources` | `b5c759f08844dfda19c79aa870866ace8d4c7b3a` |
| `sdformat_tools` | `47c2d1a37372281e5627d444f7537efc0bba11df` |

上游 [simulation_robot.sdf.xmacro](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description/blob/a0541dddcbfe376f369a6345b532a10923ad9149/resource/xmacro/simulation_robot.sdf.xmacro) 组合了：

- `rm25_example_robot` 车体；
- 前置 RPLIDAR A2、倾斜安装的 Livox 模型、云台工业相机；
- 关节状态发布、灯光、底盘、云台及射击插件。

其中 Livox 宏调用使用 `update_rate=20`、`samples=1875`。上游导航 README 明确说明仿真扫描并非真实 MID360 扫描模式，额外点云工具用于补充 Point-LIO 所需字段。这里不接 Point-LIO 时，不必为导航点云强制引入其时间字段处理链。

底盘 [MecanumDrive2.cc](https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo/blob/7443ff06c345752d0e23172c479d99b27207e2c7/rmoss_gz_plugins/plugins/mecanum_drive2/MecanumDrive2.cc) 的实质是速度 PID → `AddWorldWrench()`，符合本次接受的简化执行模型。不要因名称含 Mecanum 就把它描述为实车舵轮动力学。

## 3. 最小接入架构

新增独立适配包，建议命名 `pb_vehicle_adapter`，放在 `meshnav_demo_ws/src/`。上游包保留原文件和许可证，本地话题、TF、启动裁剪集中在适配包内。

```text
                    当前 RMUC2026 world
                             │
              北极熊车辆 SDF + 原全向插件
                   │                   ▲
       点云/扫描/关节/真实位姿       ignition.msgs.Twist
                   ▼                   │
             ros_gz_bridge       单一速度适配/超时停车
                   │                   ▲
          /cloud /scan /tf_gt           │
                   │            /cmd_vel (TwistStamped)
           TF 与里程计适配         ▲                 ▲
                   │          MeshNav          JIE 类型适配
                   ▼                                ▲
              RViz / 两个导航器               /cmd_vel_jie (Twist)
```

约束：一次只允许一个导航控制源驱动车辆。可同时显示两种规划结果，但不能让两个跟踪器同时发布最终速度。

## 4. 文件修改清单

以下路径相对 `/home/rainple/nav_test`。

| 文件/目录 | 修改要求 |
| --- | --- |
| `meshnav_demo_ws/src/pb_vehicle_adapter/`（新增） | 集中管理模型展开、生成、桥接、TF、速度适配、RViz 和验证 |
| 新包 `package.xml`、`CMakeLists.txt` 或 Python 安装配置 | 声明实际使用的 ROS/Gazebo 依赖，安装 launch/config/models/rviz；不得安装无关比赛系统 |
| 新包 `models/pb_navigation_robot.sdf.xmacro` | 包装上游模型，保留外观、3D/2D 雷达、相机与关节；去掉射击功能；不改原仓库文件 |
| 新包 `config/pb_ros_gz_bridge.yaml` | 独立的 PB 话题桥，不覆盖旧桥 |
| 新包 `src` 或 Python 模块 | 实现速度超时停车、真实位姿到导航 TF/odom 的适配 |
| 新包 `launch/pb_vehicle_sim.launch.py` | 启动当前场地、PB 车辆、描述发布、传感器桥与位姿适配；不自动启动任何导航器 |
| 新包 `launch/pb_meshnav.launch.py` | 复用上述基础仿真，再启动当前 MeshNav server 与 RViz |
| 新包 `launch/pb_jie.launch.py` | 复用基础仿真，再组合当前 JIE 地图/规划/跟踪器与 RViz；不启动 MeshNav 跟踪器 |
| 新包 `rviz/pb_navigation.rviz` | RobotModel、PointCloud2、LaserScan、TF、路径；默认可见且固定帧 map |
| `.../mesh_navigation_tutorials_sim/launch/base_simulation_launch.py` | 若统一旧入口，新增 `robot_model` 分支；旧分支原样保留，PB 分支不能继续生成 ceres URDF |
| `.../mesh_navigation_tutorials_sim/launch/simulation_launch.py` | 透传车辆选择参数，PB 分支接入自己的定位/TF，不重复启动旧定位源 |
| `.../mesh_navigation_tutorials/launch/mesh_navigation_tutorials_launch.py` | 透传车辆选择；车辆相关高度、膨胀与坡道约束按 profile 区分 |
| `doc/START_NAVIGATION.md` | 接入验收通过后再更新正式入口；此前本文命令只作为接口设计 |

省略号分别代表当前 `meshnav_demo_ws/src/mesh_navigation_tutorials/` 下对应包路径。可以先只做新包的独立 launch，验证完成后再接旧入口，避免在运行中的配置上来回修改。

## 5. 依赖与磁盘空间控制

不要直接执行整套导航仓库的 `git clone --recursive`、全量 `vcs import`、Docker 镜像下载或无差别 `rosdep install`。本任务不需要比赛场地资源、先验点云、实车雷达驱动、SLAM 和裁判网页。

实施前只读检查：

```bash
df -h /home/rainple/nav_test
du -sh /home/rainple/nav_test/meshnav_demo_ws
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_sim
```

`df` 看可用空间；`du` 看当前工作空间占用；两个 `prefix` 确认已有桥和 Gazebo 启动支持，不重复安装。

获取上游时建议使用隔离的源码缓存，不直接把所有仓库放入现有 `src`。使用浅层/部分克隆并禁用自动 LFS 下载，再按固定 commit 检查需要的文件；部分克隆并不保证模型文件已下载齐全。

最小资源闭包须从 XMacro 的 `model://` 引用递归得到，至少覆盖 `rm25_example_robot`、`rplidar_a2`、`mid360`、`industrial_camera` 及其材质、网格和子模型。检查 LFS 指针、缺失纹理以及资源搜索路径。保留 license 和来源清单，不手工猜测只拷一个 STL 就足够。

插件构建优先使用原 `rmoss_gz_plugins` 包。该包的实际构建依赖以固定版本 `package.xml/CMakeLists.txt` 为准；仅在依赖确实缺失时安装。若需要裁剪为独立底盘插件库，应明确记录源码来源、许可证和修改，不伪装成未经修改的上游版本。

## 6. 车辆生成与外观显示

上游使用 **SDF XMacro，不是 ROS xacro URDF**。不得直接用当前 `xacro ceres.urdf.xacro` 的处理方式替换文件名，也不能把 SDF 字符串直接交给仅接受 URDF 的 RobotModel 链路。

实施顺序：

1. 用固定版本的上游展开工具生成完整 SDF，检查所有 `model://` URI 能解析。
2. 生成一个固定名称 `robot` 的实例；保留 PB 模型内部 `chassis`、雷达和云台 link 名称，避免大范围重命名。
3. 用 `ros_gz_sim create -file` 生成车辆，不再从旧 `/meshnav/robot_description` 生成 ceres。
4. 通过上游描述工具提供的转换/发布能力生成 RViz 所需 URDF；若存在不能转换的 SDF 插件，物理模型仍使用原 SDF，显示用 URDF 只保留一致的 link、joint、visual 和真实外参。
5. 发布新的 `/meshnav/robot_description`，RViz RobotModel 显式订阅它。关节状态只使用 PB 车辆对应话题。
6. 保留灯光及云台可视化；导航测试默认云台固定，不启动自主旋转和跟随云台控制。相机保留，允许用开关暂停图像输出以降低 GPU 压力。

生成命令接口示例（展开后的路径需由适配包提供）：

```bash
ros2 run ros_gz_sim create -file /绝对路径/pb_navigation_robot.sdf \
  -name robot -x X -y Y -z Z
```

这是格式示例，`X/Y/Z` 必须替换为通过检查的出生位置，不能直接复制执行。PB 整车尺寸不同，旧 `spawn_z` 和坡脚坐标不能未经检查照搬。模型自身 pose 和生成偏移只设置一套明确约定，避免重复加高。

不要把新车压缩到旧代理车尺寸来“保证过隧道”。先测量新车展开后的外廓和碰撞高度，再决定导航测试车辆 profile；确实放不下的通道不应承诺可通行。

## 7. 速度和传感器接口：具体映射

### 7.1 速度

固定模型名为 `robot` 时，已核实原 `MecanumDrive2` 订阅 **`/robot/cmd_vel`**，不是旧模型的 `/model/robot/cmd_vel`。

建议保留现有 ROS 导航接口 `/cmd_vel`，其类型为 `geometry_msgs/msg/TwistStamped`。新增单一适配节点：

```text
MeshNav /cmd_vel : TwistStamped ─────────────────────┐
                                                  ├─ 速度适配 → /pb/cmd_vel_safe : Twist
JIE /cmd_vel_jie : Twist → 现有 jie_twist_stamper ───┘
                                                        │
                                           ROS_TO_GZ /robot/cmd_vel
```

箭头汇合表示两种可选运行模式，不允许同时发布。适配器不重新做导航，只完成：

- 检查有限数值、消息新鲜度、所选控制源；
- 确认命令在底盘坐标系，必要时通过 TF 旋转速度；`TwistStamped.header.frame_id` 不会被桥自动用于坐标转换；
- 去掉时间头后发布普通 `Twist`；
- 约 0.5 s 无新指令时持续发零，取消导航和切换控制源时先停车。优先使用稳态时钟监控接收超时，仿真暂停/恢复另做测试；
- 与原 PB `rmoss_gz_base` 底盘控制节点二选一，本方案直接桥插件，不同时启动它的跟随云台控制。

原因：已核实的 `MecanumDrive2` 会保留最后速度目标，没有看到输入超时停车逻辑。必须避免停止导航发布后车继续走。适配进程自身崩溃时无法发零，若需覆盖该故障，应在插件加最小超时保护或由独立看门狗处理；这不属于复杂动力学改造。

**拟新增桥配置片段：**

```yaml
- ros_topic_name: /pb/cmd_vel_safe
  gz_topic_name: /robot/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: ignition.msgs.Twist
  direction: ROS_TO_GZ
```

PB 分支必须禁用旧车的速度桥、`SlopeAwareHolonomicDrive` 和 DiffDrive。不能在 PB 车上同时保留两种车体驱动。

### 7.2 传感器

| 功能 | 建议 ROS 话题 | ROS 类型 | 接入要求 |
| --- | --- | --- | --- |
| 仿真时间 | `/clock` | `rosgraph_msgs/msg/Clock` | 单一来源 |
| 3D 雷达 | `/cloud` | `sensor_msgs/msg/PointCloud2` | `PointCloudPacked` 单向桥接；保留测量坐标和 frame_id |
| 2D 雷达 | `/scan` | `sensor_msgs/msg/LaserScan` | 保留模型已有 RPLIDAR；不是 3D 点云伪造的替代品 |
| 相机 | `/camera/image_raw`、`/camera/camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` | 光学坐标、时间、内参需一致 |
| IMU（若完整展开模型实际包含） | `/imu/data` | `sensor_msgs/msg/Imu` | 先确认实际传感器；没有就明确新增，不冒称上游已有 |
| 关节状态 | `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo `Model` 消息转换，检查名称和频率 |
| 原始 PB 里程计 | `/pb/odom_raw` | `nav_msgs/msg/Odometry` | 已核实 Gazebo 来源 `/robot/odometry`；仅用于对照 |
| 导航里程计 | `/odom` | `nav_msgs/msg/Odometry` | 与导航 TF 适配器一致 |
| Gazebo 真实位姿 | `/tf_gt` 或 `/pb/poses_raw` | `tf2_msgs/msg/TFMessage` | 由新增 PosePublisher/世界姿态话题获取，不直接桥到 `/tf` |

雷达/相机的最终 Gazebo 话题取决于展开后的嵌套模型和 sensor scope，本次未完整展开全部资源，**不能声称其名称与旧车一致**。实施时先列举并读取真实类型：

```bash
ign topic -l
ign topic -i -t /实际的Gazebo话题
ros2 topic list -t
ros2 topic info /cloud --verbose
ros2 topic echo /cloud --once --field header --qos-reliability best_effort
```

`ign topic` 检查 Gazebo 侧；`ros2 topic` 检查桥后的 ROS 侧。示例中的“实际话题”必须替换。点云桥只转消息格式，不改变点的坐标，也不会自动修正 frame_id。

传感器使用 volatile、兼容 best-effort 的订阅；静态描述、`/tf_static` 保留 transient-local 语义。RViz 点云设置为 Best Effort/Volatile，避免再次出现 QoS 不匹配。具体桥 QoS 配置能力以本机安装版本为准，用 `topic info --verbose` 验证，不凭 YAML 外观判断生效。

现有 world 要保留 Physics、SceneBroadcaster、Sensors 等必要系统，GPU 雷达需要可用渲染后端。只看到车不代表雷达运行正常；检查点云频率和点数。

## 8. TF 与里程计：推荐真值导航模式

推荐第一阶段采用一个适配器统一发布：

```text
map ──静态单位变换──> odom ──Gazebo 三维真值──> base_footprint
                                                    └─固定外参─> chassis
                                                                   ├─> 雷达
                                                                   └─> 云台/相机
```

这里 `base_footprint` 定义为随底盘倾斜的导航参考点，保留 roll/pitch/z；不是额外水平投影。若希望采用水平 footprint，需增加另一 link 并同步修改导航参考帧，不能混用两种定义。

实施细节：

1. 从真实 Gazebo `chassis` 位姿获取 `T_world_chassis`，不是未经验证地把 model 原点当作底盘原点。
2. 根据展开模型确定常量 `T_base_chassis`。计算 `T_world_base = T_world_chassis × inverse(T_base_chassis)`。
3. 当前场地坐标与全局地图对齐后设置 `map→odom` 为单位变换，发布 `odom→base_footprint = T_world_base`。如果地图有额外转换，必须应用已记录的变换，不再假定单位关系。
4. `/odom` 的 pose 与这个 TF 完全一致；twist 表示同一参考点、同一约定下的速度，转换参考点时考虑角速度引起的线速度差异。
5. 保留 `base_link` 兼容帧：如果它与 `chassis` 同原点同朝向，可作为其固定子帧；否则使用真实外参。不能为了名字一致到处加单位 TF。
6. PB 分支禁用原 `ground_truth_localization` 的 map→odom 发布，以及旧 `/tf_odom` 转发，避免重复 TF 所有者。
7. 机器人描述发布器负责内部 link TF；真值适配器只负责导航根部。确保 TF 没有环、没有同一个 child 被两个父节点发布。

为什么不直接使用 PB `/robot/odometry`：已检查上游代码，它仅写入 x/y 和 yaw，未写入完整 z/roll/pitch；frame 名称带 `robot/` 前缀。直接重命名 frame_id 并不能补回三维信息。导航测试继续采用真值定位即可，不必引入 Point-LIO 来解决它。

也可延续原 `ground_truth_localization.cpp` 结构，但必须先正确生成 odom→base 并转换 PB 真值的参考点；不能同时使用本文的“map→odom 单位变换”方案。第一版只选择一种方案。

## 9. 接入 MeshNav 和 JIE

### MeshNav

- 保留现有 mesh 文件、planner/controller 插件和 goal 接口。
- `robot_frame` 与上述参考点一致，点云输入保持 `/cloud`，`use_sim_time=true`。
- PB 车保持全向控制开关启用。
- 重新测量车辆外廓后更新 inscribed/inflation、障碍高度与净空配置。旧值 `0.28`、`0.225` 是旧低矮代理车配置，不代表 PB 整车。
- 现有坡道居中约束默认在 PB profile 关闭，先验证新车普通导航表现；不能把旧车 y=5.72 的碰撞结论直接套用到 PB 车。若仍需约束，再针对 PB 外廓校准并单独验证。
- 先测静态地图与路径跟踪，再启用点云动态障碍处理。点云可视化成功不等于动态避障已启用。

### JIE

- 保留 `jie_octomap import_pcd_map.launch.py`、`octo_planner meshnav_ceres_controller.launch.py` 等现有链路；launch 名称含 ceres 不表示代码一定绑定车体，须检查实际 frame/topic/尺寸参数。
- 保留 `/planned_path → d1_controller → /cmd_vel_jie → jie_twist_stamper → /cmd_vel`，最终由 PB 适配器接收。
- 重新设置机器人半径、高度、地形接触相关参数，不能继续无条件采用旧 `rmuc2026_profile` 的低矮车数值。
- 原始场地 PCD 可以保留，但依赖车辆尺寸生成的可通行层、膨胀和缓存需要重新构建。
- 静态 PCD/OctoMap 规划不因有实时雷达就自动具备动态避障；需核实当前执行链是否消费实时障碍，再独立验收。

两者不得各自再启动一辆机器人或一个 Gazebo 世界。需要并行对比规划时，只保留一套仿真和 TF，并禁止未选中控制器输出速度。

## 10. 拟提供的启动入口与命令含义

**本节是适配完成后必须实现的命令接口，现在不能直接运行。** `pb_vehicle_adapter` 包及下列 launch/参数尚未创建。

所有终端共用环境（域值以当前测试约定为准；42 仅为现有文档示例）：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
```

`ROS_DOMAIN_ID` 使相关 ROS 节点互相发现；不同域隔离 ROS，但不自动隔离 Gazebo Transport。`ROS_LOCALHOST_ONLY` 限制 ROS 在本机通信。`source` 加载已有 ROS 包与插件环境。独立对照实验还应为 Gazebo 设不同 `IGN_PARTITION`，且对照实例内部保持一致。

单独看车、检查传感器：

```bash
ros2 launch pb_vehicle_adapter pb_vehicle_sim.launch.py \
  world_name:=rmuc2026_field start_gazebo_gui:=True start_rviz:=True
```

完整 MeshNav 测试：

```bash
ros2 launch pb_vehicle_adapter pb_meshnav.launch.py \
  world_name:=rmuc2026_field map_name:=rmuc2026_field \
  start_gazebo_gui:=True start_rviz:=True
```

完整 JIE 测试：

```bash
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash
ros2 launch pb_vehicle_adapter pb_jie.launch.py \
  world_name:=rmuc2026_field start_gazebo_gui:=True start_rviz:=True
```

三个入口三选一，不叠加运行。后两个应自行包含基础仿真；组合 launch 内统一使用真值定位和 PB profile。`world_name` 选现有场地，`map_name` 选 MeshNav 全局地图；GUI/RViz 参数确保可视化可用。

适配包建好且上游依赖已经成功构建后，增量编译：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
colcon build --packages-select pb_vehicle_adapter --symlink-install
source install/setup.bash
```

此命令不负责自动构建缺失依赖。上游依赖的构建清单应由实施者按实际包图列出；不要在本方案里假定任意包名都存在。描述或碰撞模型修改后应重启测试实例，单纯 source 不能更新已生成的车。

## 11. 分阶段实施与验收标准

| 阶段 | 实施内容 | 必须留下的证据 |
| --- | --- | --- |
| A：资源接入 | 固定版本、完整展开 SDF、验证 URI、导入旧场地 | 版本清单、资源清单、无缺失网格日志、Gazebo 截图 |
| B：显示与传感器 | URDF/TF、关节、3D/2D 雷达、相机 | RViz 车体显示、点云与墙面对齐、话题类型/频率 |
| C：速度链 | 单一速度源、全向驱动、超时停车 | 前后、横移、旋转、停止发布后停车记录 |
| D：定位链 | 真值三维 TF 与 odom 一致 | 上坡时 z/pitch 变化、无重复 TF、无时间外推错误 |
| E：导航 | 分别启动 MeshNav/JIE | 各自规划、跟踪、到达、取消；无另一个控制源干扰 |
| F：场地任务 | 平地、短坡、隧道、窄道 | 新车尺寸下的成功/失败记录，不直接继承旧车通过结论 |
| G：动态感知（按当前功能） | 启用实时障碍链后测试 | 障碍点云被消费、导航作出相应反应；仅显示不算通过 |

通用检查命令（接入后执行）：

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic info /pb/cmd_vel_safe --verbose
ros2 topic hz /cloud
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint front_mid360
ros2 topic echo /odom --once
```

雷达 frame `front_mid360` 是上游命名线索，完整展开后若不同，使用点云 header 中的实际名称。`tf2_echo` 验证链路存在；点云墙面对齐和上坡测试才验证外参正确，不能仅凭 TF 查询成功判定通过。

小体积记录：

```bash
ros2 bag record /cmd_vel /pb/cmd_vel_safe /odom /tf /tf_static /clock
```

记录一次用例后 Ctrl+C；默认不录大型点云/图像，必要时短时另录。每次写明模型版本、地图版本、出生点、目标点、控制器、结果及日志路径。

## 12. 回退与交付

实施前保存当前源码差异与运行参数，不使用 `git reset --hard` 或覆盖用户手动修改。旧 ceres 模型、旧桥和旧 launch 原样保留。新包运行失败时停止新实例，恢复旧入口即可；同一个实例不混用两套桥/TF/驱动。

最终应交付：

1. 独立适配包与上游版本、许可证清单；
2. 可切换的 PB/旧车模型配置，两个导航器明确的启动入口；
3. 实际展开后的话题、消息类型、TF 树和尺寸 profile；
4. Gazebo/RViz 显示、传感器数据、停车保护和两个导航器的运行记录；
5. 固化的接口测试，尤其是速度超时、TF 单一所有者和 frame/topic 一致性。

本文不要求新增真实舵轮动力学，不要求迁移整套北极熊导航，也不要求重做已有场地。以最小车辆与传感器适配完成导航仿真测试，就是本次任务的完成标准。
