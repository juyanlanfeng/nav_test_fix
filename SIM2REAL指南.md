# FAST-LIVO2 → PCL 背景差分 → STVL 时间层 → MeshNav：sim2real 指南

更新：2026-10-10。依据本仓库 `3f0c57a`（「AI实现stvl+清理项目」）及当日读取的源码编写。

文档位于仓库根目录 `/home/rainple/nav_test`；ROS 2 工作空间为其下的 `mesh_navigation_tutorials`。本文只讨论 MeshNav + MeshController，不涉及其他导航框架。

**目标：先证明建图、定位、障碍感知和底盘接口分别正确，再验证全向导航及“旧路可行就不换路”的行为。换用真机可以排除 Gazebo 渲染、仿真时间和接触模型的影响，但不会自动修复地图、向量场或重规划代码的问题。**

本次更新保留仿真，主线改为当前已接入的 **PCL 背景差分 → TemporalObstacleLayer → 动态膨胀 → final**。RMCL 分割是可选替代输入；FAST-LIVO2 与实车驱动仍需按实际部署版本接入。下文的实车 YAML、launch 均为待创建模板，本次更新只修改本指南。

## 0. 先整理项目，保留可复现的仿真

### 0.1 当前清理状态与保留清单

本次检查时，旧顶层 `mesh_navigation/`、完整 STVL 参考仓库和 DDDMR 依赖目录已不在项目中。实际工作空间是 `mesh_navigation_tutorials/`，其 `src/mesh_navigation/` 必须保留。`field/` 和 `field/pcd/rmuc2026_field.pcd` 当前存在；不要继续按此前临时缺失的状态操作。

| 路径或内容 | 操作 | 原因 |
|---|---|---|
| `mesh_navigation_tutorials/src/` | 保留 | 当前导航、地图、控制、定位和仿真源码 |
| `src/mesh_navigation/mesh_layers/src/stvl_frustum/` | 保留，含许可证和 `UPSTREAM.md` | 正在编译的三个上游几何文件；不需安装整个 STVL/OpenVDB |
| `src/pb_vehicle_adapter/`、`src/mesh_navigation_tutorials_sim/`、`src/pb_gazebo_sim_support/` | 保留 | 仿真入口、车辆、场地与插件；协调节点也在 PB 包中 |
| `third_party/pb2025_sources/`、`pb2025_install/`、`pb2025_build/` | 一起保留 | 已安装 PB 插件是指向 build 内 `.so` 的绝对软链接，不能只删 build |
| `third_party/pb2025_python/`、`pb2025_tools_venv/` | 保留 | 重新生成 PB 模型时使用 |
| `field/pcd/rmuc2026_field.pcd`、当前 PLY、场地碰撞/视觉 STL | 保留并单独备份 | PCL 固定背景与现有仿真依赖 |
| `field/pcd_to_nav_mesh.py`、`check_nav_mesh_route.py`、`requirements-conversion.txt` | 保留 | 实车 PCD 转导航网格与预检工具 |
| `field/` 中 CAD、重复的原始 PCD、旧分析产物 | 确认引用后归档 | 体积大；不能整目录删除，里面仍有仿真资源 |
| `log/`、`__pycache__/`、`.pytest_cache/` | 保存有价值记录后清理 | 可再生成 |
| 工作空间 `build/`、`install/` | 暂时保留；需要时一起停机重建 | 保留可启动基线；迁移到另一台机器则重新编译 |
| `.git/`、`validation/`、STVL 实施状态文档 | 保留 | 版本与回归证据 |

注意表中的 `src/` 均相对于 `mesh_navigation_tutorials/`。不要用全项目 `find ... -name build` 批量删除；PB 的 build 目录目前是运行依赖。

### 0.2 先备份，再处理残留引用

停掉仿真与导航后，在项目外建立备份。以下是操作示例，本次文档更新没有执行这些命令：

```bash
cd /home/rainple/nav_test
backup_dir="/home/rainple/nav_test_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
git status --short > "$backup_dir/git-status.txt"
git diff --binary HEAD > "$backup_dir/worktree.patch"
git rev-parse HEAD > "$backup_dir/project_commit.txt"
tar -czf "$backup_dir/source-and-config.tgz" \
  mesh_navigation_tutorials/src mesh_navigation_tutorials/launch \
  mesh_navigation_tutorials/validation \
  mesh_navigation_tutorials/STVL_MESHNAV_IMPLEMENTATION_PLAN.md \
  mesh_navigation_tutorials/STVL_MESHNAV_IMPLEMENTATION_STATUS.md \
  tools SIM2REAL指南.md
```

**Git 没有备份所有运行资产。** `.gitignore` 忽略 PCD、H5、STEP、部分 STL、转换产物及 diagnostics。上述源码压缩包也不包含全部 `field/` 和 `third_party/`。另将当前背景 PCD、地图原件、标定、场地 STL、PB 依赖和重要 bag 复制到外盘，并记录 `sha256sum`。`git restore` 无法恢复从未提交的文件。

目前仍有这些待清理项：

1. `pb_vehicle_adapter/package.xml` 仍声明已移除的 `octo_planner`、`jie_octomap`。移除这两个依赖，并核查 JIE/DDDMR 分支对应的 console entry、源码、配置和测试。`setup.py` 中 `pb_dddmr_map_publisher`、`pb_pcd_publisher` 仍指向当前不存在的模块，应一起整理；保留 `meshnav_navigator` 与仿真入口。
2. `third_party/build_dddmr_native.sh`、`setup_dddmr_env.sh`、DDDMR 专用 patch 可随历史项目归档。不要运行仍指向不存在工作空间的环境脚本。
3. 根 README、`field/README.md`、PB README 还有旧 `doc/*`、`meshnav_demo_ws`、JIE/DDDMR 步骤。逐一更新这些入口；`field/verify_rmuc_project.py` 也仍按旧目录组织检查，当前不能把它当作总验收命令。

先查看引用，再决定归档范围：

```bash
rg -n 'octo_planner|jie_octomap|dddmr|meshnav_demo_ws' \
  mesh_navigation_tutorials/src/pb_vehicle_adapter tools third_party \
  README.md field/README.md
ls -l third_party/pb2025_install/rmoss_gz_plugins/plugins/
test -s field/pcd/rmuc2026_field.pcd
```

### 0.3 仿真与实车配置分开

现有 `mbf_mesh_nav.yaml`、PCL YAML 和 `pb_vehicle_profile.yaml` 继续服务仿真。第 8 节建议在 `mesh_navigation_tutorials/real/` 新建实车配置与入口，实车不覆盖仿真文件。

当前仿真总入口 **没有启动 PCL 差分节点**：默认 `obstacle_segmentation: rmcl_seg` 启动的 RMCL 发布 `/obstacle_points`，时间层订阅的却是 `/dynamic_obstacles`。使用本文主链时，应另开终端启动 PCL；若不使用 RMCL 分割，可将仿真配置中的 `meshnav_launch.obstacle_segmentation` 改为 `none`。定位选择 `rmcl_micpl` 时仍需保留 RMCL 定位。一个障碍输入话题只安排一个已验收的来源。

```bash
# 终端 A：保留原 PB 仿真入口。
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash
ros2 launch pb_vehicle_adapter meshnav_pb_sim.launch.py
```

```bash
# 终端 B：PCL 仿真差分；先确认当前背景 PCD 与 Gazebo 场地对齐。
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash
ros2 launch pcl_obstacle_point_subtraction pcl_obstacle_point_subtraction.launch.py
```

启动后核查 `/cloud`、`/dynamic_obstacles` 和 final 代价更新，再做原场景的箱体出现、遮挡、移走与绕行回归。`pb_preflight` 的通过不能单独证明 PCL 差分与时间层清除正确。`start_sim: "False"` 表示沿用已有仿真，仍会切换 PB 速度来源，不能作为实车模式。

## 1. 当前具备什么，还缺什么

| 项目 | 当前状态 | 真机工作 |
|---|---|---|
| PCD → 可行驶面 PLY | 已有 `field/pcd_to_nav_mesh.py` | 使用真实场地 PCD，按车辆能力重新验收 |
| MeshNav 网格、静态/动态代价层 | 已有，动态输入使用 `TemporalObstacleLayer` | 切换地图、缓存、车体参数和真实时钟 |
| PCL 固定背景差分 | 已有，缓存 KD 树、队列深度 1、过滤车体周围点 | 背景配准、QoS、外参及过滤范围需按实车修改 |
| STVL 时间衰减 | 已有线性/指数/持久模型与视野加速 | 是 MeshNav 适配实现，未移植整个 STVL，也没有完整自由空间/遮挡判定 |
| MeshController 全向控制 | 已有，`holonomic: true` | 校验 vx、vy、wz 的方向、单位与底盘能力 |
| 旧路保留协调节点 | 已有 `meshnav_navigator`，受阻后取消旧执行并停车重试规划 | 可以独立运行；还需验证真实底盘停止与恢复 |
| FAST-LIVO2 驱动、外参、时间同步 | 本次未提供实际部署版本 | 现场确认并冻结配置 |
| 重启后对旧地图定位 | 本仓库未提供 FAST-LIVO2 的完整接入 | 需要实际定位模块，或经验证的短时初始化方案 |
| 真机里程计/TF 适配 | 不能用 Gazebo 真值代替 | 由实际里程计、定位节点和标定外参提供 |
| 真实底盘速度接口与停止机制 | PB 仿真适配器不能代表实车接口 | 对接底盘驱动、速度仲裁、失联停车 |
| 真机总 launch | 当前没有已验证的专用入口 | 第 8 节提供待落地模板，驱动部分需现场接入 |

本文是操作与接入指南；文中的真机 YAML、launch 是**需要创建和填写的模板**，本次只交付文档，不会自动改动当前仿真参数，也没有声称已完成实车驱动适配。

当前导航配置快照：投影高度 **0.50 m**，静态/动态内圈 **0.15/0.30 m**，动态外圈 **0.60 m**，线速度上限 **0.8 m/s**；PCL 背景匹配阈值 **0.08 m**。动态内圈的旧注释仍提到 0.40 m，与实际数值不符；`obstacle_inflation.inscribed_value: 0.99` 也与 `mesh_planner.cost_limit: 0.99` 相等，不能把它当成留有阈值余量的实车设置。**这些均是当前仿真配置，不是实车推荐参数。** 后续以实际参数回读、测量和路线验收为准。

### 1.1 先填现场信息

| 内容 | 现场填写 |
|---|---|
| FAST-LIVO2 仓库 URL、分支、commit；ROS 1 / ROS 2 | 待填 |
| 上位机系统、ROS 发行版、CPU/GPU、是否分布式 | 待填 |
| 雷达型号、驱动版本、点云格式、时间戳来源 | 待填 |
| IMU、相机、内参、外参、同步方式 | 待填 |
| 整车含附件的长 L、宽 W、最高碰撞高度 H | 待填，单位 m |
| 前后左右突出物、雷达/相机是否高于车顶 | 待填 |
| 最大允许坡度、可跨台阶、最小实测刹车减速度 | 待填 |
| 底盘命令话题、消息类型、参考坐标系、失联超时 | 待填 |
| 实际里程计话题、pose/twist 坐标、是否发布 TF | 待填 |
| 地图定位方案、初始位姿输入方式、失效判据 | 待填 |

未确认 FAST-LIVO2 版本时，不能照搬某个 ROS 2 分支的 launch 名字。官方当前 README 使用 catkin 和 `roslaunch`，属于 ROS 1 用法；你们若使用 ROS 2 移植版，应以该仓库为准。[FAST-LIVO2 官方说明](https://github.com/hku-mars/FAST-LIVO2)

## 2. 总体流程与接口

```mermaid
flowchart TD
    A[真实雷达 / IMU / 相机] --> B[FAST-LIVO2 建图与连续里程计]
    B --> C[固定坐标系下的场景 PCD]
    C --> D[PCD 转换与地图验收]
    D --> E[可行驶面 PLY / 配套 H5]
    C --> F[旧地图定位 / 重定位]
    B --> F
    F --> G[地图位姿与 TF]
    A --> H[当前扫描：去畸变 / 自身点过滤 / 坐标与 QoS 适配]
    C --> I[PCL 固定背景差分]
    G --> I
    H --> I
    I --> T[TemporalObstacleLayer 障碍历史]
    T --> U[动态膨胀]
    E --> S[静态代价与膨胀]
    U --> J[final：静态与动态取最大值]
    S --> J
    J --> K[GetPath / MeshController]
    G --> K
    L[Mesh Goal] --> M[旧路可行性检查协调节点]
    M --> K
    K --> N[速度仲裁 / 限幅 / 看门狗]
    N --> O[真实底盘]
```

地图有不同用途，必须区分：

- **场景 PCD**：保存地面、墙、柱、天花板等结构，用于检查建图与旧地图定位。
- **固定背景 PCD**：PCL 差分的参照，必须与导航 map 对齐，建图中的临时物体应剔除。可以来自场景 PCD，但要单独验收；背景中已有的物体不会因为再次扫描而变成差异点，其固定碰撞约束还需由导航地图/静态代价表达。
- **导航 PLY**：机器人能够行驶的表面。不能把墙、顶棚直接当作行驶面。
- **H5**：MeshNav 的工作缓存，依赖 PLY 和代价参数；不提供 SLAM 重定位。
- **当前扫描**：实时障碍输入。不能用累计建图点云替代，否则移动物体会留下历史点。

### 2.1 约定导航侧接口

以下名称是本文模板约定，真实驱动通过适配或 remap 接入。

| 接口 | 类型 | 约束 |
|---|---|---|
| `/cloud` | `sensor_msgs/msg/PointCloud2` | 当前扫描，XYZ 属于配置的 `lidar_frame`；采样时间有效；当前 PCL 输入订阅是 Reliable |
| `/dynamic_obstacles` | `sensor_msgs/msg/PointCloud2` | PCL 差分输出，Reliable、队列深度 1；frame 为 `base_footprint`，保留采样时间；正常无障碍也发布有效空帧 |
| `/obstacle_points` | `sensor_msgs/msg/PointCloud2` | 仅选择 RMCL 分割时使用；需改时序层 topic 或将分割输出 remap 到 `/dynamic_obstacles` |
| `/odom` | `nav_msgs/msg/Odometry` | 连续里程计，必须提供可信实际速度，不能只有 pose、twist 全为零 |
| `/tf`、`/tf_static` | TF | `map` 到导航车体参考点、雷达在扫描时间可查询 |
| `/nav/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 本文实车模板将 MBF `~/cmd_vel` remap 到此处；改话题名不会改变消息类型 |
| 底盘最终命令话题 | 现场确定 | 若驱动收 Twist，需实际转换节点提取 `twist`，同时校验时效、限幅和失联停车；不能只 remap |
| `/rviz/goal_pose` | `geometry_msgs/msg/PoseStamped` | `frame_id: map`，目标落在正确层的网格上 |
| `/meshnav_navigator/status` | `std_msgs/msg/String` | JSON：状态、执行路径计数、阻断段等 |

**速度必须经过一个明确的最终发布者。** 遥控、导航、驻停控制不能同时直接写电机命令；底盘侧应在命令过期、急停、导航撤销或定位失效时停止。不能只依赖 ROS 节点正常退出时发出的最后一条零速度。

## 3. FAST-LIVO2 建图

### 3.1 版本与 ROS 环境

推荐保留你们已经验证过的 FAST-LIVO2 建图环境。先记录版本，再决定如何把在线数据接到 ROS 2 Humble：

- **ROS 2 FAST-LIVO2**：可直接按接口对接，检查消息定义、QoS、时间戳和 TF。
- **ROS 1 FAST-LIVO2**：离线 PCD 可以直接复制到本仓库；在线导航还需要桥接标准消息或经过验证的转换进程。不要把“能复制 PCD”当作实时接口已打通。
- `ros1_bridge` 需要可共存的 ROS 1/ROS 2 构建环境。官方说明对 Ubuntu 22.04 + Humble 的组合列出源码构建限制；不能假定当前机器安装一个包就能桥接。优先桥接 `PointCloud2`、`Odometry`、TF 等标准接口；Livox 自定义消息只有在对应接口参与桥接构建后才可能支持。[ros1_bridge 官方说明](https://github.com/ros2/ros1_bridge)

ROS 1、ROS 2 普通工作终端分别加载各自环境；只在专门的桥接终端按桥接工程要求加载两套环境。

建议为建图数据建立独立目录：

```bash
cd /home/rainple/nav_test
mkdir -p field/real/site01_v1/{raw,calibration,reports,mesh}
mkdir -p mesh_navigation_tutorials/diagnostics/real_site01
```

保存软件 commit、驱动配置、FAST-LIVO2 YAML、相机标定和整车传感器外参。后续更换外参、地图坐标或重建地图时使用新的版本目录。

### 3.2 标定与采集

1. 固定雷达、IMU、相机安装，检查支架松动和电机振动；测量雷达到车体导航参考点的外参。
2. 按实际 FAST-LIVO2 版本确认 LiDAR–IMU、LiDAR–Camera 变换的方向，不能仅凭参数名字猜测是正变换还是逆变换。
3. 确认雷达逐点时间、IMU 时间、图像曝光时间处于相容时间基准；跨机器同步系统时钟，记录同步偏差。
4. 按该版本的惯导初始化要求起步；慢速走完整个场地，补扫地面、转角、坡道上下端、隧道内地面。
5. 对关键窄通道从两侧重复采集，覆盖车体将经过的整个宽度，避免仅扫描到墙或顶棚。
6. 尽量减少建图期间的移动物体；保留原始 bag，以便定位误差与地图缺洞可以回放分析。
7. 回到起点检查墙面重影、地面分层及重复经过处的偏差。绕回起点本身不证明系统执行了回环优化。

FAST-LIVO2 的 PCD 导出设置在官方 Avia 示例中包括 `pcd_save_en`、`type`、`filter_size_pcd`、`interval`。其中 `type: 0` 表示世界系输出，`interval: -1` 表示累计保存；具体版本的实际保存行为必须核对源码，不能把这个 Avia 文件整体用于其他雷达。[官方参数示例](https://github.com/hku-mars/FAST-LIVO2/blob/main/config/avia.yaml)

```yaml
# 仅是官方 ROS 1 参数结构示意，不是本项目 ROS 2 参数文件。
pcd_save:
  pcd_save_en: true
  type: 0                 # 地图世界系；不要直接拼接未经位姿变换的 body 系分帧。
  filter_size_pcd: 0.05    # 示例导出分辨率，须按点密度和最小结构尺寸选定。
  interval: -1            # 小场地可用；长时间累计可能占用大量内存。
```

官方当前 `LIVMapper.cpp` 在正常结束循环后调用 `savePCD()`，相关输出包括 `Log/pcd/all_raw_points.pcd`，视觉分支还可能输出降采样文件。因此应正常退出并确认落盘，避免强杀丢失缓存。该源码的默认里程计帧名为 `camera_init` / `aft_mapped`，并有使用发布时间的代码；你们的版本必须实查，不能假设它已经输出导航需要的测量时刻、车体系和速度。[官方发布与保存实现](https://github.com/hku-mars/FAST-LIVO2/blob/main/src/LIVMapper.cpp)

### 3.3 PCD 接收标准

将选定 PCD 复制为：

```text
/home/rainple/nav_test/field/real/site01_v1/raw/site01.pcd
```

随 PCD 写一份 `map_metadata.yaml`，至少记录：

```yaml
site: site01
version: v1
source_frame: 填写建图世界系
navigation_frame: map
units: m
# T_map_slam 把原建图坐标转换到最终导航 map 坐标；以下由标定结果填写。
T_map_slam: 待填写4x4矩阵
slam_commit: 待填
calibration_version: 待填
pcd_sha256: 待填
```

验收内容：米制尺寸、重力方向、地面覆盖、墙厚/重影、关键通道净空、重复经过误差、动态残影。Z 原点可以在传感器初始位置，地面 Z 为负不构成错误；不能为了让地面变成 0 而只移动 PCD，却忘记实时定位也需要同样的坐标变换。

为 PCL 另外准备 `raw/site01_background_map.pcd`：应用已记录的 `T_map_slam`，剔除建图时的移动物体，保存哈希和处理记录。背景可以包含固定墙、柱、地面等结构，导航 PLY 仍只表达经验证的可行驶面。背景不会给 MeshNav 自动生成静态碰撞约束；在背景中被抵消的固定障碍，必须由可行驶面边界、静态层或其他已验收的固定约束覆盖。

## 4. PCD → 可行驶面 PLY → H5

### 4.1 使用已有转换器

在项目根目录执行；转换虚拟环境仅用于离线工具，不要用它启动 ROS 节点：

```bash
cd /home/rainple/nav_test
# 已有此虚拟环境时直接使用；首次建立时执行下面两行。
python3 -m venv field/.step_convert_venv
field/.step_convert_venv/bin/pip install -r field/requirements-conversion.txt

field/.step_convert_venv/bin/python field/pcd_to_nav_mesh.py inspect \
  field/real/site01_v1/raw/site01.pcd \
  --report field/real/site01_v1/reports/pcd_inspection.json
sha256sum field/real/site01_v1/raw/site01.pcd
```

脚本支持 PCD `ascii` 和未压缩 `binary`，至少需要 XYZ；额外强度/颜色字段可以保留。遇到 `binary_compressed`，先保留原文件，再用 PCL 工具转成普通 binary：

```bash
pcl_convert_pcd_ascii_binary input_compressed.pcd output_binary.pcd 1
```

转换器的 `--scale` 负责单位倍率，`--translate` 仅负责平移，**不负责旋转校正**。需要重力对齐或地图旋转时，使用经过验证的点云变换工具生成新文件，保存变换矩阵，并同步修改在线定位到 map 的变换。不要反复手动旋转几个副本而不记录来源。

### 4.2 参数选择与转换命令

先填写以下 Bash 变量，然后执行转换。示例分辨率用于首次检查；最大坡度和整车高度必须填写测量结果：

```bash
cd /home/rainple/nav_test
read -r -p '整车最高碰撞包络高度加净空余量，单位 m: ' VEHICLE_CLEARANCE_HEIGHT
read -r -p '已验证可通行的最大坡度，单位 deg: ' VEHICLE_MAX_SLOPE
export VEHICLE_CLEARANCE_HEIGHT VEHICLE_MAX_SLOPE

field/.step_convert_venv/bin/python field/pcd_to_nav_mesh.py convert \
  field/real/site01_v1/raw/site01.pcd \
  field/real/site01_v1/mesh/site01_nav.ply \
  --scale 1.0 --translate 0 0 0 \
  --voxel-m 0.025 --normal-k 24 --grid-m 0.10 \
  --max-slope-deg "${VEHICLE_MAX_SLOPE:?必须填写实测坡度}" \
  --layer-merge-m 0.04 \
  --robot-height-m "${VEHICLE_CLEARANCE_HEIGHT:?必须填写整车净空高度}" \
  --min-points-per-cell 3 --min-component-area-m2 1.0 \
  --report field/real/site01_v1/reports/site01_nav.pcd_to_mesh.json
```

| 参数 | 作用与调整依据 |
|---|---|
| `voxel-m` | 输入降采样尺度；过大可能抹去窄坡、台阶边缘 |
| `grid-m` | 输出表面网格尺度；过小且点稀疏时会碎裂，过大可能错误跨越缺口 |
| `normal-k` | 无可靠法向时估计法向的邻点数；跨越两个不同面时可能混入错误法向 |
| `max-slope-deg` | 依据真实抓地、负载和稳定性测试，不能照搬算法默认 55° |
| `layer-merge-m` | 同一 XY 柱中的高度聚类阈值；过大可能合并上下层 |
| `robot-height-m` | 离线净空筛选高度，包含会发生碰撞的雷达/相机/附件和余量 |
| `min-points-per-cell` | 支撑一个候选表面所需点数；降低不等于缺失地面得到证实 |
| `min-component-area-m2` | 连通分量面积筛选；0 表示只保留最大块，不是“完全关闭筛选” |

**离线净空高度和在线 `mesh_map.obstacle.robot_height` 是两个不同概念。** 后者是障碍点向网格投影的最大距离，详见第 7 节。两者不能因为都叫 robot_height 就强行填写同一个数。

点云没有提供完整自由空间证据。仅靠几何法向不能可靠区分所有地板、桌面、楼顶和天花板；需要对原始 PCD 与 PLY 叠加检查。若手工删除/补齐三角形，应生成新版本并记录依据。

### 4.3 地图验收与无电机规划

检查转换 JSON 的输入/输出路径、哈希、边界、连通分量及保留面积。为每条隧道、坡道、回程和跨区域路线各选一组起终点，Z 使用实际导航表面高度。

```bash
# SX SY SZ GX GY GZ 为本场地实际选点；R 为后文确定的保守包络半径。
field/.step_convert_venv/bin/python field/check_nav_mesh_route.py \
  field/real/site01_v1/mesh/site01_nav.ply \
  --start "$SX" "$SY" "$SZ" --goal "$GX" "$GY" "$GZ" \
  --clearance-m "$R" \
  --report field/real/site01_v1/reports/route_01.json
```

该脚本只预检起终点贴近网格、几何连通及端点边界距离，不能代替整条路线的碰撞验收。

已有 `meshnav_map_test.launch.py` 可以预览 PCD、PLY 和 GetPath，但它会把 MBF 速度映射到 `/cmd_vel`。**预览时不要接通真实底盘订阅；只运行 `meshnav_plan_only`，不要发送 ExePath。** 该入口的地图参数、H5、静态膨胀参数必须显式传入实车值，不能沿用旧场地示例：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash
ros2 launch mesh_navigation_tutorials meshnav_map_test.launch.py \
  mesh_map_path:=/home/rainple/nav_test/field/real/site01_v1/mesh/site01_nav.ply \
  mesh_map_working_path:=/home/rainple/nav_test/mesh_navigation_tutorials/site01_v1_preview.h5 \
  source_pcd_path:=/home/rainple/nav_test/field/real/site01_v1/raw/site01.pcd \
  publish_source_cloud:=true \
  static_inscribed_radius:="$R" \
  static_inflation_radius:="$R_OUTER" \
  height_diff_threshold:="$STEP_LIMIT"
```

`R_OUTER >= R`；`STEP_LIMIT` 是结合网格分辨率与车辆台阶能力验收的高度差阈值。这些变量须在对应终端填写，不能使用空字符串。

另一终端运行：

```bash
ros2 run mesh_navigation_tutorials meshnav_plan_only \
  --start "$SX" "$SY" "$SZ" --goal "$GX" "$GY" "$GZ"
```

确认 `PATH_READY`、完整路线、正确楼层，并逐段检查车体余量。测试完成后关闭地图预览服务，避免后续有两个 `/move_base_flex`。

H5 建议放在工作空间根目录，如 `mesh_navigation_tutorials/site01_v1_real.h5`。更换地图或会改变缓存层结果的参数时使用新文件名，或在停机后备份旧缓存再重建。不要加载旧 RMUC 的 H5 来测试真实场地。

当前 MeshMap 还提供显式保存层的服务。地图完成计算后，可以在机器人停稳、场地清空、动态障碍已清除的状态下调用，并检查返回的 `success` 和日志；H5 文件存在本身不证明所有层已完整写入：

```bash
ros2 service call /move_base_flex/save_map std_srvs/srv/Trigger '{}'
```

当前 ObstacleLayer/TemporalObstacleLayer 的动态历史不读写 H5；保存服务不能替代动态历史备份或跨重启保持。仍应在重载后检查动态层初始状态和静态缓存是否对应本地图。


## 5. 在线定位、坐标与 TF：先解决“车在哪”

### 5.1 导航参考系

目标 TF 关系如下；每条边只能有一个发布者：

```text
map                            固定在导出并验收的地图上
 └─ odom                       到旧地图的定位修正
     └─ base_footprint          连续车体导航参考点，必须与网格高度约定一致
         └─ base_link          实际车体参考点
             ├─ lidar_link
             ├─ imu_link
             └─ camera_link
```

名称可以不同，但所有节点参数、外参和命令参考系必须一致。按 ROS 约定使用米、秒、弧度，车体 x 向前、y 向左、z 向上；相机 optical 坐标有单独约定，不能直接当作车体系。[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)

`odom` 应连续，`map` 负责全局定位修正；重定位跳变不应直接变成底盘速度突变。[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

坡道/多层地图不能把全场车辆 Z 强制设为 0。若导航参考点使用底盘底部的固定参考点，可以通过真实刚体外参获得；若使用随地面投影的 `base_footprint`，它与 `base_link` 的关系需要按实际姿态计算，不能用一个不符合实车的静态零变换代替。首先核对该参考点到网格的偏差，并据此验证路径投影容差。

### 5.2 FAST-LIVO2 输出不等于最终车体位姿

令 `T_A_B` 表示把 B 系坐标变换到 A 系。假设 LIVO 输出 IMU 的位姿：

```text
T_map_base = T_map_slam × T_slam_imu × T_imu_base
T_map_odom = T_map_base × inverse(T_odom_base)
```

如果 LIVO 输出的是雷达或其他 body 系，则替换对应外参。旋转时，IMU/雷达与底盘参考点之间的杆臂也影响平移速度；只把消息中的 child_frame_id 改成 base_link 不能完成位姿和速度变换。

现场必须检查：

- `Odometry.pose` 相对于哪个世界系、描述哪个刚体点。
- `Odometry.twist` 是否真实填写，速度在哪个坐标系表达，是否补偿杆臂。
- TF、点云和 pose 是否对齐同一测量时刻；不能全部盖成当前时间掩盖延迟。
- 是否已经有 FAST-LIVO2、底盘驱动、EKF 或 RMCL 发布同一条 TF。
- 位姿适配与速度估计暂未实现时，先记录原始数据完成适配验收，再接导航。

### 5.3 地图保存后重启怎么办

推荐部署为：**FAST-LIVO2/底盘里程计提供连续运动估计，独立的旧地图定位提供 `map → odom`。** 若你们使用的 FAST-LIVO2 分支本身已有经过验证的旧地图定位功能，也可以使用，但应记录其初始化与重定位接口。

必须区分三种情况：

| 场景 | 可用条件 | 限制 |
|---|---|---|
| 建图进程未重启，导出地图后继续运行 | 世界系未变化，导出文件与当前位姿同源，且版本支持不退出保存 | 可做短时验证；仍需监测漂移 |
| 重启后人工对齐一次 `map → odom` | 有测量/配准支持的初始位姿，所有传感器坐标已对齐 | 只适合受控短程测试，不具备持续漂移修正 |
| 重启后自动定位到旧 PCD | 具有实际部署并验证的定位模块与失效检测 | 正式跨场地导航采用此类方案 |

官方保存路径可能在进程退出时才落盘，因此不要默认能在同一会话保存地图再无缝导航。**每次重启都设单位变换不是重定位；从同一物理位置开机，也不保证初始化世界系完全相同。**

当前 `rmcl_launch.py` 会默认把导航 PLY 同时用于 RMCL。若该 PLY 主要是地面，不能假定它足够约束 XY 和 yaw 定位；真机首测建议使用已验证的 PCD 定位方案。需要 RMCL 定位时，另行准备适合定位的场景几何并验证可观测性，避免用可行驶面地图代替完整定位地图。

### 5.4 开电机前检查

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo map lidar_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 topic info /odom -v
```

`tf2_echo` 只能确认最新 TF；还要确认 `/cloud.header.stamp` 对应时刻的变换可用，分割日志没有过去/未来外推失败。静止、前进、横移、原地旋转分别观察一次；RViz 中墙面相对地图应稳定，不应随转向扭动。

## 6. 把真实雷达接到障碍链路

### 6.1 当前主链：PCL 背景差分

```text
去畸变后的当前扫描 /cloud（lidar_frame，保留采样时间）
  → 变换到背景比较坐标系（实车背景在 map 时，odom_frame 配为 map）
  → 与固定背景 PCD 最近邻差分（KD 树初始化一次）
  → 变回 base_footprint，按 robot_radius 过滤车体周围点
  → /dynamic_obstacles
  → TemporalObstacleLayer 向下投影到导航网格，保存回波/命中面历史
  → obstacle_inflation
  → final 与静态膨胀代价取最大值
```

PCL 识别的是“与背景不一致的点”，不是物体语义分类，也不是地面分割。定位偏差、地图缺点、外参错误、畸变和背景变化都可能产生差异点；最终仍受距离、投影高度和网格命中约束。空场地应先证明背景能正确抵消，再放新障碍验证。

实车接入前逐项处理：

| 项目 | 当前实现 | 实车操作 |
|---|---|---|
| 点云输入坐标 | 按配置 `lidar_frame` 计算变换；代码只检查 header 非空 | XYZ 必须真正位于该帧，header 与配置一致；不能仅改名字或 remap 世界系云 |
| 背景比较坐标 | 参数叫 `odom_frame`，实际可指定 `map`；加载 PCD 时只赋 frame 名 | 将背景 PCD 实际配准到 map，再设置 `odom_frame: map`，保证扫描时刻 TF 可查 |
| Gazebo 原点修正 | C++ 仍硬编码雷达局部 Z **+0.03 m** | 修改为可配置偏移，仿真保持原值，实车按测量原点外参设值；TF 已描述真实测量原点时通常为零。此项尚未实现，仅改 YAML 无法去掉硬编码 |
| QoS | 输入、输出均 Reliable，KeepLast(1) | BestEffort 驱动需适配发布 Reliable `/cloud`，或修改并验证 PCL 订阅 QoS；只改时间层 qos 无效 |
| PCD 加载 | 失败只报错，空背景随后把整帧当障碍 | 启动前检查路径和非零加载点数；后续代码应在背景加载失败时拒绝进入工作状态 |
| 匹配阈值 | `distance_threshold: 0.08`，内部平方后传给 PCL | 填实际距离，结合静态定位/地图误差与最小待检障碍测量；调大也可能抹去贴墙新障碍 |
| 自身过滤 | `robot_radius: 0.4`，base 系 XY 圆柱内的点全部删除 | 按实车包络验收；它也会删掉该圆柱内的外部障碍，不是完整 URDF 自滤波 |
| TF 发布 | `need_odom_tf`、`need_static_tf` 当前均 false | 里程计/URDF 已提供对应 TF 时保持 false；避免重复发布 |

**PCL 的点坐标修正和时间层的 `sensor_offset_xyz` 是两处不同设置。** 前者影响背景匹配和障碍坐标，后者影响时间层的测量原点、距离和视锥。TF、两处设置和点云坐标必须采用同一原点约定；只把时间层偏移改成零，不会取消 PCL 中的 +0.03 m。

优先接去畸变后的单次扫描，并变换到扫描参考时刻的真实雷达系。Livox 自定义消息需要实际转成 PointCloud2。不要把累计地图、历史彩色点云或建图全局拼接云送入时间层；它们会不断刷新本应过期的历史。

### 6.2 消息新鲜度、空帧与原点

当前时间层接受 FLOAT32 XYZ 的合法 PointCloud2，可处理组织点云行填充和大端数据。输出即使零点也应带合法 XYZ 字段、正确 frame 和新的采样时间；不要用无字段的默认消息伪装有效空帧。

当前 `max_observation_age: 0.30`：采样到处理的年龄超限、未来超过 1 ms、重复/乱序时间戳、无效数据布局、TF 失败都会保持原历史。TF/投影结束后还会再次检查年龄。正常空帧推进衰减；断流、过期帧不代表“障碍消失”。多机系统须同步系统时钟，并确认驱动 stamp 的时基；不能用重写为当前时间掩盖延迟。

检查频率之外还要量化延迟，看到 `/dynamic_obstacles` 不等于消息被时间层接受。之前仿真曾有约 0.55 s 上游积压，使全部消息被 0.30 s 检查拒绝；缓存背景 KD 树和减小队列后该轮 P95 降至 38 ms。这是历史仿真记录，实车仍需重测。

TemporalObstacleLayer 按 TF 和 `sensor_offset_xyz` 得到真实雷达原点，范围相对该原点计算；输入障碍点可以位于 base、雷达或其他可转换帧。**回退到传统 ObstacleLayer 时**仍按输入点云原点的 `p.norm()` 过滤，不能直接接世界系点云。这不改变 PCL 对原始 `/cloud` 必须属于 `lidar_frame` 的要求。

```bash
ros2 topic info /cloud -v
ros2 topic echo /cloud --once --field header
ros2 topic hz /cloud
ros2 topic info /dynamic_obstacles -v
ros2 topic echo /dynamic_obstacles --once --field header
ros2 topic hz /dynamic_obstacles
ros2 param get /move_base_flex mesh_map.obstacle.type
ros2 param get /move_base_flex mesh_map.obstacle.topic
ros2 param get /move_base_flex mesh_map.obstacle.max_observation_age
```

RViz 同时看当前扫描、背景 PCD、障碍点、obstacle、obstacle_inflation、final。前、后、左、右多距离放箱体，横移/旋转后复测；再测试侧面遮挡、箱体移走和点云断流，分别记录标记、保持与清除。四向一次检测成功不代表所有盲区已覆盖。

### 6.3 可选替代：RMCL 分割

选择 RMCL 时，链路为 `/cloud → conv_pc2_to_o1dn_node → /rmcl_inputs/cloud → o1dn_map_segmentation_embree_node → /obstacle_points`。将时间层 topic 改为 `/obstacle_points`，或将 RMCL `outlier_scan` remap 到 `/dynamic_obstacles`；停掉 PCL 的同话题发布，保持一个已验收来源。

RMCL 转换器输入同样使用 Reliable；`sensor_frame` 仅用于帧名，不会自动把 XYZ 变换过去。使用可行驶面 PLY 做分割时，墙面等静态结构也可能成为 outlier。RMCL 分割与 RMCL 定位是不同用途；PCL 可与单独的 RMCL 定位共存，但不能因此重复发布 map→odom。

这条分支需要另外配置真实雷达模型、独立地图与阈值、`use_sim_time: false`，并记录 `/rmcl_inputs/cloud`、`/obstacle_points` 和节点参数。第 8 节的模板默认使用 PCL，不同时启动 RMCL 分割。

## 7. 真机参数如何确定

### 7.1 车辆平面尺寸

以居中的矩形近似：

```text
内切半径 r_in = min(L, W) / 2
外接半径 r_out = sqrt((L/2)^2 + (W/2)^2)
```

导航参考点不在车体中心，或有突出物时，直接计算参考点到最远碰撞点的距离。MeshNav 的标量膨胀不等于完整朝向相关 footprint 检查；**全向首测建议以外接包络加误差余量设置保守硬间距**。不能因为参数叫 `inscribed_radius`，就认定填半车宽能保证旋转时车角不碰撞。

| 参数 | 真机设置原则 |
|---|---|
| `static_inflation.inscribed_radius` | 静态障碍的硬间距，依据车体包络、定位误差和地图误差 |
| `obstacle_inflation.inscribed_radius` | 动态障碍硬间距，增加感知与执行误差余量 |
| 两层 `inflation_radius` | 软代价范围，至少覆盖对应硬间距 |
| 两层 `inscribed_value` | 与 `mesh_planner.cost_limit` 联动；模板用 1.0 对 0.99 留出明确区分 |
| `height_diff.threshold` | 网格邻域高度差阈值，需要在真实坡道/台阶上验证，不能直接等同离线坡度参数 |
| `mesh_controller.max_lin_velocity` | 首测可从 0.10～0.15 m/s 量级开始，之后依据制动和延迟逐步提高 |
| `mesh_controller.max_ang_velocity` | 首测可从 0.20 rad/s 量级开始，结合底盘转向能力验证 |
| `planner_frequency` | 协调节点检查旧路的频率；当前 2 Hz 不表示每秒强制换两次路 |
| `path_check_step` | 旧路采样间距，当前 0.025 m；不是传感器空间分辨率 |
| `path_check_max_distance` | 到网格表面的投影容差；不能用放大它掩盖车体参考点或定位高度错误 |

以上速度是调试起点，不是对未知底盘的性能保证。膨胀余量和可通行通道必须联合验证；保守参数导致隧道不通时，先测量，不靠随意缩小半径获得“成功路线”。

### 7.2 在线高度过滤

`mesh_map.obstacle.robot_height` 限制障碍点沿指定方向投影到导航网格的距离。它不改变真实车高，也不是自动识别悬空障碍的完整碰撞算法。

已有仿真证据：0.5 m 方块可能只有 0.22 m 或 0.41 m 高度回波；0.15 m 的投影阈值会漏掉它。**不能把仿真 16 线采样规律当成真实 MID360 的采样规律，也不能直接把历史测试的 0.42/0.50 m 当成真机答案。** 应从真实多视角障碍点高度分布确定检测阈值，同时验证顶棚/悬物误判。

`axis_frame` 与 `down_axis` 要明确：模板采用重力对齐的 `map` 下 `[0,0,-1]`。如果使用车体 Z 轴，坡道上投影方向会随车倾斜，两种方式应在实际地形上比较。

PCL 的 `distance_threshold` 是与背景最近邻的距离容差，不是世界系高度阈值。选择 RMCL 分割时，`min_dist_outlier_scan` 是扫描相对地图表面的分割判据，也不是“保留所有高于该值的世界系点”。不要把这些值和 `robot_height` 当作固定高度区间。

### 7.3 反应距离与失效停车

把延迟分开测量：

```text
扫描 → 点云适配 → 背景差分 → 时间层/膨胀/final → 旧路检查
     → 取消旧执行 → 重规划/必要的无路重试 → 新指令 → 底盘响应
```

2 Hz 检查在相位不利时会引入接近 0.5 s 的额外等待。粗略保守估算：

```text
需要的检测/制动距离 ≥ v × 总反应时间 + v² / (2 × 实测有效减速度) + 误差余量
```

总反应时间应使用端到端测量的高分位/最坏观测值，加上必要的不确定性余量。坡道、横移、不同载荷分别测制动。动态障碍可能直接出现在制动范围内时，仅靠全局重规划不能保证避碰，需要底盘侧独立的停止/近距保护。

当前时间层已有内存历史，断流保持代价；这只能避免误把断流当作空闲，不能检测断流期间出现的新物体。协调节点的反馈超时检查也不能代替点云/定位的新鲜度看门狗。取消旧 ExePath 并不证明实车已经物理停稳；最终零指令、失联停车与实测速度应在底盘侧验收。

### 7.4 STVL 时间参数与实现边界

当前默认 `clearing_mode: stvl`、`decay_model: 0`、`obstacle_keep_time: 10.0`、`decay_acceleration: 0.0`。含义是：持续检测到的记录刷新；未再检测到的记录在**严格超过最后检测时间 10 秒后的下一有效新帧**清除，合法空帧也能推进。没有独立过期定时器，点云停了 10 秒不会自动清空。

| 参数 | 实车选择与验证 |
|---|---|
| `history_voxel_size` | 当前 0.05 m；与地图分辨率、回波密度、内存和更新耗时联合验收 |
| `clearing_mode` | 首测使用 `stvl`；`visible_timeout` 是显式兼容模式，其缺失窗口不能提供遮挡证明 |
| `decay_model` | 首测保持 0 线性；-1 持久不按时间清除，重启或已定义复位事件清除；1 指数不能简单解释成 keep 秒寿命 |
| `obstacle_keep_time` | 取决于遮挡绕行时长和移走后允许的残留时间；10 s 是当前仿真值 |
| `decay_acceleration` | 首测 0；正值在视锥内累计 `a*t³/6` 加速，可能提前清除被遮挡侧面，需单独验证 |
| `max_observation_age` | 当前 0.30 s；依据延迟测量设定，先处理积压和时间同步 |
| `sensor_frame` / `sensor_offset_xyz` | 真实测量原点约定；同时处理 PCL 的硬编码修正 |
| 范围与水平/垂直 FOV | 使用真实雷达有效覆盖并包含姿态；当前仿真的 360°、-7°～52°不可盲抄 |
| `localization_reference_frame` | 使用连续 `odom` 参考，检测 map←odom 修正跳变 |
| 位姿跳变阈值 | 当前 0.75 m、45°、短间隔 0.20 s；按真实运动与定位修正幅度验证 |

指数模型采用 `keep * exp(-age)`，过期条件是剩余量减加速量严格小于零；加速为零时通常不会有限时间自然清除。不要为了“清得更快”直接换成指数模式。

历史只存在内存中，不读写 H5。时钟倒退、网格切换和检测到定位/位姿跳变时，在下一次成功处理观测时清旧历史；应把此类复位作为定位事件记录。当前没有单独的清历史服务，取消导航也不会清除障碍历史。新插件的投影、历史和视野参数只读，改 YAML 后重启，不依赖在线 `ros2 param set`。

本实现借用了三个 STVL 视锥文件和时间衰减公式，仍采用 MeshNav 的三维体素/命中面记录、顶点投影与事件更新。没有引入 OpenVDB，也没有完整 STVL 观测缓冲、独立更新周期或自由空间/遮挡判定。视锥内缺少回波不能证明物体已移走。

2026-10-09 的原箱体场景修复后有一轮成功到达，但行驶约 **39.8 m、重规划 5 次**。这是有限场景验证，尚不能证明所有复杂场景的路线效率或稳定性；实车前继续保留同场景回归。记录见 [实施状态](mesh_navigation_tutorials/STVL_MESHNAV_IMPLEMENTATION_STATUS.md) 与 `mesh_navigation_tutorials/validation/stvl_box_20261009/`。

## 8. 真机配置与启动模板

### 8.1 不复用仿真总入口

不要把 `meshnav_pb_sim.launch.py` 直接作为真机入口。它还会涉及 Gazebo 资源、真值适配、PB 速度来源切换及仿真 preflight；`start_sim=False` 也不是完整真机模式。`pb_preflight` 当前检查 `/clock` 和仿真速度适配器，`pb_nav_goal` 也强制启用仿真时钟，不能直接用作实车验收/发目标工具。实车使用 Mesh Goal 或发布符合接口的 PoseStamped，并核查协调节点状态。

通用 `mbf_mesh_navigation_server_launch.py` 当前只支持地图与 H5 路径覆盖，固定读取仿真 YAML，**没有 `params_file` 参数**。下面直接创建 MBF Node、先加载基础配置再加载实车覆盖，避免虚构已有实车入口。

保留原仿真配置，建议现场创建：

```text
mesh_navigation_tutorials/real/
  meshnav_real.yaml
  pcl_real.yaml
  meshnav_real.launch.py
```

### 8.2 配置模板

先完成第 6.1 节的 PCL 原点修正/QoS 接入并重新构建，再使用这些模板进行无电机检查。下列 `__...__` 必须替换为现场数值或真实帧名。路径示例指向第 4 节生成的地图。
原 YAML 作为基础配置，后加载此文件覆盖真机差异；每项参数的最终值以 ROS 回读为准。
数值占位符替换后填写浮点数（例如 `1.0`），不要留下字符串；此模板未填写前不可用于启动。

```yaml
move_base_flex:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_frame: base_footprint
    odom_topic: /odom
    planners: [mesh_planner]
    controllers: [mesh_controller]
    planner_frequency: 2.0
    controller_frequency: 20.0
    path_check_step: 0.025
    path_check_max_distance: 0.4  # 首先核对导航参考点，之后按投影误差验收。
    mesh_planner:
      cost_limit: 0.99
      step_width: 0.10
    mesh_controller:
      type: mesh_controller/MeshController
      holonomic: true
      max_lin_velocity: 0.15     # 首测限速，还需底盘侧限制加速度与失联停车。
      max_ang_velocity: 0.20
    mesh_map:
      mesh_file: /home/rainple/nav_test/field/real/site01_v1/mesh/site01_nav.ply
      mesh_working_file: /home/rainple/nav_test/mesh_navigation_tutorials/site01_v1_real.h5
      default_layer: final
      height_diff:
        threshold: __实测验收的高度差阈值m__
      static_inflation:
        inscribed_radius: __静态硬间距m__
        inflation_radius: __静态软代价外圈m__
        inscribed_value: 1.0
      obstacle:
        type: mesh_layers/TemporalObstacleLayer
        topic: /dynamic_obstacles
        qos: Reliable           # 当前 PCL 差分输出为 Reliable。
        tf_tolerance: 0.10
        robot_height: __多视角测试确定的投影高度m__
        max_obstacle_dist: __实测有效障碍检测距离m__
        axis_frame: map         # 前提：map 的 Z 与重力方向对齐。
        down_axis: [0.0, 0.0, -1.0]
        history_voxel_size: 0.05
        clearing_mode: stvl
        decay_model: 0
        obstacle_keep_time: __实测确定的障碍保留时间s__
        decay_acceleration: 0.0
        max_observation_age: 0.30
        sensor_frame: __真实雷达帧名__
        # 本模板假定 TF 已描述真实测量原点；PCL 硬编码修正需另外处理。
        sensor_offset_xyz: [0.0, 0.0, 0.0]
        min_sensor_range: __雷达有效近距m__
        horizontal_fov_deg: __实测水平视野deg__
        vertical_fov_min_deg: __实测垂直视野下限deg__
        vertical_fov_max_deg: __实测垂直视野上限deg__
        localization_reference_frame: odom
        pose_jump_translation: 0.75
        pose_jump_rotation_deg: 45.0
        pose_jump_max_interval: 0.20
      obstacle_inflation:
        inscribed_radius: __动态硬间距m__
        inflation_radius: __动态软代价外圈m__
        inscribed_value: 1.0

meshnav_navigator:
  ros__parameters:
    use_sim_time: false
    goal_topic: /rviz/goal_pose
    planner: mesh_planner
    controller: mesh_controller
    planner_frequency: 2.0       # 独立节点，需要显式设置，和上方保持一致。
    get_path_action: /move_base_flex/get_path
    exe_path_action: /move_base_flex/exe_path
    check_path_service: /move_base_flex/check_path_cost
    pose_timeout: 2.0           # 协调节点反馈超时，不等于传感器安全超时。
    path_progress_window: 2.0
    action_timeout: 15.0
    replan_patience: 15.0        # 受阻起的总等待时限，含取消等待，按墙钟计。
    replan_retry_period: 0.5

rviz2_meshnav_real:
  ros__parameters:
    use_sim_time: false
```

控制器增益等未列项仍继承基础 YAML；因此还需审查基础文件的完整参数，不能只检查这段覆盖。不要同时再加载另一份包含同名参数的未知覆盖文件。

另存 `real/pcl_real.yaml`；这是完整的 PCL 参数文件，不同时加载含 `use_sim_time: true` 的仿真 PCL YAML。这里假定里程计和真实 URDF 已提供 TF：

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    base_frame: base_footprint
    lidar_frame: __真实雷达帧名__

obstacle_sub:
  ros__parameters:
    input_cloud_topic: /cloud
    output_cloud_topic: /dynamic_obstacles
    pcd_file_path: /home/rainple/nav_test/field/real/site01_v1/raw/site01_background_map.pcd
    odom_frame: map             # 该 PCD 的 XYZ 已实际配准到 map。
    distance_threshold: __实测背景匹配容差m__
    robot_radius: __实测自身点过滤半径m__

cloud_tf_publisher:
  ros__parameters:
    odom_topic: /odom
    need_odom_tf: false
    need_static_tf: false
    # 两个开关关闭时不广播以下外参；仍要满足节点必填参数类型。
    # 若将来开启静态 TF，先填写标定值并去掉其他同边发布者。
    lidar_xyz: [0.0, 0.0, 0.0]
    lidar_rpy: [0.0, 0.0, 0.0]
```

`site01_background_map.pcd` 是另外整理并配准的固定背景；只复制为这个文件名不会完成配准。确认它、导航 PLY、定位地图和在线 TF 属于同一 map。真实底盘不支持 vy 时将 `holonomic` 设 false，并重新验证受限运动学与路径执行。

### 8.3 简单 launch 模板

保存为 `real/meshnav_real.launch.py`。此入口只启动导航、PCL 和 RViz；FAST-LIVO2、定位适配、真实 URDF、硬件驱动及速度看门狗由现场部署提供。模板尚未创建、占位符未填写或第 6.1 节待修改项未处理时，不能把它当作已验收的实车入口。

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mesh_share = get_package_share_directory('mesh_navigation_tutorials')
    pb_share = get_package_share_directory('pb_vehicle_adapter')
    base_yaml = os.path.join(mesh_share, 'config', 'mbf_mesh_nav.yaml')
    real_yaml = os.path.join(os.path.dirname(__file__), 'meshnav_real.yaml')
    pcl_yaml = os.path.join(os.path.dirname(__file__), 'pcl_real.yaml')

    return LaunchDescription([
        Node(
            package='mbf_mesh_nav', executable='mbf_mesh_nav',
            name='move_base_flex', output='screen',
            parameters=[base_yaml, real_yaml],
            remappings=[('/move_base_flex/cmd_vel', '/nav/cmd_vel')],
        ),
        Node(
            package='pcl_obstacle_point_subtraction',
            executable='pcl_obstacle_point_subtraction',
            name='obstacle_sub', output='screen', parameters=[pcl_yaml],
        ),
        Node(
            package='pcl_obstacle_point_subtraction', executable='cloud_tf_publisher',
            name='cloud_tf_publisher', output='screen', parameters=[pcl_yaml],
        ),
        Node(
            package='pb_vehicle_adapter', executable='meshnav_navigator',
            name='meshnav_navigator', output='screen',
            parameters=[base_yaml, real_yaml],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_meshnav_real',
            output='screen', parameters=[real_yaml],
            arguments=['-d', os.path.join(pb_share, 'rviz', 'pb_meshnav.rviz')],
        ),
    ])
```

使用 PB 包中的协调节点不代表需要使用 PB 的仿真底盘。RViz 配置可作为起点，真机外观和 TF 应加载真实 URDF，不使用仿真车辆尺寸作碰撞依据。

### 8.4 构建与逐终端启动

先整理第 0.2 节失效的依赖和 entry point。已有完整工作空间时，可在新终端只加载 ROS underlay 后，构建目标包及其工作空间依赖：

```bash
source /opt/ros/humble/setup.bash
cd /home/rainple/nav_test/mesh_navigation_tutorials
colcon build --symlink-install \
  --packages-up-to pb_vehicle_adapter pcl_obstacle_point_subtraction
source install/setup.bash
ros2 pkg executables mbf_mesh_nav
ros2 pkg executables pb_vehicle_adapter
ros2 pkg executables pcl_obstacle_point_subtraction
ros2 launch pcl_obstacle_point_subtraction pcl_obstacle_point_subtraction.launch.py --show-args
```

选择 RMCL 分割或定位时，再把 `rmcl_ros` 加入 `--packages-up-to` 并检查其 executables。首次更新过 `AbstractLayer` 虚析构的旧构建目录还需重编译 `mesh_map` 及全部下游，见 [STVL 实施状态](mesh_navigation_tutorials/STVL_MESHNAV_IMPLEMENTATION_STATUS.md)，不能只替换新层的 `.so`。

全新上位机从源码构建并安装实际依赖，不能直接复制旧机器的 build/install。`source_dependencies.yaml` 中上游分支名也不能代替当前已修改源码的版本锁定；保留本仓库 commit 和实际源码。当前 PB/教程包仍带仿真依赖，保留仿真阶段先保留这些包；后续若要做纯实车安装，再拆出共享协调节点和运行依赖。

**重建后先回归仿真，再接实车。** 不因指南模板语法通过就跳过第 6.1 节的代码适配、接口检查与 R0～R4。需要清理工作空间缓存时，停机、保存地图/记录，在新终端仅加载 ROS，然后将工作空间自己的 build/install/log 一起归档再重建；不要触碰 PB 插件的第三方 build。

每个 ROS 2 终端加载同一环境、同一 ROS_DOMAIN_ID 和已验证的 RMW 配置。按顺序启动：

| 终端 | 启动内容 | 在继续前确认 |
|---|---|---|
| A | 真正的雷达、IMU、相机驱动 | 频率、时间戳、字段、QoS |
| B | 现场 FAST-LIVO2 启动命令 | 初始化稳定，里程计有效 |
| C | 旧地图定位、TF/里程计/点云适配 | map 对齐；`/cloud`、`/odom` 符合本文接口 |
| D | 底盘驱动、命令仲裁和看门狗 | 保持禁能或手动模式，不自动接受导航 |
| E | 以下导航 launch（含 PCL） | 固定背景非零、节点/地图/参数正确、动态代价正常 |
| F | 记录 bag 和诊断信息 | 记录开始后再使能低速测试 |

A～D 的可执行命令取决于尚未提供的真机驱动和 FAST-LIVO2 版本，必须在现场运行记录中补齐。E 在模板已创建、占位符已填写后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash
ros2 launch /home/rainple/nav_test/mesh_navigation_tutorials/real/meshnav_real.launch.py
```

参数和连接核验：

```bash
ros2 param get /move_base_flex use_sim_time
ros2 param get /meshnav_navigator use_sim_time
ros2 param get /obstacle_sub use_sim_time
ros2 param get /cloud_tf_publisher use_sim_time
ros2 param get /move_base_flex mesh_map.mesh_file
ros2 param get /move_base_flex mesh_map.mesh_working_file
ros2 param get /move_base_flex mesh_map.obstacle.robot_height
ros2 param get /move_base_flex mesh_map.obstacle.type
ros2 param get /move_base_flex mesh_map.obstacle.topic
ros2 param get /move_base_flex mesh_map.obstacle.clearing_mode
ros2 param get /move_base_flex mesh_map.obstacle.obstacle_keep_time
ros2 param get /move_base_flex mesh_map.obstacle.sensor_frame
ros2 param get /move_base_flex mesh_map.obstacle.sensor_offset_xyz
ros2 param get /obstacle_sub pcd_file_path
ros2 param get /obstacle_sub odom_frame
ros2 param get /obstacle_sub lidar_frame
ros2 param get /meshnav_navigator planner_frequency
ros2 param get /meshnav_navigator replan_patience
ros2 param get /meshnav_navigator replan_retry_period
ros2 topic info /cloud -v
ros2 topic info /dynamic_obstacles -v
ros2 topic info /nav/cmd_vel -v
ros2 service list
ros2 action list -t
```

真机这些节点和定位/驱动/RViz 都应为 `use_sim_time: false`，不需要伪造 `/clock`。`/nav/cmd_vel` 应显示 TwistStamped，订阅者应是经过接口验收的速度仲裁/底盘适配节点；若底盘收 Twist，另外核对转换输出。默认模板使用 PB RViz，可先作为显示起点；将点云显示 topic 改成 `/dynamic_obstacles`，RobotModel 接到真实 robot_description，不能沿用 PB 仿真外观当车体尺寸依据。

## 9. 从静止到封堵隧道的测试顺序

每个阶段先设定测量门槛并记录结果；不把“action 成功”单独作为通过标准。

| 阶段 | 操作 | 通过依据 | 失败时优先检查 |
|---|---|---|---|
| R0 硬件接口 | 在受控区域分别给小 vx、vy、wz，测试禁能与失联停止 | 方向正确；真实速度正确；停止距离/时间有记录 | 底盘坐标、单位、限幅、看门狗 |
| R1 静止定位 | 停在几个实测点，旋转并观察地图重合 | 位姿/点云不随转向偏移；误差低于分配的安全余量 | 外参、时间同步、杆臂、重定位 |
| R2 手动巡场 | 纯遥控走坡道、隧道和多层区域 | TF 连续、Z 层正确、没有点云拖影 | 定位几何退化、点云畸变、地图坐标 |
| R3 障碍感知与历史 | 先空场对背景，再多距离/方位放箱体；遮挡、移走、断流分别测 | 原始点→差异点→obstacle→final 正常；遮挡保持、有效空帧推进过期、断流不误清 | 配准、原点偏移、QoS、时间戳、过滤、投影高度与衰减 |
| R4 只规划 | 底盘禁能，测试跨场地/双隧道 GetPath | 每条完整路径在正确层，车体包络可通过 | 网格连通、缓存、静态代价 |
| R5 短程闭环 | 平地直行、横移、转弯、到点 | 实测误差/间隙合格，无异常停车；旧路畅通时 plans 不增加 | 控制器、实际速度反馈、定位与执行状态 |
| R6 长程无障碍 | 左上→右下，再反向走 | 完整到达，记录所有停车和换路原因 | 向量场、路径回溯、真实代价变化 |
| R7 出发前堵路 | 先封住一条隧道，再发目标 | 另一通道规划完整并实际通过 | 隧道几何、可通行代价、定位 |
| R8 行驶中堵路 | 在实测制动范围之外封住原路线 | 阻断后取消旧执行、实车停止、从当前位姿换路；短暂无路在时限内重试，超时仍保持停止 | 端到端延迟、旧路检查、取消、底盘停止、等待状态与路径接替 |
| R9 取消与故障 | 取消目标，分别模拟命令/传感器/定位中断 | 实车在约定时间/距离内停止，恢复后不擅自执行旧目标 | 底盘独立保护与恢复流程 |

R8 应使用可移动软障碍，记录其摆放时间和位置，避免人员站在机器人制动范围内。先做低速单障碍，再增加距离、转弯与遮挡复杂度。

### 9.1 发送目标与停止

使用模板 RViz 中的 **Mesh Goal**，目标落在正确层的可行驶面；不要用直接调用 GetPath→ExePath 的旧面板验证重规划，因为它绕过协调节点。

```bash
ros2 topic echo /meshnav_navigator/status
ros2 service call /meshnav_navigator/cancel std_srvs/srv/Trigger '{}'
```

取消服务返回成功仅表示请求已接收，还应观察状态、最终底盘命令和实测速度停止。硬件急停不依赖该服务。

当前 `plans` 是已采用并下发的执行路径计数，不计每次检查和失败规划。FREE 时继续旧路；LETHAL/OUTSIDE 时取消旧执行，等旧 action 结束再从当前 TF 位姿请求新路径。当前代码不再采用“边走旧路边拿候选，得到候选后复查旧路”的流程，也没有通过轮速确认物理停稳。

| 状态 | 当前含义 |
|---|---|
| `planning` | 首次请求路径 |
| `following` | 已下发路径执行请求；接收成功后跟随并周期检查，不单凭状态证明车辆已运动 |
| `replanning` | 旧路受阻，取消旧执行或正在请求新路径 |
| `waiting_for_path` | 受阻重规划遇到 NO_PATH_FOUND/BLOCKED_START/BLOCKED_GOAL，等待下一次重试 |
| `failed` | 超时或不可恢复错误；须结合 outcome、日志和底盘停止情况排查 |
| `cancel_requested` / `canceled` | 用户取消请求 / 导航取消结束 |
| `succeeded` | 执行返回到达成功，仍需验收真实目标误差 |

当前受阻重规划每 0.5 s 墙钟重试，受阻起最多等待 15 s，含取消等待；初次规划失败和其他错误仍直接失败。重试时限不是实车停车超时，底盘必须更早依靠自身保护停止。若代价持续反复阻断路径，仍可能多次重规划，须一起记录障碍历史、定位和控制器事件。

### 9.2 预先写下验收门槛

至少为以下指标填写允许值，并说明测量方法：

- 静止与运动定位误差、重定位跳变、点云与地图对齐误差。
- 检测到 final、final 到新指令、总反应时间的 P50/P95/最大值。
- 前进/倒退/横移/转弯及坡道的停车时间与距离。
- 最小真实车体间隙、是否接触、目标误差、停车次数和重规划次数。
- 障碍持续可见时的漏检比例，以及各朝向的盲区。

地图误差、定位误差和执行误差总量应小于配置的实际余量。门槛不是本文能替未知机器人给出的固定常数，必须由车辆尺寸、场地通道及制动测量共同确定。

## 10. 数据记录：用结果区分环境和代码问题

建议每轮单独目录：

```bash
cd /home/rainple/nav_test/mesh_navigation_tutorials
REAL_RUN_DIR="$PWD/diagnostics/real_site01/$(date +%Y%m%d_%H%M%S)_R5"
mkdir -p "$REAL_RUN_DIR"
export REAL_RUN_DIR
ros2 param dump /move_base_flex > "$REAL_RUN_DIR/move_base_flex.yaml"
ros2 param dump /meshnav_navigator > "$REAL_RUN_DIR/navigator.yaml"
ros2 param dump /obstacle_sub > "$REAL_RUN_DIR/obstacle_sub.yaml"
ros2 param dump /cloud_tf_publisher > "$REAL_RUN_DIR/cloud_tf_publisher.yaml"
ros2 node list > "$REAL_RUN_DIR/nodes.txt"
ros2 topic list -t > "$REAL_RUN_DIR/topics.txt"
git rev-parse HEAD > "$REAL_RUN_DIR/project_commit.txt"
sha256sum /home/rainple/nav_test/field/real/site01_v1/mesh/site01_nav.ply \
  > "$REAL_RUN_DIR/map_sha256.txt"
sha256sum /home/rainple/nav_test/field/real/site01_v1/raw/site01_background_map.pcd \
  > "$REAL_RUN_DIR/background_sha256.txt"
```

ROS 2 bag 基础记录示例：

```bash
ros2 bag record --include-hidden-topics -o "$REAL_RUN_DIR/bag" \
  /tf /tf_static /odom /cloud /dynamic_obstacles \
  /nav/cmd_vel /rviz/goal_pose /meshnav_navigator/status \
  /move_base_flex/mesh /move_base_flex/path \
  /move_base_flex/vertex_costs /move_base_flex/vertex_costs/updates \
  /move_base_flex/get_path/_action/status \
  /move_base_flex/exe_path/_action/status \
  /move_base_flex/exe_path/_action/feedback /rosout
```

另加入实际驱动的原始点云、IMU、图像（若需分析视觉）、FAST-LIVO2 原始里程计、定位质量、底盘最终命令、实测轮速/舵角和急停状态话题。上述命令没有猜测这些现场话题名，必须根据实际系统补齐。

选择 RMCL 分支时，再记录 `/rmcl_inputs/cloud`、`/obstacle_points` 并 dump 转换器与分割节点参数。每轮保存完整 launch、实车覆盖文件、TF 发布者分工和第 6.1 节原点修正的实际代码版本。

注意：

- QoS 不匹配可能让 bag 缺数据；尤其 `/tf_static` 和一次性网格/完整代价消息，确认记录器收到初始快照，必要时使用 rosbag2 QoS override。记录结束后用 `ros2 bag info` 核对各话题数量。
- 仅有 action status/feedback 不包含完整 GetPath 请求/响应或所有 ExePath 下发路径；保留节点日志、目标事件和成功路径快照。若分析控制器向量场失败，还应按实际话题清单记录向量场。
- 用外部视频/距离测量标注障碍摆放、接触、到点、急停事件。不能只凭轮式里程计证明“没有碰撞”。
- 对 `/dynamic_obstacles` 与代价更新可利用保留的消息时间戳匹配；TF、规划、驱动未必沿用同一 stamp，跨阶段还需接收时刻或专门时间日志。不要把不同含义的 header.stamp 直接相减。
- 真机主要看 CPU/GPU 占用、调度、网络延迟和消息新鲜度，不再用 Gazebo RTF 衡量运行速度。
- 回放时隔离真实底盘。允许回放节点使用仿真时钟，不代表在线真机也应开启 `use_sim_time`。

每轮填写 `notes.md`：地图/软件/参数版本、起终点、障碍尺寸与位置、执行顺序、实际偏离、结果、最小间隙、所有停车原因。失败时先保存现场，避免立即调参覆盖证据。

## 11. 故障定位速查

| 现象 | 第一检查点 | 更可能属于 |
|---|---|---|
| PCD 能显示，PLY 只有几小块 | 地面覆盖、分辨率、法向、连通分量筛选 | 地图输入/转换 |
| 新开机后所有路线整体偏移 | 旧地图定位、T_map_slam、初始位姿 | 定位接入 |
| 原地转向时点云绕车“甩动” | 外参杆臂、时间戳、畸变、重复 TF | 标定/接口 |
| 驱动点云正常，PCL/RMCL 无输入 | Reliable/BestEffort、消息类型、配置 lidar_frame 与真实 XYZ | QoS/适配 |
| 空场中大量固定结构成为差异点 | 背景点数/坐标、map 对齐、时间同步、PCL 3 cm 修正 | 背景配准/外参/定位 |
| 离地图原点越远越难检测障碍 | 是否回退传统 ObstacleLayer，或真实 sensor TF/原点错误 | 距离过滤语义 |
| 障碍点存在但 obstacle 不更新 | 点云年龄、超前/重复/乱序、sensor/reference TF、投影高度与网格 | 新鲜度/坐标/投影 |
| obstacle 有代价但 final 无影响 | 输入层关系、default_layer、阈值、缓存 | 代价配置 |
| 障碍在不同视角反复消失 | clearing_mode、keep、加速、雷达覆盖、自身过滤、定位/时钟跳变事件 | 感知/衰减/定位 |
| 箱体移走仍阻路约 10 s | 最后检测时间、线性 keep、是否有有效新帧 | 当前时间模型的正常残留，需实测权衡 |
| 断流后一直保留旧障碍 | 是否只有超龄帧/TF失败、未收到有效新帧 | 当前保持历史行为；同时核验底盘已停止 |
| 路径到隧道口断开 | GetPath 结果码、地图连通、回溯、是否显示失败后的部分路径 | 地图/规划实现 |
| 旧路未阻断却切换 | check_state、blocked_segment、实际 final、是否有其他 goal 客户端 | 协调/接口 |
| 控制器没有速度，向量场访问失败 | 当前 face、向量场有效范围、旧/新执行的日志 | 控制器/规划衔接 |
| 机器人进入 waiting_for_path | outcome、当前起点/目标是否受阻、剩余重试时限 | 停车重规划等待，不是仍沿旧路行驶 |
| 有导航速度但实车不走或横移方向错 | 仲裁、驱动使能、单位、vy 支持、里程计 | 底盘接入 |
| 雷达/定位掉线后继续运动 | 独立新鲜度看门狗、底盘命令失联停车 | 尚未补齐的实车保护 |

已有长路线测试出现过 MeshController 在当前 face 取不到向量场并返回失败；旧路保留修复并没有同时修复这个问题。真实场地若出现同样日志，应保留现场的 pose、path、mesh、final 和向量场，不能仅归因于 FAST-LIVO2 或实车参数。

## 12. 建议的第一轮交付

第一轮先完成 R0～R5，然后提交：

1. 本文 1.1 的信息表，以及实际 A～D 启动命令。
2. FAST-LIVO2 原始 bag、PCD、标定配置和地图变换矩阵。
3. PLY、转换报告、关键通道尺寸、无电机 GetPath 结果。
4. TF 拓扑、各关键话题类型/QoS、完整运行时参数。
5. 背景配准/空场差分、四向障碍、遮挡/移走/断流的分层代价结果和低速停车测量。
6. 一轮短程全向导航的 bag、日志、视频和事件记录。

这些数据通过后再做 R6～R9。这样若真机仍卡死或换路异常，可以明确追踪到定位、感知、代价、规划、控制或底盘中的具体环节。

## 13. 本地依据

- [点云转换流程及限制](field/README.md)
- [PCD 转换器](field/pcd_to_nav_mesh.py)
- [几何预检工具](field/check_nav_mesh_route.py)
- [当前导航参数](mesh_navigation_tutorials/src/mesh_navigation_tutorials/config/mbf_mesh_nav.yaml)
- [PB 仿真入口](mesh_navigation_tutorials/src/pb_vehicle_adapter/launch/meshnav_pb_sim.launch.py)
- [PB 仿真资源与插件路径](mesh_navigation_tutorials/src/pb_vehicle_adapter/config/pb_vehicle_profile.yaml)
- [PCL 仿真参数](mesh_navigation_tutorials/src/pcl_obstacle_point_subtraction/config/pcl_obstacle_point_subtraction.yaml)
- [PCL 参数文件入口](mesh_navigation_tutorials/src/pcl_obstacle_point_subtraction/launch/pcl_obstacle_point_subtraction.launch.py)
- [PCL 差分与仿真原点修正](mesh_navigation_tutorials/src/pcl_obstacle_point_subtraction/src/pcl_obstacle_point_subtraction.cpp)
- [PCL TF 发布开关](mesh_navigation_tutorials/src/pcl_obstacle_point_subtraction/src/cloud_tf_publisher.cpp)
- [RMCL 启动入口](mesh_navigation_tutorials/src/mesh_navigation_tutorials/launch/rmcl_launch.py)
- [点云转换器源码](mesh_navigation_tutorials/src/rmcl/rmcl_ros/src/nodes/conversion/pc2_to_o1dn.cpp)
- [障碍投影源码](mesh_navigation_tutorials/src/mesh_navigation/mesh_layers/src/obstacle_layer.cpp)
- [时间衰减与历史源码](mesh_navigation_tutorials/src/mesh_navigation/mesh_layers/src/temporal_obstacle_layer.cpp)
- [三个上游文件的来源、许可证与适配差异](mesh_navigation_tutorials/src/mesh_navigation/mesh_layers/src/stvl_frustum/UPSTREAM.md)
- [STVL 实施、测试与箱体绕行结果](mesh_navigation_tutorials/STVL_MESHNAV_IMPLEMENTATION_STATUS.md)
- [MBF 原始 TwistStamped 发布](mesh_navigation_tutorials/src/move_base_flex/mbf_abstract_nav/src/abstract_navigation_server.cpp)
- [重规划协调节点](mesh_navigation_tutorials/src/pb_vehicle_adapter/pb_vehicle_adapter/meshnav_navigator.py)
- [路线检查服务](mesh_navigation_tutorials/src/mesh_navigation/mbf_mesh_nav/src/mesh_navigation_server.cpp)
