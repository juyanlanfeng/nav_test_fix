# PB 车辆 + RMUC2026：三导航框架接入、仿真修复与验收方案

日期：2026-09-08。框架：MeshNav、JIE、DDDMR。

> **范围变更（2026-09-15，用户决定）：DDDMR 不再要求适配与验收。** 本任务收口为
> **MeshNav + JIE 双框架**；DDDMR 相关产物（`dddmr_rmuc2026.yaml`、`pb_dddmr.launch.py`、
> `pb_dddmr_map_publisher`、地图生成器、第三速度源 `dddmr`）保留可用、可回退，但不再投入
> 闭环验收，相关结论见第 17.4 节（记录保留，避免以后重复踩坑）。

**本文是实施任务书，只新增文档，不代表代码已修改或闭环已通过。** 与 [PB 车辆接入方案](PB2025_VEHICLE_INTEGRATION_PLAN.md) 配套；本文重点补充第三套导航和完整运行验收。继续使用 PB 简化全向施力模型，不要求真实舵轮动力学。

**修订说明：第 11～16 节补齐从实施到直接运行的交付要求。此前只有建议包名和启动示例，不足以作为“照做即可直接运行”的保证。实施者必须同时交付地图产物、完整配置、启动入口、检查程序和实跑记录，不能只创建同名 launch 即宣告完成。文档无法替代代码实现和运行验证。**

## 1. 当前状态与完成标准

| 项目 | 当前证据 | 缺口 |
| --- | --- | --- |
| PB 车辆适配 | 已有模型、传感器桥、真值 TF、速度适配；7 个测试独立复跑通过 | Gazebo/RViz 实际运行、传感器对齐和导航闭环 |
| MeshNav | 已有 `pb_meshnav.launch.py`，补入 MBF 面板 | 目标拾取、action 执行、到达和取消实测 |
| JIE | 已有 `pb_jie.launch.py`，补入点击选择器 | 三维起终点选择、规划、执行及停车实测 |
| DDDMR | 已独立核实 24 包成功构建记录；运行环境脚本和启动结果由用户提供 | RMUC 地图接口、PB 速度输入、专用配置/入口、完整导航 |

完成标准不是“节点没退出”：同一套场地和 PB 模型下，三个框架分别可以选择目标、生成路径、执行到达、取消停车，并完成平地、坡道、隧道用例；失败必须可解释并留有记录。

不承诺“所有仿真错误都消失”。将每个已发现问题关联到复现条件、修改和回归用例，全部必测用例通过才签收。

## 2. 统一架构：一套仿真，三选一导航

```text
RMUC2026 world + PB robot（唯一实例）
  ├─ /cloud /scan /imu/data /camera/* /joint_states
  ├─ 真值适配 → /odom、map→odom→base_footprint→车辆/传感器
  └─ /robot/cmd_vel ← Gazebo 桥 ← /pb/cmd_vel_safe
                                      ↑
                      单一速度选择器 + 稳态时钟看门狗
                         ↑          ↑          ↑
                      MeshNav      JIE       DDDMR
```

不要同时启动三个完整 launch，它们若各自包含仿真，会生成重复世界、车辆、`/clock` 和 TF。

拟为三个组合入口增加 `start_sim:=true/false`：默认 true 用于独立启动；共同仿真已运行时设 false，只启动导航。导航切换必须先取消当前 action、停用当前控制器、确认速度为零，再启动另一框架。第一版不要求无缝热切换。

DDDMR 的隔离 PCL/GTSAM 环境只用于 DDDMR 终端；不要在启动 Gazebo、MeshNav/RViz 的终端无条件加载其动态库路径。公共通信通过 ROS 完成，无需把不同版本 PCL 的组件加载到同一个进程。

## 3. 需要修改和新增的文件

以下均相对 `/home/rainple/nav_test`。新增名称为建议接口，当前不一定存在。

| 位置 | 工作 |
| --- | --- |
| `meshnav_demo_ws/src/pb_vehicle_adapter/pb_vehicle_adapter/cmd_vel_adapter.py` | 增加第三来源 `dddmr` 和独立订阅；沿用时间戳、有限值、frame 检查及稳态超时停车 |
| 同包 `launch/pb_vehicle_sim.launch.py` | 扩展控制源 choices，保持独立启动；核对 world 参数与桥配置一致 |
| 同包 `launch/pb_meshnav.launch.py`、`pb_jie.launch.py` | 增加可关闭基础仿真的分支，分别使用导航专用 RViz 配置 |
| 同包 `launch/pb_dddmr.launch.py`（新增） | 仅组合 DDDMR 必要导航节点，支持 `start_sim`；不启动另一定位链 |
| 同包 `config/dddmr_rmuc2026.yaml`（新增） | 三维地图话题、全向轨迹、评分器、车辆尺寸、传感器外参和仿真时间 |
| 同包 `config/pb_vehicle_profile.yaml` | 作为三框架共用尺寸来源；明确测量值、近似值和单位 |
| 同包 `rviz/pb_meshnav.rviz`、`pb_jie.rviz`、`pb_dddmr.rviz`（新增/拆分） | 显示全局地图、车、点云、路径；绑定各自目标与取消入口 |
| 同包 `tools/build_dddmr_maps.*`（新增） | 由已审计场地数据生成 DDDMR 全场表面云与可通行地面云 |
| 同包地图发布模块（新增） | 发布 `mapcloud`、`mapground`，提供就绪状态和地图元数据 |
| 同包 `test/` | 补充第三控制源、切换停车、地图格式、launch 配置与 TF 接口测试 |
| `doc/START_NAVIGATION.md` | 只有实跑通过后才更新为正式三框架使用说明 |

保持上游 DDDMR 工作树干净，配置和适配集中在本地包内。若发现必须修改上游的 bug，使用可追踪的补丁，不直接覆盖源码后宣称原版兼容。

## 4. 先完成 PB 仿真底座验收

### 4.1 可视化与目标输入

不能只加入 `MeshGoal` 工具：它需要可拾取的地图几何。为 MeshNav RViz 增加现有网格显示插件，沿用原教程的 geometry/cost 话题，保留 `MbfGoalActions` 的 `/move_base_flex/get_path`、`/move_base_flex/exe_path` 和 `/rviz/goal_pose`。

JIE 的 Publish Point 必须能点击到对应的三维表面；显示 OctoMap 或选择器支持的地图几何，不能仅靠地面 Grid 猜 z。点击选择器应能显示起终点，随后确认 `/planned_path` 更新；按控制器实际开关发开始/停止命令。选择测试起点应与真实车位一致，不通过修改真实定位假装车辆已在所选位置。

DDDMR 使用其 action 或专用 RViz 工具。不要让三个框架共享一个未区分用途的目标话题而产生多次执行。

### 4.2 TF、点云与时间

保留 PB 真值适配器作为唯一导航根 TF 来源。逐项检查：

- `/clock` 唯一，导航、地图处理、RViz 均 `use_sim_time=true`。
- `map→odom` 和 `odom→base_footprint` 各只有一个发布者；内部 link TF 由 PB 描述发布器负责。
- `/cloud.header.frame_id` 与真实雷达 link 一致，点云不重复变换；坡上 z/roll/pitch 连续。
- PB 桥中世界位姿话题若写死 `/world/rmuc2026_field/dynamic_pose/info`，则 launch 要么限制只支持该世界，要么生成动态桥配置；不能接受其他 world 名却静默失去真值。
- 真值消息中 model pose 与 chassis pose 的合成需检查时间一致性；数据缺失/过旧时报告未就绪，不发布伪造的新鲜定位。
- 相机内参和光学帧一致，雷达订阅 QoS 与桥相容。频率查询成功不等于外参正确，要对照墙、地面、坡面。

### 4.3 速度和停车

现有稳态定时器与时间戳校验已修复，继续补充运行回归：不发布 `/clock`、仿真暂停/恢复、无输入、取消、切换来源、错误 frame、NaN、未来/过期时间戳。

普通关闭适配节点时发送一次零速，不等于进程崩溃时一定停车。PB 上游插件会保持最后目标；若要覆盖适配器异常退出，应增加独立看门狗或插件输入超时。仅做导航测试也应避免失去输入后持续移动。

不能同时运行旧车的速度覆盖插件、PB `MecanumDrive2` 以及 PB 原跟随云台控制节点。第一版固定云台，减少速度参考系歧义。

## 5. DDDMR 地图接入：不能只换 PCD 文件路径

本次核查本地代码发现：

- `src/dddmr_p2p_move_base/launch/p2p_move_base_localization_launch.py` 会启动 LeGO-LOAM 特征节点、MCL、地图服务器和硬编码雷达外参，不能整份直接包含。
- `launch/p2p_wo_mcl.launch` 使用 `global_planner/occupancy2ground`，默认地图是 warehouse PGM，不能用这个二维转换保留隧道上下层和坡道。
- `src/dddmr_pg_map_server/src/dddmr_pg_map_server.cpp` 读取 `poses.pcd` 及 `pcd/N_feature.pcd`、`N_surface.pcd`、`N_ground.pcd`。单个场地 PCD 不是它要求的 pose graph 数据集。
- `src/dddmr_perception_3d/plugins/static_layer.cpp` 支持从 `mapcloud` 和 `mapground` 接收 PointCloud2，因此可增加轻量静态地图发布器，避开本次不需要的 SLAM/pose graph 生成。

### 推荐实现

1. 生成两个米制、map 坐标系文件：`rmuc2026_mapcloud.pcd`（全场几何）和 `rmuc2026_mapground.pcd`（可供全局搜索的地面采样）。使用新目录，不覆盖 JIE 原 PCD。
2. mapcloud 包含墙、隧道顶棚等真实障碍表面；mapground 包含地面、可通过坡面和正确连接关系。不要把全场点云直接复制成 mapground，也不要沿 XY 只保留最高表面。
3. 同一 XY 下有多个高度时保留正确地形层。地面候选需检验坡度、支撑和车体净空；隧道顶面不能错误连接到底面。
4. 点云字段按当前 StaticLayer 消费代码验证，至少检查 XYZ、是否需要 intensity、NaN、空云、点密度、frame_id；intensity 的业务含义不能凭雷达反射率猜测。
5. 发布器在地图加载完成后发布两种 PointCloud2。静态模式 `mapping_mode=false` 时，源码订阅使用 Reliable/Transient Local；发布端也应提供持久化 QoS，使导航晚启动仍能收到地图。
6. 启动全局规划器后验证它确实收到两张云并构建搜索数据，再允许发 goal。地图缺失时不得仅靠节点存活判就绪。

数据流：

```text
已审计场地几何 → 离线采样/地形筛选
                      ├─ mapcloud.pcd → /dddmr/mapcloud : PointCloud2
                      └─ mapground.pcd → /dddmr/mapground : PointCloud2
                                             ↓
                          DDDMR StaticLayer / global_planner
```

话题名 `/dddmr/*` 是拟新增隔离接口，须在 DDDMR 全局和局部感知配置中一起改，不能只重映射一个订阅。

## 6. DDDMR 导航节点、全向控制和实时感知

### 6.1 节点集合

新增 launch 从上游启动逻辑中选择 `global_planner_node`、`p2p_move_base_node` 及它们所需的感知/局部规划组成。注意部分可执行程序内部构造多个节点，不要依据包名再重复启动相同感知实例。

首版不启动 `mcl_3dl`、`mcl_feature`、LeGO-LOAM 定位、硬编码 `base_link→laser_link` 静态 TF，不再发布 map→odom。沿用 PB 的 `/odom` 和真值 TF。

所有参数段的节点名必须以代码实际创建的名称为准。不能给复杂多节点可执行程序随意指定单一 `name=`，导致 YAML 节点段不匹配；通过运行时 `ros2 param dump` 核验。

### 6.2 全向轨迹

上游 `p2p_wo_mcl.yaml` 默认选择差速轨迹，不能只把最终输出话题接到全向车辆就认为使用了全向导航。

按 `src/dddmr_local_planner/trajectory_generators/trajectory_generators.xml` 注册的 `trajectory_generators::OmniSimpleTrajectoryGeneratorTheory` 配置局部轨迹生成器；连同评分器所引用的生成器名称一起调整。核对当前版本参数，设置非零横向采样范围、速度及加速度限制。不要混入 Ackermann 的转向约束。

先用较低速度验证横移、旋转和组合运动，确认评分器不会始终排除横向轨迹。恢复行为也应与全向底盘一致，不把原差速参数全部照搬。

### 6.3 第三速度来源

本地 `src/dddmr_p2p_move_base/src/p2p_move_base.cpp` 根据 `use_twist_stamped` 分支发布 `cmd_vel_stamped` 或 `cmd_vel`。推荐开启 stamped 分支并重映射：

| 框架 | 导航侧输出 | 速度适配输入 |
| --- | --- | --- |
| MeshNav | `/cmd_vel`，TwistStamped | 保持现有 MeshNav 输入 |
| JIE | `/cmd_vel_jie`，Twist → stamper | `/pb/jie_cmd_vel_stamped` |
| DDDMR | `cmd_vel_stamped`，TwistStamped | `/pb/dddmr_cmd_vel_stamped`（新增） |

新增 `control_source=dddmr`，同时更新节点校验、参数切换、三个 launch 的 choices 和测试。检查 DDDMR 实際输出的时间戳和 frame；如果上游未填写符合适配器要求的 header，增加明确的类型/坐标适配，不放松所有输入检查来绕过问题。

### 6.4 目标接口

源码注册 `/p2p_move_base` action，类型 `dddmr_sys_core/action/PToPMoveBase`。可参考 `src/dddmr_p2p_move_base/script/clicked2goal.py`，但要检查它订阅的话题、goal 字段以及取消逻辑后再复用。

通过 `ros2 interface show dddmr_sys_core/action/PToPMoveBase` 确认结构。不要把 Nav2 NavigateToPose 或 MBF GetPath 的 goal YAML 直接套用。必须暴露“规划执行状态”和“取消”入口。

### 6.5 动态障碍

静态地图接通不等于动态避障接通。实时点云 `/cloud` 应进入实际启用的 DDDMR 雷达感知插件，使用 PB 雷达 frame、实际扫描角度、频率和机器人外廓。

北极熊模拟 MID360 的扫描并非真实非重复扫描，配置须以仿真 sensor 为准。若使用 `MultilayerSpinningLidar`，核实层数和角度参数，不沿用其他雷达配置；激光自体滤除不能把坡面或隧道顶棚全部删掉。

分别测试障碍出现时减速/停止/绕行、移除后清除，不把雷达空数据当作无障碍。先静态闭环通过再启用动态层，方便定位问题。

## 7. 运行环境与构建

普通 PB / MeshNav / JIE 终端：

```bash
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
```

DDDMR 终端额外显式加载：

```bash
source /home/rainple/nav_test/third_party/setup_dddmr_env.sh
```

43 是隔离测试域示例，所有相关终端一致即可，不能与运行中的另一个测试混用。若需要并行 Gazebo 实例，还需不同 `IGN_PARTITION`；ROS 域本身不隔离 Gazebo Transport。

保持系统 PCL 不变。专用脚本提供 GTSAM/metis、PCL 1.15、small_gicp 的运行库路径。各 DDDMR 包实际 PCL 版本以各自 CMakeCache 为准，不声称所有包都使用 1.15；组件同进程混用不同 PCL 版本时需要额外做库加载/运行检查。

实施后只重建所改适配包：

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
colcon build --packages-select pb_vehicle_adapter --symlink-install
colcon test --packages-select pb_vehicle_adapter
colcon test-result --test-result-base build/pb_vehicle_adapter
```

若新增地图发布器采用独立 C++ 包，需要把它加入构建清单。不要在 `/home/rainple/nav_test` 根目录直接递归构建所有仓库，以免扫描到重复包。修改后重启测试节点，不能只 source 后认为旧进程已经更新。

## 8. 拟定统一启动方式

**以下 `start_sim` 参数及 `pb_dddmr.launch.py` 是待实现接口，不是当前可直接运行的命令。**

模式一：独立测试，三个命令三选一，各自启动仿真：

```bash
ros2 launch pb_vehicle_adapter pb_meshnav.launch.py start_sim:=true
ros2 launch pb_vehicle_adapter pb_jie.launch.py start_sim:=true
ros2 launch pb_vehicle_adapter pb_dddmr.launch.py start_sim:=true
```

JIE 终端还需 source `jie_3d_nav/install/setup.bash`；DDDMR 使用上一节专用环境。

模式二：一套基础仿真依次比较三个框架：

```bash
# 终端 A：只启动一次；现有基础入口已存在。
ros2 launch pb_vehicle_adapter pb_vehicle_sim.launch.py \
  start_gazebo_gui:=True start_rviz:=False

# 终端 B：以下一次只运行一条；参数需实施后才存在。
ros2 launch pb_vehicle_adapter pb_meshnav.launch.py start_sim:=false
ros2 launch pb_vehicle_adapter pb_jie.launch.py start_sim:=false
ros2 launch pb_vehicle_adapter pb_dddmr.launch.py start_sim:=false
```

导航入口应校验/设置速度选择器的来源，并等待切换完成；不能假设基础仿真默认 meshnav 就能接收 DDDMR。切换必须有停车握手；若其他导航器仍在执行，拒绝切换并提示取消。

为三个导航入口各自启动一个适用 RViz，不让基础仿真和导航再各开一个。若用户选择无 GUI 测试，仍保存话题和 action 状态，不能省略验收记录。

## 9. 必测用例及通过条件

| 用例 | 通过条件 |
| --- | --- |
| 基础启动 | 唯一车辆、唯一 clock；无插件/网格缺失；车和地图可见 |
| 点云与 TF | 直墙/地面/坡面重合；车体移动后点云无重复变换；TF 连续 |
| 目标选择 | 三种 RViz 分别触发对应规划器；有路径、状态、取消操作 |
| 平地到达 | 三框架分别完成同一合法起终点，不靠人工持续发速度 |
| 横向运动 | 全向轨迹实际产生并被 PB 执行，不被适配器错误丢弃 |
| 短坡双向 | 新 PB 模型能够通过的几何区域，分别完成上下坡；不能引用旧车结果 |
| 隧道/多层 | 路径不串层，点云顶部不被当作底面，车体尺寸允许时可通过 |
| 取消/超时 | 当前导航取消后输出零；输入断流约 0.5 s 后停车；暂停恢复无过期指令重放 |
| 来源切换 | 未选中的两个来源不能影响最终速度；无重复 TF/桥 |
| 障碍出现/移除 | 仅对实际启用动态感知的框架验收；能响应且能清除，不以显示代替避障 |
| 失败处理 | 无地图、无 TF、不可达目标、超时有明确状态；不会无限顶撞 |

每项记录 commit/未提交 diff、依赖版本、profile、起终点、仿真时间、action 结果。建议相同用例重复三次；三次通过是回归证据，不是稳定性统计保证。

小体积 ROS 记录：

```bash
ros2 bag record /cmd_vel /pb/jie_cmd_vel_stamped /pb/dddmr_cmd_vel_stamped \
  /pb/cmd_vel_safe /odom /tf /tf_static /clock
```

DDDMR 话题是待新增项；录制不会自动创建它。每个用例结束 Ctrl+C，点云/图像仅在分析感知问题时短时录制，避免磁盘占满。

7 个现有单测保留，再固化至少：第三来源屏蔽、切换立即清零、暂停时真实定时器触发、地图 QoS 晚订阅、配置实际加载、三个入口不会重复启动基础仿真。不要把手工改私有变量的单测当成消息链端到端验证。

## 10. 实施顺序与签收

1. 保存用户已有修改；完成 PB 车、点云、TF、停车基础验收。
2. 分别打通 MeshNav 和 JIE 的 RViz 目标到到达闭环；按实际错误修复，不在没有复现时继续堆参数。
3. 为 DDDMR 生成/发布正确的 mapcloud、mapground，先完成全局路径验证。
4. 接入 DDDMR 全向局部规划、第三速度源和取消链路，完成平地闭环。
5. 三框架依次完成坡道、隧道和必要的动态感知测试。
6. 将真实可执行命令、实际话题和通过记录写入正式启动文档；保留旧配置和模型作为回退。

交付报告分为“通过 / 失败 / 未测”，必须逐项列出，不用“都正常”概括。DDDMR 包可启动、PB 单测通过均属于必要条件，而非本任务的终点。

## 11. 补齐后的启动契约：实施者必须实现，用户不再手工拼节点

最终使用方式固定为“公共仿真一个终端，导航一个终端”，优先交付此模式，再提供一键组合启动。禁止启动后要求用户额外手动发布地图、补 TF、修改内部 YAML 或猜测 action 类型。

三个导航入口必须支持相同参数：

| 参数 | 默认/行为 |
| --- | --- |
| `start_sim` | true；false 时不启动 Gazebo、车辆、桥、根 TF、速度适配器 |
| `start_rviz` | true；只开当前框架专用 RViz |
| `world_name` | rmuc2026_field；暂不支持其他世界时遇到其他值明确失败 |
| `use_sim_time` | true；向可执行程序内部的所有节点传入，不仅外层 launch |
| `map_bundle` | 经校验的 RMUC2026 地图清单绝对路径；由适配包默认解析 |
| `vehicle_profile` | PB 尺寸配置文件绝对路径；不能隐藏地使用旧 Ceres 参数 |
| `startup_timeout_s` | 建议 120；超时列出缺失条件并保持零速，而不是盲目开始导航 |

`start_sim=false` 时必须连接现有速度选择器，切换到对应来源并确认成功；不存在选择器就启动失败，不自行再生成第二个。三个入口对已有控制者的竞争采用“拒绝并提示先停止旧框架”，不是后启动者抢占。

新增只读检查程序 `pb_preflight`，接口约定：

```bash
ros2 run pb_vehicle_adapter pb_preflight --framework meshnav
ros2 run pb_vehicle_adapter pb_preflight --framework jie
ros2 run pb_vehicle_adapter pb_preflight --framework dddmr
```

这三个命令目前待实现。程序检查 clock 推进、地图文件与元数据、所需包/插件、TF、点云、速度来源、action/server；成功返回 0，失败返回非零并逐项说明。启动本身可由同一逻辑做就绪门控；不要只靠固定 sleep。

## 12. DDDMR 完整配置的生成规则

不从零猜测数百行插件配置。以当前固定 commit 的 `p2p_wo_mcl.yaml` 为基线，生成新 `dddmr_rmuc2026.yaml`，保留评分器和恢复行为的完整结构，执行以下明确修改。

### 12.1 保持真实节点名称

代码已核实：

| 可执行程序 | 内部节点 |
| --- | --- |
| `global_planner/global_planner_node` | `perception_3d_global`、`global_planner`、`dynamic_window_aware_global_planner` |
| `p2p_move_base/p2p_move_base_node` | `trajectory_generators`、`mpc_critics`、`perception_3d_local`、`recovery_behaviors`、`local_planner`、`global_plan_manager`、`p2p_move_base` |

每个节点的 YAML 段都要有 `ros__parameters`，全都使用仿真时间。启动这两个可执行程序时不统一覆盖其 node name。参数检查应遍历上述节点，不能仅检查 `/p2p_move_base`。

### 12.2 最小配置变更表

| 参数段 | 必改内容 |
| --- | --- |
| `occupancy2ground` | 删除该段及节点，用新增三维地图发布器替代 |
| `p2p_move_base` | `use_twist_stamped: true`；`main_trajectory_generator` 指向启用的全向生成器实例名 |
| `global_plan_manager` | 保留有效的 `global_planner_action_name: get_plan`；运行时核对 action 名 |
| `local_planner` | `/odom`；控制频率与生成器一致；cuboid 八顶点来自 PB profile |
| `perception_3d_local/global` | `global_frame: map`，`robot_base_frame: base_footprint`；StaticLayer 的 map/ground 指向 `/dddmr/mapcloud` 和 `/dddmr/mapground`；静态阶段 `mapping_mode: false` |
| `trajectory_generators` | 主生成器换成 Omni；恢复行为依赖的旋转/单状态生成器保留，不能从插件列表直接删掉 |
| `mpc_critics` | 更新主生成器关联名称，保留碰撞评分器；初期不启用追随距离目标功能 |
| `recovery_behaviors` | 引用的生成器必须存在；速度低限和 cuboid 与主配置一致 |

为减少引用改动，可以保留基线主实例名 `differential_drive_simple`，只把其 `plugin` 改成 `trajectory_generators::OmniSimpleTrajectoryGeneratorTheory`；但需注释该名字是兼容旧引用，不代表差速模型。若改名 `omni_simple`，则全文件逐处更新引用，并测试每个恢复/评分分支。

初始低速值可取：`min_vel_x=-0.2`、`max_vel_x=0.2`、`min_vel_y=-0.2`、`max_vel_y=0.2`、`max_vel_trans=0.2`，角速度范围不超过 ±0.5 rad/s，控制频率 10 Hz。它们是验收起始值，不是实车能力结论；其余必需参数（加速度、采样数、预测时域、粒度）保留基线并按当前 Omni 实现声明类型验证。

cuboid 必须覆盖车身和轮组，不只覆盖底盘箱体。八点按 `flb/frb/flt/frt/blb/brb/blt/brt` 定义，从 PB 导出的碰撞外廓统一生成，不能在多个插件中各自维护一份不同尺寸。

启动后必须保存：

```bash
ros2 param dump /p2p_move_base
ros2 param dump /trajectory_generators
ros2 param dump /perception_3d_global
ros2 param dump /perception_3d_local
```

检查输出值与新文件一致、无 warehouse 路径和旧雷达外参，再进入运动测试。

## 13. 地图产物与发布程序的完整交付契约

地图目录建议为 `field/converted_rmuc2026/dddmr_nav/`，最终必须包含：

```text
dddmr_nav/
  rmuc2026_mapcloud.pcd
  rmuc2026_mapground.pcd
  map_manifest.yaml
  validation_report.json
```

manifest 至少记录：两个文件的绝对/可解析相对路径及 SHA256、frame=map、单位=m、原始来源文件与哈希、转换矩阵、采样间距、PB profile 哈希、地形层保留策略。不得把“文件存在”当作格式和连通性合格。

StaticLayer 当前使用 `pcl::PointXYZI`，所以两个文件和 PointCloud2 应提供 float32 的 `x/y/z/intensity`，坐标有限；初版非语义强度可统一置 0，并注明它不是雷达反射率、也不是地形标签。后续如引入强度代价，应单独定义契约。

生成步骤必须落为可重复脚本：

1. 读当前经过验证的场地几何和转换元数据；拒绝毫米/米或坐标版本不明的输入。
2. 对完整几何采样生成 mapcloud。
3. 对候选承载面采样生成 mapground，保留坡面和隧道下层；计算 PB 外廓净空，禁止跨越顶板/侧壁建立假连接。
4. 对已知短坡和隧道输出局部截面与连通性报告。对难以确认的区域报未验证，不靠插值无条件补洞。
5. 用 DDDMR 自身的全局规划接口复测入口到出口。离线连通不代表 DDDMR 的邻域参数一定能连通，必须两层都测。

地图发布器须在启动时加载一次、校验后发布 Reliable/Transient Local 的两张云；加载失败退出非零，不能发布空云并继续宣告就绪。晚启动的全局与局部感知都应收到。静态 TF map 可用后正常发布时间戳；不将文件路径 String 当作点云发送。

若无法在现有场地数据上生成正确地面云，实施仍未完成：这不是可以留给用户启动后自行处理的可选项，也不能改用二维 PGM 宣称复杂地形已接入。

## 14. 目标、取消和一键验证接口

新增统一测试客户端 `pb_nav_goal`（待实现），为三框架封装各自原生接口，不要求普通用户手写三种 action：

```bash
# 数值是命令格式示例，必须替换成该车辆可通行且已验证的目标。
ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav --x X --y Y --z Z --yaw YAW
ros2 run pb_vehicle_adapter pb_nav_goal --framework jie --x X --y Y --z Z --yaw YAW
ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --x X --y Y --z Z --yaw YAW
ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --cancel
```

上面不是当前可运行命令，也不是要求用户保留字母占位执行。最终交付须提供每框架一个经过验证的 `smoke_goal.yaml`，再支持 `--case smoke`，用户无需猜坐标。

客户端职责：

- MeshNav：调用 MBF GetPath，再在成功且路径非空时调用 ExePath；反馈规划失败、执行失败或到达；取消正在执行的 goal。
- JIE：以实时 TF 生成与真实车位一致的起点，发送起终点及终点朝向，等待本次请求对应的新路径，再按当前 `require_start_command` 约定启动；取消时发停止。必须避免使用上一请求的缓存路径。
- DDDMR：使用 `/p2p_move_base` 的 `dddmr_sys_core/action/PToPMoveBase`。已核实 goal 字段为 `target_pose`、`target_value`，结果为 `status`、`result`。普通到点模式不启用 escort/ranging，`target_value` 按该模式设定并核实，不能套用 Nav2 goal。
- 所有接口使用 map 帧和仿真时间，保存 goal handle、请求编号和结果；超时后取消并检查速度归零。

上游 `clicked2goal.py` 可以作为目标字段参考，但当前实现没有完整取消/结果管理，不能独自承担上述验收客户端职责。

RViz 三套配置与这个客户端能力对应：目标设置、执行状态和停止入口缺一不可。DDDMR action 的 SUCCESS=1 与其他框架枚举不同，报告不可统一用数值 0 表示成功。

## 15. 最终可直接运行的交付顺序

实施者必须将下列步骤实际执行并保存结果，之后才能把文档改成“运行手册”：

1. 构建适配包，三框架包可见；DDDMR 环境脚本下动态库无缺失。
2. 生成并校验地图 bundle，清单路径固定且可从安装包解析。
3. 三个 launch 的 `--show-args` 能执行，无缺失包；参数与第 11 节一致。
4. 在新终端按第 7 节环境启动公共仿真；基础 preflight 通过。
5. 依次执行 MeshNav/JIE/DDDMR 的导航-only launch 和 preflight，执行已提供的 `--case smoke`，等待实际到达，再取消/停止并退出该框架。
6. 执行坡道、隧道用例；完成后保留“通过/不适用/失败”及原因，不将车辆物理放不下误报成地图可达。
7. 最后验证各自 `start_sim=true` 的一键入口，不重复产生车辆/TF/桥。

正式交付时 README 顶部必须给出可以复制的完整环境加载与启动命令，不能再含 `X/Y/Z`、未知地图路径、未创建的包名、需要手动改 YAML 的步骤。未来机器迁移另有路径配置说明，不依靠启动者之前终端中偶然残留的环境。

### 故障定位表

| 现象 | 优先检查 | 不应采取的绕过方式 |
| --- | --- | --- |
| 有车无点云 | Sensors 系统、GPU 渲染、真实 sensor 话题、桥类型、frame、QoS | 只添加 RViz 显示后宣告修复 |
| RViz 点不到地图 | 地图几何 Display、选择工具与话题、三维 z | 在多层场地统一把 z 设为 0 |
| goal 被接受但无路径 | 静态两张云就绪、全局 TF、地图连通、frame/尺寸 | 把 mapcloud 全部标成 mapground |
| 有路径不走 | action 执行状态、选择器来源、stamped header、最终 Twist | 同时启动多个速度发布者试撞 |
| DDDMR 找不到插件/库 | 专用环境、plugin XML、实际 package prefix | 全机升级 PCL 或删除系统库 |
| 重启偶发 TF 错误 | 唯一所有者、旧进程、仿真时间回退与缓存 | 随意加单位静态 TF |
| 取消后还在走 | 是否取消真实执行 goal、零速到 Gazebo、插件最后目标 | 仅清空 RViz 路径 |

## 16. 对“按文档修改后能否直接运行”的明确回答

**按本文完成全部实现、地图生成、环境与接口检查，并完成第 15 节实跑后，应交付可直接启动的三框架仿真。仅按修改清单写完代码，不能保证可运行。**

目前仍不存在文档本身能够证明三框架闭环成功的事实。不得将本次文档完善说成已修复仿真；实际验收是实施的必需步骤，不是交付后再让用户补做的步骤。

## 17. 实跑记录（2026-09-15）

本节只记录**实际执行过**的命令与结果；未执行的项目明确写“未测”。

### 17.1 运行方式

环境脚本 `log/accept_env.sh` 一次性加载**三个工作空间**（缺一个就会出现“包找不到”或“规划器不响应”）：
`/opt/ros/humble`、`meshnav_demo_ws/install`（MeshNav + pb_vehicle_adapter）、
`jie_3d_nav/install`（jie_octomap + octo_planner）、`third_party/setup_dddmr_env.sh`（DDDMR 专用）。
它同时设置 `ROS_DOMAIN_ID=${ACCEPT_DOMAIN:-101}`、`ROS_LOCALHOST_ONLY=1`、`ROS_HOME` 与抬高 DDS 参与者上限的 `CYCLONEDDS_URI`。

**渲染模式自动判定**（`ACCEPT_HEADLESS=auto|1|0`）：本机桌面会话有 RTX 5070 + Xorg，而**执行验收的 agent 沙箱没有 `/dev/dri`、`/dev/nvidia*`、X socket，`nvidia-smi` 也无法连接驱动**，只能用软件渲染。脚本据此自动选择：
- 检测到渲染节点/ X socket → 保留 `DISPLAY` 与 GPU，`ACCEPT_SENSORS=True`、`ACCEPT_GUI=True`（即完整传感器 + GUI）；
- 什么都没有（当前 agent 沙箱）→ `unset DISPLAY` + 软件 EGL，`ACCEPT_SENSORS=False`、`ACCEPT_GUI=False`。

因此 17.3 第 4 行的 0.059 实时率是**沙箱无 GPU 环境**的测量值，不是这台电脑的能力上限；在桌面会话里用默认传感器跑即可，可用 `bash log/probe_rtf2.sh`（sim 秒/墙钟秒）自测。

```bash
source /home/rainple/nav_test/log/accept_env.sh      # 需要新域时：ACCEPT_DOMAIN=120 source ...
ros2 launch pb_vehicle_adapter pb_vehicle_sim.launch.py world_name:=rmuc2026_field \
  start_gazebo_gui:=False start_rviz:=False control_source:=meshnav \
  spawn_rendering_sensors:=False spawn_x:=-11.9 spawn_y:=-4.4 spawn_z:=0.25
ros2 launch pb_vehicle_adapter pb_meshnav.launch.py start_sim:=False start_rviz:=False
ros2 run pb_vehicle_adapter pb_preflight --framework meshnav
ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav --case smoke --timeout 300

# JIE（control_source:=jie；JIE 规划器首次启动要 ~2 min 建可通行图）
ros2 run pb_vehicle_adapter pb_preflight --framework jie
ros2 run pb_vehicle_adapter pb_nav_goal --framework jie --case smoke --timeout 240
```

一键复现脚本：`log/accept_meshnav_run.sh`（MeshNav 两终端模式）、`log/accept_jie_run.sh`（JIE）、
`log/accept_onekey_meshnav.sh`（`start_sim:=True` 一键入口 + 单实例检查）、
`log/accept_dddmr_run.sh`（DDDMR，已退出范围，保留作参考），
单进程控制链探针：`log/probe_dddmr_control.py`（同时抓 `/global_path`、`/pb/dddmr_cmd_vel_stamped`、`/pb/cmd_vel_safe`、TF），
车辆执行能力探针：`log/probe_velocity_chain.py`。

脚本化验收建议 `start_click_selector:=False`（少一个 DDS 参与者），并每次用新域：`ACCEPT_DOMAIN=120 bash log/accept_jie_run.sh`。

### 17.2 已通过（有输出为证）

| 项 | 证据 |
| --- | --- |
| `pb_preflight --framework dddmr` 实机态 7/7 | clock/tf/topics/nodes/actions/velocity-source/map-artifacts 全 PASS，退出码 0 |
| `pb_preflight --framework meshnav` 实机态 6/6 | 同上（无 map-artifacts 项），退出码 0 |
| `pb_preflight --framework jie` 实机态 6/6 | clock/tf/topics(`/planned_path`)/nodes(`jie_path_node`,`d1_controller`)/actions/velocity-source 全 PASS |
| **MeshNav 平地闭环通过** | `GetPath outcome=0 ... points=3` → `ExePath outcome=0 message=Controller succeeded; arrived at goal!`；起点 (-11.900,-4.402)，终点 (-12.561,-3.811)，距目标 (-12.647,-3.698) 0.14 m < 0.3 m 容差；脚本 `log/accept_meshnav_run.sh` |
| **JIE 平地闭环通过** | `Goal snapped to free cell: [-12.70,-3.74,0.10]` → `Received planned_path with 20 poses` → `Navigation execution started` → `Final tracking point reached`；客户端判定 `arrived: 0.291 m from the goal (tolerance 0.30 m)`，终点 (-12.461,-3.922)；脚本 `log/accept_jie_run.sh` |
| **一键入口（`start_sim:=True`）不重复建车** | `log/accept_onekey_meshnav.sh`：`OK creation of entity` ×1；`pb_ground_truth_adapter`/`pb_cmd_vel_adapter`/`pb_ros_gz_bridge`/`pb_robot_state_publisher` 各 1 个、mbf server 1 个；随后 preflight 6/6，且经该入口执行 smoke 仍 `Controller succeeded; arrived at goal!`（终点距目标 0.13 m） |
| DDDMR 静态图与全局规划 | `Static graph is generated with graph size: 189439`；`Path found from: 134845 to 61038`（0.01 s） |
| 全局路径正确且直达 | `/global_path` 前 11 点从 (-11.99,-4.33) 单调走到 (-12.647,-3.698)，方向与目标一致 |
| 动作接口与 goal 生命周期 | `/p2p_move_base` 接受 goal，取消时 `P2P move base cancelled`，速度归零 |
| 速度链端到端 | 探针向 `/pb/dddmr_cmd_vel_stamped` 注入指令，`/pb/cmd_vel_safe` 同步、Gazebo 中整车确实位移 |
| 第三速度源与看门狗 | 适配器只转发选中来源，超时/切换清零（pytest 41 项通过） |
| 就绪门控 | launch 内 `pb_preflight --wait-timeout <startup_timeout_s>` 实际逐项重试并打印缺失清单 |

### 17.3 实跑中发现并修复的缺陷

| # | 现象 | 根因 | 修复 |
| --- | --- | --- | --- |
| 1 | 任意两点都 `No path found`，地图却完全连通 | `mapcloud` 按“完整几何”生成，包含地面本身；StaticLayer 用 `mapcloud` 计算每个 ground 节点的 lethal 距离，于是全部节点 < `inscribed_radius` 被判致命 | 生成器改为“非承载面（墙/顶/陡面）采样 + 剔除低于局部承载面的点”；`mapcloud` 396568 → 93936 点 |
| 2 | `p2p_move_base_node` 启动即崩：`libackermann_msgs__rosidl_typesupport_introspection_cpp.so: cannot open shared object file` | 隔离 apt 前缀的 `lib` 目录不在 `LD_LIBRARY_PATH`，而该库由 dlopen 加载，`ldd` 查不出来 | `third_party/setup_dddmr_env.sh` 增加 `dddmr_apt/opt/ros/humble/lib` 与对应 python 路径 |
| 3 | 有路径也有轨迹，但 `Actuator type is not defined in Commanding Trajectory!`，车辆完全不动 | 上游 06d50cc 从未调用 `configurateActuatorType()`，`actuator_type_` 是未初始化值，`publishVelocity()` 丢弃所有轨迹 | `third_party/patches/dddmr-06d50cc-actuator-type.patch`：在 `initialize()` 中调用该钩子，并给成员默认值 |
| 4 | 仿真 3–5 s 的事在墙钟上要 180 s+，日志看似“规划延迟 120 s” | 在**无 GPU 可见**的 shell（agent 沙箱：无 `/dev/dri`、无 X socket）里，`gpu_lidar`×2 + `camera` 退化为软件渲染，把实时率压到 0.059（实测 sim 0.891 s / wall 15.006 s；开传感器复测 0.855 s / 15.008 s = 0.057） | 新增 `spawn_rendering_sensors`：`tools/reduced_robot_model.py` 去掉三个渲染传感器后以 `create -string` 生成，实时率回到 0.997。**有 GPU 的桌面会话保持默认 True 即可** |
| 5 | 起步对齐耗时超过 15 s，每次都被判振荡转恢复 | 对齐生成器只在 `[min_vel_theta, max_vel_theta]` 内采样角速度，默认上限 0.1 rad/s，转 136° 需 24 s | 两个旋转生成器 `max_vel_theta: 0.4`（仍在 §12.2 的 ±0.5 rad/s 内） |
| 6 | 即使低速前进也会被判“振荡”，随后一直停在恢复行为 | 看门狗只有在累计位移 ≥ `oscillation_distance`(5 m) 或转角 ≥1 rad 时才复位；PB 上限 0.2 m/s，15 s 内走不到 5 m | `oscillation_distance: 1.0`，并新增回归测试约束 `oscillation_distance < max_vel_trans × oscillation_patience` |
| 7 | 长时间验收会话中新节点随机报 `Failed to find a free participant index for domain` | CycloneDDS 每域 participant 索引表（默认 120）被大量短命 CLI 进程占满；本机同时存活的 DDS 参与者上限约 11–12 个 | `log/accept_env.sh` 用 `CYCLONEDDS_URI` 提到 2048，并支持 `ACCEPT_DOMAIN=<n>` 每次验收换新域；`pb_jie.launch.py` 增加 `start_click_selector` 以便脚本化验收少起一个节点 |
| 8 | JIE：起终点已发出但规划器毫无反应，`/planned_path` 永不到来 | `jie_path_node` 以 `QoS(1).transient_local().reliable()` 订阅 `/start_point`、`/goal_point`，而客户端用默认 VOLATILE 发布 → DDS 端点不兼容，消息静默丢弃 | `pb_nav_goal` 对这两个话题改用 `TRANSIENT_LOCAL + RELIABLE`，并加回归测试 |
| 9 | JIE：路径已规划、控制器已启动，但客户端立即返回，无法判断是否到达 | `d1_controller` 没有到达/结果话题，只有日志 | `pb_nav_goal` 新增 `wait_for_arrival()`：按 map 位姿与目标的距离判定（默认 0.30 m），超时给出“最近距离” |

第 1–9 项均已修复，并各有单测或实跑证据；`pytest` 43 项通过。

### 17.4 DDDMR（已退出范围，记录保留）

**用户决定（2026-09-15）：DDDMR 不再要求适配与验收。** 以下内容作为已完成的工作与已知限制保留，后续如需重启 DDDMR 接入可直接从这里接着做。

**DDDMR 平地 smoke 闭环未到点。** 现象：全局路径正确、`cmd_vel` 已下发且被适配器转发，但整车位移远小于指令；20 s 内既未走够距离，也随后进入恢复行为，360 s 预算内未到达。

根因已定位在**车辆仿真执行能力**，而非 DDDMR 集成本身。同一注入通道直接实测（`log/probe_velocity_chain.py`）：

| 姿态 | 指令 (vx, vy) | 实测 |
| --- | --- | --- |
| yaw ≈ 0 | (0.15, 0) | 0.13 m/s |
| yaw ≈ 0 | (0, 0.15) | 0.043 m/s |
| yaw ≈ 1.88 | (0.15, 0) | 0.078 m/s |
| yaw ≈ 1.85 | (0.10, -0.05)（DDDMR 实际发出的量级） | 0.012 m/s |

对照实验说明问题不在 DDDMR：同一目标下 **MeshNav 与 JIE 都能在 ~0.3 m 容差内到达**，二者的控制器则主要使用前进+转向（实测前进可达成 85%）。DDDMR 的全向控制器会给出带明显侧移分量的指令，而侧移在本仿真里几乎执行不出来。

上游 `MecanumDrive2` 不做麦轮逆解：它对底盘施加经机体系旋转的 PID 力（`xErr = linearVel.X() - target.x` 后 `RotateVector` 到世界系），而模型四个轮子是各向同性 `mu=mu2=0.2` 的圆柱（无辊子），横移/斜行只能靠滑动。已实测两种各向异性摩擦组合（`1.0/0.02`、`0.02/1.0`），横移反而降到 ~0，故已还原 `0.2/0.2`。

若以后要恢复 DDDMR 接入，按优先级：
1. 给轮子加入真正的辊子各向异性摩擦（正确使用 `fdir1`/`mu`/`mu2` 约定）或换用按轮驱动力矩控制、会做麦轮逆解的驱动插件，然后用 `log/probe_velocity_chain.py` 复测横移/斜行；
2. 车辆执行能力达标前，可只收窄 `differential_drive_simple` 的侧移权限（`max_vel_y`/`min_vel_y`）作为临时手段，并在报告中标注这是仿真限制、不是实车能力结论。

未测：坡道与隧道用例、动态障碍用例、取消链路的实跑确认（`pb_nav_goal --cancel` 已实现并有单测覆盖，但本阶段没有跑“执行中取消”的实景）、JIE 的一键入口实跑（MeshNav 已验）、RViz 图形界面目视确认（本机无 GPU/GLX）。
