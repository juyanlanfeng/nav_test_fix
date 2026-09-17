# PB 低矮导航测试车修改方案与验收

日期：2026-09-15。本文是实施方案，**本次没有修改车辆、导航配置或生成地图，也没有完成新车闭环验收**。

## 1. 目标与已知问题

目标：保留 PB 全向运动仿真、3D 雷达和导航接口，删除不必要的高处结构，让测试车能够实际通过 RMUC 场地隧道。外观不要求还原哨兵，也不代表真实哨兵的机械通过能力。

当前配置的车辆碰撞包络最高点为 0.4491 m，导航高度为 0.50 m。场地碰撞 STL 在隧道中心抽样得到地面 z≈0.0029 m、顶板下表面 z≈0.2501 m，局部净高约 **0.2472 m**。原车尺寸与该洞口明显不匹配。

抽样位置包括 `(x=-1.0,y=5.95)` 及对称侧 `(x=1.0,y=-5.95)`，单位 m。这只是局部垂直净高，不能代替整个车辆扫掠体检查。

必须区分两个问题：

- **物理通过**：Gazebo 的车辆 collision 不能碰顶、撞侧壁或在坡口托底。
- **规划通过**：MeshNav/JIE/DDDMR 必须保留洞内地面、正确区分顶棚和地面，且代价与净空判断允许路径通过。

降低车辆不必然消除 MeshNav 的红色区域。红色要结合当前显示的代价层、致命阈值及地图连通性判读，不能靠改配色或强制清零代价解决。

## 2. 修改范围与原则

工作包：`/home/rainple/nav_test/meshnav_demo_ws/src/pb_vehicle_adapter`。

| 文件 | 要做的修改 |
|---|---|
| `models/pb_navigation_robot.sdf.xmacro` | 改为低矮底盘，移除云台、弹仓及附属高结构，降低传感器 |
| `tools/generate_descriptions.py` | 原则上沿用；确认能从修改后的源生成 SDF 和 URDF |
| `models/pb_navigation_robot.sdf` | 由生成器重新生成，Gazebo 实际使用 |
| `urdf/pb_navigation_robot.urdf` | 同步生成，供 RViz/TF 使用 |
| `config/pb_vehicle_profile.yaml` | 更新实际包络及规划高度 |
| `config/dddmr_rmuc2026.yaml` | 同步所有 cuboid，而非只改一处 |
| `launch/pb_meshnav.launch.py` | 使用新的独立缓存；必要时调整已查明的代价问题 |

不要修改 `third_party/pb2025_sources` 的上游资源。不要只编辑生成后的 SDF，否则下次生成会恢复旧车。不要整体缩放机器人：整体缩放会改变轮径、轴距、传感器安装和控制关系。

## 3. 具体车辆改法

### 3.1 在本地源文件中定义精简底盘

当前 wrapper 调用了 `rm25_example_robot` 宏，该宏会展开云台、装甲、弹仓等完整结构。因此仅删除两个云台控制插件还不够。

建议：在 wrapper 内新增本地 `pb_low_chassis` 宏，将原来 `<xmacro_block name="rm25_example_robot" ... />` 替换为本地宏调用。可参考上游文件：

`third_party/pb2025_sources/rmoss_gz_resources/resource/models/rm25_example_robot/rm25_example_robot.def.xmacro`

只复制其中的 `base_footprint`、`base_to_chassis`、`chassis` 和四个轮子调用到本地宏，按以下清单修改。上游 include 可以暂时保留以解析轮子和基础几何宏，但不再实例化完整车辆。

保留：

- `base_footprint` 和 `base_to_chassis`，安装偏移仍为 `0 0 0.076`。
- `chassis` 及其 IMU，传感器名仍为 `chassis_imu`。
- 四轮及原关节名，轮径、轴距、轮距和摩擦参数先不改。
- `chassis_base_collision`：中心 `0 0 0.045`，尺寸 `0.36 0.24 0.13`。
- 前后轮支架碰撞体，先保留原定义。

删除：

- `chassis_projectile_container_collision`，这是底盘 link 内的高弹仓，不能漏删。
- `gimbal_yaw_odom`、`gimbal_pitch_odom`、`gimbal_yaw`、`gimbal_pitch` 及其关节。
- 装甲与装甲支架、灯条、测速模块等附属 link 和 joint。
- 两个控制云台的 `JointController` 插件。

将底盘原 `chassis_base.dae` 外观替换为与底盘碰撞体一致的简单 box，位置同为 `0 0 0.045`，尺寸同为 `0.36 0.24 0.13`。否则外观中仍可能保留弹仓，看起来还是高车。

底盘 box 顶面相对 `base_footprint` 高度为：

```text
0.076 + 0.045 + 0.13 / 2 = 0.186 m
```

保留的底盘质量/惯量可以作为导航测试初值，但删除附属结构后总质量已经改变，必须重新观察姿态与控制稳定性；不能称为真实哨兵动力学模型。

### 3.2 传感器改为低位固定安装

以下为第一版候选安装值，均相对 `chassis`，单位 m/rad：

| 传感器宏 | parent | pose | 说明 |
|---|---|---|---|
| `rplidar_a2` | `chassis` | `-0.10 0 0.11 0 0 0` | 后部低位 2D 雷达 |
| `livox` | `chassis` | `0.10 0 0.11 0 0 0` | 前部水平 3D 雷达，取消旧的倾斜安装 |
| `industrial_camera` | `chassis` | `0.16 0 0.065 0 0 0` | 前向相机，不再依赖已删除的云台 |

保持三个宏的 `prefix="front_"`、采样率和传感器名称不变，避免破坏生成器按名称设置 topic 的逻辑。

建议将 MID360 和 2D 雷达的外观统一简化为传感器本地坐标中中心 `0 0 0.02`、尺寸 `0.08 0.08 0.04` 的 box，碰撞体与此外观一致。通过在本地宏中定义传感器，或在本地生成器中明确替换这些几何实现，**不要直接改上游宏**。MID360 当前宏没有本体 collision，必须补上，不能让传感器穿顶。

此简化传感器的顶面为 `0.076+0.11+0.04=0.226 m`，水平时局部理论余量约 `0.2472-0.226=0.0212 m`。这只是候选设计，**不是已通过的最终参数**。

若希望进一步降高，应连同底盘 box、惯量和传感器安装重新设计；不能把雷达简单塞到底盘实体里面。还要确认 3D 点云未被底盘遮挡，且雷达能看见洞口前方与侧壁。

保留 `JointStatePublisher`、`PosePublisher`、`MecanumDrive2`，以及原四个轮关节名。桥接 topic 保持不变。相机不再随云台转动是预期变化。

### 3.3 姿态与 TF

保留 `base_to_chassis_xyz: [0.0, 0.0, 0.076]` 及单位四元数。由于上述方案未改变轮心和底盘坐标关系，不需要修改真值转换的该项外参。

传感器外参必须来自重新生成的 URDF，禁止另外发布一份旧静态 TF。检查所有 joint 的 parent/child 和 SDF 的 `relative_to` 都指向仍存在的实体。

## 4. 同步三个导航的尺寸

### 4.1 共享 profile

修改 `config/pb_vehicle_profile.yaml`：

- `robot_height`：候选 **0.23 m**，前提是生成模型实测高度不超过 0.226 m。
- `collision_aabb_min/max`：从新模型所有碰撞几何计算，不能沿用旧的 z=0.4491，也不能只凭视觉估计。
- `base_to_chassis_xyz/xyzw`：如上保留。
- `ramp_corridors_enabled`：仍保持 false；旧 Ceres 通道特例不能当作新车通过证明。

计算碰撞包络时必须串联 joint origin 和 collision origin，并考虑旋转。对 cylinder 要按旋转后的实际范围计算；以未旋转尺寸相加会把横向车轮算错。

水平尺寸先不改。当前 `static_inscribed_radius=0.33` 不等于完整车体的外接半径，而 JIE 又直接用它作为 `robot_radius_xy`，三者的几何含义并不一致。不能为变绿而统一改成很小的半径。如果侧向膨胀仍封死通道，应分别检查矩形车体、圆形碰撞近似与各规划器半径语义，再决定是否改车宽或改碰撞判定。

**排查结论（见 §9.5.2）**：`0.33` 既不是内切半径也不是外接半径，而它确实把物理上能过的 0.85 m 隧道判成过不去（可用带宽被压到 0.10–0.15 m）。按上面要求分清语义后，把两个半径改为**包络的内切圆半径 = 半宽 = 0.216608 m**：直走廊里 `w − 2r` 恰好等于车体能否通过的判据（`2r = 0.4332 m` 就是车宽），更小会让规划器穿过车体挤不过去的缝。该值由 `pb_vehicle_adapter.robot_envelope.footprint_radii()` 推导，并有 `test_low_vehicle_description.py::test_footprint_radius_matches_the_disc_model` 双向卡住（不得小于半宽、隧道必须留出 >0.30 m）。

### 4.2 MeshNav

当前 `pb_meshnav.launch.py` 会把 profile 高度传给 `obstacle_robot_height`，但这不等于所有静态层都会自动按这个高度重新计算。它还传入 `height_diff_threshold=0.2`，并存在静态膨胀参数。

验收时分别查看：原始 mesh 是否有洞内地面；高度差/粗糙度/边界层是否先把地面判致命；膨胀是否进一步封死；顶棚是否混入地面邻域。不要盲目提高全图高度差阈值。

使用新的工作缓存路径，保留原 `.h5` 不删除：

```text
/home/rainple/nav_test/meshnav_demo_ws/rmuc2026_pb_low_navigation.h5
```

### 4.3 JIE

当前 `pb_jie.launch.py` 从共享 profile 读取 `robot_height` 和半径，更新后要重启节点。降低高度不能补回 PCD/OctoMap 丢失的洞内支撑面，也不能修复错误的地图分层。

检查路径始终位于洞内地面，而不是吸附到顶棚。若失败，保留具体碰撞/支撑失败位置，区分“身体碰顶”“脚下无支撑”“离散高度跳变”。

### 4.4 DDDMR

`config/dddmr_rmuc2026.yaml` 当前有 **五处 cuboid**，都必须同步为新 profile 包络的八角点，避免全局和局部使用不同尺寸。

仓库还有 `tools/build_dddmr_maps.py` 和地图 bundle 配置。按其当前 CLI 帮助检查 profile 关联与生成方式；如 bundle 保存了旧 profile/包络，必须重新生成独立 bundle。不要只改 manifest 文字以绕过一致性检查。

## 5. 生成、编译和启动

以下命令是在完成上述源文件修改后执行，不会替代修改步骤。

### 5.1 重新生成车辆文件

```bash
cd /home/rainple/nav_test
source /opt/ros/humble/setup.bash
PYTHONPATH="/home/rainple/nav_test/third_party/pb2025_python:${PYTHONPATH:-}" \
python3 meshnav_demo_ws/src/pb_vehicle_adapter/tools/generate_descriptions.py \
  --xmacro /home/rainple/nav_test/meshnav_demo_ws/src/pb_vehicle_adapter/models/pb_navigation_robot.sdf.xmacro \
  --robot-description-root /home/rainple/nav_test/third_party/pb2025_sources/pb2025_robot_description \
  --resources-root /home/rainple/nav_test/third_party/pb2025_sources/rmoss_gz_resources \
  --output-sdf /home/rainple/nav_test/meshnav_demo_ws/src/pb_vehicle_adapter/models/pb_navigation_robot.sdf \
  --output-urdf /home/rainple/nav_test/meshnav_demo_ws/src/pb_vehicle_adapter/urdf/pb_navigation_robot.urdf
```

`PYTHONPATH` 仅为此命令提供项目隔离安装的 xmacro/sdformat_tools。两个 root 使用绝对路径，以免生成的 URDF mesh URI 依赖运行目录。两个 output 分别供 Gazebo 和 RViz 使用；命令会覆盖这两个生成文件，执行前保存你手动添加的内容到源文件。

### 5.2 安装包并测试

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pb_vehicle_adapter --symlink-install
source install/setup.bash
colcon test --packages-select pb_vehicle_adapter
colcon test-result --verbose
```

build 更新安装空间；source 使启动命令找到新安装包；test 运行已有测试。**已有测试通过不表示新车钻洞通过**，还需下一节的新增验收。

### 5.3 重启并分别运行

先在原终端 Ctrl+C 关闭上一轮导航和仿真，确认原 Gazebo 退出。仅重启 RViz不会替换 Gazebo 内已生成的车辆。三个框架一次只运行一个，以下三组是替代关系，不是连续执行。

MeshNav（新终端）：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 launch pb_vehicle_adapter pb_meshnav.launch.py \
  mesh_map_working_path:=/home/rainple/nav_test/meshnav_demo_ws/rmuc2026_pb_low_navigation.h5
```

新缓存避免沿用旧车的 MeshMap 工作结果；仍需确认其静态代价计算实际使用新参数。

JIE（关闭上一组后，新终端）：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 launch pb_vehicle_adapter pb_jie.launch.py
```

DDDMR（关闭上一组后，新终端）：

```bash
source /home/rainple/nav_test/third_party/setup_dddmr_env.sh
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 launch pb_vehicle_adapter pb_dddmr.launch.py
```

DDDMR 的专用环境加载隔离 PCL/GTSAM 等运行库，不修改系统安装。若另存了新 bundle，需要通过该 launch 的 `map_bundle` 参数选择新 bundle；未重建或仍引用旧包络时不要直接判定验收通过。

## 6. 数据流与必须保持的接口

```text
本地 XMacro → 生成 SDF → Gazebo 碰撞、轮子、传感器
            → 生成 URDF → robot_state_publisher → 传感器 TF / RViz 外观
共享 profile → MeshNav / JIE 尺寸参数
             → DDDMR cuboid 与关联地图配置
```

生成器保持 `/robot/imu`、`/robot/scan`、`/robot/cloud/points`、`/robot/camera/image`。桥接后预期分别为 `sensor_msgs/msg/Imu`、`LaserScan`、`PointCloud2`、`Image`。修改安装位置会改变传感器坐标到车体坐标的 TF，而不是改变消息类型。

车辆运动继续使用现有速度适配器、桥接及 PB MecanumDrive2，保持现有控制话题和速度消息转换。不新增第二个速度发布器与导航抢控制。

## 7. 验收标准：不能只看“不红了”

按顺序进行，前一级失败先修前一级：

1. **描述一致性**：SDF/URDF 不再含云台和弹仓碰撞；四轮关节、三个传感器均存在；无悬空 parent/child/relative_to；实测包络与 profile/DDDMR 一致。
2. **静态净空**：对两侧完整洞内区域采样，确认净高与净宽，不只是中心一点。车、相机和雷达 collision 均计入。
3. **坡口姿态净空**：沿真实支撑面求 roll/pitch，检查车体及传感器扫掠体。长度约 0.57 m 的车倾斜时，水平车高 0.226 m 并不能保证钻过 0.247 m 的洞；若卡顶，进一步降低/缩短底盘或调整传感器，而不是关闭碰撞。
4. **独立通行**：在关闭导航控制竞争的条件下低速居中进洞，双向通过两个洞；观察顶碰、侧碰、托底、抖动和打滑。不要用瞬移、穿透或隐藏碰撞来通过。
5. **导航规划**：分别在洞两侧地面选目标，检查三套规划器路径均在洞内，不能绕远却被算作钻洞成功，也不能沿顶棚走。
6. **闭环执行**：至少居中双向、略偏侧进入、坡上重新规划、停车后继续。验证点云、TF、里程计持续更新，目标到达且无持续碰撞。

每次保留车辆版本、profile、实际地图路径、缓存路径、起终点坐标、路径和视频。若只完成生成/编译，应报告“低车模型已生成，闭环未验收”；若物理能过但仍全红，应报告“车辆通过问题已解决，静态地图/代价问题未解决”。

## 8. 实施交付清单

- 低矮模型源 + 可重复生成的 SDF/URDF，不改上游第三方资源。
- 新碰撞包络计算记录；所有导航尺寸同步。
- 新增描述一致性和高度包络回归测试。
- 独立的新缓存/必要时新地图 bundle，旧文件保留可回退。
- 两侧隧道的双向物理及导航闭环证据。

完成这些验收后才能说“车能过洞且导航能规划通过”。本方案中的 0.226 m 外形和 0.23 m 规划高度是首轮设计值，不是未经运行即可保证通过的承诺。

## 9. 实施与验收记录（2026-09-15）

本节只记录**实际执行过**的内容；未执行的项目明确写“未测”。

### 9.1 已完成的修改

| 文件 | 修改 |
|---|---|
| `models/pb_navigation_robot.sdf.xmacro` | 不再实例化上游 `rm25_example_robot`；新增本地 `pb_low_chassis` 宏（base_footprint / base_to_chassis / chassis / 四轮），底盘外观与碰撞同为 `0 0 0.045` 的 `0.36×0.24×0.13` box；删除云台/装甲/灯条/测速/弹仓；三传感器改为本地宏低位固定安装 |
| `pb_vehicle_adapter/robot_envelope.py` | 新增：按 joint 链 + 旋转把每个碰撞体表面点变换到 `base_footprint`，输出包络（车轮圆柱按旋转后取样，不用未旋转尺寸相加） |
| `pb_vehicle_adapter/robot_model.py` | 未改（仍用于生成去渲染传感器模型） |
| `models/pb_navigation_robot.sdf`、`urdf/pb_navigation_robot.urdf` | 由 `tools/generate_descriptions.py` 重新生成（SDF 13 KB，URDF 7.2 KB；旧车文件备份在 `log/pb_robot_high_backup.{sdf,urdf}`） |
| `config/pb_vehicle_profile.yaml` | `robot_height: 0.50 → 0.23`；`collision_aabb_*` 换成实测包络；`static/obstacle_inscribed_radius` 与膨胀半径**保持不变**（§4.1：横向尺寸未变，不为变绿而改半径） |
| `config/dddmr_rmuc2026.yaml` | **5 处 cuboid 全部**同步为新包络八角点，脚本校验 5 块完全一致 |
| `test/test_low_vehicle_description.py` | 新增 5 项回归：无高处结构残留、四轮与三传感器保留且话题不变、无悬空 parent/child/relative_to、包络与 profile 一致、5 处 cuboid 与包络一致 |
| `test/test_robot_model.py` | 去掉“传感器数量固定为 6/3”的硬编码，改为按名字断言 |

传感器安装（相对 `chassis`）：2D 雷达 `-0.10 0 0.11`、MID360 `0.10 0 0.11`（水平，取消旧倾斜安装）、相机 `0.16 0 0.065`。
两处与方案文字不同的地方，已按方案本身的检查要求处理：

1. **相机传感器位姿**：链接仍用方案的 `0.16 0 0.065`，但把 camera sensor 放在外壳前缘 `+0.045`。按方案字面值相机的近裁剪面（0.1 m）会落在底盘 box 内部（box 前脸 x=0.18，相机原点 x=0.16），画面会被自身车体挡住。
2. **两个雷达补了 collision**：方案要求 MID360 必须补，2D 雷达一并补上，外观/碰撞同为 `0 0 0.02` 的 `0.08×0.08×0.04` box。

### 9.2 实测隧道几何（碰撞 STL，射线分层）

| 项 | 正 Y 隧道 | 负 Y 隧道 |
|---|---|---|
| 顶板覆盖范围 | x ∈ [-1.4, -0.7]，y ∈ [5.5, 6.3] | x ∈ [0.7, 1.4]，y ∈ [-6.3, -5.5]（点对称） |
| 地面 z | 0.0029（x=-1.0, y=5.95） | 0.0029（x=1.0, y=-5.95） |
| 顶板下表面 / 上表面 | 0.2501 / 0.4237 | 0.2501 / 0.4237 |
| 局部净高 | **0.2472** | **0.2472** |

与方案 §1 的抽样值一致。注意 y=5.95 上 x≈-0.2 起是**短坡**（0.0214→0.183，约 11.4°）与高平台（0.203），与隧道是两件事，隧道验收不应把坡也算进去。

### 9.3 实测包络（`python3 -m pb_vehicle_adapter.robot_envelope`）

```text
min  [-0.2828, -0.216607, 0.000198]
max  [ 0.2828,  0.216607, 0.226000]
size [ 0.5656,  0.433215, 0.225802]
```

最高点来自两个雷达 box（0.226），底盘 box 顶面 0.186，车轮底部 0.0002。水平尺寸与旧车一致（由车轮决定）。与方案预测的 0.226 m 外形、0.23 m 规划高度一致。

### 9.4 验收结果

| 验收项（§7） | 结果 | 证据 |
|---|---|---|
| 1 描述一致性 | **通过** | `test_low_vehicle_description.py` 6 项（含半径与包络一致性）；生成模型 10 个 link、9 个 joint、5 个 sensor，四轮关节名与三传感器话题不变，无悬空引用；包络与 profile/5 处 cuboid 一致。实机态复核（`log/probe_low_sensors.sh`，`spawn_rendering_sensors:=True`）：`/scan`=LaserScan、`/cloud`=PointCloud2、`/camera/image_raw`=Image、`/camera/camera_info`=CameraInfo、`/imu/data`=Imu、`/odom`=Odometry、`/joint_states`=JointState，与第 6 节要求一致 |
| 2 静态净空 | **通过** | `field/check_tunnel_clearance.py`：每条隧道 792 个位姿（横向 ±0.05 m × 航向 ±3° × 88 个纵向站点），其中顶板下方 162/153 个；正 Y **0 失败**、负 Y **0 失败**；最坏顶棚余量 **+0.0194 / +0.0203 m** |
| 3 坡口姿态净空 | **通过（同一工具）** | 每个位姿按四轮接触面拟合 roll/pitch 并抬升到"无轮下陷"，再检查车体/雷达/相机扫掠体；顶棚余量即上表；坡口处出现 56/10 个“轮-地形”告警（最大 0.033 m），是刚体平面在坡脚的保守估计，非车体穿透，报告里与车体穿透分开统计 |
| 4 独立通行 | **通过** | `log/accept_low_vehicle_tunnel.sh`：正 Y 从 x=-1.906 直行到 -0.579 再倒回 -1.906；负 Y 从 1.907 到 0.581 再倒回 1.908；轨迹 20 Hz 记录。`field/check_trajectory_clearance.py` 复核：**车体穿透 0、顶棚碰撞 0、轮-地形告警 0**，最小顶棚余量 **+0.0196 / +0.0205 m**，最小侧向净空 **0.211 m** |
| 5 导航规划 | **通过（两框架 / 两方向）** | 每条隧道都用「洞外起点 → 洞外 0.1 m 处中线目标」单独冷启动验证。MeshNav：24 点路径，y 只占 `5.940…6.000`（正 Y）/ `−6.000…−5.941`（负 Y），全程 z<0.10，未吸附到顶板。JIE：`/planned_path` 35 点（正 Y）/ 32 点（负 Y）。取证工具 `log/probe_tunnel_plan_grid.sh`（逐点请求 + 网格扫描）与 `log/probe_jie_plan_grid.sh`。上一轮的负 Y 与 JIE 洞内失败原因见 9.5.1/9.5.2，不是地图层判致命 |
| 6 闭环执行 | **通过（两框架 / 两方向）** | 四次独立冷启动，均为 `arrived`：MeshNav 正 Y 终点 (−0.730, 5.968)，负 Y (0.731, −5.966)；JIE 正 Y (−0.673, 5.934)，负 Y (0.697, −5.997)，均在洞口外。四条监控轨迹经 `field/check_trajectory_clearance.py` 复核：**车体穿透 0、顶棚碰撞 0、轮-地形告警 0**，最小顶棚余量 **+0.0196 / +0.0204 m**（MeshNav 正/负 Y），最小侧向净空 **0.169 / 0.170 m** |

产物：`field/converted_rmuc2026/tunnel_clearance/tunnel_clearance_report.json`、`.../nav_{plus_y_217,minus_y_216,jie_plus_y_218,jie_minus_y_220}_report.json`、`log/low_{plus_y,minus_y}_trajectory.csv`、`log/tunnel_nav_{meshnav,jie}_{plus_y,minus_y}_<domain>_{plan,trajectory}.csv`（文件名带 domain 以区分不同运行）。

工具：`field/read_mesh_layers.py`（读 `~/save_map` 写出的层）、`field/tunnel_cross_section.py`（网格横断面）、`field/explain_ceiling_strike.py`（把一次“顶棚碰撞”归因到具体面）、`log/probe_tunnel_plan_grid.sh` / `log/probe_jie_plan_grid.sh`（逐点规划网格）、`log/verify_ros_domains.sh`（可用 DDS 域）。

### 9.5 导航侧实测细节（§7.5/§7.6 的证据）

本轮把上一轮“隧道段被高度差层判致命”的结论推翻并重做，两个框架现在都能双向穿过隧道。

#### 9.5.1 代价层的直接取证（不再靠推断）

`mbf_mesh_nav` 的层只在内存里计算，正常跑完的 `.h5` 缓存里**没有**层数据（用 h5py 列出
`rmuc2026_pb_low_navigation.h5`：只有 `mesh/geometry`、`mesh/edge_attributes`、
`mesh/vertex_attributes/vertex_normals`）。所以本轮先用 `~/save_map`
（`std_srvs/Trigger`）把层写进缓存副本 `log/diag_layers.h5`，再用
`field/read_mesh_layers.py` 读出来：

| 层 | 运行期实测 | 结论 |
|---|---|---|
| `border` | **12693** 个致命顶点（`Found 12693 lethal vertices`），与离线复现（`field/check_mesh_layers.py`，同为 12693）完全一致 | 边界层语义已复现，但隧道地面**不在**其中 |
| `height_diff` | **1821** 个致命顶点；`radius=0.2`、`threshold=0.2` | 隧道地面实测 `height_diff` 只有 **0.0000–0.0051 m**（`log/diag_layers.h5` 与离线报告一致）。上一轮“净高 0.247 m 只比阈值大 0.047 m，所以地面被判致命”的解释**不成立**：`calcVertexHeightDifferences` 把邻域距离投影到顶点法平面，竖直距离被投影掉，顶板不会污染下方地面 |
| `roughness` | **0** 个致命顶点 | 与本车无关 |
| `static_inflation` | 113682 个顶点有代价（`inscribed 0.33 / inflation 0.70`），其余顶点为 0 | 真正封住隧道的是这一层：隧道**井壁**（比地面高 0.35–0.40 m，是真实台阶）的底排顶点被判致命后，向两侧各膨胀一个半径 |

**量出来的可用带宽（规划器自己的回答）**：用 `log/probe_tunnel_plan_grid.sh`
从洞外起点逐点请求 `GetPath`，只有「目标面三个顶点都拿到前驱」才返回 `outcome=0`。

| 配置 | 洞内横断面（`|x|=1.00`）可用 y | 宽度 | 洞口外（`|x|=0.60`） |
|---|---|---|---|
| `inscribed=0.33`（旧） | −6.05 … −5.95（3 列顶点） | 0.10–0.15 m | 只剩 1 列，且多为 `Could not find a valid path, while back-tracking` |
| `inscribed=0.216608`（新） | −6.15 … −5.85（7 列顶点） | 0.30 m | −6.10 … −5.90 全部 `outcome=0` |

同时隧道两壁附近的带状区域在两种配置下都仍然被拒（`y=−6.20/−6.25/−6.30` 与
`−5.80/−5.75/…` 全部 `Predecessor of the goal is not set`）——即放大分辨率而不是整体放开。

用最终配置（profile 已是 0.216608）重跑同一网格，结果见 `log/probe_grid_minus_y_final.txt`：
隧道中心 7 列全部 `outcome=0`（16–17 点路径），洞口外 3 列（24–25 点），两壁附近与洞口外侧
仍全部被拒；横跨全场的远目标 (−11.90, −4.40) 也从负 Y 起点可规划（341 点）——上一轮它在这个
起点是失败的，说明负 Y 一侧的场内连通性随带宽一起恢复。

**几何对称性**：把网格按原点镜像后做最近邻匹配（`scipy.cKDTree`），180417 个顶点里只有
**487 个**（0.27%）在 5 cm 内找不到镜像伙伴，且全部位于 `|x| ≥ 10 m` 的外围结构；
两条隧道及其引道（各 879/880 个顶点）**0 个**不对称。因此上一轮“正 Y 能过、负 Y 过不去”
不是地图左右不同造成的，而是 0.33 m 膨胀把可用带宽压到 0.10–0.15 m（2–3 列顶点）之后的
**数值临界**：同一次运行里正 Y 成功、镜像的负 Y 在洞口 `back-tracking` 失败。

#### 9.5.2 半径语义与最终取值（§4.1 的“再决定”结论）

§4.1 要求先分清“矩形车体 / 圆形碰撞近似 / 各规划器半径语义”，再决定怎么改。实测结论：

* **车体是矩形**：0.5656 × 0.4332 m，内切圆半径 = 半宽 = **0.216608 m**，外接圆半径
  0.356223 m（`field/*` 与 `pb_vehicle_adapter.robot_envelope.footprint_radii()` 同源）。
* **两个规划器都按圆模型**：MeshNav 的 `inscribed_radius` 是“致命顶点周围置为致命的圆盘”；
  JIE 的 `robot_radius_xy` 是直立圆柱半径。直走廊里，宽 w 的通道留给路径的带宽是
  `w − 2r`，所以取 `r = 半宽` 时圆模型**恰好**拒掉车体过不去的通道（`2r = 0.4332 m`
  正是车宽），取更小的半径就会让规划器穿过车体挤不过去的缝。
* **隧道净宽 0.85 m**（网格里两壁底排 y=−6.40/−5.55，碰撞 STL 里实测 ≈0.81 m），物理通行
  实测两侧各剩 0.19–0.21 m（§9.4 第 4 项）。`r=0.2166` 时理论带宽 0.417 m，实测
  0.30 m（多出的部分来自井壁相邻两排地面顶点也被 `height_diff` 判致命，偏保守）。

因此 `pb_vehicle_profile.yaml` 的 `static_inscribed_radius`/`obstacle_inscribed_radius`
由 0.33 改为 **0.216608**（由包络推导，`test_low_vehicle_description.py::
test_footprint_radius_matches_the_disc_model` 会同时在“不得小于半宽”和“隧道必须留出
>0.30 m”两侧卡住这个值）。这不是“为变绿把半径调小”：0.33 对矩形车体既非内切也非外接，
且它把物理上能过的 0.85 m 隧道判成过不去；改后模型仍然拒绝所有窄于车宽的通道。

#### 9.5.3 闭环验收需要的其它条件（都写进了工具与配置）

| 现象 | 原因 | 处理 |
|---|---|---|
| 车停在洞口、`ExePath outcome=106 Robot ignored velocity commands` | 默认 `step_width=0.4 m` 比 0.30 m 的可用带宽还粗，发布出来的折线在洞内来回摆，控制器在洞口要求一个很大的航向变化；车体与井壁底座板（y≈±6.36 处 4–7 cm 高的台阶）接触后卡死 | `mesh_planner.step_width: 0.05`（24 点路径，y 只在 −6.00…−5.94 之间） |
| 车体横着漂、终点航向差 39° | PB 底盘是麦克纳姆轮，`mesh_controller_holonomic: true` 会边平移边偏航 | `mesh_controller_holonomic: false`（沿走廊正向行驶，与 §7.4 实车直行的方式一致） |
| 出发时原地 180° 会蹭到井壁底座板 | 出生朝向 +x、隧道在 −x 方向 | 新增 `spawn_yaw_deg`，负 Y 用例出生即朝 −x；目标姿态取行驶方向（+Y 0°、−Y 180°） |
| 到达判定过早（“arrived 0.299 m，容差 0.30 m”，车还在洞里） | `pb_nav_goal` 只看一次距离 | JIE 用其控制器自身的 `goal_position_tolerance=0.1`，并要求连续 3 次采样都在容差内（`settled`） |
| 停在目标点后原地转 41°，尾轮压在底座板上 2.7 cm | `angle_tolerance: 0.8 rad` 允许 46° 误差 | `angle_tolerance: 0.35 rad`，`cmd_vel_ignored_tolerance: 25 s`（低速旋转需要时间） |

#### 9.5.4 环境侧必须修的两件事（否则证据不可信）

1. **幽灵车辆**：Gazebo（ignition）传输不受 ROS domain 限制，上一轮遗留的仿真服务器会继续
   响应 `ros_gz_sim create` 并发布 `/odom`、`/tf`。表现是“读回来的位姿属于另一台车”
   （最远差 7 m），验收脚本因此可能判错对象。现在 `log/accept_env.sh` 为每次运行导出唯一的
   `IGN_PARTITION`/`GZ_PARTITION`，`accept_low_vehicle_tunnel_nav.sh` 还会在发请求前核对
   `/odom` 是否等于本次 spawn（多次采样，避免上一轮进程尚未退干净时误判）。
2. **DDS 参与者表**：CycloneDDS 的 `MaxAutoParticipantIndex` 默认只有 9（一次验收里仿真 4–5
   个节点 + 导航 4–6 个 + 每个 `ros2 run` 探测各一个，很容易用满），表现为节点创建失败
   `rmw_create_node: failed to create domain`。`log/cyclonedds.xml` 现在设
   `ParticipantIndex=auto` / `MaxAutoParticipantIndex=120`；注意 XML 属性是 `Id` 而不是
   `id`，而 `MaxParticipants` 并不是 CycloneDDS 的选项（上一轮改它没有生效）。
   另外 CycloneDDS 的发现端口是 `7400 + 250·domain`，所以 `ROS_DOMAIN_ID > 232` 的域
   **根本无法创建**（`log/verify_ros_domains.sh` 用它排查可用域）。

#### 9.5.5 最终四次闭环（每例独立冷启动、独立 domain、独立 Gazebo partition）

| 用例 | 路径 | 执行 | 终点 | 轨迹复核 |
|---|---|---|---|---|
| MeshNav +Y | 24 点，`y 5.940…6.000`，`z 0.0018…0.0600` | `ExePath outcome=0 arrived` | (−0.730, 5.968)，航向 0.073 rad，越过洞口 0.030 m | 7559 采样，**穿透 0 / 顶棚 0 / 轮-地形 0**，最小顶棚余量 **+0.0196 m**，最小侧向净空 **0.169 m** |
| MeshNav −Y | 24 点，`y −6.000…−5.941` | `ExePath outcome=0 arrived`（1.9 s） | (0.731, −5.966)，航向 3.213 rad，越过洞口 0.031 m | 2482 采样，**穿透 0 / 顶棚 0 / 轮-地形 0**，最小顶棚余量 **+0.0204 m**，最小侧向净空 **0.170 m** |
| JIE +Y | 35 点 | `/planned_path` + 到达判定（0.098 m） | (−0.673, 5.934)，航向 0.004 rad，越过洞口 0.027 m | 7500 采样，**穿透 0 / 顶棚 0 / 轮-地形 0**，最小顶棚余量 **+0.0196 m**，最小侧向净空 **0.198 m** |
| JIE −Y | 32 点（`y = −5.980` 恒定） | `/planned_path` + 到达判定（0.100 m） | (0.694, −5.967)，航向 3.153 rad（180.6°，与行驶方向一致），越过洞口 0.006 m | 4830 采样，**穿透 0 / 顶棚 0 / 轮-地形 0**，最小顶棚余量 **+0.0205 m**，最小侧向净空 **0.201 m** |

JIE 的终点航向对齐已关闭（`jie_3d_nav/octo_planner/config/meshnav_ceres_controller.yaml` 的
`align_final_yaw: false`）：`true` 时目标航向 180° 会被 d1_controller 解算成 +355° 的误差，
车在洞口原地旋转约 130°，前轮压到井壁底座板上（`log/traj_jie_minus_y_219.txt`）——隧道验收
只要求沿走廊穿过并停在洞口外，与 §7.4 的实车直行一致。

#### 9.5.6 验收工具的就绪判据（本轮修正）

第一次跑 §7.5/§7.6 时两个框架都在“节点已起、地图未就绪”的窗口里被请求，得到的是假失败（MeshNav `Predecessor of the goal is not set`、JIE 的 start/goal 被 `Cleared start, goal, ... after OctoMap update` 丢弃）。现在 `log/accept_low_vehicle_tunnel_nav.sh` 改用各框架自己的就绪信号：MeshNav 等 `Successfully calculated edge costs!`，JIE 等 `Preblocked costmap rebuilt`，并各自给出等待时长。

### 9.6 已知限制与遗留

1. **隧道出口后的短坡（11.4°）爬不上去。** 直驱到目标 x=-0.20 时，车停在 x=-0.506（前轮正好抵住坡脚）并保持不动，指令与 `/pb/cmd_vel_safe` 都是 0.12 m/s，属打滑而非碰撞。车轮 `mu=mu2=0.2` 与 `tan(11.4°)=0.2016` 基本相等，是抓地力临界，不是净高问题；因此 §7.4 的轨迹终点取在坡脚之前（x=-0.60），隧道本体通行结论不受影响。若要连坡一起验收，需单独处理轮地摩擦或爬坡策略。
2. **MeshNav 缓存与代价层已复核**：低矮车使用独立缓存 `meshnav_demo_ws/rmuc2026_pb_low_navigation.h5`（`mesh_map_working_path`），层在每次加载时按当前半径重算，本轮已用 `~/save_map` + h5py 直接读出（§9.5.1）。旧车 SDF/URDF 备份在 `log/pb_robot_high_backup.{sdf,urdf}`。
3. **上游 `third_party/pb2025_sources` 未改动**；旧车 SDF/URDF 备份在 `log/` 便于回退。
4. **圆模型仍是近似**：`inscribed_radius = 半宽` 在直走廊里与矩形车体等价，但转弯/原地旋转时不覆盖车体四角（外接圆 0.356 m）。隧道验收是直行，因此结论成立；若以后要在洞里做斜向/原地机动，需要按矩形足迹单独判碰（DDDMR 的 cuboid 就是这种模型）。
5. **隧道井壁底座板**：`y≈±6.36` 处有一条高 4–7 cm 的台阶（`field/explain_ceiling_strike.py` 可以从轨迹定位到它）。直线穿行时车体离它 0.17–0.21 m，安全；但车体一旦在洞里偏航 30° 以上，尾轮会压上去并卡住。因此闭环用例要求出生朝向与目标姿态都沿行驶方向（见 9.5.3）。
6. **JIE 的远距离目标**：两条隧道内部都能规划，但横跨全场的远目标（如 (−11.90, −4.40)）仍报 `A* planning failed.`（`max_iterations` 250000 用尽或目标在图外），不影响隧道验收，但属于遗留。
7. **偏侧进入、坡上重规划、停车后继续**：偏侧进入已由 9.5.1 的逐点扫描覆盖（洞内横向 ±0.10 m 全部可规划）；坡上重规划与停车后继续仍未单独做用例。

## 10. 第二轮修改与验收（2026-09-16）

本轮的目标是把 §9.6 遗留的“隧道导航侧”做完，结论是：**两个框架、两个方向都能规划并闭环穿过隧道，
轨迹复核无任何接触**。改动分四类，全部有测试或实测证据。

### 10.1 车辆与包络

| 文件 | 改动 |
|---|---|
| `config/pb_vehicle_profile.yaml` | `static_inscribed_radius` / `obstacle_inscribed_radius`：0.33 → **0.216608**（= 包络内切圆半径 = 半宽，由 `robot_envelope.footprint_radii()` 推导；理由与负向对照见 §9.5.2） |
| `pb_vehicle_adapter/robot_envelope.py` | 新增 `footprint_radii()`（内切/外接半径），CLI 一并打印 |
| `test/test_low_vehicle_description.py` | 新增 `test_footprint_radius_matches_the_disc_model`：半径必须等于包络内切半径（含 5e-4 容差）、不得小于半宽、且 0.85 m 隧道必须留出 >0.30 m 带宽、外圈半径必须大于内切半径 |

### 10.2 MeshNav 侧的启动与控制器参数

| 文件 / 参数 | 改动 | 原因 |
|---|---|---|
| `pb_meshnav.launch.py` | 新增 `static_inscribed_radius` / `static_inflation_radius` / `height_diff_threshold` / `mesh_controller_holonomic` 启动参数（默认取 profile 与 `false`） | 让足迹实验不必改共享 profile；并把控制器改成沿路径对准航向 |
| `pb_vehicle_sim.launch.py` | 新增 `spawn_yaw_deg`（转成 `-Y` 传给 `ros_gz_sim create`） | 隧道用例让车出生即朝行驶方向，避免在洞里原地掉头蹭到井壁底座板 |
| `mbf_mesh_nav.yaml` | `mesh_planner.step_width: 0.4 → 0.05`；`angle_tolerance: 0.8 → 0.35`；`cmd_vel_ignored_tolerance: 10 → 25` | 默认路径采样比 0.30 m 的可用带宽还粗；46° 的终点角误差会把尾轮压到井壁底座板上；低速旋转需要更长的看门狗 |
| JIE `meshnav_ceres_controller.yaml` | `align_final_yaw: true → false` | 终点航向 180° 被解算成 +355° 误差，导致洞口原地旋转 130° 并压到井壁底座板 |

### 10.3 验收工具与运行环境

| 文件 | 改动 |
|---|---|
| `log/accept_env.sh` | 每次运行导出唯一 `IGN_PARTITION`/`GZ_PARTITION`（Gazebo 传输不受 ROS domain 隔离，遗留仿真会造成“幽灵车辆”） |
| `log/cyclonedds.xml` | `Discovery/ParticipantIndex=auto`、`MaxAutoParticipantIndex=120`（默认 9 不够一次验收用；XML 属性是 `Id` 不是 `id`，`MaxParticipants` 不是有效选项）；关闭共享内存 |
| `log/accept_low_vehicle_tunnel_nav.sh` | 每次请求前核对 `/odom` 等于本次 spawn（多次采样）、就绪等待期间用零速度指令压住车辆防止溜车、到达判定按框架使用控制器自身容差并要求连续 3 次采样、产物文件名带 domain、目标点取隧道中线（`y=±6.00`）与行驶方向姿态 |
| `pb_nav_goal.py` | 新增 `ARRIVAL_XY_TOLERANCE`（meshnav 0.20 / jie 0.10 / dddmr 0.20）与 `settled` 判定 |
| 新增工具 | `field/read_mesh_layers.py`、`field/tunnel_cross_section.py`、`field/tunnel_vertices.py`、`field/explain_ceiling_strike.py`、`log/probe_tunnel_plan_grid.sh`、`log/probe_jie_plan_grid.sh`、`log/probe_save_layers.sh`、`log/verify_ros_domains.sh`、`log/reap_sim.sh`（已有，补充说明） |

### 10.4 复现命令

```bash
# 单测（描述一致性、包络、半径语义、导航客户端等）
source /opt/ros/humble/setup.bash && source meshnav_demo_ws/install/setup.bash
ROS_HOME=/home/rainple/nav_test/log/ros_test_home python3 -m pytest meshnav_demo_ws/src/pb_vehicle_adapter/test -q

# 四条隧道闭环（每例独立冷启动 / 独立 domain / 独立 Gazebo partition）
CASE_DOMAIN=217 bash log/accept_low_vehicle_tunnel_nav.sh meshnav_plus_y
CASE_DOMAIN=216 bash log/accept_low_vehicle_tunnel_nav.sh meshnav_minus_y
CASE_DOMAIN=218 bash log/accept_low_vehicle_tunnel_nav.sh jie_plus_y
CASE_DOMAIN=220 bash log/accept_low_vehicle_tunnel_nav.sh jie_minus_y

# 轨迹复核（每条 ~5 min）
PYTHONPATH=meshnav_demo_ws/src/pb_vehicle_adapter field/.step_convert_venv/bin/python \
  field/check_trajectory_clearance.py log/tunnel_nav_meshnav_minus_y_216_trajectory.csv

# 规划通过性网格（把 0.216608 换成 0.33 即为对照）
GRID_DOMAIN=205 bash log/probe_tunnel_plan_grid.sh minus_y
JIE_RADIUS=0.216608 JIE_DOMAIN=218 bash log/probe_jie_plan_grid.sh plus_y
```
