# STVL / MeshNav implementation and verification

实施日期：2026-10-09。代码位于当前教程工作区 `src/mesh_navigation/`。

## 已实现

- 新插件 `mesh_layers/TemporalObstacleLayer`，继承原 `ObstacleLayer`。
- 共用点云布局检查、坐标变换、距离过滤、向下投影和致命顶点提交；旧插件继续逐帧替换。
- 以三维体素和 face ID 保存真实回波、关联顶点、最后检测时间和累计衰减年龄。
- 当前默认 `clearing_mode: stvl`，线性寿命 10 秒、视野加速系数 0，真实雷达原点周围 8 米范围。
- 支持 STVL 的线性（0）、指数（1）、持久（-1）时间模型和 `a*t³/6` 视野加速；
  旧的 0.1 秒缺失窗口仅在显式选择 `visible_timeout` 模式时生效。
- 先刷新、后过期；断流不执行清除；有效空帧允许清除。TF 失败、消息过时、重复、乱序、明显超前和布局错误不更新历史。
- 处理前与 TF/投影完成后检查消息年龄。时间使用整数纳秒，未来时间仅容忍 1 ms 精度误差。
- 检测 map/reference 变化和短间隔雷达位姿跳变；通过 ROS 时钟跳变回调记录时间纪元，下一有效帧清旧历史。
- 存活记录共享的顶点取并集。致命顶点集合未变化时不通知下游。
- MID360 非对称垂直视野、完整旋转、局部 3 cm 原点偏移、球形范围与轴线保护。
- 新插件的投影、历史、视野参数只读，修改 YAML 后重启。旧插件原有可热更新参数继续有效。
- YAML 已切换到新插件，安装配置是源码的符号链接。

三个上游文件从原先放置的公开 include/src 路径移到 `mesh_layers/src/stvl_frustum/`。
来源、原始许可证和适配差异见该目录的 `UPSTREAM.md`。没有安装整个 STVL，也没有引入 OpenVDB 或 Nav2 依赖。

与初始方案相比增加一个必要修复：给 `mesh_map::AbstractLayer` 添加虚析构函数。
pluginlib 通过基类指针删除插件，此修复确保派生类的订阅、历史和时钟回调被释放。
这改变了插件基类 ABI，所以本次已重编译 `mesh_map` 及其全部工作区下游包。
在其他构建目录更新时也必须重编译这些包。

## 构建与自动测试

实际执行并通过：

```bash
cd /home/rainple/nav_test/mesh_navigation_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 MAKEFLAGS=-j3 \
  colcon build --packages-above mesh_map --parallel-workers 2 \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
OMP_NUM_THREADS=2 colcon test --packages-select mesh_layers mesh_map \
  --event-handlers console_direct+
colcon test-result --test-result-base build/mesh_layers --verbose
colcon test-result --test-result-base build/mesh_map --verbose
```

12 个包构建成功：`mesh_map`、`mesh_layers`、`mbf_mesh_core`、`mbf_mesh_nav`、
`mesh_controller`、`cvp_mesh_planner`、`dijkstra_mesh_planner`、`mesh_mppi`、
`mesh_navigation`、`mesh_navigation_tutorials`、`pb_terminal_controller`、`pb_vehicle_adapter`。
构建日志有既有 CMake/依赖库警告，没有编译错误。

首次交付的时间层测试为 29 项；本次增加 7 项 STVL 衰减与箱体侧面保留测试后，
时间层 36 项和 inflation 2 项全部通过。另有前次通过的 mesh_map 2 项，共 40 个 GTest 用例。
`colcon test-result` 将 CTest 容器也计入，mesh_layers 当前报告 40 个结果条目。

测试使用实际加载的三角 mesh、raycaster、pluginlib、可控 ROS 时钟和 TF buffer。
包括正常订阅虚派发、旧插件回归、精确时间边界、有效空帧/失败帧区分、
等待 TF 期间变旧的帧、断流恢复、共享顶点、负坐标 floor、同体素不同面、
原始回波高度、真实雷达原点、姿态与 FOV、定位跳变、暂停/时钟倒退、
只读参数与 YAML 覆盖、层间删除传播、静态代价保留、仅刷新时无下游通知、
带行填充及大端字节序点云，以及插件销毁后节点资源释放。

## 实际导航进程验证

本节为首次交付时的 `visible_timeout` 模式验证，保留原始结果作追溯；
当前默认已改为 STVL 时间衰减，见下文“箱体绕行反复折返修复”。

在隔离的 `ROS_DOMAIN_ID=193` 中启动安装后的 `mbf_mesh_nav`，加载当前场地
HDF5 的临时副本，使用实际 YAML 参数；临时覆盖输入话题为 `/stvl_smoke/obstacles`、
使用墙钟和合成 TF/点云。没有发送导航目标或车辆速度命令。

场地网格为 180,502 个顶点、348,467 个三角面。查询运行时参数确认插件为
`mesh_layers/TemporalObstacleLayer`，保留时间为 10 秒和 0.1 秒。
输入约 20 Hz，回波位于场地地面上方 0.3 米；同时监视 obstacle 与 final 的更新消息。

| 检查 | 结果 |
|---|---|
| 首次标记与重复刷新 | 通过；重复刷新未产生额外 obstacle/膨胀通知 |
| 视野内连续缺失 | 通过；清除帧时间距首次缺失帧约 0.1010 s |
| 删除传播到 final | 通过；从首次缺失发布到收到 final 更新约 0.1019 s |
| 断流超过 10 s | 通过；原动态顶点保持 |
| 恢复帧重新检测 | 通过；先刷新，仍存在的障碍没有被旧年龄清除 |
| 移出范围后的普通过期 | 通过；最后检测后约 10.0301 s 清除 |

本次记录 203 个有效帧。小点云下，投影和历史处理合计 P95 约 0.0174 ms；
发生变化的下游更新样本 P95 约 1.44 ms。这些数值仅描述本次少量合成回波，
不能作为密集真实点云或大历史表的性能结论。

本机原始日志保存在 `/tmp/mesh_stvl_runtime_d2u0xw17/`，包含 `result.json`、
`navigation.log` 和 `layer_timings.csv`。验证脚本位于 `/tmp/mesh_stvl_smoke.py`。
临时文件可能被系统清理；上表记录了交付时的关键结果。

## 使用与剩余验收

### 真实扫描链路修复与验证（2026-10-09）

用户反馈 `/dynamic_obstacles` 可见但动态代价不更新后，在其正在运行的 Gazebo 中复现：
8 秒采样中，108 帧障碍点云的年龄中位数为 0.550 s、P95 为 0.577 s，
全部超过 `max_observation_age: 0.30`，没有代价更新。TF 与订阅连接正常。
此前合成输入验证没有覆盖这一上游延迟。

原因是 PCL `SegmentDifferences::segment()` 每帧对固定的 631,508 点背景重建 KD 树，
加上 10 帧输入队列，造成持续积压；新增时间层的新鲜度检查因而拒绝所有帧。
修复如下：

- 背景 KD 树初始化一次，复用 PCL 原有 `getPointCloudDifference()` 算法和平方距离阈值。
- 点云输入与输出均使用 `KeepLast(1)`；保留原始采样时间。
- 时间层对非法布局、过时/超前帧、TF/投影后过时帧输出节流告警。
- 保持 0.30 s 新鲜度限制与现有历史清除规则。

重新构建 `pcl_obstacle_point_subtraction`、`mesh_layers`，重启背景差分和导航节点后：

| 真实链路检查 | 结果 |
|---|---|
| 20 秒采样 | 327 帧障碍点云全部在 0.30 s 内 |
| 点云年龄 | 中位数 22 ms，P95 38 ms，最大 80 ms |
| 首帧标记 | 400 条历史记录、80 个致命顶点；同帧传播到 obstacle_inflation 和 final |
| Gazebo 临时 0.4 m 方块出现 | 测试区域新增 3 个致命顶点，18 个顶点的最终代价升高 |
| 删除临时方块 | 新增的 3 个致命顶点全部清除，测试区域最终代价恢复基线 |
| 差分算法对比 | 原算法与缓存 KD 树算法输出点坐标及顺序一致，NaN 和空扫描检查通过 |
| 自动回归 | mesh_layers 与 mesh_map 全部通过，33 个 GTest 用例 |

临时方块已删除。真实链路验证没有发布车辆运动命令。
两个重启后的节点跟随原 launch 进程退出；下次仍按原启动方式运行。
本机验证记录：`/tmp/stvl_before_fix.json`、`/tmp/stvl_after_nav_restart.json`、
`/tmp/stvl_gazebo_check.json`、`/tmp/stvl_difference_check.log`；
构建与测试日志：`/tmp/mesh_stvl_latency_build.log`、`/tmp/mesh_stvl_latency_tests.log`。

### 箱体绕行反复折返修复（2026-10-09）

用户提供的 22:52:44 录屏及导航日志显示：机器人绕过场地中
`box`（中心约 `[-4.7993, -3.8462]`，边长 1 m）时，每约 3 秒重新规划，
在左右候选路线间反复切换，最终在当前面没有向量场时以 outcome 100 失败。

在同一运行中的 Gazebo 中复现，记录点云驱动的 `obstacle` / `final` 和实际车位。
机器人从箱体一侧移动到另一侧时，旧侧面虽然被遮挡，仍满足几何 FOV 判定，
被原来的 0.1 秒未观测窗口提前删除；新规划便穿过刚被删空的一侧，重新观测后再阻断。
这是清除规则与 STVL 的实际衰减机制不一致造成的循环，前次小方块出现/删除试验未覆盖它。

当前默认配置：

```yaml
clearing_mode: stvl
decay_model: 0
obstacle_keep_time: 10.0
decay_acceleration: 0.0
```

`obstacle_keep_time` 对应 STVL 的 `voxel_decay`。
参考固定上游版本的时间模型：线性剩余寿命 `keep - age`、指数剩余量
`keep * exp(-age)`、持久保留；视野内每次有效更新可额外累计 `a * age³ / 6`。
使用上游的严格小于零过期条件。当前将加速系数设为 0，保留绕行需要的侧面历史；
代价是已移走的障碍可能在最后观测后保留约 10 秒。
正的加速系数可以缩短清除时间，但不能区分遮挡、漏扫与真实消失，效果也依赖更新频率。

旧规则仍可通过 `clearing_mode: visible_timeout` 显式选择，此时才使用 `visible_keep_time`。
新增测试覆盖遮挡侧面保留及 final 层、线性过期边界、上游累计加速公式、
视野外不加速、指数模型、持久模型与时钟重置、重新观测与断流恢复。
时间层 36 项和膨胀层 2 项测试全部通过。

这次对齐了时间衰减公式，并非移植整个 STVL：仍使用 MeshNav 的体素/面记录与投影，
仍先刷新当前回波，且只在有效新鲜帧到达时推进清除；未增加 OpenVDB、Nav2 costmap、
原始扫描自由空间检测或遮挡检测。三个 frustum 文件仍是唯一整体移入的上游文件。

复测还发现协调节点在确认旧路阻挡后仍执行旧路线，且瞬时 `NO_PATH_FOUND` 会立即终止目标。
仅修改衰减规则的一次复测在场地北侧遇到该失败，停车后重新查询则又有可行路线。
因此同步修正 `meshnav_navigator.py`：

- 旧路仍可行时继续保持；确认受阻后取消旧执行，等其结束，再从当前位置请求新路。
- 保留原目标；短暂无路或起终点受阻时保持停止，以 0.5 秒间隔重试。
- 总等待上限 `replan_patience: 15.0` 秒（墙钟），超时退出；无效目标等错误不重试。
- 取消用户目标、迟到 action 接收/结果、重规划成功均正确清除等待状态。

19 项协调节点测试全部通过，使用已安装代码再运行也全部通过。
当前构建后的 `mesh_layers` 与 `pb_vehicle_adapter` 已加载到运行节点。

同一 Gazebo 场景、近似原起点和同一目标 `[-1.4217, -3.2536, 0.3765]` 的实测：

| 版本 | 结果 |
|---|---|
| 原 0.1 秒缺失窗口 | 4.49 秒仿真时间内往返约 2.99 m，3 次规划后控制失败，离目标约 2.69 m |
| STVL 时间模型 + 停车重规划/限时重试 | 成功到达，误差约 0.168 m；6 次规划（含初次） |
| 最终版中的短暂无路 | 实际触发一次 `waiting_for_path`，随后得到新路并继续完成任务 |

最终版整段运行约 53.86 秒仿真时间，实际行驶约 39.78 m。
路径包括场地北侧绕行；本结果证明该次完整路线能完成，不代表无重规划、最短路线或所有场景均已验收。
测试没有移动/删除用户放置的三个箱子。导航与协调节点跟随原 launch 进程退出。

持久记录：[结果与轨迹数据](validation/stvl_box_20261009/result.json)、
[轨迹对比图](validation/stvl_box_20261009/trajectory.png)（左右图显示范围不同）。
本机完整采样位于 `/tmp/stvl_loop_before.npz`、`/tmp/stvl_loop_coordinated.npz` 及各自 `.json`；
构建/测试日志位于 `/tmp/stvl_decay_build.log`、`/tmp/stvl_decay_test.log`、
`/tmp/stvl_navigator_build.log`、`/tmp/stvl_navigator_test.log`。

### 启动与后续验收

重新 source 工作区后，按原来的方式启动导航和 PCL 背景差分节点。
新插件继续订阅 `/dynamic_obstacles`；上游正常无障碍时必须发布保留 XYZ 字段和
采样时间的有效空帧。当前 `meshnav_pb_sim.launch.py` 的 `rmcl_seg` 选项输出
`obstacle_points`，因此仍需启动项目原有 PCL `/dynamic_obstacles` 发布流程。

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash
ros2 param get /move_base_flex mesh_map.obstacle.type
ros2 topic info /dynamic_obstacles -v
```

只修改动态历史参数不需要删除静态 HDF5 缓存，修改 YAML 并重启即可。
回退时把 `mesh_map.obstacle.type` 改回 `mesh_layers/ObstacleLayer`。

已完成代码、自动测试、合成输入验证、Gazebo 临时障碍标记/清除及本次箱体场景完整绕行验证。
持续移动物体、其他封堵布局、真实定位重置、实车外参和高负载延迟验收仍待进行。
视野判定沿用方案约定，不检查遮挡。
