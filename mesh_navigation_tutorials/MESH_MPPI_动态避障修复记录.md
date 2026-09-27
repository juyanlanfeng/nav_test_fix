# Mesh MPPI 动态避障修复记录

日期：2026-09-27。入口仍为：

```bash
cd /home/rainple/nav_test/mesh_navigation_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb_vehicle_adapter meshnav_pb_sim.launch.py
```

## 1. 修复结果

最终配置保留原来的 16 线仿真雷达。针对报告中的 0.5 m 方块，修复了高度过滤漏检、行驶中不更新全局路径，以及动态障碍安全余量不足的问题。

| 最终配置复测 | 结果 | 规划总次数（含首次） | 最小车体间隙 | RTF |
| --- | --- | ---: | ---: | ---: |
| T0：空场到点 | 成功，距目标 0.197 m | 19 | 不适用 | 0.894 |
| T5：先放方块再导航 | 成功，距目标 0.195 m | 19 | +0.373 m | 0.847 |
| T6：行驶中插入方块 | 成功，距目标 0.177 m | 23 | +0.212 m | 0.859 |

T0/T5/T6 的正式轮次没有 `NO_VALID_CMD` / `ROBOT_STUCK`。这里的间隙使用与原报告相同的车体矩形分离轴计算；它是平面 footprint 量测，不是 Gazebo 接触传感器的独立证明。T6 方块中心相对请求位置的最大平面位移约 0.0066 m，远小于 0.212 m 的间隙。

复测文件：[diagnostics/mesh_mppi_fix_20260927](diagnostics/mesh_mppi_fix_20260927)。原测试目录未修改。

## 2. 修改了什么

### 2.1 点云到障碍图层：投影门限 0.15 → 0.50 m

文件：[mbf_mesh_nav.yaml](src/mesh_navigation_tutorials/config/mbf_mesh_nav.yaml)。

参数：`move_base_flex.mesh_map.obstacle.robot_height`。

该插件实际检查的是障碍点向下投影到网格的距离。这个参数不会改变 Gazebo 车体尺寸，也不会重新生成地图净空。

原报告说明 0.23 m 仅在部分视角有效：目标点附近只能测到约 0.406～0.419 m 的回波。这次也验证了目标点视角，最终配置下 `/obstacle_points` 的方块点约为 0.405～0.418 m 高；原始障碍层有 24 个致命顶点，`final` 在方块中心为 1.0，半径 0.6 m 内有 273 个致命顶点。

起点视角也通过：原始障碍层 42 个致命顶点，`final` 中心为 1.0，附近 376 个致命顶点。

没有降低 `min_dist_outlier_scan`：仍为 0.3。最终方案未修改 RMCL 分割、雷达模型、车辆尺寸或静态膨胀参数。

**为何没有保留加密雷达方案：** 曾试验 60 线、1° 竖直采样配合 0.23 m 门限。目标点视角的原始点云仍没有低于门限的方块点，障碍层依然为零。因此撤回该模型修改，保留原 16 线，使用经过视角复测的投影门限。

**适用边界：** 0.50 m 是针对当前稀疏点云的保守处理，会把某些真实车辆能够从下面通过的低空悬物也投影为障碍。它没有实现“物体底部/净空识别”，也不能保证所有大小、距离和遮挡条件下都能检测。低净空场景需要单独验收。

### 2.2 行驶中重规划：为 RViz 目标增加协调节点

新文件：[meshnav_navigator.py](src/pb_vehicle_adapter/pb_vehicle_adapter/meshnav_navigator.py)。

流程：

```text
RViz Mesh Goal → /rviz/goal_pose → meshnav_navigator
                                  ├─ get_path：2 Hz 更新路径
                                  └─ exe_path：替换当前执行路径 → MPPI → 底盘
```

原 RViz 面板只调用一次 `get_path → exe_path`，单独配置 `planner_frequency` 不会让这个调用流程自动重规划。本地 MBF 的 `move_base` 实现也存在启动重规划线程和初始化周期的问题，因此本次通过 PB 协调节点完成该入口的闭环，没有扩展修改 MBF 上游代码。

关键行为：

- 从导航 YAML 读取默认规划器、控制器和 `planner_frequency=2.0`；频率为 0 时只规划一次。
- 新路径利用 MBF 同一控制器的路径替换机制，不在每次换路前取消控制器。
- 忽略被替换的旧路径返回结果，防止旧任务结束时误终止新任务。
- 距目标 0.4 m 内停止换路，由 MPPI 完成到点。
- 新目标先取消旧任务，等待旧请求结束再启动。
- 重规划失败时取消路径执行；不会继续沿已经失效的旧路径前进。
- 规划/目标接收超过 15 秒墙钟时间则取消任务并报告失败。

[launch 文件](src/pb_vehicle_adapter/launch/meshnav_pb_sim.launch.py)只增加普通 `Node(...)`，流程代码位于上述 Python 文件。参数仍集中在导航 YAML，没有把业务状态机放进 launch。

默认 [RViz 配置](src/pb_vehicle_adapter/rviz/pb_meshnav.rviz)移除了旧 `MbfGoalActions` 面板，避免两个客户端同时响应 Mesh Goal。仍使用顶部 **Mesh Goal** 点选目标。

### 2.3 修正路径替换引发的速度封锁

文件：[cmd_vel_adapter.py](src/pb_vehicle_adapter/pb_vehicle_adapter/cmd_vel_adapter.py)。

实测发现：新路径进入 EXECUTING 后，旧路径稍后变为 ABORTED，适配器原先仅查看“状态发生变化的目标”，因而把旧路径被替换误当作整个导航取消，持续输出零速度。

现在检查状态数组中是否仍有 ACCEPTED / EXECUTING 的目标；有活动路径时允许继续接收速度。当前任务真正取消且无活动路径时，仍封锁运动。

### 2.4 动态障碍余量和速度

同一导航 YAML 中：

| 参数 | 原值 | 最终值 | 原因 |
| --- | ---: | ---: | --- |
| `mesh_map.obstacle_inflation.inscribed_radius` | 0.20 m | 0.40 m | 车体外接半径约 0.356 m，增加约 0.044 m 余量，保护转弯时的车角 |
| `mesh_controller.kinematics.linear_velocity_max` | 1.5 m/s | 0.8 m/s | 为感知更新、规划和底盘响应留出距离 |
| `controller_frequency` | 未显式配置 | 20 Hz | 明确 MPPI 控制周期 |
| `planner_frequency` | 20 Hz，但原调用流程不使用 | 2 Hz，由协调节点使用 | 持续更新全局路径 |

动态膨胀外圈仍为 1.0 m。静态层参数保持原值。动态内圈加大可能使更窄的障碍间隙被判为不可通行。

## 3. T6 的时序证据

方块位置沿用原报告的 `(-8.19, -2.00)`；车辆距离该点约 1.7 m 时调用 Gazebo 创建服务。创建过程中不暂停仿真。

| 从创建请求起算 | 实测 |
| --- | --- |
| `final` 首次出现障碍 | 0.443 秒墙钟时间；约 0.261 秒仿真时间 |
| 首次代价 | 方块中心 1.0，半径 0.6 m 内 369 个致命顶点 |
| 插入前最后一条全局路径距方块中心 | 0.115 m |
| 插入后约 0.079 秒的路径 | 0.117 m，此时新障碍代价尚未到位 |
| 插入后约 0.840 秒的路径 | 0.728 m，已绕开方块 |
| 插入后的路径发布次数 | 14 次 |
| 最小车体间隙 | +0.212 m |

时序来自 bag 接收时间和 `/clock`，包含服务创建、消息传输和记录延迟，不应解释成纯算法耗时。

图：[T6 轨迹](diagnostics/mesh_mppi_fix_20260927/T6/trajectory.png)。数据：[路径](diagnostics/mesh_mppi_fix_20260927/T6/paths.json)、[代价](diagnostics/mesh_mppi_fix_20260927/T6/costs.json)、[间隙](diagnostics/mesh_mppi_fix_20260927/T6/clearance_offline.json)。

## 4. 其他检查与测试偏离

- `colcon build --packages-select pb_vehicle_adapter mesh_navigation_tutorials --symlink-install`：两包成功。
- 23 项相关 Python 测试通过，覆盖旧路径结果、取消期间延迟接收的请求、规划失败停车、速度适配器和原模型行为。见 [tests.log](diagnostics/mesh_mppi_fix_20260927/tests.log)。
- 实际取消测试：第二次规划后调用取消服务，状态到达 `canceled`，之后采集的 40 条底盘速度指令全部为零。见 [cancel_check.json](diagnostics/mesh_mppi_fix_20260927/cancel_check.json)。
- 新目标均由 `/rviz/goal_pose` 发送，使用 RViz 的同一输入接口；没有用旧的 action 直连脚本作为重规划验收入口。
- T5/T6 起点通过正常导航返回，存在到点容差，与原报告起点不是逐毫米一致。本次是功能回归，不是单参数效果的严格对照。
- 间隙按请求的方块中心和原报告 footprint 计算。方块会随地面轻微倾斜；T5/T6 结束时中心移动约 6.5 mm，正式轮次的正间隙明显大于该量。
- `T6_goal_blocked`：旧触发条件在新路线下太晚，方块落在目标旁并封住目标。规划失败后系统停车，最小间隙 +0.900 m。这一轮不能计为绕行成功，单独保留。
- `T0_preempt_bug`：用于复现并修复速度封锁的问题。
- `T0_duplicate_sim`：重启时发现 Gazebo 子进程残留，该轮不参与最终验收；已停止残留进程后重跑。
- `dense_lidar_rejected` 和不带 `_final` 的静止视角采样：60 线尝试及否决证据，不能视为最终配置结果。
- 原有初始化 TF/渲染警告，以及回位途中一次 TF 时间外推错误，未在本任务中扩展修复。正式 T0/T5/T6 结果如上。
- 本次未测试持续移动的人/车辆、任意尺寸障碍和全场隧道；当前算法是实时障碍代价更新与重规划，没有障碍速度预测。

## 5. 日常使用

编译已完成。重新启动原入口，在 RViz 使用 **Mesh Goal** 即可。

```bash
# 观察状态；plans 包括首次规划
ros2 topic echo /meshnav_navigator/status --qos-durability transient_local

# 取消并停车
ros2 service call /meshnav_navigator/cancel std_srvs/srv/Trigger '{}'
```

协调节点的参数在启动时读取。修改 YAML 后重启；单独对 `/move_base_flex` 动态设置规划频率不会同步修改协调节点。旧的直接 `get_path → exe_path` 脚本会绕过新流程。

缓存路径仍是项目根目录的 `rmuc2026_field.h5`。本次没有删除原缓存；本地膨胀层的 `readLayer()` 返回 false，启动时重算，动态障碍层按点云更新。最终运行参数备份在结果目录的 `*_runtime.yaml` 中。

测试结束后已取消任务、移除测试方块并停止本轮 launch、Gazebo 和 RViz，未留运行进程。
