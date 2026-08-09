# SCAN-Planner 全向舵轮机器人真机部署与改造手册

> 目标平台：全向舵轮移动机器人（可同时执行机体系 `vx`、`vy`、`wz`）  
> 软件基础：Ubuntu 22.04、ROS 2 Humble、SCAN-Planner `ros2-community` 分支  
> 工作区示例：`/home/rainple/nav_test/SCAN-Planner`  
> 本文重点：执行器改造、车体碰撞模型与物理参数、输入输出话题、真机部署验收

## 1. 最终要得到的系统

推荐把规划、轨迹跟踪、安全限制和舵轮解算分成四层：

```text
定位 + LiDAR/深度相机 + 全局目标/参考路径
                    │
                    ▼
             SCAN-Planner
       三维局部地图、碰撞检查、B 样条
                    │ /planning/bspline
                    ▼
        全向底盘轨迹跟踪器（需要改造）
       世界系轨迹误差 → 机体系 vx、vy、wz
                    │ /scan_planner/cmd_vel_raw
                    ▼
        安全限制/看门狗（真机必须有）
      限速、限加速度、限 jerk、超时停机、急停
                    │ /cmd_vel
                    ▼
             已封装的舵轮底盘驱动
           接收 /cmd_vel 并完成轮级控制
                    │
                    ▼
                 电机驱动器
```

你们的底盘已经封装舵轮逆运动学和轮级控制，外部只需要发布 `/cmd_vel`。SCAN 侧不需要计算单轮转角、轮速，也不需要接触 CAN 或电机协议。接口边界固定为机体系底盘速度 `vx、vy、wz`。

## 2. 开始前必须收集的机器人数据

没有这些数据，不能安全地给出最终数值。先填写一张实测表：

| 名称 | 符号 | 单位 | 获得方法 |
|---|---:|---:|---|
| 车体碰撞外形总长 | `L` | m | 包含保险杠、传感器支架等不可穿越部分 |
| 车体碰撞外形总宽 | `W` | m | 取转向/悬挂运动时的最大宽度 |
| 车体有效高度 | `H` | m | 从地面到最高不可碰撞部位 |
| `base_link` 离地高度 | `z_base` | m | 从机械尺寸、安装图或实测确认；不要求必须有 URDF |
| 底盘前后最大速度 | `vx_max` | m/s | 空载与额定负载下实测，取较小值 |
| 底盘横向最大速度 | `vy_max` | m/s | 同上 |
| 合成平移最大速度 | `vxy_max` | m/s | 同上 |
| 最大角速度 | `wz_max` | rad/s | 同上 |
| 平移加/减速度 | `a_acc/a_brake` | m/s² | 重点实测满载制动能力 |
| 角加速度 | `awz_max` | rad/s² | 实测 |
| 平移 jerk | `jxy_max` | m/s³ | 无数据时从很保守值开始 |
| 定位平面误差 3σ | `e_loc` | m | 静止与运动数据统计 |
| 点云/外参误差 | `e_sensor` | m | 标定残差和点云重影估计 |
| 感知到制动总延迟 | `t_delay` | s | 传感器、规划、DDS、底盘执行总和 |
| 期望最小安全余量 | `m_clear` | m | 根据场地、速度和安全等级确定 |
| `/cmd_vel` 超时时间 | `cmd_timeout` | s | 查看底盘文档并断流实测 |
| 底盘是否内置速度斜坡 | — | 是/否 | 查看底盘文档并记录实际响应 |

所有速度、加速度必须以“满载、最低电量、最低附着、实际轮胎”的最差工况为准，不能直接使用电机空载理论值。

## 3. 当前代码与舵轮底盘之间的差距

### 3.1 已经具备的能力

当前 `closed_loop_controller.cpp` 已经：

- 从 `/planning/bspline` 读取位置 B 样条；
- 从里程计读取当前 XY 和 yaw；
- 计算世界坐标系下的轨迹速度与位置反馈；
- 把世界系速度旋转到车体坐标系；
- 发布 `geometry_msgs/msg/Twist`；
- 填写 `linear.x`、`linear.y` 和 `angular.z`。

所以它不是纯差速控制器，数学上已经能产生全向底盘的三个自由度命令。

### 3.2 必须改造的部分

当前实现仍有以下真机风险：

1. **强制朝向路径切线**：控制器通过前视轨迹计算期望 yaw。朝向误差超过 `heading_error_threshold` 时，把 `vx/vy` 清零并只旋转。全向舵轮不一定需要“先转头再平移”。
2. **碰撞朝向不是真实车身朝向**：多个碰撞检查函数使用路径相邻点方向估计 yaw。全向移动时，运动方向和车体方向可以不同，这可能让非圆形车体使用错误姿态做碰撞检查。
3. **只有速度限幅，没有完整执行器约束**：当前控制器没有平移加速度、角加速度和 jerk 约束，也没有按底盘公开能力建立统一限制。
4. **缺少可靠的真机看门狗**：`Twist` 没有时间戳，控制器本身也没有里程计超时、轨迹超时和驱动掉线后的独立停机链。
5. **角速度被源码硬限制到 1.0 rad/s**：即使 YAML 写更大，`kMaxVYawLimit` 也会截断。最终上限应来自实测，而不是硬编码。
6. **规划器物理限制是统一标量**：`manager.max_vel/max_acc` 和 `optimization.max_vel/max_acc` 没有区分 X/Y/Z。这里的 Z 应表达坡道或地形造成的机体高度变化，不是“向上飞”的控制自由度；但现有执行器不会输出 Z 速度，所以必须保证 Z 来自可通行地表，而不是任意空间搜索。
7. **`manager.max_jerk` 当前没有实际进入可行性检查**：它会被读取，但现有搜索结果表明没有代码使用它约束 B 样条。
8. **目标朝向没有进入轨迹**：`Bspline.msg` 定义了 `yaw_pts/yaw_dt`，当前 FSM 发布时没有填充；RViz 目标姿态和参考路径姿态也没有被用于最终 yaw 控制。
9. **真机话题被写死**：`run.launch.py` 固定使用 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar` 和 `/cmd_vel`。
10. **真机错误启动了旧平台模型**：`run.launch.py` 无论仿真还是真机都会启动现有的 `robot_state_publisher` 和旧 URDF。真机主链应直接删除这个节点；只有 RViz 需要显示完整机器人模型，或底盘驱动明确依赖 `/robot_description` 时，才需要另行加载舵轮机器人的 URDF。
11. **LiDAR 外参写死在 C++ 中**：`grid_map.cpp` 内的 `lidar_extrinsic_` 和 `depth_extrinsic_` 不能直接用于你们的机器人。
12. **里程计速度坐标系有隐患**：ROS 标准中 `Odometry.twist` 通常表达在 `child_frame_id`，而 FSM 直接把 XYZ 数值当成规划坐标系速度使用。真机必须统一语义或转换。

这些问题中，第 1、2、3、4、10、11、12 项不能只靠改 YAML 解决。

## 4. 推荐的代码改造结构

建议新增而不是直接覆盖原文件，便于保留对照：

```text
src/planner/plan_manage/
├── src/
│   ├── swerve_trajectory_controller.cpp   # B 样条到机体系速度
│   ├── cmd_vel_safety_filter.cpp          # 限制、看门狗、急停
│   └── ...
├── config/
│   ├── swerve_controller.yaml
│   └── swerve_robot.yaml
└── launch/
    └── swerve_real.launch.py
```

不新增轮级运动学适配器。已封装底盘是 `/cmd_vel` 的唯一消费者。

### 4.1 逐文件修改清单

| 文件 | 必须完成的改动 |
|---|---|
| `plan_manage/src/closed_loop_controller.cpp` | 保留作参考；新建全向舵轮轨迹跟踪器，加入朝向策略、各向异性限制、超时和状态输出 |
| `plan_manage/config/controllers.yaml` | 不再作为真机配置；新增 `swerve_controller.yaml` |
| `plan_manage/CMakeLists.txt` | 编译并安装 `swerve_trajectory_controller` 和 `cmd_vel_safety_filter` |
| `plan_manage/package.xml` | 若使用 diagnostics，增加 `diagnostic_msgs` 依赖 |
| `plan_manage/launch/run.launch.py` | 保留算法仿真；不要继续作为舵轮真机最终入口 |
| `plan_manage/launch/swerve_real.launch.py` | 新增参数化真机入口，不写死定位、点云、底盘话题和机器人模型 |
| `plan_manage/src/scan_replan_fsm.cpp/.h` | 重命名冻结接口；转换里程计速度坐标系；保存目标 yaw；明确目标/参考路径 Z 的地形语义 |
| `plan_env/src/grid_map.cpp` | 删除旧硬编码外参，改成参数或按消息时间查询 TF |
| `plan_env/include/plan_env/grid_map.h` | 第一阶段使用外接圆；最终替换为统一 footprint API |
| `bspline_opt/src/bspline_optimizer.cpp` | 保持 Z 受地形参考约束，不能把 Z 当独立避障自由度；最终接入计划 yaw 的 footprint 检查 |
| `path_searching` | 搜索状态约束在可通行地表/允许坡度和台阶上；统一调用新的 footprint 检查 |
| `traj_utils/src/planning_visualization.cpp` | 删除硬编码的 `world/map`，使用 `global_frame` 参数 |
| `scan_planner_msgs/msg/Bspline.msg` | 若采用 yaw 轨迹，明确并真正填充 `yaw_pts/yaw_dt`，或新增单独 yaw 轨迹消息 |
| `rviz/default.rviz` | Fixed Frame、机器人模型和所有话题改成舵轮系统名称 |

在 `CMakeLists.txt` 中新增可执行目标的形式应类似现有 controller。安全过滤器和轨迹控制器只依赖标准 ROS 消息，不引入底盘厂商 SDK。

### 4.2 正确约束是“贴地表”，不是“Z 永远固定”

舵轮底盘没有主动的垂直速度执行器，但可以沿坡道运动，也可能通过悬挂/轮胎越过允许的小起伏。此时世界坐标中的机体 Z 会随地形变化：

```text
z_body = terrain_height(x, y) + chassis_reference_height
```

因此真机模式必须保证：

- 起点 Z 来自定位；
- 目标和全局路径 Z 来自可通行地表高程，而不是随意常数；
- 搜索只能沿满足最大坡度、台阶高度、离地间隙和稳定性要求的地表移动；
- B 样条的 Z 跟随地形参考，不能为了绕开一个箱子而凭空升高；
- 控制器仍只输出 `vx、vy、wz`，实际 Z 变化由车轮与地面接触产生；
- 三维 GridMap 负责判断障碍物在高度方向上是否会碰到车体。

当前源码已经做了部分 Z 约束：`planner_manager.cpp` 的 `applyLinearZReference()` 在起点 Z 和局部目标 Z 之间生成参考；`bspline_optimizer.cpp` 将优化梯度的 Z 行清零，所以优化器不会主动把轨迹向上抬来避障。但这只是起终点间的线性高度插值，不等于真正理解坡面，也不检查地面支撑、坡度、台阶高度或轮地接触。

对完全平坦的场地，地形函数是常数，Z 自然固定；对坡道和起伏场地，Z 应沿可通行地表变化。

## 5. 改造轨迹执行器

### 5.1 输入和输出

新的 `swerve_trajectory_controller` 建议：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 输入 | `/planning/bspline` | `scan_planner_msgs/msg/Bspline` | SCAN 局部位置轨迹 |
| 输入 | `/localization/odom` | `nav_msgs/msg/Odometry` | 当前全局位姿和速度 |
| 输入 | `/target_heading`（可选） | `geometry_msgs/msg/PoseStamped` 或自定义消息 | 独立车身朝向目标 |
| 输入 | `/emergency_stop` | `std_msgs/msg/Bool` | 外部急停/安全 PLC 状态 |
| 输出 | `/scan_planner/cmd_vel_raw` | `geometry_msgs/msg/TwistStamped` | 带时间戳和 frame 的原始底盘命令 |
| 输出 | `/planning/execution_frozen` | `std_msgs/msg/Bool` | 是否冻结 SCAN 轨迹时间 |
| 输出 | `/controller/status` | 建议自定义诊断消息或 `diagnostic_msgs` | 超时、限幅、跟踪误差和状态 |

推荐使用 `TwistStamped` 而不是裸 `Twist`，这样安全层可以判断命令是否过期。若底盘只接受 `Twist`，由最靠近底盘的安全适配节点去掉 Header。

### 5.2 坐标系约定

统一约定：

```text
global_frame: map（或 odom，只能选一个并保持一致）
base_frame:   base_link

Twist.linear.x  车体向前为正
Twist.linear.y  车体向左为正
Twist.angular.z 俯视逆时针为正
```

SCAN 的 B 样条控制点和期望速度位于 `global_frame`。控制器应先在世界系计算位置误差：

```text
v_world_target = v_world_feedforward + Kp × (p_world_desired - p_world_actual)
```

再根据当前 yaw 转为机体系：

```text
vx_body =  cos(yaw) × vx_world + sin(yaw) × vy_world
vy_body = -sin(yaw) × vx_world + cos(yaw) × vy_world
```

当前控制器已有这部分公式，可以复用。

### 5.3 全向机器人的朝向策略

必须明确选择一种，不能沿用含糊的默认行为：

| 策略 | 行为 | 适用场景 |
|---|---|---|
| `hold` | 平移时保持当前/指定固定 yaw | 第一阶段真机测试，最安全清晰 |
| `goal` | 平移与旋转解耦，yaw 平滑走向目标姿态 | 需要到点朝向 |
| `path_tangent` | 车头沿路径切线 | 前向传感器或机构需要始终朝前 |
| `external` | 订阅独立 yaw 轨迹/目标 | 上层任务决定朝向 |

推荐第一阶段使用 `hold`，并允许在 yaw 调整时继续执行受限的 `vx/vy`。全向底盘不应默认因为朝向误差大就把平移完全清零。

如果选 `goal` 或 `external`，必须让碰撞检测使用同一份计划 yaw。只改控制器、不改碰撞检查会产生“控制器实际车身朝向”和“规划器假设朝向”不一致。

### 5.4 速度限制

目标速度至少经过两类限制：

1. 单轴限制：

```text
|vx| ≤ vx_max
|vy| ≤ vy_max
|wz| ≤ wz_max
```

2. 合成速度限制：

```text
sqrt(vx² + vy²) ≤ vxy_max
```

当前代码用 `max(max_vx,max_vy)` 限制世界系速度范数，然后再分别裁剪 X/Y，不能准确表达非对称底盘能力。改造后应先得到目标机体系 Twist，再按单轴和合成平移能力限制。

### 5.5 加速度和 jerk 限制

每个控制周期 `dt` 执行限速斜坡：

```text
a_target = (v_target - v_previous) / dt
|a_xy| ≤ a_acc       加速
|a_xy| ≤ a_brake     制动
|awz|  ≤ awz_max
```

进一步限制 jerk：

```text
j = (a_target - a_previous) / dt
|j_xy| ≤ jxy_max
|jwz|  ≤ jwz_max
```

最后积分得到本周期实际下发速度。急停是例外：物理急停由硬件安全链负责；软件安全停应使用经过验证的最大安全制动减速度，而不是无限瞬时跳零。如果底盘驱动要求“失能”才能可靠停止，急停接口不能只依赖 `/cmd_vel=0`。

### 5.6 看门狗

控制器或安全过滤器至少检查：

- 里程计年龄是否超过 `odom_timeout`；
- B 样条是否存在、是否合法、是否过期；
- 控制循环 `dt` 是否异常；
- 急停是否激活；
- 定位状态是否有效；
- 底盘驱动是否在线；
- 当前跟踪误差是否超过阈值；
- 输出是否出现 NaN/Inf。

任一关键条件失败：立即进入 `STOPPING/FAULT`，持续发布安全零速或调用底盘失能接口，不能只发布一次零速。

### 5.7 不要保留硬编码角速度上限

删除或参数化：

```cpp
static constexpr double kMaxVYawLimit = 1.0;
```

最终上限由 `swerve_controller.yaml` 给出，数值不得超过封装底盘对 `/cmd_vel` 公布并实测通过的能力。

### 5.8 重命名冻结话题

把：

```text
/planning/go2_execution_frozen
```

改成中性名称：

```text
/planning/execution_frozen
```

同步修改：

- `scan_replan_fsm.cpp` 的订阅；
- `scan_replan_fsm.h` 的变量和 subscriber 名称；
- 新舵轮控制器的 publisher；
- launch remap；
- RViz/调试脚本。

## 6. 已封装底盘的 `/cmd_vel` 接口约定

你们不需要实现舵轮逆运动学，但必须把 `/cmd_vel` 的接口合同确认清楚。

### 6.1 消息类型与字段

先检查底盘正在等待的真实类型：

```bash
ros2 topic type /cmd_vel
ros2 topic info -v /cmd_vel
```

本文按以下接口设计：

```text
topic: /cmd_vel
type:  geometry_msgs/msg/Twist

linear.x   机体系前后速度，m/s，向前为正
linear.y   机体系横向速度，m/s，向左为正
angular.z  绕车体 Z 轴角速度，rad/s，逆时针为正

linear.z = 0
angular.x = 0
angular.y = 0
```

如果实际类型是 `geometry_msgs/msg/TwistStamped`，安全过滤器应直接输出带 `header.stamp` 和 `header.frame_id=base_link` 的消息，不要同时在同名话题发布两种类型。

### 6.2 必须向底盘供应商或底盘程序确认的问题

- `/cmd_vel` 是机体系还是世界系；本文要求机体系；
- `linear.y` 是否真正执行横移，而不是被忽略；
- X/Y/旋转正方向是否符合 ROS REP-103；
- 单位是否为 m/s 和 rad/s；
- 底盘内部是否限速度、加速度和 jerk；
- 多自由度组合时是否自动缩放；
- 命令断流多久进入停车；
- 持续发布零速和停止发布命令的区别；
- 驱动故障、急停、失能状态通过什么接口反馈；
- QoS 是否与发布者兼容。

### 6.3 SCAN 到底盘的推荐连接

即使底盘直接吃 `/cmd_vel`，仍推荐保留一道软件安全层：

```text
swerve_trajectory_controller
  → /scan_planner/cmd_vel_raw (TwistStamped)
  → cmd_vel_safety_filter
  → /cmd_vel (底盘要求的 Twist)
  → 已封装舵轮底盘
```

安全层负责时间戳、限速、加减速度、jerk、急停和超时；底盘继续负责内部舵轮解算。这样没有重复实现底盘运动学。

### 6.4 唯一发布者规则

真机运行时 `/cmd_vel` 应只有安全过滤器一个发布者：

```bash
ros2 topic info -v /cmd_vel
```

如果遥控节点、旧控制器和 SCAN 同时发布，速度会互相覆盖。需要手动/自动切换时使用明确的 velocity mux，并让急停拥有最高优先级。

## 7. 改造碰撞模型和障碍物膨胀

### 7.1 当前模型的实际含义

当前 GridMap 先把每个占用体素在 XY 平面膨胀成半径为 `double_cylinder_radius` 的圆柱，然后在碰撞查询点前后偏移 `double_cylinder_offset`，分别检查两个圆柱中心。

这本质上是“两个相同圆柱近似长车体”，不是任意矩形 footprint。

### 7.2 全向舵轮最安全的第一阶段方案

第一阶段使用**车体外接圆**，让碰撞模型与 yaw 无关：

```text
double_cylinder_offset = 0
double_cylinder_radius = 0.5 × sqrt(L_safe² + W_safe²)

L_safe = L + 2 × margin_xy
W_safe = W + 2 × margin_xy
```

其中：

```text
margin_xy ≥ m_clear + e_loc + e_sensor + vxy_max × t_delay
```

优点是全向横移、斜移和原地旋转时不会因为预测 yaw 错误漏碰撞。缺点是在窄通道中比较保守。

### 7.3 两圆柱近似矩形

确认第一阶段闭环安全后，才考虑减小保守性。对安全尺寸 `L_safe × W_safe`，一个能覆盖矩形的对称初值是：

```text
offset = L_safe / 4
radius = sqrt((L_safe / 4)² + (W_safe / 2)²)
```

必须在 RViz 中画出两个圆柱，并对 0～360° 车身朝向做几何验证。不能简单使用：

```text
radius = W / 2
offset = (L - W) / 2
```

因为两个离散圆之间或矩形角部可能没有被完整覆盖。

### 7.4 更推荐的最终方案：定向矩形/多边形

舵轮底盘通常是矩形，且能横移和独立旋转。最终建议把 `getInflateOccupancy(pos,yaw)` 改为定向矩形 footprint 检查：

1. 定义 `footprint` 顶点；
2. 按计划车身 yaw 旋转顶点；
3. 在多边形内部采样占用栅格，或对障碍距离场查询；
4. 加入定位和控制误差形成的安全 margin；
5. 在 A*、优化器、目标修正和安全检查中使用同一个 footprint API。

当前这些位置都调用或间接调用了碰撞查询，改造时必须全局检查：

- `path_searching/dyn_a_star.h`；
- `bspline_opt/src/bspline_optimizer.cpp`；
- `plan_manage/src/scan_replan_fsm.cpp`；
- `plan_env/include/plan_env/grid_map.h`。

只改 FSM 的安全检查是不够的，搜索和优化阶段仍可能使用旧模型。

### 7.5 全向运动下的 yaw 一致性

当前多处代码用路径切线估算 yaw：

```text
yaw = atan2(next_y - y, next_x - x)
```

这对“车头沿运动方向”的机器人近似成立，对横移的舵轮机器人不成立。

有两种安全路线：

- 第一阶段使用外接圆，完全消除 yaw 对 footprint 的影响；
- 最终规划独立 yaw 轨迹，让规划器碰撞检查和控制器都读取同一时刻的 yaw。

在独立 yaw 轨迹完成之前，不要用紧贴车身的非圆 footprint 上真机。

### 7.6 Z 方向膨胀

参数：

```yaml
grid_map.obstacles_inflation_z_up
grid_map.obstacles_inflation_z_down
grid_map.ground_height
grid_map.body_height
```

注意代码是“围绕障碍体素向上/向下膨胀”，然后在机器人规划点的 z 高度查询。不能简单把 `H` 原样填到某一个参数。

先确定：

- 里程计 pose 的 z 表示 `base_link`、地面还是机体中心；
- 点云是否已经去除地面；
- 要检测的是低矮障碍、车身高度障碍还是悬空障碍；
- 机器人是否只在平面运行。

对舵轮机器人，推荐把点云分成“可通行地表”和“不可通行障碍”，由地表高程给轨迹提供 Z 参考，再用实测障碍样本标定上下膨胀。完全平地时可以使用固定 Z。若既把地面点当普通障碍保留，又把查询高度设置错误，整张地图可能被判定为碰撞。

## 8. 调整规划物理限制

### 8.1 当前参数不是完整舵轮动力学

当前规划器主要使用：

```yaml
manager.max_vel
manager.max_acc
manager.max_jerk
optimization.max_vel
optimization.max_acc
manager.feasibility_tolerance
```

其中速度和加速度限制对 XYZ 使用同一个标量。优化阶段有逐轴惩罚，最终动态可行性又检查三维轨迹导数的向量范数。坡道上的 Z 导数会计入空间轨迹速度/加速度，这是有意义的；但它不是底盘可直接执行的 `linear.z`。当前模型仍不包含坡度相关降速、底盘内部执行器模型、角速度、角加速度或横向/纵向不同能力。

### 8.2 第一阶段保守配置

在未完成各向异性规划约束前：

```text
planner_v_limit = min(vx_safe, vy_safe, vxy_safe)
planner_a_limit = min(ax_safe, ay_safe, axy_safe, a_brake_safe)
```

并保持：

```yaml
manager.max_vel:             planner_v_limit
optimization.max_vel:        planner_v_limit
manager.max_acc:             planner_a_limit
optimization.max_acc:        planner_a_limit
```

规划器和优化器的限制必须一致。控制器/安全层的限制应等于或略低于规划限制，不能让规划器认为可执行而底盘长期饱和。

`manager.feasibility_tolerance` 当前默认 `0.5`，意味着部分检查允许较大比例超限。真机初期应显著降低，并以日志确认轨迹不会频繁超限后再调整。

### 8.3 jerk 必须在执行层真正实现

当前 `manager.max_jerk` 只被加载，没有进入现有 B 样条可行性判定。真机不能因为 YAML 中出现该参数就认为 jerk 已受限。

至少在 `cmd_vel_safety_filter` 中实现平移和角 jerk 限制；长期方案是在轨迹可行性检查中加入 B 样条三阶导数约束，使规划轨迹本身就可执行。

### 8.4 制动距离和规划视野

保守制动距离：

```text
d_stop = v² / (2 × a_brake) + v × t_delay + margin_xy
```

应满足：

```text
local_update_range_xy > d_stop + 车体外接半径
planning_horizon      > d_stop + 车体外接半径
传感器有效距离        > d_stop + 车体外接半径
```

`fsm.emergency_time` 也不能随便沿用。它应大于从发现危险到安全停止的最坏时间，并通过实车低速制动测试验证。

### 8.5 栅格分辨率

`grid_map.resolution` 越小，边界更细但计算量按三维快速增加。选择时考虑：

- 最窄必须识别的障碍尺寸；
- 定位和点云噪声；
- 车体安全 margin；
- CPU 在实际点云频率下的耗时。

不要为了看起来精细就盲目设为 1 cm。先记录 GridMap 更新耗时、规划耗时和 CPU 占用，再决定。

## 9. 推荐参数文件结构

新增 `config/swerve_robot.yaml`，把真机参数集中管理。下面只展示字段结构，`<...>` 必须替换为实测值：

```yaml
scan_planner_node:
  ros__parameters:
    use_sim_time: false

    grid_map.frame_id: map
    grid_map.sliding_map_frame_id: sliding_map
    grid_map.resolution: <resolution_m>
    grid_map.sliding_map_size_x: <local_map_x_m>
    grid_map.sliding_map_size_y: <local_map_y_m>
    grid_map.sliding_map_size_z: <local_map_z_m>
    grid_map.local_update_range_x: <sensor_effective_x_m>
    grid_map.local_update_range_y: <sensor_effective_y_m>
    grid_map.local_update_range_z: <sensor_effective_z_m>

    # 第一阶段推荐外接圆
    grid_map.double_cylinder_radius: <circumscribed_radius_plus_margin_m>
    grid_map.double_cylinder_offset: 0.0
    grid_map.obstacles_inflation_z_up: <validated_z_up_m>
    grid_map.obstacles_inflation_z_down: <validated_z_down_m>
    grid_map.body_height: <base_or_body_reference_height_m>
    grid_map.ground_height: <map_ground_z_m>

    manager.max_vel: <conservative_planar_speed_mps>
    manager.max_acc: <conservative_planar_acc_mps2>
    manager.max_jerk: <documented_but_not_yet_enforced_mps3>
    manager.planning_horizon: <horizon_m>
    manager.feasibility_tolerance: <small_tolerance>

    optimization.max_vel: <same_as_manager_max_vel>
    optimization.max_acc: <same_as_manager_max_acc>
    optimization.dist0: <collision_optimization_clearance_m>
```

新增 `config/swerve_controller.yaml`：

```yaml
swerve_trajectory_controller:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    base_frame: base_link
    control_rate: 100.0
    heading_mode: hold

    kp_x: <value>
    kp_y: <value>
    kp_yaw: <value>

    max_vx: <value>
    max_vy: <value>
    max_vxy: <value>
    max_wz: <value>
    max_ax: <value>
    max_ay: <value>
    max_axy: <value>
    max_awz: <value>
    max_jxy: <value>
    max_jwz: <value>

    odom_timeout: 0.2
    command_timeout: 0.2
    trajectory_timeout: <value>
    max_position_error: <value>
    finish_distance: <value>
    finish_yaw_error: <value>
```

这里的数值不能从原项目默认值照抄。

## 10. 改造真机 launch

### 10.1 不要继续硬编码话题

新增 `swerve_real.launch.py`，至少声明：

```text
odom_topic
sensor_pose_topic
cloud_topic
depth_topic
goal_topic
initial_path_topic
cmd_vel_raw_topic
cmd_vel_topic
global_frame
base_frame
sensor_type
geometry_config
controller_config
```

launch 通过 remap 接线，而不是在 Python 中写死厂商话题。

### 10.2 删除旧机器人描述；URDF 按需使用

你们不使用仿真，而且底盘已经封装并直接接收 `/cmd_vel`，因此 **SCAN 真机运行并不要求提供 URDF**。`run.launch.py` 中无条件启动的旧平台 `robot_state_publisher` 应从真机启动链删除，而不是强制换成另一份 URDF。

这里要把三类数据分开：

1. **车体位姿**：机器人现在位于哪里、姿态如何。当前 SCAN 通过 `body_pose`（`nav_msgs/msg/Odometry`）直接接收，规划核心不需要从 URDF 获取。
2. **传感器位姿或外参**：点云如何从 LiDAR 坐标系转换到规划坐标系。当前代码通过 `sensor_pose` 加 `lidar_extrinsic_` 处理；改造后也可以按点云时间戳查询 TF。外参可以来自标定参数或 `static_transform_publisher`，并不要求完整 URDF。
3. **机器人可视模型**：只用于 RViz 显示车体各 link，或供明确依赖 `/robot_description` 的驱动、`ros2_control` 使用。这才是 URDF 的主要用途。

因此按实际系统选择：

- 定位同时提供正确的 `body_pose` 和 `sensor_pose`：不启动 `robot_state_publisher`，不需要 URDF；
- 定位只提供车体位姿，点云仍在 `lidar_link`：提供 `base_link → lidar_link` 的静态外参即可，可用 `tf2_ros static_transform_publisher`，仍不需要完整 URDF；
- 需要在 RViz 中显示完整舵轮机器人，或底盘驱动明确要求 `/robot_description`：再由底盘 bringup 加载自家 URDF 和一个 `robot_state_publisher`，SCAN 不重复加载。

注意：`robot_state_publisher` 主要根据 URDF 发布机器人各 link 之间的关系，通常**不会替定位系统发布** `map/odom → base_link`。全局到车体的动态位姿仍来自 LIO、SLAM 或定位节点。

### 10.3 建议的真机话题命名

```text
/localization/odom
/sensors/lidar/points
/goal_pose
/global_path
/planning/bspline
/scan_planner/cmd_vel_raw
/cmd_vel
/emergency_stop
/controller/status
```

厂商原始话题可以继续存在，通过 launch remap 到上述稳定接口。

## 11. SCAN-Planner 输入接口详解

以下是当前源码的真实接口，以及舵轮改造后的建议映射。

### 11.1 机体里程计

| 项 | 内容 |
|---|---|
| 节点内部话题 | `body_pose` |
| 当前真机映射 | `/LIO/odom_vehicle` |
| 建议映射 | `/localization/odom` |
| 类型 | `nav_msgs/msg/Odometry` |
| QoS | SensorDataQoS |
| 使用者 | FSM、GridMap 滑窗、轨迹控制器 |

必须满足：

- `header.frame_id` 是 `map` 或 `odom` 全局规划坐标；
- `child_frame_id` 是 `base_link`；
- pose 是机器人基准点在全局坐标系的位姿；
- 四元数合法且归一化；
- 时间戳单调、与传感器同一时间源；
- 更新频率和延迟满足控制要求。

**速度坐标系特别注意**：标准 Odometry 的 twist 通常表达在 `child_frame_id`。当前 FSM 直接读取数值并作为世界系初速度参与 B 样条。如果定位输出是机体系速度，应在 FSM 中根据 yaw 转到世界系，或增加一个 `odom_adapter` 输出明确的世界系速度。不要只改 frame 字符串而不变换数值。

### 11.2 LiDAR 点云

| 项 | 内容 |
|---|---|
| 节点内部话题 | `cloud` |
| 当前真机映射 | `/LIO/clouds_lidar` |
| 建议映射 | `/sensors/lidar/points` |
| 类型 | `sensor_msgs/msg/PointCloud2` |
| QoS | SensorDataQoS |
| 关键参数 | `grid_map.cloud_is_world` |

当 `cloud_is_world=false` 时，源码不使用 PointCloud2 的 TF 自动变换，而是把每个 XYZ 当成传感器坐标，再用最近一次 `sensor_pose` 旋转和平移到世界系。

因此必须保证：

- 点坐标确实位于预期传感器坐标系；
- 点云和传感器 pose 时间接近；
- 点中没有 NaN/Inf；
- 地面、机器人自身点和远距离噪声得到正确处理；
- `max_ray_length` 与传感器可靠量程匹配。

当前 LiDAR 点云与 pose 没有 message_filters 同步，只使用“最近一次 pose”。高速运动或高延迟网络下建议改为按点云时间戳查询 TF，或增加同步器。

### 11.3 传感器位姿

| 项 | 内容 |
|---|---|
| 节点内部话题 | `sensor_pose` |
| 当前真机映射 | `/LIO/odom_imu` |
| 建议方式 | 由 TF 按点云时间查询 `global_frame → lidar_link` |
| 当前类型 | `nav_msgs/msg/Odometry` |

当前 `need_extrinsic=true` 时还会应用 C++ 中写死的外参矩阵。舵轮机器人必须：

- 将矩阵改为本机标定值；或
- 推荐改为 tf2 查询，删除硬编码外参；或
- 上游直接发布已是 LiDAR 光心/原点的世界位姿，并设置 `need_extrinsic=false`。

三种方式只能选一种，不能在上游已经应用外参后又让 GridMap 应用一次。

### 11.4 深度图

| 项 | 内容 |
|---|---|
| 内部话题 | `depth` |
| 当前真机映射 | `/camera/aligned_depth_to_color/image_raw` |
| 类型 | `sensor_msgs/msg/Image` |
| 同步 | 与 `sensor_pose` 使用 ApproximateTime |

必须配置真实相机的 `cx/cy/fx/fy`、深度缩放、最小/最大距离和像素跳采样。使用 LiDAR 时可以不发布深度图。

### 11.5 手动目标

| 项 | 内容 |
|---|---|
| 内部/当前话题 | `/move_base_simple/goal` |
| 建议话题 | `/goal_pose` |
| 类型 | `geometry_msgs/msg/PoseStamped` |
| 使用条件 | `navi_mode=1` |

当前 FSM 只使用目标 XY，目标 z 被替换为首次里程计的 z，目标 orientation 被忽略。若需要舵轮到点朝向，必须另行保存 goal yaw 并交给 yaw 控制器/轨迹规划。

### 11.6 全局参考路径

| 项 | 内容 |
|---|---|
| 当前话题 | `/initial_path` |
| 建议话题 | `/global_path` |
| 类型 | `nav_msgs/msg/Path` |
| 使用条件 | `navi_mode=3` |

当前实现读取 poses 的位置，忽略每个姿态的 orientation，并额外给 z 加上 `body_height`。必须定义上游路径 Z 究竟是“地表高度”还是“base_link 高度”：只有前者才应该增加机体参考高度；如果路径已经表示 `base_link`，再次相加会导致整条轨迹错误抬高。

### 11.7 执行冻结和急停

当前 FSM 输入 `/planning/go2_execution_frozen`。应重命名为 `/planning/execution_frozen`。

此外需要新增 `/emergency_stop`。规划器内部的 `EMERGENCY_STOP` 只处理规划碰撞失败，不等价于硬件急停。

## 12. SCAN-Planner 输出接口详解

### 12.1 局部 B 样条

| 项 | 内容 |
|---|---|
| 话题 | `/planning/bspline` |
| 类型 | `scan_planner_msgs/msg/Bspline` |
| 主要字段 | `order`、`traj_id`、`start_time`、`knots`、`pos_pts` |

`pos_pts` 和 `knots` 定义位置 B 样条。执行器据此求位置、速度和加速度。当前 `yaw_pts/yaw_dt` 没有被 FSM 填充，不能当作有效 yaw 轨迹使用。

### 12.2 底盘速度

当前：

```text
/cmd_vel
geometry_msgs/msg/Twist
linear.x, linear.y, angular.z
```

推荐改造后：

```text
/scan_planner/cmd_vel_raw
geometry_msgs/msg/TwistStamped
frame_id = base_link

/cmd_vel
geometry_msgs/msg/Twist 或 TwistStamped
由安全过滤器输出给底盘
```

不要让规划控制器绕过安全过滤器直接连接电机驱动。

### 12.3 规划状态数据

| 话题 | 类型 | 说明 |
|---|---|---|
| `/planning/data_display` | `scan_planner_msgs/msg/DataDisp` | 规划过程中的五个标量数据 `a～e` |
| `/planning/execution_frozen` | `std_msgs/msg/Bool` | 是否冻结轨迹时钟 |
| `/self_inflation` | `visualization_msgs/msg/Marker` | 当前碰撞近似模型可视化 |

建议新增标准 diagnostics，明确 FSM 状态、最后轨迹时间、跟踪误差、限幅原因和故障码。

### 12.4 GridMap 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/grid_map/occupancy` | `sensor_msgs/msg/PointCloud2` | 原始占用体素 |
| `/grid_map/occupancy_inflate` | `sensor_msgs/msg/PointCloud2` | 膨胀后的占用体素 |
| `/grid_map/unknown` | `sensor_msgs/msg/PointCloud2` | 未知体素可视化 |
| `/grid_map/depth_cloud` | `sensor_msgs/msg/PointCloud2` | 深度图投影后的点云 |
| `/grid_map/sliding_map_bbox` | `visualization_msgs/msg/Marker` | 滑动地图边界 |
| `/grid_map/sensor_pose_extrinsic` | `nav_msgs/msg/Odometry` | 应用外参后的传感器位姿调试输出 |

GridMap 还发布 `global_frame → sliding_map` TF。

### 12.5 规划可视化输出

```text
/goal_point
/global_list
/init_list
/optimal_list
/a_star_list
```

类型均为 `visualization_msgs/msg/Marker`。当前部分可视化代码硬编码 `world`，另一些路径工具使用 `map`；真机改造时统一使用参数化 `global_frame`，否则 RViz 会出现数据存在但不显示。

## 13. 真机完整话题接线表

推荐最终接口：

| 上游节点 | 发布话题 | SCAN/控制链消费者 | 必需 |
|---|---|---|---|
| 定位/LIO | `/localization/odom` | planner、controller、sliding map | 是 |
| LiDAR 驱动 | `/sensors/lidar/points` | GridMap | LiDAR 模式是 |
| 定位或传感器位姿适配器 | `body_pose`，以及 `sensor_pose` 或 `global_frame→sensor` TF | GridMap/调试 | 是；传感器位姿消息与 TF 方案二选一 |
| 上层导航/RViz | `/goal_pose` | FSM mode 1 | mode 1 是 |
| 全局规划器 | `/global_path` | FSM mode 3 | mode 3 是 |
| SCAN planner | `/planning/bspline` | swerve controller | 是 |
| swerve controller | `/scan_planner/cmd_vel_raw` | safety filter | 是 |
| 急停/安全系统 | `/emergency_stop` | safety filter | 是 |
| safety filter | `/cmd_vel` | 舵轮底盘驱动 | 是 |
| 底盘驱动 | `/joint_states`/状态/故障 | diagnostics/TF | 强烈建议 |

### 13.1 当前接口到舵轮接口的迁移表

| 当前源码/launch 接口 | 舵轮真机建议接口 | 操作 |
|---|---|---|
| `body_pose` → `/LIO/odom_vehicle` | `/localization/odom` | launch 参数化 remap，并修正 twist 坐标语义 |
| `sensor_pose` → `/LIO/odom_imu` | 按时间查询 `map→lidar_link` TF | 修改 GridMap，避免使用最近 pose 和硬编码外参 |
| `cloud` → `/LIO/clouds_lidar` | `/sensors/lidar/points` | launch 参数化 remap |
| `depth` → RealSense 固定话题 | `/sensors/camera/depth` | 仅深度模式使用，参数化相机内参 |
| `/move_base_simple/goal` | `/goal_pose` | remap；若需要终点 yaw，修改 FSM 保存 orientation |
| `/initial_path` | `/global_path` | remap；统一 z 和 orientation 语义 |
| `/planning/go2_execution_frozen` | `/planning/execution_frozen` | FSM、controller、变量名和调试工具全部重命名 |
| controller 直接 `/cmd_vel` | `/scan_planner/cmd_vel_raw` | 改为 `TwistStamped`，先进入安全过滤器 |
| 无独立安全输出 | `/cmd_vel` | 由安全过滤器发布，底盘只订阅这一条 |
| 无外部急停输入 | `/emergency_stop` | 新增；硬件急停仍必须独立于 ROS |
| 真机无条件加载旧 URDF | 真机默认不加载机器人描述 | 删除旧 `robot_state_publisher`；只有可视化或驱动明确需要时，才由底盘 bringup 加载自家 URDF |

### 13.2 完成改造后的预期启动形式

下面是目标形态，不是当前仓库已经存在的 launch；只有完成第 4～10 节改造后才能使用：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/SCAN-Planner/install/setup.bash

ros2 launch scan_planner swerve_real.launch.py \
  odom_topic:=/localization/odom \
  cloud_topic:=/sensors/lidar/points \
  goal_topic:=/goal_pose \
  initial_path_topic:=/global_path \
  cmd_vel_raw_topic:=/scan_planner/cmd_vel_raw \
  cmd_vel_topic:=/cmd_vel \
  global_frame:=map \
  base_frame:=base_link \
  sensor_type:=lidar \
  navi_mode:=1 \
  geometry_config:=/absolute/path/to/swerve_geometry.yaml \
  controller_config:=/absolute/path/to/swerve_controller.yaml \
  use_sim_time:=false
```

launch 启动前应先确认定位、LiDAR、传感器位姿/外参、底盘驱动和硬件急停都已经由各自 bringup 独立验证。若采用 TF 转换点云，再额外确认 TF 时间和坐标链；若继续使用 `sensor_pose` 消息，则不应为了形式完整而强行增加 URDF。

## 14. 编译与静态检查

安装依赖：

```bash
cd /home/rainple/nav_test/SCAN-Planner
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
sudo apt install -y libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

编译：

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=OFF
source install/setup.bash
```

运行测试：

```bash
colcon test
colcon test-result --verbose
```

执行器改造后至少新增单元测试：

- 世界系到机体系速度旋转；
- X/Y/合成速度限幅；
- 加速、制动、角加速度和 jerk 限制；
- NaN/Inf 拒绝；
- 里程计/轨迹超时持续零速；
- 急停优先级；
- `/cmd_vel` 三个有效自由度的符号与单位；
- X/Y/旋转单独及组合限幅；
- 命令断流后的停车行为；
- 横移时 footprint 使用正确 yaw 或外接圆。

## 15. 不接电机的接口验收

先只启动定位、传感器、SCAN 和控制器，底盘驱动保持断使能。

### 15.1 检查输入类型和 QoS

```bash
ros2 topic list -t
ros2 topic info -v /localization/odom
ros2 topic info -v /sensors/lidar/points
ros2 topic hz /localization/odom
ros2 topic hz /sensors/lidar/points
```

不要直接 `echo` 完整 PointCloud2，数据量很大。

### 15.2 检查时间和 TF

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map lidar_link
```

检查：

- 位姿连续，无跳变；
- 静止时 yaw 不漂移到不可接受范围；
- 点云时间戳不是零，也不是另一台电脑的错误时间；
- TF 能查询到点云时间附近，而不只是“现在”；
- 真机使用 `use_sim_time=false`，除非系统确实有统一仿真时钟。

### 15.3 检查地图和 footprint

```bash
ros2 topic info -v /grid_map/occupancy
ros2 topic info -v /grid_map/occupancy_inflate
ros2 topic hz /grid_map/occupancy_inflate
```

在 RViz 中：

1. Fixed Frame 设为 `map`；
2. 观察原始点云是否与真实墙面重合；
3. 推动机器人但不使能电机，观察滑动地图是否跟随；
4. 用已知尺寸纸箱放在车体前、后、左、右；
5. 检查膨胀区域是否覆盖所有不允许车体中心进入的位置；
6. 原地改变车体 yaw，确认 footprint 没有漏角。

### 15.4 检查控制输出但不接驱动

```bash
ros2 topic echo /scan_planner/cmd_vel_raw
```

分别下发前、后、左、右、斜向和旋转目标，确认符号：

```text
前进  linear.x > 0
左移  linear.y > 0
逆时针 angular.z > 0
```

触发急停、断开里程计、停止发布点云、让轨迹过期，确认安全输出持续为零并给出明确故障状态。

## 16. 真机分阶段验收

### 阶段 0：机械和安全

- 硬件急停可切断驱动；
- 有独立安全员；
- 机器人周围设置隔离区；
- 轮胎、舵向零位、编码器方向和限位已检查；
- 底盘自身看门狗已启用。

### 阶段 1：架空/支撑测试

不启动 SCAN，只用受控测试节点分别发送极小的：

- `+vx/-vx`；
- `+vy/-vy`；
- `+wz/-wz`；
- 组合 `vx+vy`；
- 组合 `vx+wz`。

核对底盘的实际前后、横向和旋转方向，不允许方向靠猜。轮级行为由封装底盘自行保证。

### 阶段 2：落地开环底盘测试

- 速度限制为最终计划值的 10%～20%；
- 测试直行、横移、斜移、原地旋转；
- 记录命令与里程计响应；
- 测出延迟、最大制动距离、横向漂移和 yaw 超调；
- 回填参数表。

### 阶段 3：只跟踪无障碍短轨迹

- SCAN 感知保持运行；
- 目标距离 0.5～1 m；
- 第一阶段使用外接圆；
- heading mode 使用 `hold`；
- 检查位置误差、速度饱和和停止精度。

### 阶段 4：静态障碍

- 使用软质障碍物；
- 从低速开始；
- 依次测试正面、侧面、角部和狭窄通道；
- 专门测试横移时车体角部；
- 验证规划失败时安全停车。

### 阶段 5：动态障碍与路线引导

前四阶段稳定后，才接入 `/global_path` 和动态障碍测试。记录每次规划耗时、最小障碍距离、命令延迟、制动距离和故障码。

## 17. 参数调试顺序

一次只调一层：

1. 坐标系、时间戳、外参；
2. 点云滤波和地面处理；
3. 车体 footprint 与膨胀；
4. 底盘速度、加速度、jerk 和 `/cmd_vel` 超时限制；
5. 规划器 `max_vel/max_acc`；
6. 控制器 `kp_x/kp_y/kp_yaw`；
7. 规划视野、重规划与紧急时间；
8. 最后才调优化代价权重。

如果坐标或 footprint 还没正确，不要通过调大优化权重来掩盖问题。

## 18. 上真机前的强制检查清单

- [ ] 机器人明确是全向舵轮，底盘接口能独立执行 `vx/vy/wz`；
- [ ] 原旧平台 URDF、`robot_state_publisher`、话题和命名已从真机 launch 移除；
- [ ] `body_pose` 正确；传感器位姿采用 `sensor_pose` 或 TF 中的一种，外参和时间正确；
- [ ] Odometry 的 pose 和 twist 坐标语义已确认；
- [ ] LiDAR 点云坐标和外参只转换一次；
- [ ] 地面与机器人自身点不会把整车判为碰撞；
- [ ] 第一阶段 footprint 使用包含安全 margin 的外接圆；
- [ ] 横移时碰撞检查不再假设车头沿路径切线；
- [ ] 规划器和控制器速度/加速度限制来自实测；
- [ ] jerk 在执行安全层中真正生效；
- [ ] `/cmd_vel` 的类型、坐标系、单位和三个自由度方向已经实测；
- [ ] 底盘公开限制和组合运动能力已经实测；
- [ ] 里程计、轨迹、通信和驱动超时都会持续安全停机；
- [ ] 硬件急停不依赖 ROS 2；
- [ ] `/cmd_vel` 前有安全过滤器，规划控制器不直接接电机；
- [ ] 不接驱动的接口验收全部通过；
- [ ] 架空测试和低速落地测试全部通过；
- [ ] 所有测试都有 rosbag、日志和参数版本记录。

只有全部通过，才进入有障碍的自主导航测试。
