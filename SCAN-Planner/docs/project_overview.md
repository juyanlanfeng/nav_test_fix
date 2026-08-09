# SCAN-Planner 项目总览（小白友好版）

> 面向路线引导四足长程导航的空间碰撞感知局部规划器（ROS 2 Humble 移植版）。
> 本文从零开始介绍：这个项目是做什么的、代码怎么组织、系统是怎么跑起来的、每一步背后发生了什么。

---

## 目录

1. [项目简介](#1-项目简介)
2. [预备知识（看不懂代码前先看这里）](#2-预备知识)
3. [整体架构：一张图](#3-整体架构)
4. [仿真模块：让规划器"以为"自己在真实世界](#4-仿真模块)
5. [感知与地图模块：GridMap](#5-感知与地图模块gridmap)
6. [规划模块：全局轨迹 + 局部 B 样条优化](#6-规划模块)
7. [状态机 FSM：规划的"大脑"](#7-状态机-fsm)
8. [控制器：把轨迹变成速度指令](#8-控制器)
9. [完整运行机制：一条数据怎么在系统里流动](#9-完整运行机制)
10. [参数配置详解](#10-参数配置详解)
11. [代码阅读指南](#11-代码阅读指南)

---

## 1. 项目简介

**SCAN-Planner** 是一个给**四足机器人（宇树 Go2）**使用的**局部路径规划器**。所谓"局部规划"，就是机器人在知道自己要去哪（全局目标）的前提下，根据传感器实时看到的环境，**在局部范围内规划出一条安全、平滑、能避开障碍物的运动轨迹**，并不断"边看边改"。

这个仓库是 ROS 2 Humble 的原生移植版（原项目是 ROS 1）。核心算法、参数文件结构与原版一致，但所有 ROS 接口都改成了 ROS 2 风格（rclcpp、colcon、launch 文件等）。

几个关键特点：

- **面向四足**：速度上限低（0.75 m/s）、加速度上限低（0.5 m/s²），并且把机器人自身建模成"**双圆柱**"形状来做碰撞检测（四足机器人身体长，用两个圆柱更贴合）。
- **空间碰撞感知**：障碍物不仅做平面膨胀，还做**三维膨胀**（z 方向上下都膨胀），规划器生成的是 3D 轨迹，不只是平面 2D 路径。
- **自带一套确定性仿真器**：不依赖 Gazebo 物理仿真也能完整跑通（默认模式），适合调试和回归测试；Gazebo Fortress 物理仿真作为可选方案。
- **"路线引导"（route guidance）**：支持先有一条全局参考轨迹（比如一条路径/一串路径点），规划器在局部范围内对它做避障优化——这是论文里"路线引导"的含义。

---

## 2. 预备知识

### 2.1 ROS 2 基础概念

| 概念 | 通俗解释 |
|------|---------|
| **节点（Node）** | 一个独立的程序进程，干一件专门的事，比如"规划"、"控制"、"仿真传感器" |
| **话题（Topic）** | 节点之间通信的"广播频道"。发布者往频道里发消息，订阅者收消息。一对多 |
| **消息（Message）** | 话题里传的数据，有类型定义，比如 `nav_msgs/Odometry`（位置速度） |
| **参数（Parameter）** | 节点的配置项，运行时可以改。本项目的参数放在 YAML 文件里，名字用点号分隔，如 `grid_map.resolution` |
| **launch 文件** | 一键启动多个节点并配置好它们之间关系的脚本（Python） |
| **remapping** | 把节点内部用的话题名映射到外部实际话题名，比如把规划器内部的 `cloud` 映射到 `/quad_0/cloud` |
| **回调（Callback）** | 收到话题消息或定时器触发时自动执行的函数。ROS 节点平时"没事干"，靠回调驱动 |

本项目几乎所有节点都遵循一个模式：**程序入口创建一个 `rclcpp::Node`，然后把它传给一个普通 C++ 类，由这个类去建订阅、发布、定时器**。例如 `scan_planner_node.cpp` 创建节点后交给 `SCANReplanFSM::init(node)`。

### 2.2 算法概念

| 概念 | 通俗解释 |
|------|---------|
| **占用栅格地图（Occupancy Grid）** | 把空间切成一个个小立方体（体素），每个体素记录"这里有障碍物的概率"。本项目分辨率 0.05 m，即 5 cm 一个格子 |
| **log-odds（对数几率）** | 存储概率的数学技巧。把概率 p 换成 `log(p/(1-p))`，好处是**多次观测可以直接相加**，不用乘。命中障碍物加正数，穿过（没碰到）加负数 |
| **射线投射（Raycasting）** | 模拟激光/深度射线：从传感器位置发射一条线到障碍物点，**线上的格子标记为"被穿过"（miss），终点格子标记为"命中"（hit）**。这是建图的核心 |
| **B 样条（B-Spline）** | 用一串"控制点"定义的光滑曲线。轨迹用 B 样条表示后，**调整控制点 = 调整轨迹**，且轨迹天然光滑。3 次 B 样条保证位置连续且加速度连续 |
| **梯度下降 / LBFGS** | 数学优化方法。定义一个"代价函数"（违反约束越多代价越大），然后迭代地朝让代价变小的方向调整控制点。LBFGS 是带历史信息加速的梯度下降 |
| **代价函数（Cost Function）** | 把"轨迹好不好"量化成一个数：平滑性代价 + 碰撞代价 + 物理可行性代价 + 贴参考轨迹代价，加权求和 |
| **状态机（FSM）** | 程序在几个固定状态之间切换：初始化 → 等目标 → 生成轨迹 → 重规划 → 执行 → 紧急停止。任何时刻程序只处于一个状态 |
| **min-snap（最小加加速度）** | 用多项式函数生成一条经过若干点的光滑轨迹，作为优化前的"初值" |

---

## 3. 整体架构

### 3.1 工作空间布局

```
SCAN-Planner/                     ← 就是 colcon 工作空间根目录
├── src/
│   ├── planner/                  ← 规划栈（本项目的核心）
│   │   ├── scan_planner_msgs/    ← 自定义消息（Bspline、DataDisp）
│   │   ├── plan_env/             ← 占用栅格地图 GridMap（感知）
│   │   ├── path_searching/       ← A* 路径搜索（DynAStar）
│   │   ├── bspline_opt/          ← B 样条 + LBFGS 优化（核心算法库）
│   │   ├── traj_utils/           ← 可视化、多项式轨迹工具
│   │   └── plan_manage/          ← 应用层：节点、FSM、控制器、launch、配置
│   └── simulator/                ← 仿真
│       ├── mockamap/             ← 程序化生成环境点云（随机地图/迷宫等）
│       ├── map_generator/        ← 读取 PCD 文件发布点云地图
│       ├── local_sensing/        ← 模拟传感器：从全局地图"渲染"出激光/深度数据
│       └── Utils/
│           ├── go2_description/  ← Go2 机器人 URDF + Gazebo 物理仿真
│           ├── odom_visualization/
│           ├── waypoint_generator/
│           └── pose_utils/
├── map.pcd                       ← 现成的点云地图文件（48 MB，测试用）
├── build/ install/ log/          ← colcon 构建产物
└── docs/                         ← 本文档
```

### 3.2 依赖关系（构建顺序）

```
scan_planner_msgs（消息定义，最底层）
        ↓
plan_env → path_searching → bspline_opt → traj_utils
        ↓
           plan_manage（依赖上面所有库）
```

改 `plan_env`、`bspline_opt` 等库的头文件后，**必须重新构建对应库**，`plan_manage` 里跑的才是新代码（`colcon build --packages-select <包名>` 逐个构建）。

### 3.3 运行时的节点全景（默认仿真 + 闭环模式）

```
┌─────────────────────────────────────────────────────────────────┐
│                         run.launch.py 启动                        │
│                                                                 │
│  ┌─────────────┐   body_pose(100Hz)   ┌──────────────────────┐  │
│  │go2_kinematic │ ───────────────────▶ │  scan_planner_node   │  │
│  │    _sim      │ ◀─────────────────── │  (FSM + 规划器)        │  │
│  │  (仿真机器人)  │     cmd_vel(10ms)    │                      │  │
│  └──────┬──────┘                      └─────────┬────────────┘  │
│         │body_pose                             │planning/bspline│
│         ▼                                       ▼               │
│  ┌─────────────┐                     ┌──────────────────────┐  │
│  │local_sensing │  cloud+sensor_pose │     closed_loop_      │  │
│  │  (传感器仿真)  │ ─────────────────▶ │    controller(控制器)   │  │
│  └──────┬──────┘     10Hz           └──────────────────────┘  │
│         │global_map                                            │
│         ▼                                                      │
│  ┌─────────────┐   mock_map / global_cloud                     │
│  │ mockamap /  │◀───────────────────────────────────────────   │
│  │ map_generator│                                              │
│  └─────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
```

> 闭环（closed_loop）：规划器算轨迹 → 控制器把轨迹变成 `cmd_vel` → 仿真机器人积分速度得到 `body_pose` → 传感器仿真根据新位置"看到"新环境 → 地图更新 → 规划器再规划……形成一个完整的感知-规划-控制循环。

---

## 4. 仿真模块

仿真的目标：**让规划器收到的传感器数据和真实世界一致**，这样规划算法本身可以被独立调试。

### 4.1 环境地图（两种来源）

| 来源 | 包/节点 | 说明 |
|------|---------|------|
| 程序化生成 | `mockamap` | 用噪声/算法生成随机地图（`type` 可选：1=柏林噪声地形、2=随机障碍物、3=2D 迷宫、4=3D 迷宫），默认 type=2，40×40×5 m，500 个随机障碍物。发布 `mock_map` 话题 |
| 真实点云地图 | `map_generator` | 读取 PCD 文件（如仓库根目录的 `map.pcd`）发布为 `global_cloud`，低频（0.2 Hz）发布即可 |

### 4.2 传感器仿真（local_sensing）

这是"确定性仿真"的核心。它订阅机器人的 `body_pose`，从全局地图点云中**实时渲染**出传感器数据：

- **深度相机**（`sensor_type:=depth`）：把全局地图点投影到相机平面上，用"z-buffer"方式（保留最近的）生成深度图，模拟 640×480 的 RealSense 深度相机。
- **激光雷达**（`sensor_type:=lidar`，默认）：按雷达线束模型（可配置 360° 水平视场、90° 垂直视场、最小探测距离等）生成点云。
- 传感器位姿（`sensor_pose`）也一并发布，供建图模块做坐标变换。

有两个实现：`pcl_render_node`（CPU，默认）和 `opengl_render_node`（GPU，需 `-DUSE_GPU=ON` 构建，依赖系统 GLFW/GLEW）。

> 注意：这个仿真**没有物理**——机器人运动由 `go2_kinematic_sim` 纯运动学积分得到（见 8.2 节）。Gazebo Fortress + 宇树 Go2 的物理仿真在 `go2_description` 包里，是可选方案（`go2_sim.launch.py`），提供 12 关节控制、IMU、足端接触力等。

---

## 5. 感知与地图模块：GridMap

文件：[grid_map.cpp](src/planner/plan_env/src/grid_map.cpp)、[grid_map.h](src/planner/plan_env/include/plan_env/grid_map.h)、[raycast.cpp](src/planner/plan_env/src/raycast.cpp)

GridMap 是"空间碰撞感知"的核心，把传感器原始数据变成规划器能查询的占用地图。它作为 `GridMap` 类被规划管理器持有（`planner_manager_->grid_map_`），规划器查询"某个点是否被占用"来检测碰撞。

### 5.1 数据结构

- 一个固定大小的三维数组 `occupancy_buffer_`，每个体素存一个 **log-odds 值**（默认分辨率 0.05 m，地图 10×10×5 m → 200×200×100 个体素）。
- 初始值：`clamp_min_log - unknown_flag`，表示"未知"。
- 每个体素还有一个**膨胀标记** `occupancy_buffer_inflate_`（0/1）和计数 `occupancy_buffer_inflate_cnt_`。

### 5.2 建图流程（射线投射，每 50 ms 执行一次）

```
激光点云 / 深度图
    │
    ▼
① 把点变换到世界系（cloud_is_world=true 时直接用；false 时用 sensor_pose 变换）
    │
    ▼
② 对每个点：终点格子记"命中"(hit)，从传感器到终点的射线经过的格子记"穿过"(miss)
    │（用 RayCaster 逐格走，网格越小步长越细）
    ▼
③ 每个格子用计数投票：这一帧里 hit 次数 ≥ miss 次数 → 加 p_hit 的 log-odds，否则加 p_miss
    │
    ▼
④ log-odds 累加（多次观测不断更新），并钳制在 [clamp_min, clamp_max]
    │
    ▼
⑤ 格子从"未占用"变"占用"（超过阈值 p_occ）时，更新膨胀层
```

关键参数（`planner.yaml`）：

- `p_hit: 0.85` / `p_miss: 0.30`：一次命中/穿过对概率的影响。转成 log-odds 后 `prob_hit_log ≈ 1.735`，`prob_miss_log ≈ -0.847`
- `p_min: 0.12` / `p_max: 0.98`：概率钳制上下限（防止观测次数过多导致值爆炸）
- `p_occ: 0.80`：超过这个概率视为"被占用"
- `max_ray_length: 5.0`：射线最大长度，超过则只更新到 5 m 处
- `local_update_range`：只更新传感器附近 5×5×2.5 m 的区域（远处观测不值得更新，也省计算）

### 5.3 膨胀层（双圆柱自身体模型）—— 本项目特色

普通 2D 规划器把机器人当成一个圆；Go2 是**长条形的四足机器人**，所以用**两个圆柱**近似身体（前后各一个，半径 0.25 m，间距 0.18 m），z 方向上下也膨胀（上 0.1 m、下 0.4 m，给腿和身体留空间）。

- 预计算膨胀偏移表 `inflate_offsets_`（一个圆柱半径内的所有体素偏移）。
- 某个格子变占用时，把偏移表内的格子计数 +1；变空闲时 -1。计数 > 0 的格子标记为"已膨胀"。
- 规划器查询碰撞时用 `getInflateOccupancy()`——把查询点按**机器人当前朝向**同时查两个圆柱的膨胀区域，任何一个被占用即视为碰撞。这就是"空间碰撞感知"。

### 5.4 滑动地图

机器人走远了怎么办？地图窗口**跟随机器人滑动**（`map_sliding_en: true`）：

- 机器人移动超过阈值（0.2 m）时，把整个地图窗口平移，丢掉身后超出范围的数据（`updateSlidingMap`）。
- 地图原点移到新位置，出界的体素清零。窗口始终以机器人为中心（10×10×5 m）。

---

## 6. 规划模块

### 6.1 两个层次的轨迹

| 层次 | 内容 | 何时生成 |
|------|------|---------|
| **全局轨迹（global）** | 从起点到目标的整条光滑多项式轨迹（min-snap），仅用于提供"大方向" | 收到目标时生成一次 |
| **局部轨迹（local）** | 从全局轨迹上取一小段（约 7.5 m 的规划视野），用 B 样条 + 优化得到可执行轨迹 | 每 10 ms 可能重规划 |

**为什么分两层？** 全局轨迹不管障碍物（只保证光滑），局部规划器只盯着眼前一段做避障。这样每次优化的问题规模小（控制点少），10 ms 内能算完，机器人才能"边跑边想"。

### 6.2 全局轨迹生成

文件：`planner_manager.cpp` 的 `planGlobalTraj()` / `planGlobalTrajWaypoints()`

1. 起点 + 目标点（或一串路径点）→ 距离超过 4 m 的线段自动插入中间点。
2. 每段时间 = 距离 / 最大速度（首尾段 ×2，起停要缓）。
3. 用 `PolynomialTraj::minSnapTraj` 生成一条最小加加速度的多项式轨迹，存入 `global_data_`。

### 6.3 局部规划三步走（核心：reboundReplan）

文件：`planner_manager.cpp` 的 `reboundReplan()`，算法实现在 `bspline_opt` 包。

```
STEP 1  INIT    生成初始 B 样条
         - 首次/换新目标：用全局轨迹采样出一串点（间距约 0.2 m），参数化成 B 样条
         - 连续重规划：沿用上一条轨迹，采样 + 末端接一段多项式到局部目标
         - 失败多次时：随机扰动中间点，尝试不同的初值（"随机多项式"）
         - 用 A* 搜索障碍物方向，给每个控制点标出"该往哪边躲"（rebound 机制）

STEP 2  OPTIMIZE  B 样条优化（LBFGS 梯度下降）
         代价函数 = λ1·平滑性 + λ2·碰撞 + λ3·物理可行性 + λ4·贴参考
           λ1=1.0  λ2=1.0  λ3=0.1  λ4=1.0
         - 平滑性：加加速度（jerk）平方和
         - 碰撞：控制点到障碍物的距离低于安全距离(0.2 m)时产生代价，
           梯度方向 = A* 给出的躲障方向
         - 可行性：速度/加速度超过上限的部分产生三次方惩罚
         - 贴参考：轨迹偏离参考路径的部分产生代价

STEP 3  REFINE   时间重分配
         - 检查轨迹速度/加速度是否超限（checkFeasibility）
         - 超限则把时间轴拉长（lengthenTime），重新采样、再优化一次
         - 最终再采样检查动态可行性（checkDynamicFeasibility）
```

三个环节都通过后，轨迹存入 `local_data_`（位置/速度/加速度三个 B 样条），并由 FSM 发布出去。任何一步失败就整体失败，由 FSM 决定重试或紧急停止。

> 物理极限（`planner.yaml`）：`max_vel: 0.75 m/s`、`max_acc: 0.5 m/s²`、`max_jerk: 4.0`——四足机器人的运动能力远低于无人机，这些值直接约束轨迹的激进程度。

### 6.4 局部目标怎么选（getLocalTarget）

FSM 每次重规划前调用 `getLocalTarget()`：

1. 沿全局轨迹从 `last_progress_time_` 往后扫，找到第一个**离当前位置 ≥ planning_horizon（7.5 m）**的点作为局部目标。
2. 如果局部目标被占用，沿轨迹前后搜索最近的空闲点替代。
3. 离终点足够近时（以 v²/2a 的刹车距离衡量），直接以终点为目标并把速度设为 0。

---

## 7. 状态机 FSM

文件：[scan_replan_fsm.cpp](src/planner/plan_manage/src/scan_replan_fsm.cpp)、[scan_replan_fsm.h](src/planner/plan_manage/include/plan_manage/scan_replan_fsm.h)

FSM 是规划器的"大脑"，两个定时器驱动它：

| 定时器 | 周期 | 回调 | 职责 |
|--------|------|------|------|
| `exec_timer_` | **10 ms** | `execFSMCallback` | 状态机主循环：决定何时规划、何时执行 |
| `safety_timer_` | **50 ms** | `checkCollisionCallback` | 安全检查：采样检查当前轨迹是否撞上障碍物 |

### 7.1 状态转移图

```
       收到里程计 + 收到目标
INIT ─────────────────────▶ WAIT_TARGET
                              │ 有目标（have_target_）
                              ▼
                           GEN_NEW_TRAJ ──规划成功──▶ EXEC_TRAJ
                              │                        │  │
                          规划失败（重试）              │  │ 轨迹走完→回WAIT_TARGET
                              │                        │  │ 偏离过大→REPLAN_TRAJ
                              └────────────────────────┘  │
                                                          │
                           REPLAN_TRAJ ──规划成功──▶ EXEC_TRAJ
                              │
                          规划失败（计数超限）
                              ▼
                           EMERGENCY_STOP ──停稳后──▶ GEN_NEW_TRAJ 或 WAIT_TARGET
```

### 7.2 EXEC_TRAJ 状态里在做什么（10 ms 一次）

拿到当前轨迹后，每 10 ms 检查一次：

1. `t_cur > duration`：轨迹走完了 → 回 WAIT_TARGET 等新目标（路径点模式下切下一个路径点）。
2. 离终点很近（< `no_replan_thresh` 0.1 m）：什么都不做，继续执行。
3. 离起点很近（< `replan_thresh` 1.0 m）：刚出发，轨迹还新鲜，不重规划。
4. 否则：**重规划**（进入 REPLAN_TRAJ）。原因通常是偏离轨迹了或地图更新后旧轨迹不再安全。

重规划时（`planFromCurrentTraj`）以当前里程计为起点、当前轨迹的速度/加速度为初速度/初加速度，保证新旧轨迹平滑衔接。

### 7.3 安全检查（50 ms 一次）

`checkCollisionCallback` 沿当前轨迹逐点（0.01 s 步长）查询 `getInflateOccupancy`：

- **前方 2/3 段**是检查范围（最后 1/3 留给规划器刹车）。
- 发现碰撞 → 先尝试立即重规划（"给一次机会"）。
- 重规划失败且碰撞点距当前 < `emergency_time`（1 s）：进入 **EMERGENCY_STOP**，发布一个停在当前点的轨迹（6 个控制点全在同一点），机器人原地停下。
- 重规划失败但碰撞还很远：进入 REPLAN_TRAJ 慢慢想办法。

### 7.4 轨迹冻结机制（go2_execution_frozen）

四足机器人转向慢。当控制器发现**朝向偏差过大**（比如轨迹要求转向 90°，机器人得先转过去）时，会发布 `planning/go2_execution_frozen = true`（冻结执行），FSM 收到后把轨迹的 `start_time_` 不断往后推（`updateLocalTrajTimeFreeze`）。

效果：**轨迹的"时钟"暂停了**——机器人停在原地转头，但轨迹不会因为时间流逝而"过期"。转好了控制器发布 false，轨迹时钟继续走。

### 7.5 三种导航模式（navi_mode）

| 模式 | 目标来源 | 说明 |
|------|---------|------|
| 1 | RViz 2D Goal 工具 | 在 RViz 里点目标点，订阅 `move_base_simple/goal`。目标高度取当前机体高度 |
| 2 | 预设路径点 | `fsm.waypoints` 参数（xyz 三元组扁平数组），到达一个后自动规划下一个。路径点文件用 `keypoint_recorder.py` 录制 |
| 3 | 参考路径 | 订阅 `initial_path`（全局路径），沿路径逐段做局部避障——对应"路线引导"论文场景 |

模式 3 收到路径后会把每个点的高度加上 `body_height`（0.4 m，因为路径通常是地面高度，而机体中心离地 0.4 m）。

---

## 8. 控制器

### 8.1 闭环控制器（closed_loop_controller，默认）

文件：[closed_loop_controller.cpp](src/planner/plan_manage/src/closed_loop_controller.cpp)

订阅 `planning/bspline` 和 `body_pose`，每 10 ms 算一次 `cmd_vel`：

1. **期望朝向**：看轨迹上当前时间再往前 0.8 s 的点，算"该朝哪个方向"。
2. **朝向偏差** > 0.8 rad（约 46°）：发布"冻结执行"信号 + 只转不走的 `cmd_vel`，先转头。
3. 朝向 OK：**位置误差反馈**（`kp_pos` 比例增益）+ 轨迹期望速度，合成目标速度向量，限制在 `max_vx/max_vy` 内，旋转到机体坐标系后发布。
4. 轨迹走完且误差 < `finish_dist`（0.15 m）：发零速度，任务完成。

### 8.2 仿真机器人（go2_kinematic_sim）

文件：[go2_kinematic_sim.cpp](src/planner/plan_manage/src/go2_kinematic_sim.cpp)

没有物理的"假机器人"：订阅 `cmd_vel`，每 10 ms 按 `x += v·dt` 积分位置和朝向，发布 `body_pose` 里程计（100 Hz）。带超时保护：0.3 s 没收到新指令就自动停下。

> 开环模式（`controller_mode:=open_loop`）下没有这个闭环，`open_loop_controller` 直接沿着轨迹求值发布 `body_pose`——机器人"完美跟随"轨迹，适合验证规划器本身，跳过控制器问题。

### 8.3 go2_gait_publisher

纯可视化节点：根据运动速度生成四条腿的摆动关节角（trot 步态，频率 2.2 Hz），发布 `joint_states`，让 RViz 里的 Go2 模型看起来在"走路"。不影响任何算法。

---

## 9. 完整运行机制

### 9.1 启动一条命令发生了什么

```
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 \
  sensor_type:=lidar controller_mode:=closed_loop
```

1. `run.launch.py` 校验参数（sensor_type/controller_mode/navi_mode 必须合法，navi_mode=2 必须有路径点文件）。
2. 启动 `scan_planner_node`（参数 = planner.yaml + 启动参数覆盖 + 话题重映射）。
3. 启动 `robot_state_publisher`（加载 Go2 的 xacro 模型）。
4. 按 controller_mode 启动 `closed_loop_controller` + `go2_kinematic_sim`，或 `open_loop_controller`。
5. 仿真模式下再启动 `go2_gait_publisher` 和 `simulator.launch.py`（mockamap/map_generator + 传感器仿真 + odom_visualization）。

### 9.2 一次完整导航的时序（手动模式，闭环仿真）

```
t=0s    各节点启动。go2_kinematic_sim 开始发 body_pose
t≈0.1s  scan_planner_node 收到第一个里程计 → FSM 从 INIT 进入 WAIT_TARGET
         （odometryCallback 里记录 rviz_goal_height = 当前机体高度）
t=5s    用户在 RViz 点 2D Goal → 收到 move_base_simple/goal
        waypointCallback：
         - 生成全局轨迹（min-snap）
         - 检查终点是否被占用，被占用则沿轨迹找最近空闲点
         - have_target_=true → FSM 进入 GEN_NEW_TRAJ
t=5.01s FSM(GEN_NEW_TRAJ)：
         - getLocalTarget() 取 7.5 m 视野内的局部目标
         - reboundReplan()：多项式初值 → LBFGS 优化 → 可行性检查
         - 成功 → 发布 planning/bspline（含控制点、节点向量、时间戳、编号）
         - 进入 EXEC_TRAJ
t=5.02s closed_loop_controller 收到 B 样条，开始 10ms 周期：
         - 朝向偏差 < 0.8 rad → 正常跟踪：位置反馈 + 期望速度 → cmd_vel
         - 朝向偏差 > 0.8 rad → 发 frozen=true + 原地转向
        go2_kinematic_sim 积分 cmd_vel → 新 body_pose
        local_sensing 按新位置渲染出新的激光/深度数据（10 Hz）
t=5.05s GridMap 收到新点云 → 更新占用栅格 + 膨胀层（50 ms 周期）
        FSM 安全检查（50 ms）：采样当前轨迹，查碰撞
t=5.1s+ FSM(EXEC_TRAJ) 每 10ms 检查：
         - 偏离轨迹 > 1.0 m → REPLAN_TRAJ（用当前状态重新规划）
         - 地图出现新障碍 → 安全检查失败 → 尝试重规划
t=...   （重复以上循环，机器人边前进边刷新轨迹）
t=结束  轨迹走完，距离终点 < 0.15 m → 控制器发零速 → FSM 回 WAIT_TARGET
```

### 9.3 关键话题清单

| 话题 | 类型 | 发布者 → 订阅者 | 说明 |
|------|------|----------------|------|
| `body_pose` | nav_msgs/Odometry | 仿真机器人/实机里程计 → 规划器、控制器、传感器仿真、GridMap | 机体位姿（实机是 `/LIO/odom_vehicle`） |
| `sensor_pose` | nav_msgs/Odometry | 传感器仿真 → GridMap | 传感器位姿（实机 `/LIO/odom_imu`） |
| `cloud` | sensor_msgs/PointCloud2 | 传感器仿真 → GridMap | 激光点云（实机 `/LIO/clouds_lidar`） |
| `depth` | sensor_msgs/Image | 传感器仿真 → GridMap | 深度图（depth 模式） |
| `move_base_simple/goal` | geometry_msgs/PoseStamped | RViz → 规划器 | 手动目标（navi_mode=1） |
| `initial_path` | nav_msgs/Path | 外部 → 规划器 | 参考路径（navi_mode=3） |
| `planning/bspline` | scan_planner_msgs/Bspline | 规划器 → 控制器、RViz | 规划结果（控制点+节点向量+时间） |
| `planning/go2_execution_frozen` | std_msgs/Bool | 控制器 → 规划器 | 转向时冻结轨迹时钟 |
| `cmd_vel` | geometry_msgs/Twist | 控制器 → 仿真机器人/实机 | 速度指令 |
| `grid_map/occupancy` | sensor_msgs/PointCloud2 | GridMap → RViz | 占用地图可视化 |
| `grid_map/sliding_map_bbox` | visualization_msgs/Marker | GridMap → RViz | 滑动地图范围框 |

---

## 10. 参数配置详解

所有参数在 `src/planner/plan_manage/config/` 下三个 YAML 文件：

### planner.yaml（规划器本体）

```yaml
fsm.*            # 状态机：navi_mode、重规划阈值(1.0m)、规划视野(7.5m)、紧急时间(1.0s)、失败保护
grid_map.*       # 地图：分辨率(0.05m)、滑动地图大小(10×10×5)、膨胀参数(双圆柱0.25/0.18)、
                 #      射线概率(p_hit 0.85/p_miss 0.30/钳制0.12-0.98)、相机内参
manager.*        # 运动极限：max_vel 0.75、max_acc 0.5、max_jerk 4.0、控制点间距 0.2m
optimization.*   # 优化权重：λ_smooth 1.0、λ_collision 1.0、λ_feasibility 0.1、λ_fitness 1.0、安全距离 0.2m
```

### controllers.yaml（控制器）

```yaml
closed_loop_controller:   # 前瞻时间 0.8s、朝向阈值 0.8rad、位置增益 0.8、速度上限 0.75/0.35/1.0
open_loop_controller:     # 发布频率 100Hz、初始位置
go2_kinematic_sim:        # 仿真频率 100Hz、指令超时 0.3s、初始位置(-19, 1, 0.3)
go2_gait_publisher:       # 步态频率 2.2Hz、关节摆动幅度
```

### simulator.yaml（仿真器）

```yaml
mockamap_node:   # 地图类型(type 2=随机障碍物)、40×40×5、500 个障碍物、随机种子 127
pcl_render_node: # 传感器类型、视野(360°/90°)、最小探测距离 1.0m、点云降采样 0.1m、
                 #     相机内参 640×480、动态障碍物开关(默认关)
odom_visualization:
```

---

## 11. 代码阅读指南

建议按这个顺序读，从"入口"到"核心算法"层层深入：

1. **`src/planner/plan_manage/launch/run.launch.py`** — 先看系统怎么拼起来、话题怎么接的（30 分钟理解全局）。
2. **`src/planner/plan_manage/src/scan_planner_node.cpp`** — 入口，只有 30 行，理解"节点交给 FSM"的模式。
3. **`src/planner/plan_manage/src/scan_replan_fsm.cpp`** — 状态机主循环，读 `execFSMCallback` 和 `checkCollisionCallback` 两个回调，理解"什么时候规划、什么时候停"。
4. **`src/planner/plan_manage/src/planner_manager.cpp`** — `reboundReplan` 三步走，理解"怎么规划"。
5. **`src/planner/bspline_opt/src/bspline_optimizer.cpp`** — 四个代价函数（`calcSmoothnessCost`/`calcDistanceCostRebound`/`calcFeasibilityCost`/`calcFitnessCost`），理解"轨迹好不好"怎么量化。
6. **`src/planner/plan_env/src/grid_map.cpp`** — `raycastProcess` 和 `updateInflation`，理解"地图怎么建、膨胀怎么算"。
7. **`src/planner/plan_manage/src/closed_loop_controller.cpp`** — 控制器，理解"轨迹怎么变成速度"。
8. **`src/simulator/local_sensing/src/pointcloud_render_node.cpp`** — 传感器仿真，理解"假数据怎么生成"。

### 调试小贴士

- 终端输出里 `[FSM]: state: ...` 每 1 秒打印一次当前状态，`[rebo replan]:` 打印每次规划的起终点和耗时（绿色 = 成功）。
- 规划失败时看 `continuous_failures_count_` 相关输出：多次失败会启用随机扰动初值。
- RViz 里看：绿色路径 = 全局轨迹，白色 = 局部优化结果，`grid_map/occupancy` = 占用地图，`self_inflation` = 双圆柱模型可视化（蓝色半透明圆柱，跟随机器人）。
- 修改参数后不需要重新编译——YAML 是运行时加载的，改完重启节点即可（但要注意 launch 文件里有部分参数覆盖）。
