# PB 仿真坡道：可运行配置与本轮修复

本轮按“先能运行”的要求交付独立仿真配置。**原失败区域到原目标的长路径已经闭环到达；不代表所有起点和 T0–T8 全部通过。** 不连接真机，不自动提交。本报告补充 `PB_SLOPE_NEXT_STEPS_COMPLETION_REPORT.md`，旧报告的历史结果保留。

## 1. 直接运行

先构建（已构建的当前工作区可跳过）：

```bash
cd /home/rainple/nav_test
bash tools/pb_sim/build_and_test.sh
```

终端一启动仿真与导航：

```bash
cd /home/rainple/nav_test
source /opt/ros/humble/setup.bash
source meshnav_demo_ws/install/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=pb_practical
ros2 launch pb_vehicle_adapter pb_meshnav_sim_ready.launch.py
```

不需要窗口时加 `start_gazebo_gui:=False start_rviz:=False`。默认关闭渲染传感器，使用仿真真值定位；这不等于动态障碍感知已经实现。默认起点 `(-1.9, 5.95)`、车头沿 +X，是本轮实际通过的坡前起点。

终端二加载相同环境并发目标：

```bash
cd /home/rainple/nav_test
source /opt/ros/humble/setup.bash
source meshnav_demo_ws/install/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=pb_practical
ros2 run pb_vehicle_adapter pb_preflight --framework meshnav --wait-timeout 120
ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav \
  --controller pb_terminal_controller --x 1.35 --y 5.95 --z 0.203 --yaw 0
```

取消：`ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav --cancel`。
重新发送上面的目标会创建新任务。退出用启动终端的 Ctrl+C。不要同时在同一域运行第二套仿真。

原失败区域的运行配置：启动命令追加
`spawn_x:=-4.63 spawn_y:=-3.5 spawn_yaw_deg:=-90`，目标改成
`--x 11.704056 --y 5.089724 --z 0.025379 --yaw 0`。
这是**绕开不可通过地形到达**，不是证明车辆能强行翻过原卡点。

## 2. 修复内容与边界

### 独立的有界 PI 仿真驱动

新增 `pb_gazebo_sim_support::VelocityDrive`，仅通过运行时模型替换 `MecanumDrive2`。
保持原来的 X/Y 力与 yaw 力矩上限（100 N、200 N、100 Nm），增加积分与抗饱和，
有 0.5 s 指令接收超时。施加物理力，不直接设置位姿/速度，不改质量、摩擦或碰撞几何。
PI 单独使用仍在原卡点失败，因此不能把这个补丁单独称为根因修复。

`pb_vehicle_sim.launch.py` 和原 `pb_meshnav.launch.py` 默认仍用 `legacy`；
新增 `pb_meshnav_sim_ready.launch.py` 明确选用 `pi` 和 `pb_terminal_controller`。
原公共控制器与第三方驱动未为本轮场景增加坐标特判。

### 保守的仿真通行参数

实用入口把 MeshNav 高差阈值从 0.2 m 改为 **0.08 m**，使原长路径选择其他路线。
这是该测试车辆的仿真参数，可能拒绝更多地形，不是实车越障能力标定。
使用独立缓存 `~/.ros/pb_practical_navigation.h5`；地图网格不变。
长路径本轮实际用等价参数及独立 `log/repair_clearance_map.h5` 完成。

### 取消、驻停与数据有效性

- 已终止的目标现在查询真实结果；ABORTED 不再因“零速且停下”返回成功。
- 驻停需要有效、及时的位姿和此前接受的运动任务；重复/倒退/陈旧位姿不能延续旧保持目标。
- 取消或失败抑制旧保持，新任务可重新激活。纯零速与物理停下仍分别检查。
- 冷启动复验发现终端插件自己的取消标记未在 `setPlan()` 清零（公共跟踪器已正确清零）。本轮补上新任务重置，并清除旧进展/时间历史；取消标记改用原子变量。
- 采集工具增加驱动、诊断、高差阈值、缓存和完成后采集时长选项，保存 `run_config.json`；资产记录改为实际使用的缓存。

### 接触数据

原实现找错了实体层级。现在在本模型的 collision 实体上请求 `ContactSensorData` 状态，
由物理引擎填充，默认关闭。无需修改世界或增加接触传感器。
这是请求状态输出，不是写力/位姿指令；旧报告“从不创建任何组件”的描述不准确。
本轮得到车轮与场地的接触位置，但引擎没有提供法向/深度/接触力字段。
不能将缺失字段当作零，也不能据轮速单独确定“支架顶住、一个轮卸载”。

## 3. 闭环证据

| 用例 | 结果 | 原始证据目录（均在 log/ 下） |
|---|---|---|
| 原卡点，PI + 原 0.2 m 阈值 | FAIL，仍卡在约 (-4.627,-4.626) | `repair_pi_original_report_20260922_101518` |
| 原失败区域 → 原目标，PI + 0.08 m 阈值 | PASS，动作成功，约 106 s，终点 (11.624,5.068)，误差约 0.083 m | `repair_route_clearance_report_20260922_102204` |
| +Y 坡前 → 坡顶 | PASS，动作成功，误差约 0.026 m | `repair_pi_plus_report_20260922_102241` |
| +Y 到达后驻停 | PASS，连续 66.47 s，最大三维位移 0.01092 m（从成功时刻计，不裁掉整定） | 同上；`repair_closed_loop_metrics.json` |
| −Y 通道 | PASS（本轮一次），终点误差约 0.061 m | `repair_pi_center_report_20260922_101908` |
| −Y 最终 0.08 m 配置复验 | PASS，约 24.4 s，终点 (1.275,-5.969)，误差约 0.077 m；无需世界 contact 插件即收到 1863 条接触对 | `repair_final_minus_report_20260922_133202` |
| 指定偏侧起点 (-0.5,5.72) | FAIL，落稳约 (-0.5,5.710,z=0.150)，GetPath outcome=56，无路径 | `repair_pi_offset_report_20260922_102014` |
| 实用入口冷启动、取消后立即重发 | PASS，取消 CANCELED、零速与停下均通过；新任务约 6.35 s 到达 (1.368,5.961)，误差约 0.021 m | `ready_fixed_preflight.log`、`ready_fixed_cancel.log`、`ready_fixed_resume.log`、`ready_fixed_bag/` |

每个目录含实际指令、6-DOF 真值、里程计、动作状态、控制器状态和结果。
第一行旧采集客户端曾返回 rc=0，但目标未到达，**本报告判 FAIL**；此误报已补回归测试。
−Y 一次成功不推翻旧报告中的间歇失败，需要结合后续复验。

构建五包、20 项终端逻辑 gtest、10 项诊断 gtest、84 项适配器 pytest、61 项工具 pytest
全部通过，记录在 `log/repair_final_build_tests.log`。之后发现的插件取消重置修复另行构建并执行现场取消/重发验证；单元测试不是闭环通过的替代。
该补丁构建及 20 项终端逻辑测试也通过（`repair_cancel_build.log`、`repair_cancel_gtest.log`）。
原失败复验保存在 `ready_smoke_*`，修复后的复验保存在 `ready_fixed_*`，不覆盖失败证据。
源码基线和证据 SHA256 清单在 `log/repair_final_manifests/`，原始记录留在本地、不自动上传。

## 4. 尚未承诺的能力

偏侧起点自动脱困/居中仍未实现，不要用该起点作为默认启动位置。未新增
“动态机器人堵隧道 → 局部规划失败 → 全局换路”的完整闭环；当前直接 GetPath→ExePath
客户端不等于全局自动重规划管理器。坡上重发、隧道上下层的完整组合矩阵仍未全部复验。
这些内容需要后续 SCAN 接入与任务管理，不能用本次几条通过记录替代。

## 5. 真机隔离

实用入口只用于 Gazebo。真机不得启动它或 PI/真值插件，也不要照搬低矮模型包络和 0.08 m 阈值。
真机应由定位提供 map/odom/base TF 与里程计，由硬件适配发布底盘速度；
设置 `use_sim_time=false`，按实际制动、速度、包络配置。驻停功能默认代码关闭，
本轮只在仿真显式启用；真机需另行验证急停、看门狗和坡面制动。
本轮没有连接实机。
