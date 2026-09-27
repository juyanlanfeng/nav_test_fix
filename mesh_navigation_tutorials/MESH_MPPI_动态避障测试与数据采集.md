# Mesh MPPI 动态避障测试与数据采集

适用入口：`ros2 launch pb_vehicle_adapter meshnav_pb_sim.launch.py`。

本文用于收集能够定位问题的数据。完成后提供结果目录或压缩包，即可继续分析。测试参数用于对照实验，不代表最终调参结果。

## 1. 要回答的问题

按以下顺序判断动态障碍在哪一步丢失：

```text
Gazebo 新增物体
  → /cloud 原始点云
  → /rmcl_inputs/cloud 射线数据
  → /obstacle_points 分割后的障碍点
  → obstacle 图层的致命顶点
  → obstacle_inflation 膨胀代价
  → final 最终代价
  → MPPI 预测轨迹与控制指令
  → /pb/cmd_vel_safe
  → /odom 实际车辆运动
```

当前怀疑的过滤冲突是：RMCL 的 `min_dist_outlier_scan=0.3` 与障碍层的 `robot_height=0.15`。
在平地、模拟射线命中地面的情况下，前者要求实测点偏离地面超过 0.3 m，后者只接受向下投影到地面不超过 0.15 m 的点。这一情况下没有交集。
模拟射线未命中地图时分割逻辑不同，因此不能仅凭配置认定所有障碍都被过滤，必须实际采集。

最近日志已经确认 `mesh_mppi/DiffDriveMPC` 成功加载，但出现过 `NO_VALID_CMD` 和 `ROBOT_STUCK`。需要记录错误出现时的障碍代价、车辆速度和目标，不能仅凭停住判断控制器坏了或避障成功。

## 2. 测试场景与记录要求

先在平坦、开阔、有导航网格覆盖的区域测试，避开坡道、窄门、网格边界和原有障碍膨胀区。

- 初始位置和目标间距建议约 4～5 m，两侧均留出绕行空间。
- 新增一个有 **visual 和 collision** 的方块，建议尺寸 `0.5 × 0.5 × 0.5 m`。底部贴地，平地上中心高度是地面高度加 0.25 m。记录实际尺寸和位置。
- 优先用 Gazebo 自带方块。只有碰撞体、没有可被渲染雷达看到的外观时，GPU 雷达可能看不到它。
- 静态感知测试时，方块放在车前约 1～2 m，车辆不发送导航目标。
- 每轮使用同样的位置、尺寸和朝向；对照测试中只修改表格指定参数。
- 放置后等待物体稳定。明确记录是暂停后搬动、瞬间移动，还是持续平移，以及暂停和恢复时间。
- 记录 Gazebo 的实时因子 RTF。下面的等待时间优先按仿真时间计算；例如 RTF=0.2 时，10 秒仿真时间需要约 50 秒墙钟时间。

需要记录：车辆初始位置、目标位置和朝向、方块尺寸/位置、是否移动、发送目标的工具、是否撞击/停车/绕行/中止。

## 3. 准备结果目录和终端环境

### 3.1 创建一次测试会话

先停止已有的同一套仿真，避免启动两份。终端 1 执行：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash

export TEST_ROOT="/home/rainple/nav_test/mesh_navigation_tutorials/diagnostics/mesh_mppi_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_ROOT/config" "$TEST_ROOT/ros_logs" "$TEST_ROOT/runs" "$TEST_ROOT/screenshots"
printf 'export TEST_ROOT=%q\n' "$TEST_ROOT" > /tmp/mesh_mppi_test.env

cp /home/rainple/nav_test/mesh_navigation_tutorials/src/mesh_navigation_tutorials/config/mbf_mesh_nav.yaml "$TEST_ROOT/config/"
cp /home/rainple/nav_test/mesh_navigation_tutorials/src/mesh_navigation_tutorials/config/rmcl.yaml "$TEST_ROOT/config/"
cp /home/rainple/nav_test/mesh_navigation_tutorials/src/pb_vehicle_adapter/config/pb_vehicle_profile.yaml "$TEST_ROOT/config/"
cp /home/rainple/nav_test/mesh_navigation_tutorials/src/pb_vehicle_adapter/config/pb_ros_gz_bridge.yaml "$TEST_ROOT/config/"

git -C /home/rainple/nav_test rev-parse HEAD > "$TEST_ROOT/git_head.txt"
git -C /home/rainple/nav_test status --short > "$TEST_ROOT/git_status.txt"
git -C /home/rainple/nav_test diff -- mesh_navigation_tutorials/src/mesh_navigation_tutorials/config > "$TEST_ROOT/config_diff.patch"
```

后续新开的每个终端都先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/mesh_navigation_tutorials/install/setup.bash
source /tmp/mesh_mppi_test.env
```

`/tmp/mesh_mppi_test.env` 是本次会话的路径记录。创建下一次会话前，请完成并归档本次结果。

### 3.2 启动并完整保存日志

终端 1：

```bash
export ROS_LOG_DIR="$TEST_ROOT/ros_logs"
set -o pipefail
ros2 launch pb_vehicle_adapter meshnav_pb_sim.launch.py 2>&1 | tee "$TEST_ROOT/launch.log"
```

保留终端。实验结束时用 Ctrl+C 正常停止。

## 4. 首先保存实际运行状态

节点启动完成后，在终端 2 执行。源码 YAML 不能代替这些运行时参数。

```bash
ros2 node list > "$TEST_ROOT/nodes.txt"
ros2 topic list -t --include-hidden-topics > "$TEST_ROOT/topics.txt"
ros2 action list -t > "$TEST_ROOT/actions.txt"

ros2 param dump /move_base_flex > "$TEST_ROOT/config/mbf_runtime_original.yaml"
ros2 param dump /rmcl_seg > "$TEST_ROOT/config/rmcl_seg_runtime_original.yaml"
ros2 param dump /rmcl_lidar3d_conversion > "$TEST_ROOT/config/conversion_runtime.yaml"
ros2 param dump /pb_cmd_vel_adapter > "$TEST_ROOT/config/cmd_adapter_runtime.yaml"

ros2 topic info /cloud -v > "$TEST_ROOT/cloud_endpoints.txt"
ros2 topic info /rmcl_inputs/cloud -v > "$TEST_ROOT/rmcl_inputs_endpoints.txt"
ros2 topic info /obstacle_points -v > "$TEST_ROOT/obstacle_endpoints.txt"
ros2 topic info /move_base_flex/vertex_costs/updates -v > "$TEST_ROOT/cost_update_endpoints.txt"

ros2 param get /move_base_flex mesh_controller.type
ros2 param get /move_base_flex mesh_map.default_layer
ros2 param get /move_base_flex controller_frequency
ros2 param get /rmcl_seg min_dist_outlier_scan
ros2 param get /move_base_flex mesh_map.obstacle.robot_height
```

应确认：插件类型为 `mesh_mppi/DiffDriveMPC`，默认代价层为 `final`。参数缺失、节点不存在或命令失败时，保存错误文本，先停止后续参数对照实验并反馈。

另外记录实际安装位置，便于排除 source 了其他工作区：

```bash
ros2 pkg prefix mesh_navigation_tutorials > "$TEST_ROOT/package_mesh_navigation_tutorials.txt"
ros2 pkg prefix pb_vehicle_adapter > "$TEST_ROOT/package_pb_vehicle_adapter.txt"
ros2 pkg prefix mesh_mppi > "$TEST_ROOT/package_mesh_mppi.txt"
```

## 5. 每轮测试统一采集方法

### 5.1 每轮开始前建立目录并保存参数

终端 2 执行下面模板，把 `T0_free_drive` 替换为当前测试编号。

```bash
export RUN_NAME=T0_free_drive
export RUN_DIR="$TEST_ROOT/runs/$RUN_NAME"
mkdir -p "$RUN_DIR"
printf 'export RUN_DIR=%q\n' "$RUN_DIR" > /tmp/mesh_mppi_run.env
ros2 param dump /move_base_flex > "$RUN_DIR/mbf_runtime.yaml"
ros2 param dump /rmcl_seg > "$RUN_DIR/rmcl_seg_runtime.yaml"
date --iso-8601=seconds > "$RUN_DIR/start_wall_time.txt"
```

### 5.2 录制 rosbag

终端 3 先加载第 3 节的环境，然后执行：

```bash
source /tmp/mesh_mppi_run.env
ros2 bag record --include-hidden-topics --use-sim-time \
  -o "$RUN_DIR/bag" \
  /clock /tf /tf_static /rosout /parameter_events \
  /cloud /rmcl_inputs/cloud /obstacle_points \
  /move_base_flex/mesh \
  /move_base_flex/vertex_costs /move_base_flex/vertex_costs/updates \
  /move_base_flex/path /move_base_flex/optimal_trajectory \
  /cmd_vel /pb/cmd_vel_safe /odom /pb/truth_health \
  /move_base_flex/get_path/_action/status \
  /move_base_flex/exe_path/_action/status \
  /move_base_flex/exe_path/_action/feedback \
  /move_base_flex/move_base/_action/status \
  /move_base_flex/move_base/_action/feedback \
  2>&1 | tee "$RUN_DIR/recorder.log"
```

看到 recorder 开始订阅话题后再操作障碍物、发送目标。每轮结束先 Ctrl+C 停止 recorder，等待正常退出，再执行：

```bash
ros2 bag info "$RUN_DIR/bag" > "$RUN_DIR/bag_info.txt"
```

注意：

- 每轮通常记录 40～90 秒仿真时间即可，低 RTF 时墙钟时间会更长。点云占用空间较大，不必长时间空录。
- `bag` 目录必须不存在；重做一轮时改用 `T1_baseline_repeat2` 等新编号。
- 未使用 `move_base` action 时，它的 feedback/status 没消息可以接受；尚未发送目标时，MPPI 预测轨迹没消息也正常。
- `/clock` 没消息时，带 `--use-sim-time` 的 recorder 不会写入数据，这是诊断信号。
- 录制了反馈和状态，但 bag 不能完整替代 action 最终返回结果。请保存 RViz 面板的最终错误文字或截图，以及整个 launch 日志。
- 检查 `bag_info.txt` 是否真的包含 `/cloud`、`/obstacle_points`、`/tf`、`/tf_static`。若成本全量消息或静态 TF 未录到，记下来；先保留已有数据，不要把无消息直接解释为没有障碍。

### 5.3 记录事件时间

在终端 2 运行：

```bash
source /tmp/mesh_mppi_run.env
mark_event() {
  printf '\n%s | %s\n' "$(date --iso-8601=seconds)" "$*" >> "$RUN_DIR/events.txt"
  timeout 3 ros2 topic echo /clock --once >> "$RUN_DIR/events.txt" 2>&1
}
```

每个关键操作后调用，例如：

```bash
mark_event "开始空场观察"
mark_event "方块已放置：中心(x,y,z)=实际坐标，尺寸=0.5,0.5,0.5"
mark_event "通过 RViz Mesh Goal 发送目标：实际坐标与朝向"
mark_event "方块已移出路径"
mark_event "车辆停止；面板提示：抄写完整提示"
```

事件时间允许几秒误差，写明是操作前还是操作后。不要保留示例里的“实际坐标”占位文本。

### 5.4 RViz 显示

Fixed Frame 使用 `map`：

1. 保留原始 `/cloud` 显示。
2. 新增一个 PointCloud2，话题 `/obstacle_points`，颜色与原始点云区分，QoS 可选 Best Effort。
3. Mesh Map 显示中的 `Vertex Costs Type`，依次观察 `obstacle`、`obstacle_inflation`、`final`。原配置已经连接全量和更新代价话题。
4. 新增 Path，话题 `/move_base_flex/optimal_trajectory`，用于观察 MPPI 预测轨迹；同时保留 `/move_base_flex/path` 全局路径。

截图应包含物体对应位置、车辆、点云或代价图层，并在文件名中写明轮次和图层，例如 `T2_obstacle_inflation.png`。
只看到 `/cloud` 中有点，不能证明导航已经感知障碍；只看到全局路径不变，也不能证明 MPPI 没有局部绕行。

## 6. T0：无新增障碍的行驶基线

编号：`T0_free_drive`。保持原始参数。

1. 清除本次添加的物体，选择平坦开阔路线。
2. 开始录包，静止观察约 10 秒。
3. 通过日常使用的同一个 RViz 工具发送目标，记录起点、目标和工具名称。
4. 观察到达、失败或约 30 秒；保存最终面板信息。
5. 若车辆没动，或已经出现 `NO_VALID_CMD` / `ROBOT_STUCK`，记录下来。仍可完成后面的静止感知测试，但暂不把行驶失败归因于新增障碍。

本轮要回答：没有新物体时，MPPI 能否输出有效指令、底盘能否执行、能否到达目标？

## 7. T1～T4：车辆静止，四组参数对照

测试前取消所有导航目标，等车停稳。每组都先移走方块，观察约 10 秒；再把方块放到同一位置，观察约 15 秒；最后移走，观察约 10 秒。
每组独立录包并保存运行时参数。车辆和方块的位置尽量一致。

| 编号 | min_dist_outlier_scan | mesh_map.obstacle.robot_height | 目的 |
|---|---:|---:|---|
| T1_baseline | 原始值，预期 0.3 | 原始值，预期 0.15 | 复现现象 |
| T2_seg_only | 0.05 | 原始值，预期 0.15 | 只降低分割阈值 |
| T3_height_only | 原始值，预期 0.3 | 0.23 | 只提高障碍层高度 |
| T4_both | 0.05 | 0.23 | 检查两个环节共同影响 |

表里的原始值以第 4 节运行时参数为准。如果实际不是 0.3/0.15，替换下列命令中的原始值并在报告注明。
不要同时修改采样数、权重、速度、膨胀半径或规划频率，也不要在组间更换地图/缓存。

本项目的这两个参数均有动态更新回调，可以用以下命令临时修改，无需编辑 YAML 或重启。每条命令应返回设置成功；设置后执行 `ros2 param get` 回读。

T1（还没改参数时直接测试；若需恢复到原始值）：

```bash
ros2 param set /rmcl_seg min_dist_outlier_scan 0.3
ros2 param set /move_base_flex mesh_map.obstacle.robot_height 0.15
```

T2：

```bash
ros2 param set /rmcl_seg min_dist_outlier_scan 0.05
ros2 param set /move_base_flex mesh_map.obstacle.robot_height 0.15
```

T3：

```bash
ros2 param set /rmcl_seg min_dist_outlier_scan 0.3
ros2 param set /move_base_flex mesh_map.obstacle.robot_height 0.23
```

T4：

```bash
ros2 param set /rmcl_seg min_dist_outlier_scan 0.05
ros2 param set /move_base_flex mesh_map.obstacle.robot_height 0.23
```

每组修改后：

```bash
ros2 param get /rmcl_seg min_dist_outlier_scan
ros2 param get /move_base_flex mesh_map.obstacle.robot_height
```

保持 `min_dist_outlier_map` 原值：当前导航使用的是 `outlier_scan` 输出，本轮仅检验这条链路。

每组填下面三项：

- 新增方块在原始点云里是否可见？分割后有没有同位置的点？
- `obstacle` 是否增加致命区域，`obstacle_inflation` 和 `final` 是否在同一位置产生代价？
- 移走方块后，这些点和代价是否消失？大约需要几秒仿真时间？

**0.05 m 和 0.23 m 是诊断用对照值。** 0.23 m 接近当前车体包围盒高度 0.226 m；是否还需安全余量、是否引入地面误检，要看数据。
这些测试只改动态障碍处理参数，无需为了对照实验删除 `.h5`。删除缓存会引入额外变量。

## 8. T5：确认感知有效后的行驶绕障

编号：`T5_block_path`。只有 T0 能正常行驶、且 T4 能让新增物体出现在 `final` 代价中时，才进入本轮。
继续使用 T4 参数，并保存本轮运行时参数。

1. 选择 T0 的同一路线，把方块放在路线中间，留出左右绕行空间。
2. 先录包，等 `final` 图层出现方块代价，再发送相同目标。
3. 记录全局路径是否绕开物体、MPPI 预测轨迹是否绕开、车辆实际是否绕开。
4. 如果失败，记录完整 action 错误文字和时间，不要立即点多个新目标覆盖现场。

这一步先验证“已感知到障碍时，规划与控制能否避开”。如果失败，保留数据即可；暂不继续加快障碍运动或增加 MPPI 参数。

## 9. T6：执行过程中出现障碍（通过 T5 后再做）

编号：`T6_obstacle_appears`。保持 T4 参数。

1. 移走方块，恢复车辆起点，开始录包。
2. 发送与 T0 相同的目标。
3. 车辆开始行驶后，把方块放在其前方约 1.5～2 m 的原路径上，记录实际距离和操作时间。
4. 观察它是否减速、停车、局部绕行，或者中止任务。
5. 若停车，约 10 秒后移开物体，观察任务是否恢复。记录当时 action 仍在执行，还是已经失败。

不要把物体直接放进车体内部或紧贴车辆；这只会测试已经碰撞时的处理，无法评价提前避障。
如果通过暂停仿真放置，明确记录暂停区间；如果时间变为倒退或重置，请另开一轮。

可选追加 `T7_crossing`：让物体横穿路径，记录物体移动速度、方向和轨迹。瞬间拖拽与匀速横穿需分开记录。

当前 MPPI 从实时 MeshMap 获取障碍代价，不能据此假定它会估计并预测物体未来运动。T6/T7 首先观察对更新后的代价是否及时响应。

## 10. 如何区分局部绕障与全局重规划

当前 RViz `MbfGoalActions` 面板走 `get_path → exe_path`，并非直接发送 `move_base` 目标。它在 `ROBOT_STUCK` 后有有限次数重试，但不是持续周期重规划。

- `/move_base_flex/path` 不变，但 `/move_base_flex/optimal_trajectory` 和车辆轨迹绕开物体：局部避障可以是正常工作的。
- 把 `planner_frequency` 提到 20，并不能使这个 RViz 流程自动每秒重规划 20 次。
- 本轮先保持发送目标的方式一致，不同时改成其他 action；报告中写清使用哪个 RViz 按钮/工具。
- 如果你本来使用其他工具发送 `/move_base_flex/move_base`，请提供具体命令。对应 bag 已包含该 action 的反馈和状态。

## 11. 出现异常时追加的数据

### 11.1 点云没有进入下游

在异常对应轮次的目录下保存：

```bash
source /tmp/mesh_mppi_run.env
timeout 8 ros2 topic echo /cloud --once --no-arr > "$RUN_DIR/cloud_header.txt" 2>&1
timeout 8 ros2 topic echo /obstacle_points --once --no-arr > "$RUN_DIR/obstacle_header.txt" 2>&1
```

这里看 `header.frame_id`、时间戳、`width`、`height`；`--no-arr` 省略大数组。总点数为 `width × height`，不要只看话题存在。
若命令因超时退出，保留文件并注明“8 秒无消息”；这和“持续收到 width=0 的空点云”含义不同。

也可串行测量频率，每条约 10 秒，避免同时启动很多 CLI 订阅者影响性能：

```bash
timeout 10 ros2 topic hz /cloud > "$RUN_DIR/cloud_hz.txt" 2>&1
timeout 10 ros2 topic hz /obstacle_points > "$RUN_DIR/obstacle_hz.txt" 2>&1
timeout 10 ros2 topic hz /move_base_flex/vertex_costs/updates > "$RUN_DIR/cost_updates_hz.txt" 2>&1
```

`timeout` 返回 124 表示主动结束采集，不表示节点崩溃。CLI 没测到频率不能单独证明没有数据，还需结合端点 QoS 和 bag。
记录仿真是否暂停、RTF 多大。代价更新频率也不等于成功检测障碍的频率。

### 11.2 TF 报错

若障碍点云 frame 确认为 `front_mid360`：

```bash
timeout 8 ros2 run tf2_ros tf2_echo map front_mid360 > "$RUN_DIR/tf_sensor.txt" 2>&1
timeout 8 ros2 run tf2_ros tf2_echo map base_footprint > "$RUN_DIR/tf_robot.txt" 2>&1
```

若实际 frame 不同，替换成点云 header 中的名称。`tf2_echo` 主要检查最新变换，不能代替点云时间戳对应的历史 TF 检查，所以 bag 中的 `/tf`、`/tf_static` 和 `/clock` 仍然需要。

### 11.3 控制器失败或停住

保存 RViz action 面板完整错误及截图，记录是否有以下情况：

| 观察 | 后续分析方向 |
|---|---|
| 原始点云也看不到方块 | Gazebo 物体、GPU 雷达、视场、桥接 |
| 原始点云有，障碍点云没有 | RMCL 分割、地图/TF 对齐、阈值 |
| 障碍点云有，obstacle 没代价 | 高度/距离过滤、向下投影、导航网格覆盖 |
| obstacle 有，final 没变化 | 图层传播与代价更新 |
| final 正常，NO_VALID_CMD | 采样轨迹全部无效、地图边界、碰撞阈值、运动约束等；不能仅凭错误确定具体原因 |
| 有 /cmd_vel，/pb/cmd_vel_safe 没有对应运动指令 | 底盘速度适配、驻停或健康状态 |
| 两路速度指令都有，/odom 不跟随 | 底盘运动学/动力学、碰撞、仿真性能 |
| 障碍移除后代价不清除 | 新点云是否继续到达、TF、图层清除 |

停止、绕行、任务中止是不同结果。不要将“没有撞上”一律记录为“避障成功”。

## 12. 每轮结果模板

每轮创建一个 `notes.md`，复制填写：

```text
测试编号：
日期与时间：
min_dist_outlier_scan 实际值：
mesh_map.obstacle.robot_height 实际值：
车辆起点 (x,y,z,yaw)：
目标 (x,y,z,yaw)：
目标发送工具/命令：
障碍模型名称、尺寸：
障碍放置坐标、移动方式/速度：
RTF 大致范围：
是否暂停仿真，暂停区间：
原始 /cloud 中是否看到障碍：
/obstacle_points 中是否看到同位置障碍：
obstacle 图层是否出现致命区域：
obstacle_inflation / final 是否出现代价：
移除障碍后是否清除，延迟约几秒：
全局路径是否变化：
MPPI 预测轨迹是否变化：
实际行为（继续直行/减速/停住/绕行/撞击/任务中止）：
action 完整结果/错误：
关键事件时间：见 events.txt
对应截图/视频文件名：
本轮与标准步骤不同的操作：
```

## 13. 恢复参数与提交结果

测试完成后，取消当前目标，恢复第 4 节记录的原始运行值。若原始值是 0.3 和 0.15：

```bash
ros2 param set /rmcl_seg min_dist_outlier_scan 0.3
ros2 param set /move_base_flex mesh_map.obstacle.robot_height 0.15
```

上述修改没有写入 YAML；关闭并重新启动项目也会按配置文件加载参数。
正常结束所有 recorder 后，再停止 launch，保留日志。

提交内容：

1. `config/`：源码配置和原始运行参数。
2. `launch.log`、`ros_logs/`、节点/话题/action/端点信息。
3. 各轮 `bag/` 完整目录（包含 `metadata.yaml` 和所有 `.db3` 分片），以及 `bag_info.txt`、运行参数、`events.txt`、`notes.md`。
4. 标注轮次的截图；有条件时附 T5/T6 的短视频。

同一机器继续分析时，直接把 `$TEST_ROOT` 的绝对路径告诉我即可，无需打包。需要传输时：

```bash
source /tmp/mesh_mppi_test.env
tar -czf "${TEST_ROOT}.tar.gz" -C "$(dirname "$TEST_ROOT")" "$(basename "$TEST_ROOT")"
```

时间有限时，优先完成 **T0、T1、T2、T4**。T0 行驶失败或 T4 感知仍无效时，先发送现有数据即可，不必强行完成 T5/T6。T3 用来补充分离高度参数影响，T7 为可选测试。
