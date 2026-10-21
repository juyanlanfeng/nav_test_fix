# 日志分析

## 先给结论

TF 时间戳问题这次**没有复现**：日志里没有再出现 `extrapolation into the past/future`。现在拦住机器人是另外几件事：

1. **机器人起点和后续目标点落在坏的网格面/洞上** → `MeshController` `OUT_OF_MAP`，规划器 `Could not find a face close enough`。
2. **路径检查 `check_state=2`（LETHAL）** → 导航器反复“停车重规划”，但新路径仍被判定受阻。
3. **`/Odometry` 的 twist 为 0** → MBF 以为机器人没动，报 `Robot is ignoring velocity commands`。
4. **控制器限速和 UART 的角速度通道不匹配**：`vy=-0.80`、`w=-0.50`，但 UART 只吃 `linear.x/y`，`angular.z` 被丢弃。
5. `base_footprint` 启动瞬态一条、RViz material 报错是次要问题。

---

## 1. 已经正常 / 已修好的部分

```text
mesh loaded: 98907 vertices / 174666 faces
border: 24164 lethal
height_diff: 12512 lethal
edge costs calculated
CVPMeshPlanner / MeshController loaded
planning: path length 9.79644 m, 96 poses
```

- TF 的 past/future 跳过没有再出现，说明 `mesh_layers` 改最新 TF 生效了。
- 地图、层、规划器、控制器、速度适配器都正常加载。
- 第一段 9.79 m 的路径规划成功。

---

## 2. 关键问题 A：起点/目标点落在坏的网格面或洞上

```text
[MeshMap::searchContainingFace] Failed to calculate barycentric coordinates
Point: [6.196497, -4.203503, -0.250072] Face: 70274

MeshController: lost mesh surface near raw pose (6.198, -4.203, -0.029);
last face 71504 could not be extended (search radius 0.400 m, distance 0.400 m);
reporting OUT_OF_MAP
```

```text
[MeshMap::searchContainingFace] Failed ... Point: [3.700000, -2.212923, -0.250293] Face: 95969
waveFrontPropagation(): Could not find a face close enough to the given goal pose
Predecessor of the goal is not set! No path found!
```

后面的第二个目标点也一样：

```text
Point: [4.700000, -4.281873, -0.246269] Face: 70205
Could not find a face close enough to the given goal pose
```

说明：

- `map_with_ground.ply` 的**地面网格在这些位置有退化面/洞**，`closestPointQuery` 找到的最近面无法算出有效重心坐标；
- 控制器在机器人当前位姿 `(6.198, -4.203, -0.029)` 找不到可投影的面，直接 `OUT_OF_MAP`；
- 规划器在目标点 `(3.7, -2.2)`、`(4.7, -4.28)` 也找不到面，`GetPath` 失败。

这不是控制器或规划器算法的问题，是**网格本身在这些点不可用**。机器人起点和 RViz 点的目标都踩在上面，所以第一步就失败。

**优先处理**：

1. 在 RViz 里同时显示 `final`、`border`、`height_diff`、`map_with_ground.ply`，看起点/目标点是不是在洞或边界上；
2. 把机器人挪到一块完整、开阔、远离边界的地面，再试一次；
3. 目标点在 RViz 里选一块 `final` 不是 lethal 的完整面；
4. 如果起点/目标确实在洞上，需要补洞/清理网格：删除退化三角形、重复顶点，填掉地面洞，最好只保留地面连通分量。

`map_with_ground.ply` 是 Blender 导出的 174,666 面网格，包含大量天花板/上层分量（514 m²）和 99 个连通分量；导航真正需要的是那 351 m² 的地面，当前地面还有洞/坏面。

---

## 3. 关键问题 B：路径检查 LETHAL，反复重规划

```text
planning: path length 9.79644 m
Robot is ignoring velocity commands for 1.05 / 2.10 / 3.10 / 4.15 s
replanning: Remaining route is blocked; check_state: 2, blocked_segment: 21
replanning: ... blocked_segment: 50
replanning: ... blocked_segment: 14
replanning: ... blocked_segment: 27
Predecessor of the goal is not set! No path found!
Planning reached max retries! (10)
state: waiting_for_path -> failed
```

`check_state=2` 是 LETHAL。路径刚规划出来 5 秒就被判阻塞，说明 `final` 代价在这条路径上发生了变化或被判 lethal。常见来源：

1. **机器人自身点云被当成动态障碍**：PCL 当前没有 `robot_radius` 自车过滤，`/cloud_registered` 里车体附近的点会进入 `/dynamic_obstacles`，`TemporalObstacleLayer` 投影后把起点附近路径标死。日志里阻塞段集中在 14~50，靠近机器人，符合这个特征。
2. **网格边界/洞**：`border` 24,164 个致命顶点、`height_diff` 12,512 个致命顶点；路径经过洞边或高度差位置时被判 lethal。
3. **`height_diff: 0.1` 对当前 Blender 网格偏严**：地面边缘/小起伏被大量判死。

**优先检查**：

```bash
ros2 topic echo /dynamic_obstacles --once --field header
```

在 RViz 里看机器人周围有没有一圈“动态障碍”。如果有，恢复 PCL 的 `robot_radius` 自车过滤（旧代码有，当前被删掉了），建议先取 0.4~0.5 m，按实车包络验证。

---

## 4. 关键问题 C：`/Odometry` 的 twist 为 0

```text
Robot is ignoring velocity commands for 1.05 seconds
(last command: vx=0.02, vy=-0.80, w=-0.50)
```

机器人 TF 位姿其实在动：

```text
raw pose 6.198, -4.203  ->  5.764, -4.836
```

但 MBF 仍认为“机器人没动”。原因是 FAST-LIVO2 的 `/Odometry` **只填了 pose，没有填 twist**；`Odometry_transformed` 也是直接复制 `msg->twist`，所以速度一直是 0。

影响：

- `cmd_vel_ignored_tolerance` 当前是 **10 s**；如果机器人持续被判定“没动”，控制器会在 10 s 后报错停止；
- 本次因为路径检查提前重规划，没等到 10 s，但这是随时会触发的隐患。

**处理方式（二选一）**：

1. 在 FAST-LIVO2 的 `publish_odometry()` 里用 `_state.vel_end` 填 `twist.twist.linear/angular`；或者让 `tf_transformer` 用相邻 pose 差分计算 twist；
2. 临时把 `mbf_mesh_nav.yaml` 的 `cmd_vel_ignored_tolerance` 改为 `0` 先禁用这个检查，等 twist 修好再恢复。

---

## 5. 关键问题 D：控制器限速和 UART 角速度

```yaml
max_lin_velocity: 0.8
max_ang_velocity: 0.5
ang_vel_factor: 7.0
```

日志里实际命令：

```text
vx=0.02, vy=-0.80, w=-0.50
```

问题：

- `vy=-0.80` 已经到 `max_lin_velocity` 上限，但实车 Nav2 配置里 `max_vel_y` 只有 **0.25**；
- UART 只使用 `linear.x`、`linear.y`，`angular.z` 被忽略；`w=-0.50` 实际没有下发到车体转向；
- `ang_vel_factor: 7.0` 让控制器大量输出 `w`，但这些 `w` 对当前底盘是无效的。

建议先改成：

```yaml
holonomic: true
lin_vel_factor: 1.0
ang_vel_factor: 0.0
max_lin_velocity: 0.25
max_ang_velocity: 0.0
```

如果底盘确实支持 `linear.z` 作为 yaw，再单独接；否则不要让控制器输出无效的 `w`。

---

## 6. 次要问题

1. **`base_footprint` 启动瞬态**

```text
Keeping obstacle history: required sensor/localization TF unavailable:
"base_footprint" ... does not exist
```

只出现一次，发生在导航刚启动、第一条障碍点云进来时；后续 `MeshController` 已经能拿到 `base_footprint` 的位姿。属于启动时序，不是持续 TF 失败。如果反复出现，检查 tf_transformer 是否在 nav 之前起来。

2. **RViz material 报错**

```text
Could not find material '0_TexturedMesh_0_8400GeneralMaterial_' to unload
```

是 `rviz_mesh_tools_plugins` 显示层在更新网格材质时的报错，不影响导航计算，可以忽略。

---

## 7. 建议的排查顺序

1. **先解决起点/目标点落在坏面/洞上**（最高优先级）：
   - RViz 看 `final` + 网格；
   - 把机器人挪到完整地面；
   - 目标点选在完整面；
   - 需要的话清理 `map_with_ground.ply` 的退化面/洞。
2. **看 `/dynamic_obstacles` 是否把车体标成障碍**：
   - 恢复 PCL `robot_radius` 自车过滤；
   - 再看 `blocked_segment` 是否消失。
3. **修 `/Odometry` twist 或暂时把 `cmd_vel_ignored_tolerance` 设为 0**。
4. **降控制器限速、关掉无效的 `w` 输出**。
5. 最后再调 `height_diff`/`border` 阈值和路径代价。

当前最核心的一句话：**第一段 9.79 m 路径能规划出来，但机器人在起点附近就 `OUT_OF_MAP`，目标点又在坏面上，所以后面的路径检查/重规划都是在补这个洞。先把地图和起点/目标点修好，控制器的速度参数再调，才有意义。**
