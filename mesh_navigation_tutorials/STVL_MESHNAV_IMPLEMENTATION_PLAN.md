# 将 STVL 障碍保留与视野清除机制接入 MeshNav：实施方案

> 2026-10-09 箱体绕行回归修正：本文原定的 0.1 秒视野缺失清除会把被遮挡的箱体侧面
> 误当成消失，造成反复折返。当前默认已改为 STVL 时间衰减模型、10 秒线性寿命、
> 视野加速系数 0。本文旧窗口规则仅在显式 `clearing_mode: visible_timeout` 时生效。
> 最新行为与实测结果以 [实施记录](STVL_MESHNAV_IMPLEMENTATION_STATUS.md) 为准。

更新日期：2026-10-08。适用环境：当前项目、ROS 2 Humble、MID360。

本文保留最初实施设计。2026-10-09 已完成插件、配置、自动测试和实际导航进程的合成输入验证；实现差异与验收结果见 [STVL_MESHNAV_IMPLEMENTATION_STATUS.md](STVL_MESHNAV_IMPLEMENTATION_STATUS.md)。下文中的“计划”“拟新增”等措辞描述原设计，当前状态以实施记录为准。

## 1. 采用的实现路线

新增 `mesh_layers/TemporalObstacleLayer`，继承现有 `ObstacleLayer`。从原插件中抽出投影和提交结果的公共函数，由两个插件共用。新插件在两者之间增加历史记录管理，视野判断使用 STVL 已有的雷达模型。

落地后，数据流仍是：

```mermaid
flowchart LR
    A[实时点云与完整 PCD 背景相减] --> B[/dynamic_obstacles]
    B --> C[TemporalObstacleLayer]
    C --> D[原有 obstacle_inflation]
    D --> E[原有 final]
    F[原有 static_inflation] --> E
    E --> G[原有规划、控制与重规划]
```

层实例继续叫 `obstacle`，只更换它的 `type`。这样 `obstacle_inflation.inputs: ['obstacle']` 和 `final` 的连接关系保持一致。

### 1.1 已确认的行为

| 情况 | 新插件的行为 |
|---|---|
| 检测到障碍 | 沿现有投影流程标记网格顶点，障碍本体代价为无穷大 |
| 障碍离开当前雷达视野 | 从最后一次检测起保留 10 秒，到期清除；保留期间代价不降低 |
| 历史障碍在当前视野内，但没有重新检测到 | 采用约 0.1 秒的快速过期窗口 |
| 障碍被遮挡或短暂漏扫 | 接受它在视野内被快速清除，不做遮挡判断 |
| 雷达或分割节点不再发布消息 | 保持当前动态代价，暂停执行清除 |
| 必需的 TF 查询失败 | 本帧不改变历史和代价 |
| 数据恢复 | 先标记当前障碍，再清除超时且没有被刷新的一批历史；断流时间计入年龄 |
| 收到格式、时间戳、TF 均有效的空障碍点云 | 算一次有效观测，允许清除历史 |
| 定位发生明显跳变 | 清空旧动态历史，再建立本帧障碍 |
| 标记距离 | 默认 8 米，通过 YAML 调整 |

只订阅 `/dynamic_obstacles`。本阶段按单个下方可通行表面处理，不增加多层曲面的区分逻辑。

### 1.2 三种代码复用方式的取舍

| 方式 | 实际需要的工作 | 本方案选择 |
|---|---|---|
| 直接使用 STVL 的 Nav2 插件 | 适配二维 costmap 插件接口，再接回 MeshNav 的逐顶点代价 | 不采用 |
| 抽出整个 `SpatioTemporalVoxelGrid` | 引入 OpenVDB，补 mesh 关联、断流门控和固定的视野过期窗口 | 不采用 |
| 共用当前 ObstacleLayer，再抽取 STVL 雷达视野模型 | 小幅整理原插件；适配三个上游文件；新增历史表和过期逻辑 | **采用** |

STVL 网格公开提供占用点云和二维计数结果，现成结构没有 MeshNav 的 face/vertex 关联。它的视野加速衰减也不能直接表达这里约定的独立 0.1 秒窗口。以上取舍来自对其接口和实现的核对；本方案复用 STVL 的视野几何和清除思路，时间管理采用下面明确定义的规则。[STVL 网格接口](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/include/spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp)、[网格实现](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/src/spatio_temporal_voxel_grid.cpp)

## 2. 当前项目里直接复用的代码

以下位置以编写本文时的源码为准，后续整理后行号会变化。

| 现成代码 | 复用内容 | 需要的处理 |
|---|---|---|
| `src/mesh_navigation/mesh_layers/src/obstacle_layer.cpp`，约 139～239 行 | 点云坐标变换、投影方向变换、距离过滤、批量 `castRays` | 提取到一个公共投影函数 |
| 同文件，约 239～255 行 | `robot_height` 过滤、命中 face、获取该面的顶点 | 在输出顶点的同时保留原始障碍点位置 |
| 同文件，约 260～290 行 | 致命顶点差集、写锁、替换代价、`notifyChange` | 提取到一个公共提交函数 |
| 原 `ObstacleLayer::initialize()` | 参数声明、QoS、互斥回调组、订阅 | 由新插件调用；沿用原有订阅 |
| `src/mesh_navigation/mesh_layers/src/inflation_layer.cpp` | 根据障碍致命顶点计算膨胀 | 继续使用 |
| `src/mesh_navigation/mesh_layers/src/combination_layer.cpp` | `MaxCombinationLayer` 合成静态、动态代价 | 继续使用 |
| `src/mesh_navigation/mesh_map/src/layer_manager.cpp` | 更新发布和依赖层通知 | 继续使用 |

原插件目前用本帧的障碍完全替换上一帧结果；新增插件将提交“全部尚未过期历史”的顶点并集。两者共享投影、代价提交代码，避免复制整份 `obstacle_layer.cpp` 后分别维护。

原来的 `robot_height` 仍然是沿配置投影方向到 mesh 的最大距离。它不参与历史寿命计算，第一版继续使用当前值 `0.50 m`。

## 3. STVL 直接取用哪些文件

固定使用官方仓库提交：

```text
SteveMacenski/spatio_temporal_voxel_layer
16cdf3b823e0b14886eb7571cd5f559b97fea19d
```

只取以下三个文件，不安装整个 STVL 包：

| 上游文件 | 计划放入当前包的位置 |
|---|---|
| [frustum.hpp](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/include/spatio_temporal_voxel_layer/frustum_models/frustum.hpp) | `mesh_layers/src/stvl_frustum/frustum.hpp` |
| [three_dimensional_lidar_frustum.hpp](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/include/spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp) | `mesh_layers/src/stvl_frustum/three_dimensional_lidar_frustum.hpp` |
| [three_dimensional_lidar_frustum.cpp](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/src/frustum_models/three_dimensional_lidar_frustum.cpp) | `mesh_layers/src/stvl_frustum/three_dimensional_lidar_frustum.cpp` |

这里的 `mesh_layers/` 指 `src/mesh_navigation/mesh_layers/`。这些文件作为包内部源码使用，不向其他包导出其头文件。

### 3.1 必须做的薄适配

1. 把 `IsInside()` 的查询点类型从 `openvdb::Vec3d` 改为现有的 `Eigen::Vector3d`，删除 OpenVDB include。
2. 删除没有使用的可视化等 include，修改三个文件之间的 include 路径。
3. 删除雷达类中没有使用的两个 `Dot()` 辅助函数。点类型替换后它们会出现同签名，不能原样保留。
4. 在内部使用 `mesh_layers::stvl` 命名空间，避免以后与其他插件中的上游 `geometry` 名称碰撞。
5. 在视野查询的入口加有限值、水平半径接近零的保护，并补充三维球形距离检查。

原有位姿变换和视野判定主体保留。适配后只需要项目已经使用的 Eigen 和 `geometry_msgs`；原样复制则仍然要求 OpenVDB 头文件。上游所选文件中的版权和许可头完整保留，另用一份简短 `UPSTREAM.md` 记录提交及这五项改动。[上游基类头文件](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/include/spatio_temporal_voxel_layer/frustum_models/frustum.hpp)

`UPSTREAM.md` 同时记录所选文件头中的 BSD 风格条款与上游包声明的 LGPL v2.1，并附原始声明的来源；不能用当前包的许可证推定整个上游仓库的许可。[上游包声明](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/package.xml)

### 3.2 MID360 上下视野的正确换算

当前仿真雷达配置为水平 360°、垂直 −7°～52°、20 Hz。来源为 `src/pb_vehicle_adapter/models/pb_navigation_robot.sdf` 的 `front_mid360_lidar`。

所选 STVL 实现通过垂直斜率偏移表示上下边界。不能直接把“59°宽、22.5°中心”传入构造函数，否则边界会变化。初始化时把 YAML 的真实上下边界换算为其需要的参数：[上游雷达实现](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/src/frustum_models/three_dimensional_lidar_frustum.cpp)

```text
a = tan(vertical_fov_min_deg × π / 180)
b = tan(vertical_fov_max_deg × π / 180)

传给 STVL 的 vFOV   = 2 × atan((b - a) / 2)
传给 STVL 的 offset = atan((a + b) / 2)
传给 STVL 的 padding = 0
```

对于 −7°～52°，计算结果约为 `vFOV=1.223280417 rad`、`offset=0.524519485 rad`。它们是模型参数，YAML 中仍写直观的 −7°、52°。

上游距离判定使用水平半径。适配器额外用雷达局部坐标的三维距离检查 `[min_sensor_range, max_obstacle_dist]`；模型内部最小水平半径设为 0，以免混入另一种最小距离定义。角度边界不得到达 ±90°，初始化时检查上下界顺序。

## 4. 原 ObstacleLayer 的最小整理

### 4.1 共用三个接口

在 `include/mesh_layers/obstacle_layer.h` 中增加下面的 `protected` 接口。以下是设计签名，名称和类型以实际整理结果为准：

```cpp
// 沿用原订阅。绑定该成员函数后，可虚派发到 TemporalObstacleLayer。
virtual void processPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

// false 表示本帧不能使用；true + 空 observations 是有效空观测。
bool projectObservations(
    const sensor_msgs::msg::PointCloud2& msg,
    ProjectedFrame& output,
    const Eigen::Vector3f& range_origin_in_cloud = Eigen::Vector3f::Zero());

// 共用代价写入、变化顶点计算和下游通知。
void commitLethalSet(
    const rclcpp::Time& stamp,
    std::set<lvr2::VertexHandle> active_vertices);
```

`initialize()` 从 `private` 移到 `protected`。`costs_`、`lethals_` 和订阅对象继续由基类管理；新插件只提交结果，不直接修改它们。

另外增加一个内部标志 `fixed_parameters_`，默认 `false`。新插件在调用基类初始化前设为 `true`：基类声明本层参数时设置 `ParameterDescriptor.read_only`，并跳过原有动态重配回调的注册。新插件自身参数也声明为只读。旧插件保持默认值，原有热更新行为不变。这样首版统一通过 YAML 和重启调参，避免再写一套历史关联失效与视野模型热重建逻辑。

`ProjectedFrame` 只携带本次投影必需的数据：

```text
stamp                   本帧采样时间
map_from_cloud          点云坐标系到 map 的变换
observations[]
  point_in_map          投影前的障碍回波位置，保留高度
  face                  命中的网格面
  vertices[3]           该面的三个顶点
```

### 4.2 距离过滤共用，原插件行为保留

当前 `p.norm() <= max_obstacle_dist` 从点云坐标系原点计算距离。公共函数改成：

```text
|p - range_origin_in_cloud| <= max_obstacle_dist
```

旧插件使用默认零原点。新插件传入“真实雷达测量原点在点云坐标系中的位置”，使 8 米范围与视野查询的起点一致。

空点云仍要查询必要的 TF；查询成功后直接返回有效空结果，跳过零条射线的 `castRays`。TF、消息布局或投影接口不可用时返回失败，调用方不得据此清空历史。

### 4.3 提交时只通知确实改变的顶点

共享提交函数建立新的 `0/∞` 代价集合，使用原有对称差得到新增和删除顶点。变化集合必须同时包含两者，否则删除不会向下游传播。

先在写锁内更新数据，释放锁后调用 `notifyChange()`。如果致命顶点集合没有变化，新插件只刷新历史时间，不触发膨胀重算。原 `LayerManager` 已具备变化传播能力。

## 5. 新增历史记录：一个表即可

### 5.1 记录的内容

建议使用标准库 `std::unordered_map`，按“空间体素 + 命中面”去重：

```text
ObservationKey
  floor(point.x / history_voxel_size)
  floor(point.y / history_voxel_size)
  floor(point.z / history_voxel_size)
  face_id

ObservationRecord
  point_in_map          最近一次真实回波位置
  vertices[3]           缓存的网格顶点
  last_seen             最后一次检测时间
  visible_missing_since 视野内连续未刷新开始时间；可为空
```

体素索引使用 `floor()`，尤其注意地图负坐标。`face_id` 放入 key，防止同一体素里投影到相邻面的点相互刷新寿命。记录 key 的整数范围和点坐标要校验。

默认体素边长 `0.05 m`。这是历史去重粒度，保留记录中真实点的位置用于视野判断；既不修改 PLY，也不把原始点云下采样后再做首次投影。

同一 key 本帧收到多个回波时只刷新一条记录。同一小体素中的回波合并会有最多一个体素量级的空间差异，视野边缘可能受影响，需在边界测试中验证。

不推荐为了再省一点代码只按 face 保存一个点：大三角面可能同时承载不同高度、位置的回波，用一个代表点会让视野内的点带着视野外历史一起快速清除。

### 5.2 多个历史共用顶点的处理

每次清除后遍历存活记录，把全部 `vertices[3]` 插入一个 `std::set<VertexHandle>`，提交这个并集。

这样一个记录过期时，另一个仍存活记录支持的共享顶点会保留。第一版无需维护第二张引用计数表，也无需再次投影任何历史点。

设本帧障碍点数为 N、历史记录数为 K、存活唯一顶点数为 V：新增加的工作主要是本帧哈希更新、K 次寿命/视野检查和顶点并集。顶点并集使用现有 `std::set`，不是常数时间；应测量实际 K 和耗时后再决定是否优化。

## 6. 回调的确定执行顺序

```text
收到 /dynamic_obstacles
  1. 先检测节点 ROS 时钟倒退，设置待重置纪元；再按相应纪元
     检查消息布局、frame_id、采样时间和消息新鲜度
  2. 查询同一采样时刻的必要 TF，计算真实雷达位姿
  3. 调用共用投影函数；失败则直接返回；等待和投影完成后
     再次检查消息年龄，过时也直接返回，保持历史与代价
  4. 若时间纪元、定位或配置发生需重置的变化，清空旧历史
  5. 标记本帧全部有效投影；刷新 last_seen，清空 visible_missing_since
  6. 遍历没有被本帧刷新的记录
       普通年龄达到 10 秒：删除
       否则，在当前视野内：开始/继续 0.1 秒计时，到期删除
       否则，在当前视野外：取消快速清除计时，保留普通 10 秒寿命
  7. 对存活记录的顶点取并集
  8. 调用共用提交函数，将真实变化传给 inflation 和 final
```

**先标记、后清除**是恢复数据时的关键：当前确实存在的障碍先刷新，避免刚恢复的一帧先删除它，再重新添加。

### 6.1 时间规则

- `last_seen`、普通寿命、视野缺失窗口统一使用有效点云的 `header.stamp`，对应同一 ROS 时间域。
- 用节点 ROS 时钟在入口及所有 TF/投影完成后判断消息新鲜度；超过 `max_observation_age` 的消息跳过，不用滞后的视野清历史。
- 时间戳为零、重复、乱序或明显超前的帧不作为新观测。未来时间只容忍计时精度内的误差。
- ROS 时间倒退时启动新时间纪元；在下一帧有效观测中清空旧历史和时间比较状态，不能把旧纪元的时间戳继续作大小比较。
- Gazebo 暂停使仿真时间停止，保留时钟也停止。断流但 ROS 时间继续增长时，年龄继续增长，只暂不执行删除。

无需清除 timer：正常过期和视野清除都由下一帧有效观测执行。这样消息断流时不会被定时器误删。代价在断流期间可能保留超过 10 秒，这是已约定行为。

### 6.2 空点云的接口约定

上游必须在“正常扫描完成，但没有背景之外的点”时照常发布空 `/dynamic_obstacles`，保留正确时间戳、坐标系和 PointCloud2 字段。

当前 PCL 节点的正常回调会发布分割结果，数量可以为零。收到空消息只能证明这个话题仍有有效输出；只订阅该话题无法独立验证原始雷达有没有被上游正确处理。因此上游不得在失去雷达输入后重复发布人为生成的空帧。

## 7. 真实雷达位姿与定位跳变

### 7.1 视野起点必须来自雷达

当前 `/dynamic_obstacles` 输出坐标系为 `base_footprint`。其消息原点不能直接作为雷达视野原点。

统一记号：`T_A_B` 将 B 系中的坐标变到 A 系。

```text
T_map_sensor = T_map_cloud × T_cloud_lidar × T_lidar_sensor
```

- `T_map_cloud`：公共投影函数已经取得。
- `T_cloud_lidar`：通过已有 `MeshMap::tf2Buffer()` 查询 `header.frame_id ← sensor_frame`。
- `T_lidar_sensor`：由 `sensor_offset_xyz` 给出，偏移定义在雷达 link 的局部坐标系。

将这个真实雷达的平移、完整 roll/pitch/yaw 设置到 STVL 视野模型，再查询保存的三维回波点。不能用投影后的地面顶点或 face 中心查询视野，否则会丢失高度信息。

仿真 SDF 的测量原点比 `front_mid360` link 沿局部 Z 高 `0.03 m`，所以默认偏移填 `[0.0, 0.0, 0.03]`。PCL 已经把输出点坐标转换正确，这里只用于恢复视野原点，**不再次平移整片障碍点云**。

实车若 TF 已经指向正确测量原点，偏移设为零；若存在外参误差，按标定结果设置。

### 7.2 定位跳变的最小检测

利用现有 TF，保存上次有效观测的 `T_map_odom` 和雷达 map 位姿：

1. `map ← localization_reference_frame` 的平移或旋转出现超过阈值的跳变，清空历史。
2. 对间隔不超过 `pose_jump_max_interval` 的相邻有效帧，再检查雷达 map 位姿的突变，捕获参考变换未变化但机器人位姿重置的情况。
3. 长时间断流后，不能仅因机器人移动较远就认定定位重置；此时依靠参考变换变化判断，再按寿命处理历史。
4. 在全部必需 TF 和投影成功后，先清空历史，再标记新帧，最后一次性提交顶点并集。

阈值是初值，需要结合实车最大速度、帧间隔和定位修正幅度验证。仅凭 TF 不能区分所有重定位与正常运动：若重定位既没有参考变换突变，也不表现为可检测的相邻位姿跳变，需要定位模块明确提供重置通知才能完全识别。第一版先覆盖当前可观测的跳变，不新增通知话题。

地图替换或插件重新初始化时，历史也清空；缓存的 face/vertex 关联只能用于创建它们的那张 mesh。

## 8. YAML 示例与参数含义

以下是将来放到 `src/mesh_navigation_tutorials/config/mbf_mesh_nav.yaml` 的 **obstacle 层替换片段**。实际编辑时只替换现有 `mesh_map.obstacle` 内容，不以这个片段覆盖整个参数文件。

`history_voxel_size` 及后面的参数都是拟新增参数，目前原插件不认识它们。第一版采用修改 YAML 后重启的调参方式，通过第 4.1 节的内部标志使新插件的投影、历史和视野参数只读。`ros2 param set` 修改这些参数会被拒绝，配置从 YAML 重新加载；原插件继续保留自身原有的热更新行为。

```yaml
move_base_flex:
  ros__parameters:
    mesh_map:
      obstacle:
        # ---- 共用投影与输入 ----
        type: 'mesh_layers/TemporalObstacleLayer'
        topic: '/dynamic_obstacles'   # 唯一输入；正常无障碍时也须发布有效空帧。
        qos: 'Reliable'              # 沿用当前 PCL 发布端；队列深度继续为 1。
        tf_tolerance: 0.10           # TF 查询的最长等待时间 [s]，不是时间戳补偿。
        down_axis: [0.0, 0.0, -1.0]  # 投影方向，定义在 axis_frame 中。
        axis_frame: 'map'            # 沿 map 的负 Z 投影；保留当前方向。
        robot_height: 0.50           # 回波到 mesh 沿投影方向的最大距离 [m]。
        max_obstacle_dist: 8.0        # 从真实雷达原点计算的最大三维距离 [m]。
        combination_weight: 1.0      # 继承原层的组合权重。

        # ---- 历史与观测有效性 ----
        history_voxel_size: 0.05     # 历史回波去重的体素边长 [m]。
        obstacle_keep_time: 10.0     # 最后检测后，视野外的普通保留时间 [s]。
        visible_keep_time: 0.10      # 视野内连续未刷新时的过期窗口 [s]。
        max_observation_age: 0.30    # 处理时允许的消息年龄上限 [s]；初值需实测。

        # ---- 当前雷达的几何视野 ----
        sensor_frame: 'front_mid360' # TF 中雷达 link 名；不取点云 header 原点代替。
        sensor_offset_xyz: [0.0, 0.0, 0.03]  # link 到测量原点的局部偏移 [m]。
        min_sensor_range: 0.10       # 视野清除的最小三维距离 [m]。
        horizontal_fov_deg: 360.0    # 水平视野 [deg]，以雷达局部前向为中心。
        vertical_fov_min_deg: -7.0   # 垂直视野下界 [deg]。
        vertical_fov_max_deg: 52.0   # 垂直视野上界 [deg]。

        # ---- 定位跳变：阈值为待验证初值 ----
        localization_reference_frame: 'odom' # 与 map 对齐的连续里程计参考系。
        pose_jump_translation: 0.75  # 判定明显平移突变的阈值 [m]。
        pose_jump_rotation_deg: 45.0 # 判定明显旋转突变的阈值 [deg]。
        pose_jump_max_interval: 0.20 # 对雷达位姿作相邻突变检查的最大间隔 [s]。
```

8 米标记范围和 10 秒普通寿命是已确认值。0.05 米历史粒度、0.30 秒消息年龄和位姿阈值是工程初值，不能在未测试时视为比赛环境的最终参数。

初始化时检查：寿命非负、体素尺寸为正、范围有效、`visible_keep_time <= obstacle_keep_time`、FOV 角度有效、offset 为三个有限值。对于只读的新参数，提示修改 YAML 并重启即可。

## 9. 具体改哪些文件

所有路径相对项目根目录。

| 文件 | 修改内容 |
|---|---|
| `src/mesh_navigation/mesh_layers/include/mesh_layers/obstacle_layer.h` | 公共投影结果结构、三个 protected 接口、initialize 访问权限及参数只读标志 |
| `src/mesh_navigation/mesh_layers/src/obstacle_layer.cpp` | 提取原投影与提交代码，旧回调调用它们；按标志控制参数只读 |
| `src/mesh_navigation/mesh_layers/include/mesh_layers/temporal_obstacle_layer.h` | **新增**插件声明、历史 key/record、时间与位姿状态 |
| `src/mesh_navigation/mesh_layers/src/temporal_obstacle_layer.cpp` | **新增**参数声明、TF 位姿适配、历史更新和过期 |
| `src/mesh_navigation/mesh_layers/src/stvl_frustum/` | **新增**三个上游文件和 `UPSTREAM.md`，应用第 3 节的薄适配 |
| `src/mesh_navigation/mesh_layers/mesh_layers.xml` | 增加新插件条目，保留原插件条目 |
| `src/mesh_navigation/mesh_layers/CMakeLists.txt` | 编译新插件和 frustum，增加一个测试 target |
| `src/mesh_navigation/mesh_layers/test/temporal_obstacle_layer_test.cpp` | **新增**历史、TF 失败和变化传播的测试 |
| `src/mesh_navigation_tutorials/config/mbf_mesh_nav.yaml` | 切换 obstacle.type，增加带注释的配置 |

预计修改 5 个已有文件，新增 2 个插件文件、3 个上游代码文件、1 个上游说明和 1 个测试文件。历史结构可以放在新插件头中，不另建工具库或节点包。

适配后的头文件若直接使用 `geometry_msgs`，在 CMake 和 `package.xml` 显式补充已有 ROS 依赖的声明，避免只依靠传递依赖；不增加 OpenVDB 或 Nav2。这可能再修改一个已有文件。

### 9.1 插件注册

在现有 `mesh_layers.xml` 的 library 内增加：

```xml
<class name="mesh_layers/TemporalObstacleLayer"
       type="mesh_layers::TemporalObstacleLayer"
       base_class_type="mesh_map::AbstractLayer">
  <description>Obstacle layer with history and frustum expiry.</description>
</class>
```

新 cpp 使用已有注册方式：

```cpp
PLUGINLIB_EXPORT_CLASS(mesh_layers::TemporalObstacleLayer, mesh_map::AbstractLayer)
```

### 9.2 构建接入

在原 `add_library(mesh_layers ...)` 中加：

```cmake
src/temporal_obstacle_layer.cpp
src/stvl_frustum/three_dimensional_lidar_frustum.cpp
```

包内源文件使用相对 include 路径即可。插件头只声明 STL/Eigen 状态或 frustum 前置类型，使 vendor 头无需出现在安装接口；若使用未完整定义类型的 `unique_ptr`，将插件析构函数定义放在 cpp 中。

沿用当前 `ament_cmake_gtest`，一个测试文件覆盖下节的主要行为。首版不增加清除线程、额外 timer、PCL 二次分割或新的点云输出话题。

## 10. 按什么顺序实施

### 第一步：只整理共用代码

提取 `projectObservations()` 和 `commitLethalSet()`，旧 `ObstacleLayer` 调用它们。确认旧插件仍逐帧替换障碍，距离过滤默认原点和删除通知一致。

### 第二步：接入上游视野模型

固定提交下载三个文件并适配，测试 MID360 上下角、雷达偏移和 8 米范围。此时不切换运行配置。

### 第三步：写历史管理

在新插件里实现一个历史表、有效帧门控、普通寿命、视野缺失计时和重置。新历史只缓存原投影结果，不调用第二套 raycaster。

初始化先准备新插件参数和状态，再调用基类初始化创建订阅。回调沿用基类互斥回调组，历史操作串行；下游可通过原层写锁安全读取代价。

### 第四步：注册、编译与自动测试

完成新 class 和测试 target 后运行以下命令。这里列出的是未来实施时的命令，本文编写没有执行构建。

```bash
cd /home/rainple/nav_test/mesh_navigation_tutorials
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select mesh_layers --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mesh_layers --event-handlers console_direct+
colcon test-result --verbose
```

随后切换 YAML。若配置包的安装目录不是符号链接，重新构建它，确保运行加载的是新配置：

```bash
colcon build --packages-select mesh_navigation_tutorials --symlink-install
source install/setup.bash
```

### 第五步：Gazebo 分项验证，再上车

先验证标记、保留和清除，再测试封堵路线。保留现有 launch 入口，用运行时参数确认实际加载的是新插件和 `/dynamic_obstacles`。

```bash
ros2 param get /move_base_flex mesh_map.obstacle.type
ros2 param get /move_base_flex mesh_map.obstacle.topic
ros2 topic info /dynamic_obstacles -v
ros2 topic hz /dynamic_obstacles
```

首次验证将普通寿命临时设短，例如 2 秒，确认流程后再恢复 10 秒。实车使用当前校准的 sensor_frame、offset、真实点云频率和 TF 链。

## 11. 必须通过的测试

### 11.1 自动测试

用可控 ROS 时间、简单三角 mesh 和少量合成回波，验证精确边界。涉及 TF 的测试使用本包已有 MeshMap 初始化方式和有效/无效变换，不能只测试寿命算式而遗漏插件提交结果。

| 测试 | 必须满足的结果 |
|---|---|
| 原插件回归 | 一帧有点、下一帧有效空点时，原插件仍清除本帧以外障碍 |
| 新插件普通寿命 | 连续有效观测下，视野外历史在 10 秒边界过期，不随回调次数改变 |
| 视野清除 | 进入视野且未刷新时开始 0.1 秒窗口；连续缺失到期删除；再次检测则取消窗口 |
| 空帧与失败区分 | 有效空帧允许清除；TF 失败、过时帧、乱序帧不会清除 |
| 断流恢复 | 断流超过 10 秒时保持已有代价；恢复帧刷新仍存在障碍，删除未刷新超时记录 |
| 共享顶点 | 删除某记录后，另一条记录仍支持的顶点保持致命 |
| 历史 key | 负坐标 floor 正确；同体素不同 face 不互相刷新 |
| FOV 几何 | −7°/52°两侧、360°后方、倾斜姿态和轴线保护正确 |
| 雷达原点 | base 系点云采用真实 sensor 原点；8 米球形边界和 3 厘米外参方向正确 |
| 定位跳变 | 先清旧历史，再保留本帧新障碍；长断流后的正常移动不单凭位移被判重置 |
| 时间变化 | 仿真暂停不老化；新时间纪元清空旧时间状态 |
| 参数维护 | 新插件的相关参数拒绝热修改；重启后读取 YAML 新值；旧插件保留既有参数行为 |
| 层间传播 | 删除顶点通过 obstacle→inflation→final 生效；共享支持未消失时不误清 |
| 仅刷新时间 | 致命顶点集合不变时，不触发一次新的膨胀计算 |

如果旧回调已经彻底使用公共函数，旧插件回归测试也要覆盖该函数，避免整理代码时改变旧行为。

### 11.2 Gazebo 验收场景

| 场景 | 操作 | 观察结果 |
|---|---|---|
| 持续标记 | 放置一个静止障碍，保持持续扫描 | 10 秒后仍存在；持续被检测应不断刷新 |
| 视野外保留 | 移动车辆，使历史点离开配置的有效视野/距离 | 最后检测后保留约 10 秒，再清除 |
| 重入后清除 | 障碍在视野外被移走，再让旧位置进入视野 | 约 0.1 秒缺失窗口后清除，静态地图代价保留 |
| 视野内移动 | 在雷达范围内移走障碍 | 旧位置快速清除，新位置及时标记 |
| 停止消息 | 暂停分割发布，保持仿真时钟运行超过 10 秒 | 动态层保持；恢复有效帧后按当前观测处理 |
| 定位重置 | 注入明确的 map/odom 对齐跳变 | 旧动态历史清空，新位置正常标记 |
| 路线封堵 | 导航途中堵住隧道，再移开障碍 | 路线可行性按 final 更新；检查旧位置是否有残留 |

水平 360°的雷达仅旋转 yaw 通常不能把点移出视野。“视野外”场景应通过距离、垂直角或移动雷达实现；遮挡本身不能算几何视野外。

## 12. 实时性怎么确认

约 0.1 秒是视野缺失窗口，不是从雷达扫描到最终地图更新的全链路上限。

当前仿真原始雷达 20 Hz。如果 `/dynamic_obstacles` 也有效地达到 20 Hz，首次缺失帧开始计时，删除通常发生在其后约 0.10～0.15 秒的有效帧上，再叠加处理和膨胀时间。实际频率下降、TF 等待或回调排队会延迟清除；不能只设置 `visible_keep_time: 0.10` 就宣称满足 0.1 秒实时性。

沿用已有 `LayerTimer`，在新回调里增加少量分段计时；每隔数秒汇总，避免逐点日志：

| 数据 | 用途 |
|---|---|
| 消息年龄、输入频率 | 判断延迟是否在到达插件之前已经发生 |
| TF 等待、当帧投影耗时 | 判断外参等待和 raycast 是否阻塞 |
| 历史数量 K、视野/寿命检查、顶点并集合计耗时 | 测量本次新增工作量 |
| 变化顶点数、notifyChange/下游更新耗时 | 判断实际膨胀更新开销 |
| obstacle 和 final 的变化时刻 | 量测最终可用于规划的延迟 |

在最终硬件未确定前，先把“新增历史处理 P95 不超过一个有效输入周期的 20%”作为验证目标。例如 20 Hz 时目标约 10 ms，而不是未经测量的性能承诺。达不到时按上述分段数据定位原因，再做具体优化。

当前 PCL 节点还有单独的点云相减开销；历史插件无法弥补上游长延迟。它解决保留和清除语义，也减少重复投影及无变化时的膨胀通知。项目整体动态避障能力仍需用到达 final 的实测延迟判断。

## 13. 回退与交付标准

回退只需将同一层实例的类型改回：

```yaml
type: 'mesh_layers/ObstacleLayer'
```

再重启导航。原插件注册条目保留，新参数不会被旧插件使用，必要时移除或注释新增参数。

动态历史只存在内存中，沿用 ObstacleLayer 不读写自身动态代价的行为。10 秒、0.1 秒或视野参数变化采用重启，不需要为这张历史表生成 `.h5`。静态网格及其缓存继续遵循现有缓存流程。

完成实施时应交付：新插件及最小公共函数整理、固定提交的三个上游文件和改动说明、带注释 YAML、自动测试结果，以及 Gazebo 各场景和延迟量测结果。实车测试还要确认有效空帧持续输出、采样时刻 TF 可查询和 sensor 外参正确。

这套方案新增的核心业务代码集中在一个插件中：保存少量关联记录，按有效观测更新寿命，再把存活顶点并集交回现有 MeshNav。其余投影、膨胀、组合和路线检查继续使用当前实现。
