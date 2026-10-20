# MeshNav 实车导航接口

本包保留两个节点：`meshnav_navigator` 管理导航任务，`pb_cmd_vel_adapter`
将导航速度转换为底盘驱动接收的消息。包名与可执行程序名保持现有名称。

## 文件用途

| 文件 | 用途 |
| --- | --- |
| `pb_vehicle_adapter/meshnav_navigator.py` | 接收目标，调用 MBF 规划、执行、检查剩余路线并按需重规划 |
| `pb_vehicle_adapter/cmd_vel_adapter.py` | 单路 `TwistStamped → Twist`、坐标系名称检查、指令超时清零 |
| `config/interfaces.yaml` | 集中配置话题、action/service、车体坐标系和时间参数 |
| `launch/meshnav_real.launch.py` | 启动上述节点；提供导航参数文件时同时启动 MBF |
| `launch/meshnav_rviz.launch.py` | 仅启动可视化，适合在操作电脑连接已运行的导航 |
| `rviz/meshnav_real.rviz` | 网格地图、代价、路径、点云、TF 与 Mesh Goal 工具 |

## 数据接口

```text
目标 PoseStamped → meshnav_navigator → MBF GetPath / ExePath
                                           ↓
                                 /cmd_vel (TwistStamped)
                                           ↓
                                  pb_cmd_vel_adapter
                                           ↓
                                 /base/cmd_vel (Twist)
                                           ↓
                                       实车底盘驱动
```

默认接口如下，话题名可在 `config/interfaces.yaml` 修改：

| 接口 | 类型 | 发布/服务提供方 → 使用方 |
| --- | --- | --- |
| `/rviz/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 或任务程序 → 导航协调器 |
| `/move_base_flex/get_path` | `mbf_msgs/action/GetPath` | MBF 服务端 ← 导航协调器 |
| `/move_base_flex/exe_path` | `mbf_msgs/action/ExePath` | MBF 服务端 ← 导航协调器 |
| `/move_base_flex/check_path_cost` | `mbf_msgs/srv/CheckPath` | MBF 服务端 ← 导航协调器 |
| `/meshnav_navigator/status` | `std_msgs/msg/String`，JSON | 导航协调器 → 监控端 |
| `/meshnav_navigator/cancel` | `std_srvs/srv/Trigger` | 导航协调器服务端 ← 任务程序 |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | MBF → 速度适配器 |
| `/base/cmd_vel` | `geometry_msgs/msg/Twist` | 速度适配器 → 底盘驱动 |
| `/odom` | `nav_msgs/msg/Odometry` | 外部里程计/定位系统 → MBF |

状态与取消接口使用节点私有名称 `~/status`、`~/cancel`；可通过 ROS remapping
修改。其余协调接口通过 YAML 参数修改。节点根键与 launch 中的节点名保持一致。
目标与地图的坐标系应一致。MBF 的 `robot_frame` 与适配器 `base_frame` 应一致。
适配器要求非空且匹配的 `header.frame_id`，不会旋转速度或查询 TF。

## 外部需要提供的功能

- MBF 地图、规划器、控制器和障碍层：由完整导航 YAML 配置，保留 LVR2 等核心依赖。
- 真实定位、里程计和 TF：通常为 `map → odom → base_footprint → lidar`，
  按实际定位方案指定各段的唯一发布者；本包不广播任何 TF。
- 雷达驱动及 `pcl_obstacle_point_subtraction`：单独启动；确保点云、背景地图、
  TF 对齐，PCL 输出话题与 MBF 障碍层订阅话题一致。
- 实车底盘驱动：接收配置的 `Twist` 输出，并负责底盘协议、运动学及执行。
- RViz：使用本包的实车视图在操作电脑启动，目标工具发布到配置的目标话题。

速度适配器原样保留六个速度分量，不做限速或底盘运动学转换；请在控制器及底盘
配置符合车辆能力的限速。底盘只接受 `TwistStamped` 时，可以不启动适配器，
让匹配接口的驱动直接订阅 MBF 输出。

## 启动

构建本包（其 ROS 和 MBF 消息依赖需要已安装或构建）：

```bash
source /opt/ros/humble/setup.bash
cd /home/rainple/nav_test/mesh_nav_ws
colcon build --packages-select pb_vehicle_adapter --symlink-install --parallel-workers 1
source install/setup.bash
```

只启动协调与速度接口，连接已经运行的 MBF：

```bash
ros2 launch pb_vehicle_adapter meshnav_real.launch.py
```

同时启动 MBF，指定你准备的完整导航参数文件与接口配置：

```bash
ros2 launch pb_vehicle_adapter meshnav_real.launch.py \
  navigation_params_file:=/absolute/path/to/mbf_real.yaml \
  interfaces_file:=/absolute/path/to/interfaces.yaml
```

完整导航 YAML 需要有 `move_base_flex.ros__parameters`，其中包含地图路径、
`planners`、`controllers`、插件及地图层配置；本包不复制场地地图和机器人尺寸。
启动时按“完整导航 YAML → interfaces.yaml → 系统时间”覆盖 MBF 参数。
导航协调器的参数只从接口配置读取，其中 planner/controller 必须是 MBF 中的实例名。
本入口启动的 MBF 速度输出自动重映射到适配器的 `input_topic`。
若 MBF 独立启动，需要自行配置同样的重映射及实车时间、frame、odom 参数。

所有由本入口启动的节点使用 `use_sim_time: false`。配置修改后重启生效。
启动进程不等于接口已就绪；定位、地图、障碍物和底盘接口应先完成接入。

## 可视化

导航已经运行时，在同一 ROS 域的操作电脑上单独打开 RViz：

```bash
ros2 launch pb_vehicle_adapter meshnav_rviz.launch.py
```

需要与导航接口一起启动时：

```bash
ros2 launch pb_vehicle_adapter meshnav_real.launch.py start_rviz:=true
```

同时启动 MBF 与 RViz：

```bash
ros2 launch pb_vehicle_adapter meshnav_real.launch.py \
  navigation_params_file:=/absolute/path/to/mbf_real.yaml start_rviz:=true
```

两个入口都可传入 `interfaces_file:=/absolute/path/to/interfaces.yaml` 和
`rviz_config:=/absolute/path/to/custom.rviz`。机器人端默认不打开窗口。
操作电脑需要 `rviz2` 和本仓库 `rviz_mesh_tools_plugins` 插件。

默认显示网格及 `final` 顶点代价、规划路径、雷达点云、动态障碍点云和 TF。
使用工具栏 **Mesh Goal** 在网格上发送导航目标。Fixed Frame 使用接口配置中的
`global_frame`；目标、点云和机器人描述话题也从接口配置读取。网格、代价和路径
默认连接 `/move_base_flex/...`，修改服务器命名空间时请同步修改 RViz 配置。

`RobotModel` 默认关闭；真实 `robot_state_publisher` 提供 URDF 和 link TF 后，
在 Displays 中勾选它。没有 URDF 时仍能通过 TF 查看机器人坐标系。

取消导航及查看状态：

```bash
ros2 service call /meshnav_navigator/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /meshnav_navigator/status --qos-durability transient_local
```

## 停车行为与迁移

适配器默认以 30 Hz 输出。启动时、输入超时、时间戳失效、坐标系不匹配或出现
非有限速度时，输出零速度；下一条有效输入可恢复输出。超时既检查单调时钟下的
接收间隔，也检查原始消息时间戳，重复发布不会延长指令有效期。通信两端需时间同步。
正常关闭时尽力发送零速度，进程强制退出或链路断开由底盘驱动自己的超时机制处理。
取消导航由协调器请求 MBF 停止执行；适配器不订阅 action 状态，仅处理速度流。

本次移除了多导航来源选择、真值定位、模型生成、桥接、旧启动检查和主动驻停纠偏。
旧 `meshnav_pb_sim.launch.py` 由 `meshnav_real.launch.py` 替代；旧 profile 参数不再读取。
`meshnav_topic` 改为 `input_topic`，默认底盘输出改为 `/base/cmd_vel`；
`control_source`、`hold_*`、`truth_*` 等旧参数已取消。原 PB 末端控制插件参数文件
也已移出本包；若另选控制插件，应把其参数放到自己的完整导航 YAML。

如果此前安装过旧版本，普通增量安装可能遗留旧入口和资源。使用独立 install
前缀，或在确认采用默认 isolated 安装布局后，仅清理本包的
`build/pb_vehicle_adapter` 与 `install/pb_vehicle_adapter` 再构建，避免误启动旧入口。
