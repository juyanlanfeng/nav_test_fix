# MeshNav 实车参考入口

此包放在 `doc/sim2real_templates/`，不会自动替换当前仿真工作空间。按仓库根目录
`SIM2REAL指南.md` 将两个参考包复制到独立实车准备工作空间，并应用文档提供的 PCL
补丁后构建。本包只依赖核心导航、PCL 和 `meshnav_runtime`，不启动硬件驱动。

入口固定启动四个节点：`command_gateway`、`obstacle_sub`、`move_base_flex`、
`meshnav_navigator`。启动不产生 TF 或 `/clock`；`start_rviz:=true` 才查询
`rviz2` 和 `rviz_mesh_tools_plugins`。速度网关每次启动都禁能，完整接口检查通过后
通过其 enable 服务手动使能；它不能代替底盘控制器的失联停车。

## 准备外部部署 bundle

将安装后的 `share/meshnav_real_bringup/config/` 中四份 YAML 复制到独立目录：

```text
site01_v1/
  robot.yaml
  meshnav_real.yaml
  pcl_real.yaml
  interfaces_real.yaml
  maps/
    site_nav.ply
    site_background.pcd
```

`meshnav_real.yaml` 是完整独立配置，没有仿真配置继承。固定代价参数和控制器增益仍需
读过并验收。`robot.yaml` 中的 null 要替换成实际测量值；map 路径相对于 bundle，
冻结资产后填 SHA256。导航的 TF 帧、传感器范围/FOV、平面包络/膨胀、差分阈值、
自身过滤、网关/控制器速度上限等从这份文件派生，修改时不需要改 launch 源码。

`collision_radius_m` 必须包住整车附件以及长宽矩形的四角。`safety_margin_m` 覆盖
实际地图、定位、感知和执行误差；两层膨胀内圈都取两者之和。软代价外圈不可小于
这个硬间距。`height_diff_threshold_m` 是运行层的高度差阈值，需要和离线可通行
面的生成参数分别验收。`obstacle_projection_height_m` 是回波投影距离，不是车高。

`sensors.offset_xyz` 是 lidar link 到测量原点的局部偏移，同时进入 PCL 和时间层；
外部 TF 已描述测量原点时保持零。它不是 `base → lidar` 的安装外参。FOV 为真实
有效视野。背景 PCD 已经配准到 map；PCL 的 `odom_frame` 参数名字虽然叫 odom，
本入口赋予它 map 来比较固定背景。连续里程计仍然保持 odom 帧。

`integration` 的四个布尔值记录已经完成的 PCL 源码补丁、背景配准、定位和 TF
发布者验收；不能仅改为 true 便声称完成。模板值为 false，因此原样启动会被拒绝。
校验器检查配置/路径/文件头/哈希，不证明地图连通性、外参或真实停车能力。

## 干净接口

| 边界 | 默认接口 | 硬件侧责任 |
|---|---|---|
| 当前去畸变扫描 | `/cloud` PointCloud2，lidar_link，BestEffort | 单帧 XYZ/采样时间和真正测量坐标；不能发累计地图 |
| 实际连续里程计 | `/odom` Odometry，odom → base_footprint，Reliable | 正确 pose/twist；发布 odom → base_footprint TF |
| 地图定位 | map → odom TF | 在旧地图定位，健康状态真实反映有效性 |
| 安装标定 | base_footprint → lidar_link TF | 唯一 URDF/外参发布者，覆盖扫描时刻 |
| 实际驱动可接收导航 | `/hardware/ready` Bool | 持续心跳；禁能/遥控接管等状态时 false |
| 实际急停 | `/hardware/estop` Bool | 持续心跳；急停有效时 true |
| 定位健康 | `/localization/healthy` Bool | 持续心跳；初始化/丢定位/质量不足时 false |
| 导航目标 | `/rviz/goal_pose` PoseStamped，map | 目标应落在正确层的可行驶面 |
| 唯一底盘侧导航输出 | `/hardware/cmd_vel` Twist | 实测米/秒、弧度/秒；支持 vy；最终仲裁与电机独立失联停车 |

导航原始速度内部经过 `/nav/cmd_vel` TwistStamped，网关检查新鲜度、消息坐标、
数据与健康输入、TF、有限数值和运动上限后输出 Twist。话题映射在
`interfaces_real.yaml` 中修改，frames 在 `robot.yaml` 中修改。若现有底盘驱动仅有
其他消息类型，需要独立硬件适配器；只改话题名字不完成类型转换。

用实际驱动状态生成 Bool 心跳，不能用永久发布 true 代替健康检测。网关故障后
锁定禁能，恢复数据不会自行恢复运动，重新使能也不会重发之前缓存的导航速度。
MBF 的里程计订阅使用 Reliable，实车 bundle 不接受 `odom_qos: best_effort`；
若原驱动使用 BestEffort，由硬件侧适配层保留采样时间、坐标和实际速度，并将标准
`/odom` 重新发布为 Reliable。

## 校验与启动

```bash
ros2 run meshnav_real_bringup meshnav_validate_bundle \
  --bundle-dir "$HOME/meshnav_bundles/site01_v1" --dump-effective

ros2 launch meshnav_real_bringup meshnav_real.launch.py \
  bundle_dir:="$HOME/meshnav_bundles/site01_v1" \
  cache_dir:="$HOME/.ros/meshnav/cache" start_rviz:=false
```

H5 不写入代码或地图目录。缓存名字包含地图哈希和有效导航配置哈希，地图/参数
改变时使用新缓存；时间层障碍历史仍然仅存在内存。节点参数显式使用真实时钟，
配置中出现 `use_sim_time: true` 会被拒绝。

```bash
ros2 topic echo /command_gateway/status
ros2 service call /command_gateway/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /meshnav_navigator/cancel std_srvs/srv/Trigger '{}'
```

完成真实方向/速度/失联停止和健康输入验收后，才对真实 bundle 调用 enable true。
本包不会编造 FAST-LIVO2、雷达或电机驱动启动命令。

## 桌面假硬件演练

```bash
ros2 run meshnav_real_bringup meshnav_make_smoke_bundle \
  --output "$HOME/meshnav_bundles/desktop_smoke"

# 所有演练终端使用同一隔离域，并只允许本机通信。
export ROS_DOMAIN_ID=209
export ROS_LOCALHOST_ONLY=1

# 终端 A：只发布假扫描、假里程计、TF 和假健康，不发目标、不使能。
ros2 run meshnav_real_bringup meshnav_fake_hardware \
  --bundle "$HOME/meshnav_bundles/desktop_smoke" --domain-id 209

# 终端 B：测试输出已固定到 /meshnav_smoke/chassis_cmd。
ros2 launch meshnav_real_bringup meshnav_real.launch.py \
  bundle_dir:="$HOME/meshnav_bundles/desktop_smoke" start_rviz:=false
```

合成 bundle 只能输出 `/meshnav_smoke/...`，不连接真实驱动。该演练用于检查进程、
地图加载、TF、参数/话题和保护链路；不验收真实定位、制动距离或复杂地形控制。
