# 实车接口参考文件

按 [SIM2REAL 指南第 8 节](../../SIM2REAL指南.md) 执行电脑改造、配置、检查与部署。这里提供可运行的参考实现，目录本身不会改变现有仿真安装。

已完成的本机软件检查与实际启动结果见 [验证记录](VALIDATION.md)。

| 内容 | 用途 |
|---|---|
| [tools/stage_reference.py](tools/stage_reference.py) | 默认只检查；`--apply` 将两个包复制到工作空间并应用 PCL 补丁；拒绝覆盖已有包 |
| [patches/pcl_real_interfaces.patch](patches/pcl_real_interfaces.patch) | PCL 原点偏移/QoS 参数、帧与时效检查、异常背景拒绝、扫描有效性检查 |
| [meshnav_runtime](meshnav_runtime/) | 独立协调节点、默认禁能的命令网关和测试 |
| [meshnav_real_bringup](meshnav_real_bringup/README.md) | 完整配置、外部 bundle 校验、四节点启动、合成地图与假硬件 |

从仓库根目录开始：

```bash
python3 doc/sim2real_templates/tools/stage_reference.py
python3 doc/sim2real_templates/tools/stage_reference.py --apply
```

预检查通过后才暂存。后续只构建 `--packages-up-to meshnav_real_bringup`，按指南限制最多三个包同时编译，使用独立普通安装目录。原样实车配置含未填的测量值和验收项，会拒绝启动；合成检查使用单独 `desktop_smoke` bundle、ROS 域 209 和 `/meshnav_smoke/chassis_cmd` 输出。

实车边界为当前点云、真实里程计、单一所有者的 TF、设备/急停/定位健康心跳与底盘 Twist 接口。CAN/串口、设备驱动、旧地图定位、外参和 MCU 独立失联停车由实际车辆系统提供。导航包不启动这些外部模块。

参考协调节点来自本仓库 `6a45dc4` 的 PB 实现，是用于纯实车闭包的独立快照。后续改协调逻辑时同步审阅参考版本与原版本。PCL 补丁针对该版本源码，源码改变后需要重新检查补丁。
