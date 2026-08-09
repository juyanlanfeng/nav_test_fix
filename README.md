# RMUC2026 双全局规划器仿真

本目录已经把同一个 RMUC2026 Gazebo 场景接入 Mesh Navigation 和
`jie_3d_nav`。完整的地图生成、编译、启动、数据流、QoS、等价对比和故障排查见：

- [field/CONVERSION_AND_USAGE.md](field/CONVERSION_AND_USAGE.md)

快速检查所有 canonical 地图和 ROS 副本是否一致：

```bash
cd /home/rainple/nav_test
python3 field/verify_rmuc_project.py
```

脚本通过后，优先从主文档的“最短可用流程”开始运行。
