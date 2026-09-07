# RMUC2026 双全局规划器仿真

本目录已经把同一个 RMUC2026 Gazebo 场景接入 Mesh Navigation 和
`jie_3d_nav`。日常启动请直接看：

- **[两个导航的启动步骤（按终端逐条执行）](doc/START_NAVIGATION.md)**：MeshNav 与 JIE 分开说明，包含窗口操作、开始运动、停止和切换。

地图生成、编译、数据流、QoS、等价对比和详细故障排查见：

- [doc/CONVERSION_AND_USAGE.md](doc/CONVERSION_AND_USAGE.md)
- [JIE 地形接触/碰撞代码分析与舵轮哨兵改造方案](doc/JIE_SENTRY_TERRAIN_REDESIGN.md)（设计文档，尚未实施）

快速检查所有 canonical 地图和 ROS 副本是否一致：

```bash
cd /home/rainple/nav_test
python3 field/verify_rmuc_project.py
```

脚本通过后，按[启动步骤](doc/START_NAVIGATION.md)选择 A（MeshNav）或 B（JIE）路线。
