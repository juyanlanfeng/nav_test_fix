# RMUC2026 两处隧道入口凸起修复（2026-09-24）

## 修复范围与原因

根据 Gazebo 与 MeshNav 俯视图中圈出的两个位置，在地图坐标约 `(4.65, 4.95)` 和 `(-4.65, -4.95)` 找到对称的立体标记。每处由 5 个**独立、封闭**的小 STL 构件组成，并非坡面本体：标记顶面约 `z=0.13–0.14 m`，下方连续路面约 `z=0.106 m`。车轮因此会遇到约 2–3 cm 的局部台阶；该形状也被体素地图采入。

`field/remove_rmuc2026_tunnel_relief.py` 只移除这 10 个标记，显示 STL 和碰撞 STL 各移除 664 个三角面。脚本在动手前核对原文件 SHA-256、每个构件的边界和独立封闭性；其他 STL 三角面记录逐字节保留。原始 STEP **没有修改**，所以将来从 STEP 重新导出后，需要在生成导航地图前重新执行这个补丁；源网格不匹配时脚本会拒绝按旧面号处理。

## 已同步的运行资产

| 用途 | 当前文件 |
| --- | --- |
| Gazebo 显示与碰撞 | `field/converted_rmuc2026/gazebo/models/rmuc2026_field/meshes/rmuc2026_field_{visual,collision}.stl`，并同步到 `meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim/models/rmuc2026_field/meshes/` |
| MeshNav 全局地图 | `field/converted_rmuc2026/mesh_planner/rmuc2026.ply`，并同步到 `meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials/maps/rmuc2026_field.ply` |
| JIE 占据地图 | `field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd` |
| JIE 地面显示图 | `rmuc2026_field_floor.pcd`：重建后与旧文件 SHA-256 相同，无须替换 |

MeshNav 的旧 `.h5` 工作缓存包含旧地图。`pb_meshnav.launch.py` 的默认缓存已改为新的 `rmuc2026_pb_tunnel_relief_removed.h5`，首次重新启动时会从新 PLY 构建；手动传入 `mesh_map_working_path` 时也必须使用一个**新路径**。不要复用 `rmuc2026_pb_navigation.h5`。Gazebo 已加载的模型不会热更新，必须结束旧仿真并重新启动。

## 离线验收结果

- 显示 STL：`2234919 → 2234255` 面；碰撞 STL：`499999 → 499335` 面。两侧移除后，在原标记中心沿竖直方向仍能命中下方连续路面；不存在删构件留下的孔洞。
- JIE 点云：`631564 → 631508` 点，仅在两个目标区域各减少 28 个、`z=0.14 m` 的占据体素。`field/verify_jie_tunnel_pcd.py` 的正、负 Y 低层隧道检查均为 `connected=true`，且未误走顶板。
- MeshNav PLY：`180417 → 180502` 顶点、`348297 → 348467` 三角面。旧 STL 用当前生成器重建得到与旧 PLY **完全相同**的 SHA-256，因此新 PLY 差异来自此次网格修复。四条低层通道回归均通过。
- 在 `(±4.65, ±4.95)` 附近，PLY 中标记顶面高度由约 `0.132 m` 降为路面 `0.106 m`；该点的局部高度差由约 `0.056 m` 降到 `0`。圈内另一个取样点的高度差由约 `0.089 m` 降到 `0`。
- 无 Gazebo/RViz 的独立 ROS 域冒烟测试中，`mbf_mesh_nav` 成功加载新 PLY 的 `180502` 顶点、`348467` 面，完成代价层与规划器插件初始化，生成约 3.3 MB 的临时 H5，随后按限时器正常退出。此临时 H5 未作为 PB 运行缓存使用。

上述是几何与离线地图验收，**不是** Gazebo 中机器人双向驶过及规划闭环的验收。旧图上成片的红色还可能包含附近墙体、坡边与代价膨胀；去掉这几个浮雕不保证整片区域变成自由空间。重启后应分别检查：Gazebo 中两处凸起已消失、机器人双向通过、MeshNav/JIE 路径确实从预期坡面通过。

仓库里的旧 `field/verify_rmuc_project.py` 仍写死已不存在的 `mesh_planner/rmuc2026_field.ply` 和旧工作区布局，运行它会在文件存在检查阶段失败；它不是本轮验收依据。本轮使用了重建器自身的四通道回归、JIE 双隧道检查、网格射线与显示/碰撞/导航资产哈希一致性检查。

## 复现与回退

修复前的显示/碰撞 STL、PLY、PCD、生成报告和元数据已保留在 `field/converted_rmuc2026/backups/tunnel_relief_before_20260924/`。如需复现，先从该备份恢复两个原始 STL，运行 `field/remove_rmuc2026_tunnel_relief.py --help` 查看参数，在临时目录生成补丁 STL，再分别用 `field/build_multilevel_nav_mesh.py --rmuc2026-profile`、`field/build_jie_surface_pcd.py` 重建导航图；不要直接在旧 `.h5` 缓存上验收。
