# field — RMUC 2026 场地资源转换工作区

本目录存放场地 CAD 模型或三维点云到各类导航地图
(mesh / OctoMap / 点云)的**转换脚本与说明文档**。支持两条独立输入流程：
`STP → mesh` 和 `PCD → mesh`。原始模型与转换产物**不纳入
git**,详见下文[资源文件说明](#资源文件说明)。

## 代码与文档清单

| 文件 | 用途 |
|---|---|
| `step_to_nav_maps.py` | STP → mesh 导航地图(主转换流程) |
| `pcd_to_nav_mesh.py` | PCD → 分层可行驶面候选 PLY；隧道/上下层需逐段验证 |
| `build_multilevel_nav_mesh.py` | 构建多层导航 mesh(含斜坡/隧道) |
| `build_jie_surface_pcd.py` | 生成 jie_path 使用的表面点云 PCD |
| `build_component_collision_mesh.py` | 生成部件碰撞网格 |
| `postprocess_nav_maps.py` | 导航地图后处理 |
| `conversion_metadata.py` | 安全合并可重复生成字段与人工/运行验证 provenance |
| `verify_rmuc_project.py` | 校验转换项目完整性 |
| `verify_jie_tunnel_pcd.py` | 校验隧道 PCD |
| `test_build_jie_surface_pcd.py` | 测试:jie 表面点云 |
| `test_build_multilevel_nav_mesh.py` | 测试:多层导航 mesh |
| `test_pcd_to_nav_mesh.py` | 测试:PCD 解析、降采样与分层 mesh |
| `test_conversion_metadata.py` | 测试:干净重建后的 metadata 闭环与兼容性 |
| `test_verify_rmuc_project.py` | 测试:H5 与几何/参数输入的新鲜度判定 |
| [`../doc/CONVERSION_AND_USAGE.md`](../doc/CONVERSION_AND_USAGE.md) | 转换流程与使用说明(详细文档) |
| `requirements-conversion.txt` | 转换脚本的 Python 依赖 |
| `RMUC2026_step_report.json` | STP 转换报告(元数据) |

## 资源文件说明

以下文件体积大(超出 GitHub 100MB 单文件限制)或属于可再生成的转换产物,
**已通过 .gitignore 排除,不随仓库分发**:

| 被排除路径 | 内容 | 大小 | 重建方式 |
|---|---|---|---|
| `RMUC2026_V2.0.0.stp` | RMUC 2026 场地原始 CAD | ~1.2 GB | 上游 CAD 来源,需人工获取 |
| `converted_rmuc2026/` | STP 转换产物:gazebo 模型(含 `rmuc2026_field_visual.stl`)、jie_nav PCD、mesh_planner mesh、缓存 | ~300 MB | 运行 `step_to_nav_maps.py` / `build_jie_surface_pcd.py` 等脚本重新生成 |
| `.step_convert_venv/` | 转换用 Python 虚拟环境 | ~160 MB | `requirements-conversion.txt` + `python -m venv` 重建 |

> 注意:`converted_rmuc2026/gazebo/models/.../rmuc2026_field_visual.stl`(107 MB)
> 是 Gazebo 仿真的视觉网格,超过 GitHub 单文件限制,故同样不纳入仓库。

## STP → MeshNav 导航地图

```bash
# 从项目根目录执行，确保下面的相对路径一致
cd /home/rainple/nav_test

# 1. 建立虚拟环境并安装依赖
python3 -m venv field/.step_convert_venv
field/.step_convert_venv/bin/pip install -r field/requirements-conversion.txt

# 2. 放置 RMUC2026_V2.0.0.stp 于 field/，先检查 STEP 单位和边界
field/.step_convert_venv/bin/python field/step_to_nav_maps.py inspect \
  field/RMUC2026_V2.0.0.stp \
  --report field/RMUC2026_step_report.json

# 3. 三角化 STEP 并生成基础/诊断资源；canonical PLY/PCD 还需继续执行详细文档第 7 节
field/.step_convert_venv/bin/python field/step_to_nav_maps.py convert \
  field/RMUC2026_V2.0.0.stp \
  --output field/converted_rmuc2026 \
  --model-name rmuc2026_field \
  --linear-deflection-mm 50 \
  --angular-deflection-deg 20 \
  --max-slope-deg 35 \
  --sample-spacing-m 0.08 \
  --min-points 10000 \
  --max-points 3000000 \
  --ground-bin-m 0.02 \
  --origin center-ground
```

完整的碰撞网格、JIE PCD、多层 MeshNav PLY 构建与验收步骤见
[`doc/CONVERSION_AND_USAGE.md`](../doc/CONVERSION_AND_USAGE.md)。

## PCD → MeshNav 导航地图

MeshNav 的 PLY 是**可行驶表面**，不是整栋建筑的外观网格：墙、柱、顶面不能作为机器人
行走的三角面。原始 PCD 才是三维场景证据；地图测试入口会同时显示 PCD 与导航面。
这条流程不做 Poisson 封闭重建，避免凭空封住隧道口，但也无法仅凭无射线信息的点云
自动证明每条通道可通行。生成 PLY 只是候选地图，必须检查连通性并实跑 GetPath。

2026-09-23 对 `field/pcd/map.pcd` 的实测：源点云 Z 为 -1.595～3.392 m，旧的
5 cm / 35° / 1 m² 产物仅保留 2 块，Z 为 -0.560～-0.350 m；它不能代表整个场景。
10 cm 网格的新候选 `site_v2.ply` 有 830 个原始连通块，筛选后仍有 4 块，最大一块
约 43.4 m²，整个候选地图 Z 为 -0.602～0.159 m。这**不是整栋场地导航已验收**。
用 0.22 m 内切半径、0.70 m 膨胀半径的 MeshNav 实测：局部路线
`(4.5,-1.2,-0.48) → (5.2,-0.7,-0.48)` 成功；`(-1,0,-0.5) → (3,0,-0.5)`
虽然几何连通，但两端距离网格边界分别约 0 和 0.10 m，小于 0.22 m 内切半径；
GetPath 返回 `Predecessor of the goal is not set! No path found!`。

### 1. 输入要求与检查

输入必须是 PCD v0.7，至少包含 `x y z`。支持 `DATA ascii` 和未压缩的
`DATA binary`，允许 intensity、rgb 等额外字段。如果带有有效的
`normal_x normal_y normal_z`，转换器会直接使用；没有法向或法向基本全为零时，会在体素
降采样后用局部 PCA 估计。`binary_compressed` 请先转换：

```bash
pcl_convert_pcd_ascii_binary input_compressed.pcd input_binary.pcd 1
```

PCL 1.12 在少数 `binary_compressed → binary` 转换中会在 `POINTS` 声明的数据后附加全零
填充。转换器会按字段和点数读取有效 payload，并在报告的 `trailing_padding_bytes` 中记录被忽略
的零/空白填充；如果尾部含非零未声明数据则报错，避免静默吞掉损坏或字段不匹配。

先检查字段、点数、边界、法向有效数和 SHA256，不生成地图：

```bash
cd /home/rainple/nav_test
field/.step_convert_venv/bin/python field/pcd_to_nav_mesh.py inspect \
  /path/to/map.pcd --report field/pcd_inspection.json
```

重点核对：坐标单位是否为米、Z 轴是否朝上、边界是否符合实际场地。毫米点云必须在转换时
显式使用 `--scale 0.001`。转换器不会猜单位，也不会自动配准坐标系。

### 2. 生成分层 PLY

通用地面机器人示例：

```bash
field/.step_convert_venv/bin/python field/pcd_to_nav_mesh.py convert \
  /path/to/map.pcd \
  field/converted_pcd/mesh_planner/site_v2.ply \
  --voxel-m 0.025 \
  --normal-k 24 \
  --grid-m 0.10 \
  --max-slope-deg 35 \
  --layer-merge-m 0.06 \
  --robot-height-m 0.35 \
  --min-points-per-cell 1 \
  --min-component-area-m2 1.0 \
  --report field/converted_pcd/mesh_planner/site_v2.pcd_to_mesh.json
```

参数含义：

| 参数 | 含义 |
|---|---|
| `--scale` | 输入单位到米的倍率；米为 1，毫米为 0.001 |
| `--translate X Y Z` | 缩放后施加的米制平移，用于对齐 map 原点，不负责旋转配准 |
| `--voxel-m` | 法向估计前的三维体素降采样；应小于最窄坡面/通道特征 |
| `--normal-k` | 无有效法向时，每点局部 PCA 的邻点数 |
| `--grid-m` | 输出导航网格分辨率；点间距大于格子时会破碎，增大后也必须检查是否误连墙/缺口 |
| `--max-slope-deg` | 可行驶面最大坡度 |
| `--layer-merge-m` | 同一 XY 柱内，视为同一表面的 Z 聚类阈值 |
| `--robot-height-m` | 从候选地面到上方下一表面的最小净高 |
| `--min-points-per-cell` | 一个 XY/Z 表面簇至少需要的点数；噪声多时提高到 2–3 |
| `--min-component-area-m2` | 保留的最小连通面面积；0 只保留最大连通面 |

场地存在互不相连但都要导航的平台时，设置合适的 `--min-component-area-m2`；只保留主场地时
使用 0。存在隧道时，`--robot-height-m` 必须使用真实碰撞包络高度加安全余量，不能填传感器
最高点，也不能填 0。

输出报告记录源 PCD/输出 PLY 哈希、输入字段、法向来源、各层和连通分量数量、边界及拓扑。
如果输出目录的上两级已有 `conversion_metadata.json`，还会写入独立的
`pcd_to_mesh_navigation` provenance，不会冒充 STP 产物。

不要只看总三角面数。检查 `raw_components`、`components_by_area`、
`selected_components`、`output_bounds_m`。当前 `map.pcd` 的大部分三维点是立面或顶面，
所以导航面 Z 范围显著小于场景范围是正常的；但 4 块之间无法规划跨块路线。
`meshnav_getpath_tested: false` 表示转换本身没有通过规划验收。

先对**具体**起终点做几何预检，Z 要填真实导航面高度：

```bash
field/.step_convert_venv/bin/python field/check_nav_mesh_route.py \
  field/converted_pcd/mesh_planner/site_v2.ply \
  --start 4.5 -1.2 -0.48 --goal 5.2 -0.7 -0.48 \
  --clearance-m 0.22 \
  --report field/converted_pcd/mesh_planner/route_precheck.json
```

`geometry_precheck_pass: true` 仅说明两点距离网格不超过 0.2 m、所在三角形沿边连通、
起终点距网格边界不小于指定半径；它不会检查**整条路线**的机器人净空、墙后误连、
静态膨胀、能否实际 GetPath，更不能代表闭环验收。
预检失败时不要继续用这组点测试，先修正地图/选点。
在这份点云上，`(-1,0,-0.5) → (3,0,-0.5)` 虽在同一连通块，但 0.22 m 边界
距离预检失败，实际 GetPath 也失败。

若目标是**整个场地**导航，下一步需要补采或核实地面/坡道的点云覆盖，按原始 PCD
划定可行驶面、墙体和真实开口，并对缺洞逐处审查；只有测量支持的缺口才可补面。
不要把墙和顶棚直接塞进 MeshNav 的行驶网格，也不要靠盲目放大 `--grid-m` 或缩小机器人
半径去制造虚假的连通。随后对每个入口、隧道底/顶及坡道设置起终点，逐项运行
几何预检和实际 GetPath；局部路线成功不等于跨区域连通，更不等于闭环通过。

### 3. 导入 MeshNav 并在 RViz 查看路线（不启动仿真）

下面只启动 MeshNav 地图服务器、原始点云显示和 RViz，不启动 Gazebo 或机器人。
可直接使用 `field` 输出的绝对路径，无需复制进地图包或覆盖旧 `site.ply`。
每次换 PLY 都用新的 H5 文件名，防止读到旧缓存：

```bash
# 新增 launch 和仅规划工具后，需要构建一次
source /opt/ros/humble/setup.bash
cd /home/rainple/nav_test/meshnav_demo_ws
colcon build --symlink-install --packages-select mesh_navigation_tutorials
source install/setup.bash
```

终端一启动地图服务器和 RViz。`mesh_map_working_path` 必须是可写的 `.h5` 路径，首次启动
会创建缓存，大地图可能需要等待一段时间：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 launch mesh_navigation_tutorials meshnav_map_test.launch.py \
  mesh_map_path:=/home/rainple/nav_test/field/converted_pcd/mesh_planner/site_v2.ply \
  mesh_map_working_path:=/home/rainple/nav_test/meshnav_demo_ws/site_pcd_v2_navigation.h5 \
  source_pcd_path:=/home/rainple/nav_test/field/pcd/map.pcd \
  publish_source_cloud:=true \
  static_inscribed_radius:=0.22 \
  static_inflation_radius:=0.70 \
  height_diff_threshold:=0.20
```

RViz 的 `Source PCD (scene only)` 应显示墙、柱、顶面，`MeshNav surface (planning)`
只显示可行驶候选面。两层必须重合对齐。如果转换时用了 `--scale` 或 `--translate`，
启动时还需给出同样的 `source_cloud_scale` / `source_cloud_translate_x/y/z`；
否则叠加视图会错位。两点请先通过上面的几何预检，再在终端二下发 GetPath：

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
ros2 run mesh_navigation_tutorials meshnav_plan_only \
  --start 4.5 -1.2 -0.48 \
  --goal   5.2 -0.7 -0.48
```

这组示例已实测输出 `PATH_READY poses=16`，RViz 的 `GetPath result` 会显示白色
路线。该进程每秒重发一次路径以保持显示；按 Ctrl+C 只关闭路线发布，地图服务器仍运行。
换起终点时关闭终端二并重新执行即可。

这里不要点击 RViz 的 `Mesh Goal`：仓库原有面板会使用 TF 中的当前机器人位姿，并在规划成功
后自动发送 `ExePath`。无机器人模式没有该 TF，也不应执行路径。`meshnav_plan_only` 固定使用
显式起点且只调用 GetPath。

也可以先用以下命令确认服务与显示话题：

```bash
ros2 action list | grep /move_base_flex/get_path
ros2 topic echo --once /move_base_flex/mesh
ros2 topic echo --once /move_base_flex/path
```

H5 缓存与 PLY 及导航参数绑定。更换 PLY、坡度/膨胀参数后必须删除旧 H5，让 MeshNav 重新生成。
`static_inscribed_radius` 应接近机器人占地内切半径，`static_inflation_radius` 应不小于它；
`height_diff_threshold` 是地形高差致命阈值。这三个值必须按待测试机器人配置，不能为了得到一条
路线无限缩小。若只是排查地图拓扑，可以临时减小膨胀半径做对照，但该结果不能作为通行验收。
如果 `PATH_READY` 前报 `OUT_OF_MAP`、`BLOCKED_START`、`BLOCKED_GOAL` 或 `NO_PATH_FOUND`，
先在 RViz 中核对坐标是否落在网格上，再核对两点是否属于同一连通面以及静态膨胀是否过大。

### 4. 验收与限制

先运行脚本测试：

```bash
PYTHONPATH=field field/.step_convert_venv/bin/python -m unittest \
  field/test_pcd_to_nav_mesh.py
```

然后至少检查 RViz 中的坡面连续性、隧道底面/顶面不串层、规划起终点连通、机器人净空，并做
闭环上下坡及重规划测试。点云只表达“测到了表面”，不表达传感器射线中的自由空间；缺点区域
是未知区域。无方向法向也不能仅凭几何区分地面正面与天花板背面。因此脚本成功、PLY 有三角面、
MeshNav 能加载都不能替代闭环验收。稀疏或遮挡严重的 PCD 应先补采/配准，不能靠调大
`--grid-m` 跨过真实缺口。
