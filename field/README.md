# field — RMUC 2026 场地资源转换工作区

本目录存放 RMUC 2026 场地 CAD 模型(`RMUC2026_V2.0.0.stp`)到各类导航地图
(mesh / OctoMap / 点云)的**转换脚本与说明文档**。原始 CAD 与转换产物**不纳入
git**,详见下文[资源文件说明](#资源文件说明)。

## 代码与文档清单

| 文件 | 用途 |
|---|---|
| `step_to_nav_maps.py` | STP → mesh 导航地图(主转换流程) |
| `build_multilevel_nav_mesh.py` | 构建多层导航 mesh(含斜坡/隧道) |
| `build_jie_surface_pcd.py` | 生成 jie_path 使用的表面点云 PCD |
| `build_component_collision_mesh.py` | 生成部件碰撞网格 |
| `postprocess_nav_maps.py` | 导航地图后处理 |
| `conversion_metadata.py` | 安全合并可重复生成字段与人工/运行验证 provenance |
| `verify_rmuc_project.py` | 校验转换项目完整性 |
| `verify_jie_tunnel_pcd.py` | 校验隧道 PCD |
| `test_build_jie_surface_pcd.py` | 测试:jie 表面点云 |
| `test_build_multilevel_nav_mesh.py` | 测试:多层导航 mesh |
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

## 使用

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
