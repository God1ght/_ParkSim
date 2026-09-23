# 地图扩展框架（Map Extension）

面向「把新停车场/新楼层接入 ParkSim」的完整框架：地图由什么组成、放在哪里、如何从
CAD 图纸（DWG/DXF）生成、如何接入仿真并验证。

> 现状：仿真运行时场地 = `priorFiles/data/DJI_XXXX`（嘉亭汇既有场地，30 份记录）+ 共享资产
> （spots / waypoints_graph / parking_maneuvers）。本文档定义**扩展第二类场地**的规范。
> 已有一个提取实例：`priorFiles/maps/jth_b1/`（嘉亭荟 B1，草案，见其 README）。

---

## 1. 地图的组成

仿真的一张「地图」由以下资产构成（路径由 `config/maps/<name>.yaml` 或地图目录解析：

| 资产 | 内容 | 必需 |
| --- | --- | --- |
| `spots_data.pickle` | 车位几何与语义（见 §3.1） | ✔ |
| `waypoints_graph.pickle` | 车道路网图（供 A*/Dijkstra 规划，见 §3.2） | ✔ |
| `parking_maneuvers.pickle` | 离线泊车机动库（16 组合查表） | ○ 缺省时用 `online_rs` 在线生成 |
| `agents_data_<name>.pickle` | 场景「经验初始化」的进场车辆数据 | ○ |
| `dlp/`（base_map.png 等） | 底图与障碍数据（simulator 展示层） | ○ 展示增强 |
| `layout.json` | 提取/标定的原始布局（车位框、标签、车道线） | ○ 生成用 |
| `base_map.png` | 底图渲染（网页静态层） | ○ 展示增强 |

**运行前提（按规划器语义分两类）**：
- **A 类·行式车库**（横向双排车位 + 过道，如 DJI 场地）：可直接运行；
  机动库缺省用 `maneuver_provider=online_rs`。
- **B 类·任意拓扑**（倾斜行、弧线车道等）：需扩展路网与机动生成（本文档 §6 路线）。

## 2. 目录与选择约定

```
python/parksim/priorFiles/maps/<name>/
├─ map.yaml            # 元信息 + ready 标志
├─ spots_data.pickle   # （生成后）
├─ waypoints_graph.pickle
├─ parking_maneuvers.pickle   # 可选
├─ layout.json         # 提取产物
├─ base_map.png
└─ tools/              # 提取/生成脚本（自包含）
```

`map.yaml` 示例：
```yaml
name: jth_b1
title: 嘉亭荟 B1
ready: false          # true 才会出现在网页「场地数据」列表
unit_to_meter: 0.2    # 图纸单位→米（1:200）
spots_data_path: ${PARKSIM_PRIORFILES}/maps/jth_b1/spots_data.pickle
waypoints_graph_path: ${PARKSIM_PRIORFILES}/maps/jth_b1/waypoints_graph.pickle
parking_maneuvers_path: ${PARKSIM_PRIORFILES}/maps/jth_b1/parking_maneuvers.pickle
```

加载路径（已实现）：`scenario.load_map(name)`
1. `config/maps/<name>.yaml` 预设（现有机制）→ 展开 `${PARKSIM_ROOT}`/`${PARKSIM_PRIORFILES}`；
2. `priorFiles/maps/<name>/map.yaml` 地图目录（新机制）；
3. `DJI_XXXX` 数据集名（现有机制）。

网页「场地数据」列表：`webviz/server.py::list_maps` 扫描 `priorFiles/data/DJI_*_scene.json`
与 `priorFiles/maps/*/map.yaml`（仅 `ready: true` 的地图会出现在选项中）。

## 3. 数据格式

### 3.1 spots_data.pickle
来自 `load_parking_spaces()` 读取，键：
```python
{
  'parking_spaces':        np.ndarray (N,2)  # 每个车位的中心（米）
  'overshoot_ranges':      {'pointed_right': [(i0,i1),...], 'pointed_left': [...]}
  'north_spot_idx_ranges': [(i0,i1),...]     # 「北排」（车头朝 +y）车位序号区间（按行）
  'spot_y_offset':         float             # 车位中心 → 过道对齐点的 y 距离（DJI=5.0m）
  'anchor_points':         [int, ...]        # 可选：锚点车位
  'anchor_spots':          [[int,...], ...]  # 可选：锚点分组
}
```
约定：车位序号从 0 连续编号；「北排」= 停车时车头朝 +y（`heading='up'`）。
`overshoot_ranges` 决定泊车起始点在车位前/后 4m（`compute_ref_path`）。

### 3.2 waypoints_graph.pickle
```python
{'graph': WaypointsGraph 实例, 'entrance_coords': np.array([x, y])}
```
`WaypointsGraph`（`parksim/route_planner/graph.py`）= 顶点（`Vertex.coords=np.array([x,y])`）
+ 有向边（`Edge.v1/v2/c`）。生成脚本可用
`graph.add_waypoint_list(waypoints: np.ndarray (M,2))` 加一串连通顶点，或自行构造
Vertex/Edge 后 pickle（保持类可被 `parksim` 导入即可 unpickle）。

### 3.3 parking_maneuvers.pickle（可选）
16 组合离线轨迹表：`lib[(driving_dir, x_position, spot, heading)] = traj (7, K)`
（行 = t/x/y/psi/v/u_a/u_steer）。**不提供时用 `online_rs` 在线生成**
（Reeds-Shepp，已支持邻位占用车辆/障碍考虑）。

## 4. 从 CAD 图纸生成（工具链）

以嘉亭荟 DWG 为例（全部脚本在 `priorFiles/maps/jth_b1/tools/`）：

```
DWG ──dwg2dxf(libredwg)──▶ DXF ──extract*──▶ 元素(线/多段线/文本/填充)
     ──labels2──▶ 编号标签 ──match_spots──▶ 车位框 ──render_final──▶ 底图+叠加验证
     ──build_package──▶ layout.json/map.yaml
```

1. **转换**：`brew install libredwg`（macOS）或 `apt install libredwg-tools`（Ubuntu），
   然后 `dwg2dxf -o out.dxf file.dwg`。DWG 版本老（AC1018）亦可。
2. **提取**：`analyze_dxf.py`（结构统计）→ `extract2.py`（多边形/文本/填充 JSON）→
   `extract_segs.py`（线段）。
   - 注意 PDF→CAD 矢量化的图纸：线段常为 3 顶点折线（A→B→A）+ SOLID 描边；
     文本为逐字符 TEXT，需按坐标聚类（`labels2.py`）。
3. **车位识别**：`match_spots.py` = 编号标签 + 包围它的矩形（尺寸聚类）匹配。
   - 先验证比例：行内标签间距中位数 × 比例 ≈ 车道间距（本例 12 单位 × 1:200 = 2.4m）。
4. **底图/验证**：`render_final.py` 生成干净底图与车位叠加图（人工核对对齐）。
5. **打包**：`build_package.py` 换算米制、写 `layout.json` + `map.yaml`（`ready: false`）。
6. **补全**（待做，B 类尤其需要）：倾斜行提取、无编号车位补齐、车道中心线/路网、车位朝向语义。

## 5. 接入与验收清单

1. 生成 `spots_data.pickle` + `waypoints_graph.pickle`（+ 可选机动库），放入地图目录；
2. `map.yaml` 配好路径 → `ready: true`；
3. 离线校验（不启动仿真）：
   - `load_parking_spaces` / `load_graph` / `load_maneuver` 全部可加载；
   - `graph.search(任意车位过道点)` 命中合理顶点；
4. 联机校验：
   - 网页「场地数据」选择新地图 → 重启 → `Scenario` 日志正确；
   - 车辆正常生成、路网巡航、泊入泊出（online_rs 回退率 < 阈值）；
   - 底图/车位/路径显示与实际位置一致；
5. 回归：原有 DJI 地图仍可正常选择运行。

## 6. 已知边界与路线（B 类地图）

- **倾斜/弧形车位行**：需在提取时按标签旋转角分簇，路网按行方向分段拟合；
- **任意路网规划**：现 A* 路网为图结构，任意拓扑天然支持；瓶颈在**车道中心线自动提取**
  （可从「空白过道区域」的形态学中轴或人工标注生成）；
- **机动生成**：`online_rs` 已可跨行向工作（Reeds-Shepp 任意位姿），倾斜行无需额外改动；
- **多楼层**：当作独立地图接入（各自 spots/graph），跨层坡道暂不建模。
