#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_parking_maneuvers.py
==========================

为地图上**每个泊位**离线生成一条「带避障」的泊车机动轨迹（方案 D）。

背景
----
现役泊车机动是 2024 年 DLP/DJI 遗产的固定曲线（``priorFiles/parking_maneuvers.pickle``，
16 个 key），运行时按 ``park_start_coords ± 4 m`` 在**全局 x 方向**硬平移。它既不知道
泊位在哪，也不知道障碍在哪：实测 82.8% 的泊位机动与障碍相交，终点距泊位中心
median 4.5 m（99.6% > 1 m）。

本脚本改为：以**车道接入点**为起点、**泊位中心 + 泊入朝向**为终点，用 Reeds-Shepp
（复用 ``parksim.path_planner.hybrid_astar.reeds_shepp_path_planning.calc_paths``）
生成候选，再用 ``obstacles.json`` 的多边形对车辆矩形（4.6 x 1.85 m，由
``parksim.utils.get_corners.get_vehicle_corners`` 给出）做扫掠碰撞/间隙筛选，
输出**绝对坐标**的 per-spot 轨迹表。

输出
----
``<map-dir>/parking_maneuvers_per_spot.pickle``

    {
      "meta": {...},                       # 构建参数与统计
      "spots": {
          <spot_index:int>: {
              "x": [...], "y": [...], "psi": [...],     # 绝对坐标（rotated 米制）
              "v": [...],                                # 带符号速度（RS 含倒车段）
              "t": [...], "u_a": [...], "u_steer": [...],
              "meta": {
                  "spot_id": str,
                  "start": [x, y, psi], "goal": [x, y, psi],
                  "clearance": float,      # 与障碍的最小间隙（m）
                  "tight": bool,           # 擦碰深度 <= TIGHT_DEPTH_TOL，已接受
                  "depth": float,          # 最大穿透深度（m），0 表示无碰撞
                  "collides": bool,
                  "max_curvature": float,
                  "end_pos_err_m": float, "end_yaw_err_deg": float,
                  "n_candidates": int, "path_length_m": float,
              }
          }, ...
      }
    }

用法（服务器）::

    source /media/step/data/Yccc7/ParkSim-JTH/env_ros.sh
    /home/step/anaconda3/envs/parksim/bin/python \\
        python/parksim/map/build_parking_maneuvers.py \\
        --map-dir python/parksim/priorFiles/maps/jth_b1 \\
        --min-width 0.0 \\
        --out python/parksim/priorFiles/maps/jth_b1/parking_maneuvers_per_spot.pickle \\
        --report /tmp/build_parking_maneuvers_report.json

可重跑、幂等；自带自检（曲率门槛 / 终点误差 / 间隙统计），失败退出码非 0。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pickle
import sys
import time
from collections import Counter
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 可配置参数（不要写死在逻辑里）
# ---------------------------------------------------------------------------

#: 参与碰撞检测的障碍最小宽度（米）。
#: 历史默认值 2.0 会把 243 根柱子几乎全部滤掉（实测仅剩 36 个障碍），导致
#: 绝大多数泊车机动实际穿柱而过。现改为 0.0：**不做宽度过滤**，全部 400 个
#: 障碍（pillar 243 / wall 136 / boundary 12 / core 6 / facility 3）都进入约束集。
#: 其中 width_m 落在 (0, 0.2) 的仅 16 个，未观察到数量失控，故无需再设下限；
#: 若需收紧可用 --min-width 覆盖。
OBSTACLE_MIN_WIDTH_M = 0.0

#: 允许接受的最大擦碰深度（米）。超过则标记 collides=True（仍产出轨迹，交由上层决策）。
#: 车宽 1.85 m，0.10 m 约为车宽的 5%。
TIGHT_DEPTH_TOL_M = 0.10

#: 目标最小间隙（米），用于候选打分与统计。
CLEAR_TARGET_M = 0.10

#: RS 候选解评估上限（按路径长度升序取前 N 个）。
MAX_CANDIDATES = 60

#: RS 采样步长（米）。
RS_STEP_SIZE = 0.10

#: 扫掠位姿加密步长（米）。
SWEEP_STEP_M = 0.20

#: 三点外接圆曲率估计的基线间距（米）。
CURV_BASELINE_M = 0.50

#: 起点网格化：沿车道方向（起点航向）前后平移的采样偏移（米）。
#: 为什么需要：每个泊位的 Reeds-Shepp 解族只有 2~5 条（实测分布
#: {2:3, 3:6, 4:222, 5:38}），且 212/269 个泊位「全族皆侵入」——也就是说
#: 在**单一接入点**下不存在零侵入解，改约束集或加大 MAX_CANDIDATES 都无效。
#: 把接入点沿车道方向网格化后，解空间扩大 7 倍，实测最深侵入 0.3533 -> 0.12 量级。
#: 平移只改位置、不改航向，因此起点仍是车道上同一朝向的合法接入位姿，
#: 曲率门槛与终点误差天然保持。
START_OFFSETS_M = (0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0)

#: 打分的「免罚带」（米）：侵入深度 <= 本值视为等价（都是可忽略的浅接触）。
#: 为什么需要：原打分键第一维是 ``-worst_depth``，只看**最深的那一下**，
#: 完全不看「碰了几个约束」。网格化后实测出现「多处 0.005 m 浅擦」压过
#: 「单点 0.02 m」的选择，导致侵入**对数**上升（pairs 325 -> 344）而
#: 最深/累计反而下降。加免罚带后，带内一律等价，改由第二维的
#: ``-n_hit_poses``（接触位姿数）决定，语义回到「少碰优先」。
FREE_BAND_M = 0.02

#: 网格候选硬过滤 1：起点车辆矩形不得与任何约束相交（自身/串联泊位除外）。
#: 起点压在障碍或邻位 footprint 上 = 该位姿在真实场景里不存在。
GRID_REQUIRE_CLEAR_START = True

#: 网格候选硬过滤 2：起点相对 offset=0 到车道中心线的距离增量上限（米）。
#: 只限制「被挤出车道」的方向（d_new - d_old > 本值则丢弃）；贴边更近不限制。
GRID_MAX_LANE_DELTA_M = 0.30

#: 「起点压障碍面积守卫」的容差（平方米），只对显式列入白名单的泊位生效。
#:
#: 为什么需要、以及为什么不是硬过滤：新打分里的「接触位姿数」维度在长墙上
#: 是负优化 —— 车往墙里钻反而接触次数少，而 ``depth = 2A/(P1+P2)`` 对长墙
#: （周长几十米）极度不敏感，压墙在打分上几乎不花钱。实测 M891 起点压墙
#: 从 0.361 m² 被推到 1.324 m²（±3 新打分）/ 0.825 m²（±5 新打分）。
#:
#: 守卫规则是**相对**的，不是绝对硬过滤：先算出该泊位全部采样点里
#: 「起点压障碍面积」的**可达最小值** A_min，然后只保留
#: ``area <= A_min + 本值`` 的采样点。这样：
#:   - 不可能把候选砍空（取到 A_min 的那个采样点必然保留），无需降级兜底；
#:   - 不会像绝对硬过滤那样逼着选更深的解（对比 L713 0.0733 -> 0.0894 那次）；
#:   - 对「所有采样点本来就都压墙」的泊位（如 M891，7/7 全压 o_0279）
#:     自动退化为「只在压得最轻的那一档里选」，而不是无脑禁用。
START_OVERLAP_TOL_M2 = 0.02

#: 守卫的「深度安全阀」（米）：守卫后的解若比自由搜索的解深超过本值，
#: 判定为该泊位必须借道压墙，放弃守卫。见 plan_one_grid 的深度安全阀注释。
START_OVERLAP_VALVE_DEPTH_M = 0.01

#: 泊位「互侵」约束多边形的外扩量（米）。
#: 约束多边形 = 泊位 quad ∪（按 spot_headings 停放的车辆矩形 4.6 x 1.85 外扩本值）。
#: 之所以不再只用 quad：quad 是标线框，不等于车辆实际停放 footprint —— 实测
#: 14/269 个泊位按 spot_headings 停放时车装不进自家 quad（最差 K651 只有
#: 52.2% 车身在框内、朝向与 quad 长轴差 88.3°；其余 13 个是 quad 长
#: 4.10~4.30 m 短于车长 4.6 m 或有 10~29° 角差）。只用 quad 会漏检。
#: 0.15 m ≈ 车宽的 8%，作为「邻位有车」的安全余量。
SPOT_FOOTPRINT_INFLATE_M = 0.15


# ---------------------------------------------------------------------------
# 车辆运动学门槛（与 vehicle_types.py 一致：wb=2.70 m, delta_max=40 deg）
# ---------------------------------------------------------------------------
def kinematic_limits(wheelbase: float = 2.70,
                     delta_max_rad: float = math.radians(40.0)) -> Tuple[float, float]:
    """返回 (R_min [m], k_max [1/m])。"""
    r_min = wheelbase / math.tan(delta_max_rad)
    return r_min, 1.0 / r_min


# ---------------------------------------------------------------------------
# 依赖（parksim 必须在 PYTHONPATH 里）
# ---------------------------------------------------------------------------
def _import_parksim():
    """延迟导入，保证 --help 在无 parksim 环境也能用。"""
    from shapely import STRtree
    from shapely.geometry import Point, Polygon

    from parksim.pytypes import VehiclePrediction, VehicleState
    from parksim.utils.get_corners import get_vehicle_corners
    from parksim.vehicle_types import VehicleBody, VehicleConfig
    from parksim.path_planner.hybrid_astar.reeds_shepp_path_planning import calc_paths

    return {
        "STRtree": STRtree, "Point": Point, "Polygon": Polygon,
        "VehiclePrediction": VehiclePrediction, "VehicleState": VehicleState,
        "get_vehicle_corners": get_vehicle_corners,
        "VehicleBody": VehicleBody, "VehicleConfig": VehicleConfig,
        "calc_paths": calc_paths,
    }


# ---------------------------------------------------------------------------
# 几何工具
# ---------------------------------------------------------------------------
def densify(xs: np.ndarray, ys: np.ndarray, psis: np.ndarray,
            step: float = SWEEP_STEP_M) -> List[Tuple[float, float, float]]:
    """把位姿序列按弧长加密到 <= step 米，避免扫掠漏检。"""
    if len(xs) < 2:
        return [(float(xs[0]), float(ys[0]), float(psis[0]))]
    out: List[Tuple[float, float, float]] = []
    for i in range(len(xs) - 1):
        d = float(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]))
        n = max(1, int(math.ceil(d / step)))
        for k in range(n):
            t = k / n
            out.append((xs[i] + (xs[i + 1] - xs[i]) * t,
                        ys[i] + (ys[i + 1] - ys[i]) * t,
                        psis[i] + (psis[i + 1] - psis[i]) * t))
    out.append((float(xs[-1]), float(ys[-1]), float(psis[-1])))
    return out


def max_curvature(xs: np.ndarray, ys: np.ndarray,
                  dirs: Optional[Sequence[float]] = None,
                  baseline: float = CURV_BASELINE_M) -> float:
    """三点外接圆(Menger)曲率估计——对圆弧解析精确，避免弦/弧离散误差。

    ``dirs`` 给定时跳过含行进方向反转的窗口（Reeds-Shepp 尖点不是曲率）。
    """
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    n = len(xs)
    if n < 5:
        return 0.0
    ds = np.hypot(np.diff(xs), np.diff(ys))
    mean_step = float(np.mean(ds)) if len(ds) else 0.0
    if mean_step <= 1e-9:
        return 0.0
    stride = max(2, int(round(baseline / mean_step)))
    d = None if dirs is None else np.asarray(dirs, dtype=float)
    kmax = 0.0
    hop = max(1, stride // 2)
    for i in range(0, n - 2 * stride, hop):
        a, b, c = i, i + stride, i + 2 * stride
        if d is not None and d[a] * d[c] <= 0:
            continue
        ax, ay = xs[a], ys[a]
        bx, by = xs[b], ys[b]
        cx, cy = xs[c], ys[c]
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        A = math.hypot(bx - ax, by - ay)
        B = math.hypot(cx - bx, cy - by)
        C = math.hypot(cx - ax, cy - ay)
        if abs(cross) < 1e-9 or min(A, B, C) < 1e-6:
            continue
        kmax = max(kmax, 2.0 * abs(cross) / (A * B * C))
    return float(kmax)


def count_cusps(dirs: Optional[Sequence[float]]) -> int:
    """行进方向反转次数（倒车段数）。"""
    if dirs is None:
        return 0
    d = np.asarray(dirs, dtype=float)
    if len(d) < 2:
        return 0
    return int(np.sum(d[:-1] * d[1:] < 0))


# ---------------------------------------------------------------------------
# 地图/障碍加载
# ---------------------------------------------------------------------------
def load_obstacles(obstacles_json: str, min_width: float, P):
    """读 obstacles.json，返回 (polys, metas)；按 width_m >= min_width 过滤。"""
    Polygon = P["Polygon"]
    with open(obstacles_json, "r", encoding="utf-8") as f:
        doc = json.load(f)
    polys: List = []
    metas: List[dict] = []
    for o in doc.get("obstacles", []):
        if (o.get("width_m") or 0.0) < min_width:
            continue
        pts = [(float(p[0]), float(p[1])) for p in o.get("polygon", [])]
        if len(pts) < 3:
            continue
        poly = Polygon(pts)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            continue
        polys.append(poly)
        metas.append({"id": o.get("id"), "type": o.get("type"),
                      "width_m": o.get("width_m"), "length_m": o.get("length_m"),
                      "center_rot": o.get("center_rot")})
    return polys, metas


#: layout_rotated.json 中泊位 quad 所在的文件名（rotated 米制，与绝对轨迹同帧）。
SPOT_LAYOUT_JSON = "layout_rotated.json"


def load_spot_quads(layout_json: str, n_spots: int, P):
    """读 layout_rotated.json，返回 (quads, index_by_id, through_map)。

    ``quads[i]`` 是第 i 个泊位的 4 点多边形（rotated 米制），可直接作为通用
    约束多边形塞进 poly 列表——生成器对 tree/polys 不做类型假设。

    ``through_map[i]`` 是该泊位串联依赖的泊位 id 列表（``access.through_spot_ids``），
    访问第 i 个泊位必须借道这些泊位，因此它们要和自身泊位一样被放行。
    """
    Polygon = P["Polygon"]
    quads: List = [None] * int(n_spots)
    index_by_id: Dict[str, int] = {}
    through_map: Dict[int, List[str]] = {}
    if not layout_json or not os.path.exists(layout_json):
        return quads, index_by_id, through_map
    with open(layout_json, "r", encoding="utf-8") as f:
        doc = json.load(f)
    for i, s in enumerate(doc.get("spots") or []):
        if i >= int(n_spots):
            break
        pts = [(float(p[0]), float(p[1])) for p in (s.get("quad") or [])]
        if len(pts) >= 3:
            poly = Polygon(pts)
            if not poly.is_valid:
                poly = poly.buffer(0)
            if not poly.is_empty:
                quads[i] = poly
        sid = s.get("id")
        if sid is not None:
            index_by_id[str(sid)] = i
        acc = s.get("access") or {}
        tids = [str(t) for t in (acc.get("through_spot_ids") or [])]
        if tids:
            through_map[i] = tids
    return quads, index_by_id, through_map


def build_spot_footprints(quads: Sequence, centers: np.ndarray, headings: np.ndarray,
                          n_spots: int, body, P,
                          inflate: float = SPOT_FOOTPRINT_INFLATE_M) -> Tuple[List, int]:
    """把每个泊位的 quad 与「按 spot_headings 停放的车辆矩形」取并集。

    返回 ``(footprints, n_park_rect_used)``。

    为什么要并集而不是只用 quad：
      * quad 是标线框，实际停放 footprint 是 ``parking_spaces[i] + spot_headings[i]``
        处的 4.6 x 1.85 车辆矩形。二者不一致（14/269 个泊位装不进），只用 quad 既会
        漏检（车屁股伸出框外撞到邻车却判定无碰撞），也会误判。
      * 取并集是**保守**的：约束区域只增不减。
      * 外扩 ``inflate`` 给「邻位已停放」留安全余量。
    """
    Polygon = P["Polygon"]
    get_vehicle_corners = P["get_vehicle_corners"]
    VehicleState = P["VehicleState"]

    st = VehicleState()
    footprints: List = [None] * int(n_spots)
    n_rect = 0
    for i in range(int(n_spots)):
        parts = []
        q = quads[i] if i < len(quads) else None
        if q is not None:
            parts.append(q)
        try:
            st.x.x = float(centers[i][0])
            st.x.y = float(centers[i][1])
            st.e.psi = float(headings[i])
            corners = get_vehicle_corners(state=st, vehicle_body=body)
            rect = Polygon([(float(c[0]), float(c[1])) for c in corners])
            if not rect.is_valid:
                rect = rect.buffer(0)
            if not rect.is_empty:
                parts.append(rect.buffer(float(inflate)))
                n_rect += 1
        except Exception:
            pass
        if not parts:
            continue
        u = parts[0]
        for extra in parts[1:]:
            u = u.union(extra)
        if not u.is_valid:
            u = u.buffer(0)
        if not u.is_empty:
            footprints[i] = u
    return footprints, n_rect


def file_md5(path: str) -> Tuple[str, int]:
    """返回 (md5_hex, 字节数)。文件不存在返回 ('', -1)。"""
    if not path or not os.path.exists(path):
        return "", -1
    h = hashlib.md5()
    size = 0
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
            size += len(chunk)
    return h.hexdigest(), size


def count_lines(path: str) -> Optional[int]:
    """文本文件的行数；二进制（pickle）返回 None。"""
    if not path or not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        return sum(1 for _ in f)


def describe_victim(idx: Optional[int], metas: Sequence[dict],
                    spot_ids: Sequence[str]) -> str:
    """把「侵入最深的约束多边形下标」翻译成人能读懂的对象描述。"""
    if idx is None:
        return "未知（未定位到具体约束多边形）"
    if not (0 <= int(idx) < len(metas)):
        return "poly#%s（越界）" % idx
    m = metas[int(idx)]
    if m.get("type") == "spot_footprint":
        return "泊位 %s 的停放 footprint" % m.get("id")
    return "障碍 %s (type=%s)" % (m.get("id"), m.get("type"))


def cruise_heading_at(graph, wp: Sequence[float]) -> float:
    """车道接入点处的巡航航向：用最近顶点的**入边**方向（车从哪儿开过来）。"""
    verts = list(graph.vertices)
    coords = np.array([v.coords for v in verts], dtype=float)
    idx = int(np.argmin(np.linalg.norm(coords - np.asarray(wp, dtype=float), axis=1)))
    parent_of = {}
    for v in verts:
        for c in v.children:
            parent_of[id(c)] = v
    p = parent_of.get(id(verts[idx]))
    if p is not None:
        dx = float(verts[idx].coords[0] - p.coords[0])
        dy = float(verts[idx].coords[1] - p.coords[1])
        if math.hypot(dx, dy) > 1e-6:
            return float(math.atan2(dy, dx))
    if verts[idx].children:
        c = verts[idx].children[0]
        dx = float(c.coords[0] - verts[idx].coords[0])
        dy = float(c.coords[1] - verts[idx].coords[1])
        if math.hypot(dx, dy) > 1e-6:
            return float(math.atan2(dy, dx))
    return 0.0


# ---------------------------------------------------------------------------
# 扫掠评估
# ---------------------------------------------------------------------------
def sweep_evaluate(xs: np.ndarray, ys: np.ndarray, psis: np.ndarray,
                   dirs: Optional[Sequence[float]],
                   goal: Tuple[float, float, float],
                   tree, polys: Sequence, body, state, P,
                   exclude: frozenset = frozenset()) -> dict:
    """曲率 / 终点误差 / 最小间隙 / 碰撞。

    ``exclude`` 是 poly 索引黑名单：这些多边形在当前泊位的机动里被**放行**
    （自身泊位 quad——终点就落在里面；以及串联借道的泊位 quad），不参与
    碰撞判定与间隙统计。
    """
    Polygon = P["Polygon"]
    get_vehicle_corners = P["get_vehicle_corners"]

    poses = densify(xs, ys, psis, step=SWEEP_STEP_M)
    min_clear = float("inf")
    n_hit = 0
    worst_depth = 0.0
    worst_idx = None                 # 侵入最深的那个约束多边形在 polys 里的下标
    for (x, y, psi) in poses:
        state.x.x = x
        state.x.y = y
        state.e.psi = psi
        corners = get_vehicle_corners(state=state, vehicle_body=body)
        rect = Polygon([(float(c[0]), float(c[1])) for c in corners])
        if not rect.is_valid:
            rect = rect.buffer(0)
        best_d = float("inf")
        try:
            jn = tree.nearest(rect)
            if jn is not None and jn not in exclude:
                best_d = float(rect.distance(polys[jn]))
        except Exception:
            best_d = float("inf")
        for j in tree.query(rect.buffer(0.5)):
            if j in exclude:
                continue
            obs = polys[j]
            d = float(rect.distance(obs))
            if d < best_d:
                best_d = d
            if d <= 1e-9:
                n_hit += 1
                try:
                    depth = float(2.0 * rect.intersection(obs).area /
                                  (rect.length + obs.length))
                except Exception:
                    depth = 0.0
                if depth > worst_depth:
                    worst_depth = depth
                    worst_idx = int(j)
        min_clear = min(min_clear, best_d)
    if not math.isfinite(min_clear):
        min_clear = 10.0        # 10 m 内无障碍

    k = max_curvature(xs, ys, dirs=dirs)
    end_pos_err = float(math.hypot(xs[-1] - goal[0], ys[-1] - goal[1]))
    d_yaw = (float(np.unwrap(psis)[-1]) - goal[2] + math.pi) % (2 * math.pi) - math.pi
    end_yaw_err = float(abs(d_yaw))
    length = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    return {
        "max_curvature": k,
        "end_pos_err_m": end_pos_err,
        "end_yaw_err_deg": math.degrees(end_yaw_err),
        "min_clearance_m": float(min_clear),
        "n_hit_poses": n_hit,
        "collides": bool(n_hit > 0),
        "worst_depth_m": float(worst_depth),
        "worst_idx": worst_idx,
        "path_length_m": length,
        "n_cusps": count_cusps(dirs),
        "n_samples": int(len(xs)),
    }


# ---------------------------------------------------------------------------
# 单条机动生成（Reeds-Shepp + 障碍筛选）
# ---------------------------------------------------------------------------
def plan_one(start: Tuple[float, float, float],
             goal: Tuple[float, float, float],
             tree, polys, body, state, P,
             k_max: float,
             max_candidates: int = MAX_CANDIDATES,
             step_size: float = RS_STEP_SIZE,
             exclude: frozenset = frozenset(),
             free_band: float = FREE_BAND_M):
    """为单个泊位生成一条带避障的泊车机动。返回 (stat, path) 或 (None, None)。

    ``exclude`` 透传给 sweep_evaluate（见其 docstring）——放行自身与串联泊位。
    """
    calc_paths = P["calc_paths"]
    paths = calc_paths(start[0], start[1], start[2],
                       goal[0], goal[1], goal[2],
                       k_max, step_size)
    if not paths:
        return None, None
    cands = sorted(paths, key=lambda p: abs(p.L))[:max_candidates]
    best_stat = None
    best_p = None
    best_key = None
    for p in cands:
        xs = np.asarray(p.x, dtype=float)
        ys = np.asarray(p.y, dtype=float)
        psis = np.unwrap(np.asarray(p.yaw, dtype=float))
        dirs = np.asarray(p.directions, dtype=float)
        # 侵入深度在 sweep_evaluate 内部已算出，这里直接用它排序，
        # 保证「最小化侵入优先」的语义建立在真实深度上（而非无碰撞布尔）。
        stat = sweep_evaluate(xs, ys, psis, dirs, goal, tree, polys, body, state, P,
                              exclude=exclude)
        stat["_len"] = abs(float(p.L))
        # 选优：超出免罚带的侵入深度最小 > 接触位姿数最少 > 间隙大 > 路径短。
        # 注意极性：key 取 max，所以「越小越好」的量取负号。
        #   - 第一维 max(depth - FREE_BAND_M, 0)：带内浅接触一律等价（不罚），
        #     带外越深越差。有零侵入解时本维为 0，落到下一维。
        #   - 第二维 -n_hit_poses：带内比较时按「碰了几下」排序，避免网格化后
        #     用「多处浅擦」换掉「单点稍深」。
        #   - 若所有候选都超带，则自动落到**超带量最小的退化解**，而不是判失败。
        key = (-max(stat["worst_depth_m"] - float(free_band), 0.0),
               -int(stat["n_hit_poses"]),
               min(stat["min_clearance_m"], 0.6),
               -stat["_len"])
        if best_key is None or key > best_key:
            best_key, best_stat, best_p = key, stat, p
    if best_stat is None:
        return None, None
    best_stat.pop("_len", None)
    # 把本次选优用的打分键带出去：起点网格化要在**不同起点**之间做同一把尺子的比较，
    # 必须复用同一个 key（否则跨起点并列时的 tie-break 会不一致）。
    best_stat["_score_key"] = best_key
    return best_stat, best_p


def start_pose_rect(x: float, y: float, psi: float, body, P):
    """返回起点位姿处的车辆矩形（4.6 x 1.85 m）。"""
    st = P["VehicleState"]()
    st.x.x = float(x)
    st.x.y = float(y)
    st.e.psi = float(psi)
    corners = P["get_vehicle_corners"](state=st, vehicle_body=body)
    rect = P["Polygon"]([(float(c[0]), float(c[1])) for c in corners])
    if not rect.is_valid:
        rect = rect.buffer(0)
    return rect


def start_obstacle_overlap(rect, tree, polys, n_obstacles: int) -> float:
    """起点车辆矩形与**障碍**（polys 前 n_obstacles 个）的重叠面积（m²）。

    只看障碍、不看邻位 footprint：footprint 含 0.15 m 外扩，压到外扩带不等于
    撞车；压到墙/柱才是物理上不存在的位姿。
    """
    total = 0.0
    for j in tree.query(rect):
        if j >= n_obstacles:
            continue
        if not rect.intersects(polys[j]):
            continue
        try:
            total += float(rect.intersection(polys[j]).area)
        except Exception:
            continue
    return total


def start_pose_blocked(rect, tree, polys, exclude: frozenset) -> bool:
    """起点车辆矩形是否与任一约束相交（``exclude`` 中的自身/串联泊位不算）。"""
    for j in tree.query(rect):
        if j in exclude:
            continue
        if rect.intersects(polys[j]):
            return True
    return False


def lane_segments_from_graph(graph) -> np.ndarray:
    """把 waypoints 有向图摊成线段数组，形状 (N, 2, 2)：[start_xy, end_xy]。"""
    segs = []
    for v in graph.vertices:
        a = np.asarray(v.coords, dtype=float).reshape(2)
        for c in v.children:
            b = np.asarray(c.coords, dtype=float).reshape(2)
            segs.append((a, b))
    if not segs:
        return np.zeros((0, 2, 2), dtype=float)
    return np.asarray(segs, dtype=float)


def lane_distance_m(p: Sequence[float], segs: np.ndarray) -> float:
    """点到车道图最近线段的距离（米）。segs 为空时返回 inf。"""
    if segs.size == 0:
        return float("inf")
    p = np.asarray(p, dtype=float).reshape(2)
    a = segs[:, 0, :]
    ab = segs[:, 1, :] - a
    l2 = np.einsum("ij,ij->i", ab, ab)
    num = np.einsum("ij,ij->i", (p - a), ab)          # (p-a) · ab
    t = np.where(l2 > 1e-12,
                 np.clip(num / np.where(l2 > 1e-12, l2, 1.0), 0.0, 1.0),
                 0.0)
    d = np.linalg.norm(p - (a + t[:, None] * ab), axis=1)
    return float(np.min(d))


def plan_one_grid(start: Tuple[float, float, float],
                  goal: Tuple[float, float, float],
                  tree, polys, body, state, P,
                  k_max: float,
                  max_candidates: int = MAX_CANDIDATES,
                  step_size: float = RS_STEP_SIZE,
                  exclude: frozenset = frozenset(),
                  offsets: Sequence[float] = START_OFFSETS_M,
                  lane_segs: Optional[np.ndarray] = None,
                  require_clear_start: bool = GRID_REQUIRE_CLEAR_START,
                  max_lane_delta: float = GRID_MAX_LANE_DELTA_M,
                  stats: Optional[Dict[str, int]] = None,
                  free_band: float = FREE_BAND_M,
                  n_obstacles: int = 0,
                  start_overlap_tol: Optional[float] = None,
                  start_overlap_valve: Optional[float] = START_OVERLAP_VALVE_DEPTH_M,
                  start_overlap_out: Optional[Dict[str, float]] = None):
    """起点网格化求解：沿车道方向（起点航向）前后平移起点，各解一次，取全局最优。

    返回 ``(stat, path, chosen_offset)``；无解返回 ``(None, None, None)``。

    评分键与 :func:`plan_one` 内部完全一致（``_score_key``），因此并列时的
    tie-break 顺序是「超带侵入量 -> 接触位姿数 -> 间隙 -> 路径长度」。

    网格候选在执行前先过两道**硬过滤**（任一不满足即丢弃该偏移）：

    1. ``require_clear_start``：起点车辆矩形不得与任何约束相交（自身/串联泊位
       除外）。起点压在柱子/墙/邻位 footprint 上的位姿在真实场景里不存在。
    2. ``lane_segs`` 给定时，起点到车道中心线的距离相对 offset=0 的增量
       ``d_new - d_old`` 不得超过 ``max_lane_delta``（只挡「被挤出车道」的方向）。

    **降级链（保证不出现「无解」）**：实测 ±3 m / 7 采样下，有 9/269 个泊位的
    全部候选都被过滤掉（其中 7 个是 spot_waypoint 本身就压在约束上，任何偏移
    都压）。若两道过滤后没有候选，按序降级重试：
    ``两道全开 -> 只开起点过滤 -> 全关``，并在 ``stat["grid_relaxed"]`` 里记下
    实际用到的档位（"none" / "lane" / "all"），由 build() 写进 meta 便于追溯。

    3. ``start_overlap_tol`` 给定时（且 ``n_obstacles > 0``）：先算每个采样点的
       「起点压障碍面积」，只保留 ``area <= min(area) + start_overlap_tol`` 的
       采样点。见 :data:`START_OVERLAP_TOL_M2` 的 docstring——这是相对守卫，
       不会砍空、也不会逼出更深的解。

    ``stats`` 若给定，累计各过滤原因的命中次数（用于构建报告的可见性）。
    ``start_overlap_out`` 若给定，回写守卫统计（a_min / 保留数 / 选中面积）。
    """
    psi = float(start[2])
    off_list = list(offsets) if offsets else [0.0]
    off_list_full = list(off_list)
    off_list_g: Optional[List[float]] = None
    guard_active = False

    # ---- 守卫 3：起点压障碍面积（相对守卫，只砍「明显更压墙」的采样点）----
    if start_overlap_tol is not None and n_obstacles > 0 and len(off_list) > 1:
        areas = []
        for off in off_list:
            sx = float(start[0]) + float(off) * math.cos(psi)
            sy = float(start[1]) + float(off) * math.sin(psi)
            rect = start_pose_rect(sx, sy, psi, body, P)
            areas.append(start_obstacle_overlap(rect, tree, polys, n_obstacles))
        a_min = min(areas)
        kept = [(o, a) for o, a in zip(off_list, areas) if a <= a_min + float(start_overlap_tol)]
        if stats is not None:
            stats["n_start_overlap_guard_dropped"] = (
                stats.get("n_start_overlap_guard_dropped", 0) + len(off_list) - len(kept))
        off_list_g = [o for o, _ in kept]
        guard_active = True
        if start_overlap_out is not None:
            start_overlap_out["a_min_m2"] = float(a_min)
            start_overlap_out["n_kept"] = int(len(kept))
    elif start_overlap_out is not None:
        start_overlap_out["a_min_m2"] = float("nan")
        start_overlap_out["n_kept"] = int(len(off_list))

    d0 = lane_distance_m((start[0], start[1]), lane_segs) if lane_segs is not None else None
    use_lane = bool(lane_segs is not None and max_lane_delta >= 0.0
                    and d0 is not None and math.isfinite(d0))

    def _pass(use_start_filter: bool, use_lane_filter: bool, offsets_in=None):
        """跑一轮网格；返回 (key, stat, path, off) 或 None。"""
        best = None
        for off in (off_list if offsets_in is None else offsets_in):
            s_off = (float(start[0]) + float(off) * math.cos(psi),
                     float(start[1]) + float(off) * math.sin(psi),
                     psi)
            if use_start_filter:
                rect = start_pose_rect(s_off[0], s_off[1], psi, body, P)
                if start_pose_blocked(rect, tree, polys, exclude):
                    if stats is not None:
                        stats["n_grid_filter_start_blocked"] = stats.get("n_grid_filter_start_blocked", 0) + 1
                    continue
            if use_lane_filter:
                d_new = lane_distance_m((s_off[0], s_off[1]), lane_segs)
                if math.isfinite(d_new) and (d_new - d0) > float(max_lane_delta):
                    if stats is not None:
                        stats["n_grid_filter_lane_delta"] = stats.get("n_grid_filter_lane_delta", 0) + 1
                    continue
            stat, path = plan_one(s_off, goal, tree, polys, body, state, P, k_max,
                                  max_candidates=max_candidates,
                                  step_size=step_size, exclude=exclude,
                                  free_band=free_band)
            if stat is None or path is None:
                continue
            key = stat.get("_score_key")
            if key is None:      # 兜底（理论上不会发生）
                key = (-max(stat["worst_depth_m"] - float(free_band), 0.0),
                       -int(stat["n_hit_poses"]),
                       min(stat["min_clearance_m"], 0.6),
                       -stat["path_length_m"])
            if best is None or key > best[0]:
                best = (key, stat, path, float(off))
        return best

    relaxed = "none"
    best = _pass(require_clear_start, use_lane,
                 off_list_g if guard_active else None)
    # ---- 深度安全阀 -------------------------------------------------------
    # 守卫把采样点收窄到「起点压墙最轻」的那一档后，个别泊位可能只剩运动学上
    # 很差的起点（实测 L706 被逼到 0.3116 m 侵入）。所以再跑一次**不守卫**的
    # 全网格作对照：若守卫后的解比自由搜索的解深超过 start_overlap_valve，
    # 说明这个泊位就是必须借道压墙，放弃守卫、取自由搜索的解。
    # 这与「起点硬过滤」的本质区别：硬过滤是无条件丢弃（L713 0.0733->0.0894），
    # 这里是有条件的、带对照的取舍，且只在白名单泊位上发生。
    if guard_active and best is not None and start_overlap_valve is not None:
        best_free = _pass(require_clear_start, use_lane, off_list_full)
        if best_free is not None:
            d_g = float(best[1]["worst_depth_m"])
            d_f = float(best_free[1]["worst_depth_m"])
            if d_g > d_f + float(start_overlap_valve):
                if stats is not None:
                    stats["n_start_overlap_guard_valve"] = (
                        stats.get("n_start_overlap_guard_valve", 0) + 1)
                if start_overlap_out is not None:
                    start_overlap_out["valve_fired"] = 1
                    start_overlap_out["depth_guarded"] = d_g
                    start_overlap_out["depth_free"] = d_f
                best = best_free
    if best is None and use_lane:
        relaxed = "lane"        # 只保留「起点不压约束」
        if stats is not None:
            stats["n_grid_relaxed_lane"] = stats.get("n_grid_relaxed_lane", 0) + 1
        best = _pass(require_clear_start, False,
                     off_list_g if guard_active else None)
    if best is None and require_clear_start:
        relaxed = "all"         # 全关：与 v1 行为一致，但不至于丢泊位
        if stats is not None:
            stats["n_grid_relaxed_all"] = stats.get("n_grid_relaxed_all", 0) + 1
        best = _pass(False, False, off_list_g if guard_active else None)
    if best is None:
        return None, None, None
    _, stat, path, off = best
    stat.pop("_score_key", None)
    stat["grid_relaxed"] = relaxed
    if n_obstacles > 0:
        sx = float(start[0]) + float(off) * math.cos(psi)
        sy = float(start[1]) + float(off) * math.sin(psi)
        rect = start_pose_rect(sx, sy, psi, body, P)
        stat["start_obstacle_overlap_m2"] = float(
            start_obstacle_overlap(rect, tree, polys, n_obstacles))
    return stat, path, off


def to_prediction(path, v_nominal: float, P):
    """Reeds-Shepp Path -> VehiclePrediction（绝对坐标，v 带符号）。"""
    VehiclePrediction = P["VehiclePrediction"]
    xs = np.asarray(path.x, dtype=float)
    ys = np.asarray(path.y, dtype=float)
    psis = np.unwrap(np.asarray(path.yaw, dtype=float))
    dirs = np.asarray(path.directions, dtype=float)
    ds = np.concatenate([[0.0], np.hypot(np.diff(xs), np.diff(ys))])
    t = np.cumsum(ds) / float(v_nominal)
    res = VehiclePrediction()
    res.t = t
    res.x = xs
    res.y = ys
    res.psi = psis
    res.v = dirs * float(v_nominal)
    res.u_a = np.zeros_like(t)
    res.u_steer = np.zeros_like(t)
    return res


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def build(map_dir: str, out_path: str, report_path: Optional[str],
          min_width: float = OBSTACLE_MIN_WIDTH_M,
          tight_tol: float = TIGHT_DEPTH_TOL_M,
          verbose: bool = True,
          inflate: float = SPOT_FOOTPRINT_INFLATE_M,
          start_offsets: Sequence[float] = START_OFFSETS_M,
          free_band: float = FREE_BAND_M,
          require_clear_start: bool = GRID_REQUIRE_CLEAR_START,
          max_lane_delta: float = GRID_MAX_LANE_DELTA_M,
          start_overlap_spot_ids: Optional[Sequence[str]] = None,
          start_overlap_tol: float = START_OVERLAP_TOL_M2,
          start_overlap_valve: float = START_OVERLAP_VALVE_DEPTH_M) -> dict:
    P = _import_parksim()
    r_min, k_max = kinematic_limits()
    body = P["VehicleBody"]()
    cfg = P["VehicleConfig"]()
    state = P["VehicleState"]()
    if verbose:
        print("[0] vehicle l=%.2f w=%.2f wb=%.2f | R_min=%.4f m, k_max=%.6f 1/m"
              % (body.l, body.w, body.wb, r_min, k_max))
        print("    obstacle tier: width_m >= %.2f | tight depth tol = %.2f m"
              % (min_width, tight_tol))

    spots_pickle = os.path.join(map_dir, "spots_data.pickle")
    graph_pickle = os.path.join(map_dir, "waypoints_graph.pickle")
    obstacles_json = os.path.join(map_dir, "obstacles.json")
    for p in (spots_pickle, graph_pickle, obstacles_json):
        if not os.path.exists(p):
            raise FileNotFoundError("missing input: %s" % p)

    with open(spots_pickle, "rb") as f:
        sd = pickle.load(f)
    with open(graph_pickle, "rb") as f:
        gd = pickle.load(f)
    graph = gd["graph"]

    centers = np.asarray(sd["parking_spaces"], dtype=float)
    n_spots = len(centers)
    wps = np.asarray(sd.get("spot_waypoints"), dtype=float)
    headings = np.asarray(sd.get("spot_headings"), dtype=float)
    spot_ids = list(sd.get("spot_ids") or [str(i) for i in range(n_spots)])
    if wps is None or len(wps) != n_spots:
        raise ValueError("spots_data.pickle 缺少 spot_waypoints（本脚本需要逐泊位车道接入点）")
    if headings is None or len(headings) != n_spots:
        raise ValueError("spots_data.pickle 缺少 spot_headings（本脚本需要泊入朝向）")

    polys, metas = load_obstacles(obstacles_json, min_width, P)
    n_obstacles = len(polys)

    # ---- 追加「别的泊位」作为约束多边形 -------------------------------
    # layout_rotated.json 的 spots 顺序与 spots_data.pickle 一致（index 对齐），
    # quad 已是 rotated 米制，与绝对坐标轨迹同帧，可直接进 tree/polys。
    # 约束多边形 = quad ∪（按 spot_headings 停放的车辆矩形外扩 0.15 m），
    # 见 build_spot_footprints 的 docstring（quad 不等于真实停放 footprint）。
    layout_path = os.path.join(map_dir, SPOT_LAYOUT_JSON)
    quads, index_by_id, through_map = load_spot_quads(layout_path, n_spots, P)
    footprints, n_park_rect = build_spot_footprints(
        quads, centers, headings, n_spots, body, P, inflate=inflate)
    quad_poly_index: Dict[int, int] = {}
    for i in range(n_spots):
        fp = footprints[i]
        if fp is None:
            continue
        quad_poly_index[i] = len(polys)
        polys.append(fp)
        metas.append({"id": str(spot_ids[i]) if i < len(spot_ids) else "SPOT#%d" % i,
                      "type": "spot_footprint",
                      "width_m": None, "length_m": None, "center_rot": None})
    n_quad = len(quad_poly_index)

    # 逐泊位放行集合 = 自身泊位（终点就落在自家 footprint 内，否则必然误判）
    #               + 串联借道泊位（access.through_spot_ids，访问本泊位必经）
    spot_exclude: Dict[int, frozenset] = {}
    n_through_exempt = 0
    for i in range(n_spots):
        ex = set()
        ji = quad_poly_index.get(i)
        if ji is not None:
            ex.add(ji)
        for tid in through_map.get(i, []):
            j = index_by_id.get(tid)
            k = quad_poly_index.get(j) if j is not None else None
            if k is not None:
                ex.add(k)
        if len(ex) > 1:
            n_through_exempt += 1
        spot_exclude[i] = frozenset(ex)

    # 车道中心线线段（用于网格候选「不得被挤出车道」的硬过滤）
    lane_segs = lane_segments_from_graph(graph)

    # 「起点压障碍面积守卫」的白名单：spot_id -> spot_index
    guard_idx = set()
    unknown_ids = []
    for sid in (start_overlap_spot_ids or []):
        s = str(sid).strip()
        if not s:
            continue
        if s in spot_ids:
            guard_idx.add(int(spot_ids.index(s)))
        else:
            unknown_ids.append(s)
    if unknown_ids and verbose:
        print("    [WARN] --start-overlap-spot-ids 里有地图上不存在的泊位号: %s"
              % ",".join(unknown_ids))

    tree = P["STRtree"](polys)
    if verbose:
        print("    spots=%d  obstacles(used)=%d %s"
              % (n_spots, n_obstacles,
                 dict(Counter(m["type"] for m in metas[:n_obstacles]))))
        print("    spot footprint constraints appended=%d (quad u parked-rect, "
              "inflate=%.2f m, parked-rect used on %d spots)  | total constraints=%d"
              % (n_quad, inflate, n_park_rect, len(polys)))
        print("    through-spot exemptions applied on %d spots" % n_through_exempt)
        print("    start-offset grid: %s (n=%d)" % (list(start_offsets), len(start_offsets)))
        print("    grid filters: require_clear_start=%s, max_lane_delta=%.2f m, "
              "free_band=%.3f m | lane segments=%d"
              % (require_clear_start, max_lane_delta, free_band, len(lane_segs)))
        print("    start-overlap guard: %d spots %s (tol=%.3f m^2)"
              % (len(guard_idx), sorted(spot_ids[i] for i in guard_idx), start_overlap_tol))

    t_start = time.time()
    v_nominal = 1.0
    table: Dict[int, dict] = {}
    rows: List[dict] = []
    grid_stats: Dict[str, int] = {"n_grid_filter_start_blocked": 0,
                                  "n_grid_filter_lane_delta": 0,
                                  "n_grid_relaxed_lane": 0,
                                  "n_grid_relaxed_all": 0,
                                  "n_start_overlap_guard_dropped": 0}
    start_overlap_rows: Dict[int, dict] = {}
    for i in range(n_spots):
        wp = wps[i]
        psi_c = cruise_heading_at(graph, wp)
        start = (float(wp[0]), float(wp[1]), float(psi_c))
        goal = (float(centers[i][0]), float(centers[i][1]), float(headings[i]))
        guard_out: Dict[str, float] = {}
        stat, path, chosen_off = plan_one_grid(
            start, goal, tree, polys, body, state, P, k_max,
            exclude=spot_exclude[i], offsets=start_offsets,
            lane_segs=lane_segs, require_clear_start=require_clear_start,
            # 白名单泊位**关掉车道增量过滤**：实测这些泊位的最优点恰是被车道
            # 过滤丢掉的那几个（L706 的 +3.0 m 起点压墙 0.000 m²、侵入 0.0204 m
            # 被 laneΔ=+0.680 砍掉；M891 的 −3.0/−4.0 同理）。且这些泊位本身
            # 距车道图就很远（L706 d0=10.65 m），车道增量在这个尺度上没有意义。
            max_lane_delta=(-1.0 if i in guard_idx else max_lane_delta),
            stats=grid_stats,
            free_band=free_band, n_obstacles=n_obstacles,
            start_overlap_tol=(start_overlap_tol if i in guard_idx else None),
            start_overlap_valve=(start_overlap_valve if i in guard_idx else None),
            start_overlap_out=guard_out)
        if i in guard_idx:
            start_overlap_rows[int(i)] = {
                "spot_id": spot_ids[i],
                "a_min_m2": round(float(guard_out.get("a_min_m2", float("nan"))), 6),
                "n_offsets_kept": int(guard_out.get("n_kept", 0)),
                "start_offset_m": (float(chosen_off) if chosen_off is not None else None),
                "start_obstacle_overlap_m2": round(
                    float(stat.get("start_obstacle_overlap_m2", 0.0)), 6) if stat else None,
            }
        if stat is None or path is None:
            rows.append({"spot_index": i, "spot_id": spot_ids[i],
                         "generated": False, "reason": "calc_paths returned no path"})
            continue
        # 实际使用的起点（= 车道接入点沿航向平移 chosen_off 后的位姿）
        start_used = (float(start[0]) + float(chosen_off) * math.cos(float(start[2])),
                      float(start[1]) + float(chosen_off) * math.sin(float(start[2])),
                      float(start[2]))
        # tight 现在**纯统计标记**：不再作为「接受碰撞解」的捷径——
        # 上面 plan_one 的侵入深度优先排序已保证选到的是全局侵入最小的解。
        tight = bool(stat["collides"] and stat["worst_depth_m"] <= tight_tol)
        # 退化解：没有任何零侵入候选，当前解仍带有残余侵入。
        # 用户决策：允许落地退化解（不阻断交付），但**必须逐条显式告警**，
        # 不允许再出现「211 条带残余侵入、最深 0.35 m 而日志一个字都没有」。
        degraded = bool(stat["collides"] and stat["worst_depth_m"] > 0.0)
        victim = ""
        if degraded:
            victim = describe_victim(stat.get("worst_idx"), metas, spot_ids)
            print("[DEGRADED] spot #%d %s: 残余侵入 depth=%.4f m <- %s "
                  "(clearance=%.4f m, hit_poses=%d, start_offset=%+.1f m)"
                  % (i, spot_ids[i], float(stat["worst_depth_m"]), victim,
                     float(stat["min_clearance_m"]), int(stat["n_hit_poses"]),
                     float(chosen_off)))
        pred = to_prediction(path, v_nominal, P)
        table[int(i)] = {
            "x": np.asarray(pred.x, dtype=float),
            "y": np.asarray(pred.y, dtype=float),
            "psi": np.asarray(pred.psi, dtype=float),
            "v": np.asarray(pred.v, dtype=float),
            "t": np.asarray(pred.t, dtype=float),
            "u_a": np.asarray(pred.u_a, dtype=float),
            "u_steer": np.asarray(pred.u_steer, dtype=float),
            "meta": {
                "spot_id": spot_ids[i],
                "start": [start_used[0], start_used[1], start_used[2]],
                "start_offset_m": float(chosen_off),
                "start_obstacle_overlap_m2": float(
                    stat.get("start_obstacle_overlap_m2", 0.0)),
                "grid_relaxed": str(stat.get("grid_relaxed", "none")),
                "goal": [goal[0], goal[1], goal[2]],
                "clearance": float(stat["min_clearance_m"]),
                "tight": tight,
                "degraded": degraded,
                "depth": float(stat["worst_depth_m"]),
                "collides": bool(stat["collides"]),
                "victim": victim,
                "max_curvature": float(stat["max_curvature"]),
                "end_pos_err_m": float(stat["end_pos_err_m"]),
                "end_yaw_err_deg": float(stat["end_yaw_err_deg"]),
                "n_candidates": int(len(path.x)),
                "path_length_m": float(stat["path_length_m"]),
                "n_cusps": int(stat["n_cusps"]),
                "n_samples": int(stat["n_samples"]),
            },
        }
        rows.append({"spot_index": i, "spot_id": spot_ids[i], "generated": True,
                     "tight": tight, "degraded": degraded,
                     "collides": bool(stat["collides"]),
                     "depth": float(stat["worst_depth_m"]),
                     "victim": victim,
                     "start_offset_m": float(chosen_off),
                     "grid_relaxed": str(stat.get("grid_relaxed", "none")),
                     "clearance": float(stat["min_clearance_m"]),
                     "max_curvature": float(stat["max_curvature"]),
                     "end_pos_err_m": float(stat["end_pos_err_m"]),
                     "end_yaw_err_deg": float(stat["end_yaw_err_deg"]),
                     "path_length_m": float(stat["path_length_m"]),
                     "n_cusps": int(stat["n_cusps"])})
        if verbose and (i % 50 == 0 or i == n_spots - 1):
            print("    ... %d/%d (%.0fs)" % (i + 1, n_spots, time.time() - t_start))

    # ---------------- 自检 ----------------
    gen = [r for r in rows if r["generated"]]
    curv_ok = [r for r in gen if r["max_curvature"] <= k_max * 1.01]     # 1% 离散容差
    end_ok = [r for r in gen if r["end_pos_err_m"] <= 0.30 and r["end_yaw_err_deg"] <= 5.0]
    clear_ok = [r for r in gen if r["clearance"] >= CLEAR_TARGET_M]
    collide = [r for r in gen if r["collides"]]
    tight = [r for r in gen if r.get("tight")]
    hard_fail = [r for r in collide if not r.get("tight")]
    degraded_rows = [r for r in gen if r.get("degraded")]
    degraded_sorted = sorted(degraded_rows, key=lambda r: -r["depth"])

    # ---- 地图指纹：机动表与地图的绑定凭据 -----------------------------
    # 运行时（OfflineManeuver）据此比对；地图变了而机动表没重算 -> 显式告警。
    fp_files = {
        "layout_rotated.json": layout_path,
        "spots_data.pickle": spots_pickle,
        "obstacles.json": obstacles_json,
        "waypoints_graph.pickle": graph_pickle,
    }
    map_fingerprint = {}
    for name, path in fp_files.items():
        md5_hex, size = file_md5(path)
        entry = {"md5": md5_hex, "bytes": size, "lines": count_lines(path)}
        if name == "layout_rotated.json":
            with open(path, "r", encoding="utf-8") as f:
                entry["n_spots"] = len(json.load(f).get("spots") or [])
        elif name == "spots_data.pickle":
            entry["n_spots"] = int(n_spots)
        elif name == "obstacles.json":
            with open(path, "r", encoding="utf-8") as f:
                entry["n_obstacles"] = len(json.load(f).get("obstacles") or [])
        elif name == "waypoints_graph.pickle":
            entry["n_vertices"] = len(list(graph.vertices))
        map_fingerprint[name] = entry

    report = {
        "built_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "map_dir": os.path.abspath(map_dir),
        "out_path": os.path.abspath(out_path),
        "map_fingerprint": map_fingerprint,
        "params": {
            "OBSTACLE_MIN_WIDTH_M": min_width,
            "TIGHT_DEPTH_TOL_M": tight_tol,
            "MAX_CANDIDATES": MAX_CANDIDATES,
            "RS_STEP_SIZE": RS_STEP_SIZE,
            "SWEEP_STEP_M": SWEEP_STEP_M,
            "SPOT_FOOTPRINT_INFLATE_M": float(inflate),
            "START_OFFSETS_M": [float(o) for o in start_offsets],
            "FREE_BAND_M": float(free_band),
            "GRID_REQUIRE_CLEAR_START": bool(require_clear_start),
            "GRID_MAX_LANE_DELTA_M": float(max_lane_delta),
            "START_OVERLAP_SPOT_IDS": sorted(spot_ids[i] for i in guard_idx),
            "START_OVERLAP_TOL_M2": float(start_overlap_tol),
            "START_OVERLAP_VALVE_DEPTH_M": float(start_overlap_valve),
        },
        "limits": {"R_min_m": r_min, "k_max": k_max,
                   "pos_tol_m": 0.30, "yaw_tol_deg": 5.0,
                   "clear_target_m": CLEAR_TARGET_M},
        "vehicle": {"l": body.l, "w": body.w, "wb": body.wb, "v_nominal": v_nominal},
        "obstacles_used": n_obstacles,
        "obstacle_types": dict(Counter(m["type"] for m in metas[:n_obstacles])),
        "spot_quad_constraints": n_quad,
        "spot_footprint_inflate_m": float(inflate),
        "n_parked_rect_used": n_park_rect,
        "n_constraints_total": len(polys),
        "n_through_exempt_spots": n_through_exempt,
        "start_offset_usage": dict(Counter(
            r.get("start_offset_m") for r in gen if "start_offset_m" in r)),
        "grid_filter_stats": dict(grid_stats),
        "start_overlap_guard": {
            "n_spots": len(start_overlap_rows),
            "rows": [start_overlap_rows[i] for i in sorted(start_overlap_rows)],
        },
        "n_spots": n_spots,
        "n_generated": len(gen),
        "gen_success_rate": (len(gen) / n_spots) if n_spots else 0.0,
        "n_curvature_ok": len(curv_ok),
        "max_curvature": max([r["max_curvature"] for r in gen], default=0.0),
        "n_end_ok": len(end_ok),
        "end_pos_err_max_m": max([r["end_pos_err_m"] for r in gen], default=0.0),
        "end_yaw_err_max_deg": max([r["end_yaw_err_deg"] for r in gen], default=0.0),
        "n_clear_ge_target": len(clear_ok),
        "min_clearance_m": min([r["clearance"] for r in gen], default=None),
        "n_collide": len(collide),
        "n_tight_accepted": len(tight),
        "n_hard_fail": len(hard_fail),
        "hard_fail_spots": [r["spot_index"] for r in hard_fail],
        "tight_spots": [r["spot_index"] for r in tight],
        "n_degraded": len(degraded_sorted),
        "degraded_max_depth_m": max([r["depth"] for r in degraded_sorted], default=0.0),
        "degraded_worst_spot": (degraded_sorted[0]["spot_index"] if degraded_sorted else None),
        "degraded_spots": [
            {"spot_index": r["spot_index"], "spot_id": r["spot_id"],
             "depth_m": round(float(r["depth"]), 6),
             "victim": r.get("victim", ""),
             "start_offset_m": r.get("start_offset_m"),
             "grid_relaxed": r.get("grid_relaxed", "none")}
            for r in degraded_sorted],
        "gen_time_total_s": time.time() - t_start,
        "gen_time_mean_s": (time.time() - t_start) / max(1, len(gen)),
        "rows": rows,
    }
    # tight / hard_fail 现已退化为**纯统计字段**：侵入深度优先排序本身已保证
    # 选到全局侵入最小的解，因此这里不再用 hard_fail 判 build 失败——
    # 允许存在退化解（见 n_degraded / degraded_spots），交由上层决策是否放行。
    report["PASS"] = bool(len(gen) == n_spots
                          and len(curv_ok) == len(gen)
                          and len(end_ok) == len(gen))

    # ---- DEGRADED 汇总：**无条件**打到 stdout 并写进 report -------------
    # 用户决策：退化解允许落地（PASS 不被它阻断），但绝不能再无声。
    # 这一段必须在 --quiet 下也打印，且一定进 JSON，不能被任何分支跳过。
    degraded_lines = []
    if degraded_sorted:
        degraded_lines.append(
            "!!! DEGRADED: %d / %d 条机动带残余侵入（已落地，需人工/上层确认）"
            % (len(degraded_sorted), len(gen)))
        degraded_lines.append(
            "    最深侵入 %.4f m（阈值 tight_tol=%.2f m，depth>0 即算退化）"
            % (report["degraded_max_depth_m"], tight_tol))
        degraded_lines.append("    最深的 3 条：")
        for r in degraded_sorted[:3]:
            degraded_lines.append(
                "      spot #%d %-8s depth=%.4f m  <- %s"
                % (r["spot_index"], r["spot_id"], float(r["depth"]),
                   r.get("victim", "")))
        degraded_lines.append(
            "    完整清单见 report['degraded_spots']（%d 条），逐条日志见上方 [DEGRADED] 行"
            % len(degraded_sorted))
    else:
        degraded_lines.append("DEGRADED: 0 条（全部机动无残余侵入）")
    report["degraded_summary"] = degraded_lines
    print("\n=========== DEGRADED SUMMARY ===========")
    for line in degraded_lines:
        print(line)
    print("========================================")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    with open(out_path, "wb") as f:
        pickle.dump({"meta": {k: v for k, v in report.items() if k != "rows"},
                     "spots": table}, f)

    if report_path:
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=1, default=str)

    if verbose:
        print("\n=========== BUILD REPORT ===========")
        for k in ("n_spots", "n_generated", "gen_success_rate", "n_curvature_ok",
                  "max_curvature", "n_end_ok", "end_pos_err_max_m",
                  "end_yaw_err_max_deg", "n_clear_ge_target", "min_clearance_m",
                  "n_collide", "n_tight_accepted", "n_hard_fail",
                  "n_degraded", "degraded_max_depth_m", "degraded_worst_spot",
                  "gen_time_total_s", "gen_time_mean_s", "PASS"):
            print("  %-22s %s" % (k, report[k]))
        print("  out=%s" % out_path)
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description="per-spot 带避障泊车机动表构建（方案 D）")
    ap.add_argument("--map-dir", required=True, help="地图目录（含 spots_data.pickle / "
                                                     "waypoints_graph.pickle / obstacles.json）")
    ap.add_argument("--out", default="", help="输出 pickle（默认 <map-dir>/parking_maneuvers_per_spot.pickle）")
    ap.add_argument("--report", default="", help="自检报告 JSON 路径")
    ap.add_argument("--min-width", type=float, default=OBSTACLE_MIN_WIDTH_M,
                    help="参与碰撞检测的障碍最小宽度 (m)，默认 %.1f" % OBSTACLE_MIN_WIDTH_M)
    ap.add_argument("--tight-tol", type=float, default=TIGHT_DEPTH_TOL_M,
                    help="可接受的擦碰深度上限 (m)，默认 %.2f" % TIGHT_DEPTH_TOL_M)
    ap.add_argument("--footprint-inflate", type=float, default=SPOT_FOOTPRINT_INFLATE_M,
                    help="泊位互侵约束外扩量 (m)：约束 = quad ∪ 停放车辆矩形外扩本值，"
                         "默认 %.2f" % SPOT_FOOTPRINT_INFLATE_M)
    ap.add_argument("--start-offsets", default="",
                    help="起点网格化的沿车道方向偏移 (m)，逗号分隔；"
                         "默认 %s。传 '0' 可关闭网格化（= 旧行为）"
                         % ",".join(str(o) for o in START_OFFSETS_M))
    ap.add_argument("--free-band", type=float, default=FREE_BAND_M,
                    help="打分免罚带 (m)：侵入深度 <= 本值视为等价，改按接触位姿数排序。"
                         "默认 %.3f；传 0 恢复「只看最深侵入」的旧打分" % FREE_BAND_M)
    ap.add_argument("--max-lane-delta", type=float, default=GRID_MAX_LANE_DELTA_M,
                    help="网格起点相对 offset=0 的车道中心线距离增量上限 (m)，"
                         "默认 %.2f；传负数关闭该过滤" % GRID_MAX_LANE_DELTA_M)
    ap.add_argument("--allow-blocked-start", dest="require_clear_start",
                    action="store_false", default=GRID_REQUIRE_CLEAR_START,
                    help="关闭「起点不得压约束」的硬过滤（默认开启）")
    ap.add_argument("--start-overlap-spot-ids", default="",
                    help="对指定泊位（逗号分隔的 spot_id）启用「起点压障碍面积守卫」："
                         "只保留起点压障碍面积 <= (该泊位可达最小值 + tol) 的采样点。"
                         "用于压住新打分「接触位姿数」维度把起点往墙里推的副作用。")
    ap.add_argument("--start-overlap-tol", type=float, default=START_OVERLAP_TOL_M2,
                    help="起点压障碍面积守卫的容差 (m^2)，默认 %.3f" % START_OVERLAP_TOL_M2)
    ap.add_argument("--start-overlap-valve", type=float, default=START_OVERLAP_VALVE_DEPTH_M,
                    help="守卫的深度安全阀 (m)：守卫后的解比自由搜索的解深超过本值则放弃"
                         "守卫，默认 %.3f；传负数可关闭安全阀" % START_OVERLAP_VALVE_DEPTH_M)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.start_offsets.strip():
        raw = [t for t in args.start_offsets.replace("，", ",").split(",") if t.strip()]
        try:
            offsets = [float(t) for t in raw]
        except ValueError:
            ap.error("--start-offsets 必须是逗号分隔的数字，例如 '0,1,-1,2,-2,3,-3'")
    else:
        offsets = list(START_OFFSETS_M)

    out_path = args.out or os.path.join(args.map_dir, "parking_maneuvers_per_spot.pickle")
    guard_ids = [t.strip() for t in args.start_overlap_spot_ids.replace("，", ",").split(",")
                 if t.strip()]
    report = build(args.map_dir, out_path, args.report or None,
                   min_width=args.min_width, tight_tol=args.tight_tol,
                   verbose=not args.quiet, inflate=args.footprint_inflate,
                   start_offsets=offsets, free_band=args.free_band,
                   require_clear_start=args.require_clear_start,
                   max_lane_delta=args.max_lane_delta,
                   start_overlap_spot_ids=guard_ids,
                   start_overlap_tol=args.start_overlap_tol,
                   start_overlap_valve=args.start_overlap_valve)
    return 0 if report["PASS"] else 1


if __name__ == "__main__":
    sys.exit(main())
