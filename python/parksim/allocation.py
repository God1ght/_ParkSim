#!/usr/bin/env python3
"""
ParkSim 泊位分配器（可插拔）

背景：原先入库/出库车辆的泊位选择是单一实现——「空位集合均匀随机」：
    chosen_spot = np.random.choice(empty_spots)
本模块将这一决策点抽象为可插拔的 SpotAllocator，通过 simulator.yaml 的
    allocation_method: random | nearest_entrance | graph_cost | balanced_rows | manual
选择实现。默认 random 与原有行为完全一致（消耗同一随机流）。

接口：
    allocator = make_allocator(name, context)
    spot_index = allocator.choose(kind, occupied, candidates)
        kind       : 'entering' | 'exiting'
        occupied   : 可索引的占用序列（list / np.ndarray，长度 = 车位数）
        candidates : 本次可选车位池（由 simulator_node 按语义构造）
                     entering = 当前空位（i > 0）
                     exiting  = 当前有虚拟停放车且未被在位车辆认领的车位
        返回       : 选中的车位索引（int，正数）

context 支持的键：
    parking_spaces : (N,2) 车位中心坐标（必须；距离/排类方法使用）
    entrance_coords: (x,y) 入口坐标；缺省用 DEFAULT_ENTRANCE_COORDS
    graph          : WaypointsGraph 对象（graph_cost 用）
    graph_loader   : 无参函数，返回 (graph, entrance_coords)（graph_cost 用，惰性加载）
    manual_spots   : 预设车位序列（manual 用）
    manual_fallback: manual 用尽后的回退方法名（默认 'random'）

设计说明：
- 入库与出库共用同一套选择策略，但候选池不同：入库从空位池选目标车位；
  出库从「有虚拟停放车」的车位池选出发车位（由仿真端构造）。kind 参数用于区分池语义。
- 除 random 外，各方法在给定占用状态下是确定性的（平手取较小车位索引），便于对照与复现。

串联泊位（tandem）互斥占用：
    部分场地存在「内层泊位」——入库/出库机动必须穿过「前排泊位」的框线
    （JTH B1：L729-1 穿 L729、L730-1 穿 L730、L734-1 穿 L734，侵入深度 0.6014 m，
    即整车 100% 落在前排框内）。碰撞检测拦不住这种情况：已停放的车辆不在机动表的
    约束集内（约束集只有静态障碍物多边形 + 泊位 quad）。因此只能在**分配层**互斥：

        A 与 B 配对（A 的轨迹穿过 B）⇔ A、B 不可同时被占用 / 分配、认领、封锁

    本模块提供纯逻辑实现（无 ROS / 无 IO 依赖，可单测）：
        load_through_map(...)  ->  双向索引映射 {inner: [outer], outer: [inner]}
        ThroughSpotLock        ->  从占用表实时推导锁定状态
        pick_random_occupancy  ->  初始随机占用时联动排除配对泊位
    锁状态**不单独存储**，而是每次从占用表实时推导，因此「离场释放 = 占用位清零」
    会自动解锁配对泊位，不存在遗漏释放的代码路径。
"""

import heapq
import json
import os
import pickle

import numpy as np

# 入口坐标（与 priorFiles/waypoints_graph.pickle 的 entrance_coords、vehicle.yaml 一致）
DEFAULT_ENTRANCE_COORDS = (14.38, 76.21)


class SpotAllocator(object):
    """分配器基类"""

    name = "base"

    def __init__(self, context=None):
        context = dict(context or {})
        parking_spaces = context.get("parking_spaces")
        self.parking_spaces = None if parking_spaces is None else np.asarray(parking_spaces, dtype=float)
        self.entrance_coords = np.asarray(
            context.get("entrance_coords", DEFAULT_ENTRANCE_COORDS), dtype=float)

    # ------------------------------------------------------------------
    def choose(self, kind, occupied, candidates):
        """从候选池 candidates 中返回选中的车位索引；子类实现"""
        raise NotImplementedError

    # ------------------------------------------------------------------ 工具
    def _check_free(self, candidates):
        candidates = list(candidates)
        if not candidates:
            raise ValueError("[allocation] no candidate spots available")
        return candidates

    def _require_spaces(self):
        if self.parking_spaces is None:
            raise ValueError("[allocation] '%s' requires 'parking_spaces' in context" % self.name)
        return self.parking_spaces


class RandomAllocator(SpotAllocator):
    """
    空位集合均匀随机 —— 与原有实现完全一致（默认方法）。

    为保持与旧代码相同的随机数消耗节奏，此处每次选择仅调用一次 np.random.choice。
    """

    name = "random"

    def choose(self, kind, occupied, candidates):
        return int(np.random.choice(self._check_free(candidates)))


class NearestEntranceAllocator(SpotAllocator):
    """选择距入口欧氏距离最近的空位（平手取索引小者）"""

    name = "nearest_entrance"

    def __init__(self, context=None):
        super(NearestEntranceAllocator, self).__init__(context)
        spaces = self._require_spaces()
        self._entrance_dist = np.linalg.norm(spaces - self.entrance_coords, axis=1)

    def choose(self, kind, occupied, candidates):
        free = self._check_free(candidates)
        best = min(free, key=lambda i: (float(self._entrance_dist[i]), i))
        return int(best)


class GraphCostAllocator(SpotAllocator):
    """
    以「入口顶点 → 车位最近航点」的图距离（沿车道）最小为准则。

    启动时对航点图做一次单源 Dijkstra 预计算（O(V log V + E)），之后每次选择为
    O(|free|) 的查表最小。车位到图顶点的映射与规划器一致：
    graph.search(车位中心坐标) 取最近顶点。
    """

    name = "graph_cost"

    def __init__(self, context=None):
        super(GraphCostAllocator, self).__init__(context)
        spaces = self._require_spaces()
        context = dict(context or {})
        graph = context.get("graph")
        if graph is None and context.get("graph_loader") is not None:
            graph, entrance_coords = context["graph_loader"]()
            self.entrance_coords = np.asarray(entrance_coords, dtype=float)
        if graph is None:
            raise ValueError("[allocation] 'graph_cost' requires 'graph' or 'graph_loader' in context")

        self.graph = graph
        self._vertex_cost = self._dijkstra_from_entrance()
        self._spot_cost = np.full(len(spaces), np.inf)
        for spot_index, coords in enumerate(spaces):
            vertex_index = self.graph.search(coords)
            self._spot_cost[spot_index] = self._vertex_cost[vertex_index]

    def _dijkstra_from_entrance(self):
        graph = self.graph
        num_vertices = len(graph.vertices)
        index_of_vertex = {id(vertex): i for i, vertex in enumerate(graph.vertices)}

        entrance_index = graph.search(self.entrance_coords)
        cost = [float("inf")] * num_vertices
        cost[entrance_index] = 0.0
        queue = [(0.0, entrance_index)]
        while queue:
            dist, vertex_index = heapq.heappop(queue)
            if dist > cost[vertex_index]:
                continue
            vertex = graph.vertices[vertex_index]
            for child, edge in zip(vertex.children, vertex.edges):
                child_index = index_of_vertex[id(child)]
                new_dist = dist + float(edge.c)
                if new_dist < cost[child_index]:
                    cost[child_index] = new_dist
                    heapq.heappush(queue, (new_dist, child_index))
        return cost

    def choose(self, kind, occupied, candidates):
        free = self._check_free(candidates)
        best = min(free, key=lambda i: (float(self._spot_cost[i]), i))
        return int(best)


class BalancedRowsAllocator(SpotAllocator):
    """
    按停车排均衡：优先在「当前空位最多」的排内随机选一个空位。

    排的定义直接由车位坐标得到：以车位中心 y 坐标（1 位小数）聚类，本场地共 8 条
    停车排。该方法避免车位集中消耗在同一排，便于观察跨排流量。
    """

    name = "balanced_rows"

    def __init__(self, context=None):
        super(BalancedRowsAllocator, self).__init__(context)
        spaces = self._require_spaces()
        row_coords = np.round(spaces[:, 1], 1)
        self._row_values = np.unique(row_coords)
        self._row_of_spot = np.searchsorted(self._row_values, row_coords)
        self._num_rows = len(self._row_values)

    def choose(self, kind, occupied, candidates):
        free = self._check_free(candidates)
        counts = [0] * self._num_rows
        for spot_index in free:
            counts[self._row_of_spot[spot_index]] += 1
        # 空位最多的排；平手取行号较小者
        best_row = max(range(self._num_rows), key=lambda r: (counts[r], -r))
        row_free = [i for i in free if self._row_of_spot[i] == best_row]
        return int(np.random.choice(row_free))


class ManualAllocator(SpotAllocator):
    """
    按预先给定的车位序列（如场景文件）依次消费；序列耗尽或余项不可用时，
    回退到 fallback 分配器（默认 random）。
    """

    name = "manual"

    def __init__(self, context=None):
        super(ManualAllocator, self).__init__(context)
        context = dict(context or {})
        self._queue = [int(s) for s in (context.get("manual_spots") or [])]
        fallback_name = str(context.get("manual_fallback", "random")).strip().lower().replace("-", "_")
        if fallback_name == ManualAllocator.name:
            fallback_name = "random"
        self._fallback = make_allocator(fallback_name, context)
        self.fallback_count = 0

    def choose(self, kind, occupied, candidates):
        free = self._check_free(candidates)
        free_set = set(free)
        num_spots = None if self.parking_spaces is None else len(self.parking_spaces)
        while self._queue:
            spot_index = self._queue.pop(0)
            if num_spots is not None and not (0 <= spot_index < num_spots):
                continue
            if spot_index in free_set:
                return spot_index
        self.fallback_count += 1
        return int(self._fallback.choose(kind, occupied, free))


# ==========================================================================
# 串联泊位（tandem）互斥占用 —— 纯逻辑，无 ROS / 无 websocket 依赖，可离线单测
# ==========================================================================

def _log_warn(log, message):
    """可选 logger（ROS logger / logging.Logger / None）——只用 .warn/.warning。"""
    if log is None:
        return
    for name in ('warn', 'warning'):
        method = getattr(log, name, None)
        if callable(method):
            try:
                method(message)
            except Exception:  # noqa: BLE001 —— 日志失败绝不打断主流程
                pass
            return


def _resolve_spot_index(token, id_to_index):
    """把 through_spot_ids 的元素（车位 id 字符串或整数索引）解析为车位索引。

    JTH B1 的 spot_access.through_spot_ids 存的是**车位 id 字符串**（如 'L729'），
    需要先经 spot_ids 映射回索引；也兼容直接写整数索引的旧数据。解析失败返回 None。
    """
    if isinstance(token, bool) or token is None:
        return None
    if isinstance(token, int):
        return int(token)
    if isinstance(token, float):
        return int(token) if float(token).is_integer() else None
    text = str(token).strip()
    if not text:
        return None
    if text in id_to_index:
        return id_to_index[text]
    if text.lstrip('-').isdigit():
        return int(text)
    return None


def _resolve_pairs(entries, id_to_index, num_spots, source, log=None):
    """把 [(spot_index, [through ids]), ...] 解析并校验为 [(inner, outer), ...]。"""
    pairs = []
    for index, through_ids in entries:
        for token in (through_ids or []):
            other = _resolve_spot_index(token, id_to_index)
            if other is None:
                _log_warn(log, '[tandem] %s: 索引 %d 的 through id %r 无法解析，已忽略'
                          % (source, index, token))
                continue
            if other == index:
                continue
            if num_spots and not (0 <= other < num_spots):
                _log_warn(log, '[tandem] %s: 索引 %d 的 through id %r -> %d 越界（共 %d），已忽略'
                          % (source, index, token, other, num_spots))
                continue
            if num_spots and not (0 <= index < num_spots):
                _log_warn(log, '[tandem] %s: 索引 %d 越界（共 %d），已忽略'
                          % (source, index, num_spots))
                continue
            pairs.append((int(index), int(other)))
    return pairs


def _pairs_from_spots_data(path, num_spots, log=None):
    """优先数据源：spots_data.pickle 的 spot_access（生成器写入，权威）。"""
    try:
        with open(path, 'rb') as handle:
            data = pickle.load(handle)
    except Exception as exc:  # noqa: BLE001 —— 读不到即视为该地图无此数据
        _log_warn(log, '[tandem] spots_data 读取失败（%s）：%s' % (path, exc))
        return []
    if not isinstance(data, dict):
        return []
    access = data.get('spot_access')
    if not isinstance(access, (list, tuple)) or not access:
        return []
    id_to_index = {}
    for index, spot_id in enumerate(data.get('spot_ids') or []):
        if spot_id is not None:
            id_to_index[str(spot_id).strip()] = int(index)
    entries = []
    for index, item in enumerate(access):
        if not isinstance(item, dict) or not item.get('through_spot_ids'):
            continue
        entries.append((int(index), item.get('through_spot_ids')))
    return _resolve_pairs(entries, id_to_index, num_spots, 'spots_data.pickle', log)


def _pairs_from_layout(path, num_spots, log=None):
    """回退数据源：layout_rotated.json 的 spots[].access.through_spot_ids。"""
    try:
        with open(path, 'r') as handle:
            data = json.load(handle)
    except Exception as exc:  # noqa: BLE001
        _log_warn(log, '[tandem] layout 读取失败（%s）：%s' % (path, exc))
        return []
    spots = data.get('spots') if isinstance(data, dict) else None
    if not isinstance(spots, list) or not spots:
        return []
    id_to_index = {}
    for index, spot in enumerate(spots):
        if isinstance(spot, dict) and spot.get('id') is not None:
            id_to_index[str(spot['id']).strip()] = int(index)
    entries = []
    for index, spot in enumerate(spots):
        if not isinstance(spot, dict):
            continue
        access = spot.get('access') or {}
        if not isinstance(access, dict) or not access.get('through_spot_ids'):
            continue
        entries.append((int(index), access.get('through_spot_ids')))
    return _resolve_pairs(entries, id_to_index, num_spots, 'layout', log)


def load_through_map(spots_data_path='', layout_path='', num_spots=0, log=None):
    """读取地图的串联泊位（tandem）通行关系，返回双向索引映射。

    参数：
        spots_data_path : spots_data.pickle 路径（含 spot_access / spot_ids）
        layout_path     : layout_rotated.json 路径（含 spots[].access）；仅在前者
                          读不到时使用
        num_spots       : 车位总数，用于越界校验（0 = 不校验）
        log             : 可选 logger

    返回：
        dict[int, list[int]] —— 双向（对称）映射 {inner: [outer], outer: [inner]}。
        任何一步读不到（文件缺失 / 无 spot_access 键 / 无 through 关系）都返回 {}，
        调用方据此保持既有行为不变（DJI 等无串联泊位的地图走这条路）。
    """
    pairs = []
    if spots_data_path and os.path.isfile(spots_data_path):
        pairs = _pairs_from_spots_data(spots_data_path, int(num_spots or 0), log)
    if not pairs and layout_path and os.path.isfile(layout_path):
        pairs = _pairs_from_layout(layout_path, int(num_spots or 0), log)

    through_map = {}
    for inner, outer in pairs:
        through_map.setdefault(inner, set()).add(outer)
        through_map.setdefault(outer, set()).add(inner)
    return dict((key, sorted(value)) for key, value in through_map.items())


class ThroughSpotLock(object):
    """串联泊位互斥占用锁（状态从占用表实时推导，不额外存储锁定标记）。

    语义：若泊位 A 的入库/出库机动必须穿过泊位 B（配对关系），则 A 与 B 不得同时
    被占用。判定：

        locked(A) ⇔ ∃ B ∈ partners(A)：B 已占用 / 已被认领（入库在途）/ 已被封锁

    由于不维护独立锁表，「离场释放（占用位清零）」即等于解锁配对泊位，
    不存在「忘了解锁」的代码路径（包括看门狗代发释放的异常路径）。
    """

    def __init__(self, through_map=None, num_spots=0):
        self.num_spots = int(num_spots or 0)
        pairs = {}
        for key, partners in (through_map or {}).items():
            try:
                index = int(key)
            except (TypeError, ValueError):
                continue
            if not self._valid(index):
                continue
            resolved = []
            for partner in (partners or []):
                try:
                    other = int(partner)
                except (TypeError, ValueError):
                    continue
                if other != index and self._valid(other):
                    resolved.append(other)
            if resolved:
                pairs[index] = sorted(set(resolved))
        # 对称化（防御：数据源可能只给了单向关系）
        for index, partners in list(pairs.items()):
            for other in partners:
                if other not in pairs:
                    pairs[other] = []
                if index not in pairs[other]:
                    pairs[other].append(index)
        self.through_map = dict((key, sorted(set(value))) for key, value in pairs.items())

    # ------------------------------------------------------------------ 基础
    def _valid(self, index):
        return (0 <= index < self.num_spots) if self.num_spots else index >= 0

    @property
    def enabled(self):
        """该地图是否存在串联泊位互斥约束（False = 完全不改变既有行为）"""
        return bool(self.through_map)

    def partners(self, spot_index):
        """返回该泊位的串联配对泊位索引列表（无配对则返回 []）"""
        try:
            index = int(spot_index)
        except (TypeError, ValueError):
            return []
        return self.through_map.get(index, [])

    def pairs(self):
        """去重后的无向配对列表 [(a, b), ...]（a < b），供日志/诊断使用"""
        seen = set()
        out = []
        for index in sorted(self.through_map):
            for other in self.through_map[index]:
                key = (min(index, other), max(index, other))
                if key not in seen:
                    seen.add(key)
                    out.append(key)
        return out

    def summary(self):
        """人类可读摘要（日志用）"""
        if not self.through_map:
            return 'none'
        return ', '.join('%d<->%d' % pair for pair in self.pairs())

    # ------------------------------------------------------------------ 判定
    def is_locked(self, spot_index, occupied, unavailable=None, claimed=None):
        """该泊位当前是否因串联互斥而不可用。

        occupied    : 可索引的占用序列（list / np.ndarray，长度 = 车位数）
        unavailable : 永久封锁的泊位索引集合（可选）
        claimed     : 已被认领但尚未抵达/释放的泊位索引集合（可选；入库在途必须
                      计入，否则会出现「A 已认领未抵达 → B 仍被分配 → A 抵达时穿车」）
        """
        partners = self.partners(spot_index)
        if not partners:
            return False
        unavailable = unavailable or ()
        claimed = claimed or ()
        num = len(occupied)
        for other in partners:
            if not (0 <= other < num):
                continue
            if occupied[other]:
                return True
            if other in unavailable:
                return True
            if other in claimed:
                return True
        return False

    def filter_available(self, candidates, occupied, unavailable=None, claimed=None):
        """从候选池中剔除被串联互斥锁定的泊位（保持原有顺序）。"""
        return [int(spot) for spot in candidates
                if not self.is_locked(spot, occupied, unavailable, claimed)]

    def locked_spots(self, occupied, unavailable=None, claimed=None, candidates=None):
        """返回当前被串联锁锁定的泊位索引列表（诊断/日志用）。

        注意：配对关系中只要一方被占用，另一方的 is_locked 即为 True（无论它自己
        是否被占用），因此本方法会**成对**返回。若要查「实际违规」（双方同时占用），
        请用 conflicts()。

        candidates 为空时扫描全部车位；否则只检查给定候选池。
        """
        pool = range(len(occupied)) if candidates is None else candidates
        return [int(spot) for spot in pool
                if self.is_locked(spot, occupied, unavailable, claimed)]

    def conflicts(self, occupied, unavailable=None, claimed=None):
        """返回**实际违规**的泊位：自身已占用 且 串联配对泊位也被占用。

        这是互斥约束真正要排除的状态（会生成穿车轨迹）。正常应恒为空列表。
        """
        out = []
        for spot in range(len(occupied)):
            if not occupied[spot]:
                continue
            if self.is_locked(spot, occupied, unavailable, claimed):
                out.append(int(spot))
        return out


def pick_random_occupancy(pool, count, seed, through_lock=None):
    """初始随机占用：从 pool 中无放回抽取 count 个占用泊位。

    给 through_lock（且 enabled）时，每选中一个泊位就把它的串联配对泊位从剩余池中
    移除，保证「占了前排 → 内层不可用（反之亦然）」；被联动排除的数量一并返回。

    through_lock 为空 / 未启用时，**完全沿用原实现** —— 一次
    np.random.choice(len(pool), size=count, replace=False)，随机流逐位一致，
    DJI 等无串联泊位的地图不受任何影响。

    返回 (chosen: list[int], excluded_by_tandem: int)
    """
    pool = [int(i) for i in (pool or [])]
    count = max(0, min(int(count or 0), len(pool)))
    rng = np.random.default_rng(int(seed))
    if count == 0:
        return [], 0
    if through_lock is None or not getattr(through_lock, 'enabled', False):
        picked = rng.choice(len(pool), size=count, replace=False)
        return [pool[int(i)] for i in picked], 0

    remaining = list(pool)
    chosen = []
    excluded = 0
    while remaining and len(chosen) < count:
        position = int(rng.integers(0, len(remaining)))
        spot = remaining[position]
        remaining[position] = remaining[-1]
        remaining.pop()
        chosen.append(spot)
        partners = set(through_lock.partners(spot))
        if partners:
            before = len(remaining)
            remaining = [item for item in remaining if item not in partners]
            excluded += before - len(remaining)
    return chosen, excluded


_ALLOCATORS = {
    RandomAllocator.name: RandomAllocator,
    NearestEntranceAllocator.name: NearestEntranceAllocator,
    GraphCostAllocator.name: GraphCostAllocator,
    BalancedRowsAllocator.name: BalancedRowsAllocator,
    ManualAllocator.name: ManualAllocator,
}


def available_allocators():
    """返回已注册的分配方法名列表"""
    return sorted(_ALLOCATORS.keys())


def make_allocator(name, context=None):
    """工厂：按名称实例化分配器（名称大小写、连字符不敏感）"""
    key = str(name or "random").strip().lower().replace("-", "_")
    if key not in _ALLOCATORS:
        raise ValueError(
            "Unknown allocation method '%s'. Available: %s" % (name, ", ".join(available_allocators())))
    return _ALLOCATORS[key](context)
