from typing import Dict, List, Set, Tuple
from matplotlib.pyplot import hist
import numpy as np
from pathlib import Path
import pickle
import time
import array
from collections import deque

from parksim.path_planner.offline_maneuver import OfflineManeuver

from parksim.agents.abstract_agent import AbstractAgent
from parksim.controller.stanley_controller import StanleyController

from parksim.pytypes import VehiclePrediction, VehicleState
from parksim.route_planner.a_star import AStarGraph, AStarPlanner, make_route_planner
from parksim.route_planner.graph import Vertex, WaypointsGraph
from parksim.route_planner.ref_path_generator import make_ref_path_generator
from parksim.path_planner.maneuver_providers import make_maneuver_provider
from parksim.utils.get_corners import get_vehicle_corners
from parksim.utils.interpolation import interpolate_states_inputs
from parksim.vehicle_types import VehicleBody, VehicleConfig, VehicleInfo, VehicleTask


class RuleBasedStanleyVehicle(AbstractAgent):
    def __init__(self, vehicle_id: int, vehicle_body: VehicleBody, vehicle_config: VehicleConfig, controller: StanleyController = StanleyController(), motion_predictor: StanleyController = StanleyController(), inst_centric_generator = None, intent_predictor = None):
        self.vehicle_id = vehicle_id

        # State and Reference Waypoints
        self.state: VehicleState = VehicleState() # state
        self.info: VehicleInfo = VehicleInfo() # Info
        self.disp_text: str = str(self.vehicle_id)
        
        self.state_hist = deque(maxlen=500) # State history（有界，防长仿真内存无限增长）

        self.x_ref = [] # x coordinates for waypoints
        self.y_ref = [] # y coordinates for waypoints
        self.yaw_ref = [] # yaws for waypoints
        self.v_ref = 0 # target speed

        self.task_profile: List[VehicleTask] = []
        self.task_history: List[VehicleTask] = []
        self.current_task: str = None

        self.idle_duration = None
        self.idle_start_time = None

        # Dimensions
        self.vehicle_body = vehicle_body

        self.vehicle_config = vehicle_config

        # Controller and predictor
        self.controller = controller
        self.motion_predictor = motion_predictor
        self.intent_predictor = intent_predictor # cnnV2 
        self.inst_centric_generator = inst_centric_generator

        self.target_idx = 0
        
        # parking stuff
        self.graph: WaypointsGraph = None
        self.entrance_vertex: int = None

        # ---- 多出入口 / schema v2 新字段（jth_b1 新图；旧 pickle 无 → None，行为不变）----
        self.portals = None
        self.entrance_portal_ids = None
        self.exit_portal_ids = None
        self.turns = None
        self.density = None
        self.spot_entry_portals = None
        self.spot_targets = None
        self.active_exit_target = None   # 当前激活出口目标坐标（离场 CRUISE 终点）
        self.active_spot_target = None   # 当前 (泊位,入口) 目标顶点坐标（入场 CRUISE 终点）

        self.occupancy = None
        self.parking_spaces = None
        self.north_spot_idx_ranges: List[Tuple[int, int]] = None
        self.spot_y_offset: float = None

        self.spot_index = None
        self.should_overshoot = False # overshooting or undershooting the spot?
        self.park_start_coords = None

        self.offline_maneuver: OfflineManeuver = None
        # 逐泊位绝对轨迹表路径（方案 D；None = 沿用 legacy 16 组合相对表）
        self.per_spot_maneuver_path = None
        self.overshoot_ranges: Dict[str, List[Tuple[int]]] = None

        # Planning profile（可插拔规划组件；默认 = 现状）
        # route_planner: astar | dijkstra | via; ref_path_generator: spline | linear;
        # maneuver_provider: offline | online_rs
        self.route_planner_name: str = 'astar'
        self.ref_path_generator_name: str = 'spline'
        self.maneuver_provider_name: str = 'offline'
        self.route_planner = None
        self.ref_path_generator = None
        self.maneuver_provider = None

        self.parking_start_time = float('inf') # inf means haven't start parking or unparking. Anything above 0 is parking

        # 提前（巡航规划阶段）预计算的泊车机动：
        #   1) 供参考路径可视化拼接（终点落在泊位）；
        #   2) 在线几何生成规划提前完成、提前纳入周围障碍。
        self.pending_maneuver = None

        self.parking_maneuver = None
        self.parking_step = 0
        
        # unparking stuff
        self.unparking_maneuver = None
        self.unparking_step = -1
        
        # braking stuff
        self.is_braking = False # are we braking?
        self._pre_brake_target_speed = 0 # speed to restore when unbraking
        self.priority = 0 # priority for going after braking
        self.waiting_for: int = 0 # vehicle waiting for before we go. We start indexing vehicles from 1, so 0 means no vehicle
        self.waiting_for_unparker = False # need special handling for waiting for unparker

        # ---- P1 时空窗冲突检测开关（可回退） ----
        # True = 冲突判定带时间戳（空间近但时间错开不再误判）；False = 回退到纯几何走廊旧逻辑
        self.USE_SPACETIME_WINDOW = True
        # ---- P2 ETA 让行仲裁开关（可回退） ----
        # True = 同巡航时按「谁先到冲突区」（ETA）裁决；False = 沿用几何领先者判据
        self.USE_ETA_PRIORITY = True
        # ---- P3 死锁 ETA 结算开关（可回退） ----
        # True = 互等死锁按「谁先到冲突区（ETA）」结算 + watchdog；False = 沿用更小编号
        self.USE_ETA_DEADLOCK = True
        self._wait_iters = 0  # 制动等待节拍计数（watchdog 用）
        # ---- P4 停车放行时序化开关（可回退） ----
        # True = 泊位入位/出位除瞬时几何外，再加「时间窗清空」守卫（对向车 ETA 不足则暂不放行）
        self.USE_SPARK_TIMED = True

        self.logger = deque(maxlen=100)

        # ============= Information of other vehicles ===========
        self.other_vehicles: Set(int) = set() # Other vehicle ids
        self.nearby_vehicles: Set(int) = set() # Nearby vehicles that we are interested
        self.other_state: Dict[int, VehicleState] = {}
        self.other_ref_pose: Dict[int, VehiclePrediction] = {}
        self.other_ref_v: Dict[int, float] = {}
        self.other_target_idx: Dict[int, int] = {}
        self.other_priority: Dict[int, int] = {}
        self.other_task: Dict[int, str] = {} # The current task of other vehicle
        self.other_parking_progress: Dict[int, str] = {} # Other vehicles will broadcast "PARKING" if vehicle.is_parking(), "UNPARKING" if vehicle.is_unparking(), None otherwise
        self.other_parking_start_time: Dict[int, float] = {}
        self.other_is_braking: Dict[int, str] = {}
        self.other_waiting_for: Dict[int, int] = {}
        self.other_is_all_done: Dict[int, bool] = {}

        # ============== Method to exchange information
        self.method_to_change_central_occupancy = None

        # 初始化可插拔规划组件（默认配置；load_graph/load_maneuver 后会按名称重建）
        self._refresh_planning_components()

    def set_ref_pose(self, x_ref: List[float], y_ref: List[float], yaw_ref: List[float]):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.yaw_ref = yaw_ref

        self.controller.set_ref_pose(self.x_ref, self.y_ref, self.yaw_ref)
        self.target_idx = self.controller.calc_target_index(self.state)[0] # waypoint the vehicle is targeting

    def set_ref_v(self, v_ref: float):
        self.v_ref = v_ref

    def set_target_idx(self, target_idx: int):
        self.target_idx = target_idx

    def set_vehicle_state(self, state: VehicleState = None, spot_index: int = None, heading: float = None):
        if state is not None:
            self.state = state
        elif spot_index is not None:
            assert self.parking_spaces is not None, "Please run load_parking_spaces first."

            self.spot_index = spot_index

            self.state.x.x = self.parking_spaces[spot_index][0]
            self.state.x.y = self.parking_spaces[spot_index][1]
            if heading is not None:
                self.state.e.psi = heading
            elif getattr(self, 'spot_headings', None) is not None and \
                    spot_index < len(self.spot_headings) and self.spot_headings[spot_index] is not None:
                # 与泊位方向一致（泊位朝向属性）
                self.state.e.psi = float(self.spot_headings[spot_index])
            else:
                self.state.e.psi = np.pi / 2 if np.random.rand() < 0.5 else -np.pi / 2

    def set_task_profile(self, task_profile):
        self.task_profile = task_profile

    def set_active_exit_target(self, coords):
        """设置当前激活出口目标坐标（离场 CRUISE 终点）。

        多出口地图由 vehicle_node 依据 exit_portal 传入；None 表示回退 legacy
        （与 self.exit_coords 比较）。仅改坐标来源，不新建判定分支。
        """
        self.active_exit_target = None if coords is None else [float(coords[0]), float(coords[1])]

    def set_active_spot_target(self, coords):
        """设置当前 (泊位,入口) 目标顶点坐标（入场 CRUISE 终点）。

        坐标来自建图预计算的 spot_targets[spot].by_entry[entry].coords，落在该入口
        可达弧上；None 表示回退既有 spot_waypoints 路径。
        """
        self.active_spot_target = None if coords is None else [float(coords[0]), float(coords[1])]

    def exit_targets(self):
        """返回各出口 portal 的 exit_target_rotated_m 列表（无 portals 时为空列表）。"""
        out = []
        for p in (self.portals or []):
            t = p.get('exit_target_rotated_m')
            if t is not None:
                out.append([float(t[0]), float(t[1])])
        return out

    def load_parking_spaces(self, spots_data_path: str):

        with open(spots_data_path, 'rb') as f:
            data = pickle.load(f)
            self.parking_spaces = data['parking_spaces']
            # 泊位方向属性（生成/抵达朝向应对齐泊位方向）
            self.spot_headings = data.get('spot_headings')
            # 地图级车辆限制（如 JTH 密集路网限速；DJI 无此字段则不受影响）
            self.map_vehicle_limits = data.get('vehicle_limits')
            self.overshoot_ranges = data['overshoot_ranges']
            self.north_spot_idx_ranges = data['north_spot_idx_ranges']
            self.spot_y_offset = data['spot_y_offset']
            # 每车位真实车道航向点（存在时优先于 spot_y_offset 全局偏移）
            self.spot_waypoints = data.get('spot_waypoints')

    def load_graph(self, waypoints_graph_path: str):
        """
        waypoints_graph_path: path to WaypointGraph object pickle
        entrance_coords: The (x,y) coordinates of the entrance
        """
        with open(waypoints_graph_path, 'rb') as f:
            data = pickle.load(f)
            self.graph = data['graph']
            entrance_coords = data['entrance_coords']

        # 保留入口坐标（节点侧据此设置车辆出生点与离场目标；
        # 否则会沿用 vehicle_node 中写死的旧场地坐标）
        self.entrance_coords = entrance_coords
        # 地图规则：入口朝向 + 出口坐标（出库存目标与入场生成分离）
        self.entrance_heading = data.get('entrance_heading')
        self.exit_coords = data.get('exit_coords')
        # 车道偏移模式：directed=顶点已烘焙车道线（offset≈0）；global=沿用全局 offset
        self.lane_offset_mode = data.get('lane_offset_mode', 'global')

        # Default entrance vertex
        self.entrance_vertex = self.graph.search(entrance_coords)

        # 多出入口 / schema v2 新字段（旧 pickle 无这些键 → None，运行时行为不变）
        self.portals = data.get('portals')
        self.entrance_portal_ids = data.get('entrance_portal_ids')
        self.exit_portal_ids = data.get('exit_portal_ids')
        self.turns = data.get('turns')
        self.density = data.get('density')
        self.spot_entry_portals = data.get('spot_entry_portals')
        self.spot_targets = data.get('spot_targets')
        if self.portals:
            _brief = ', '.join(
                '%s(%s:%s)' % (p.get('id'), p.get('flow'), '/'.join(p.get('role_for_b1', [])))
                for p in self.portals)
            print('[graph] portals loaded: %s | entrance_ids=%d exit_ids=%d' % (
                _brief,
                len(self.entrance_portal_ids or []),
                len(self.exit_portal_ids or [])))

        self._refresh_planning_components()

    def load_maneuver(self, offline_maneuver_path: str, per_spot_maneuver_path: str = None):
        # per_spot_maneuver_path: 逐泊位绝对轨迹表（方案 D）。
        # 为 None / 文件不存在时 OfflineManeuver.per_spot 保持 None，由 provider 显式告警后回退。
        if per_spot_maneuver_path:
            self.per_spot_maneuver_path = per_spot_maneuver_path
        self.offline_maneuver = OfflineManeuver(
            pickle_file=offline_maneuver_path,
            per_spot_file=getattr(self, 'per_spot_maneuver_path', None))
        self._refresh_planning_components()

    def set_planning_profile(self, route_planner='astar', ref_path_generator='spline', maneuver_provider='offline'):
        """
        设置可插拔规划组件（建议在 load_* 之前调用；已加载的组件会按新名称重建）
        """
        self.route_planner_name = route_planner or 'astar'
        self.ref_path_generator_name = ref_path_generator or 'spline'
        self.maneuver_provider_name = maneuver_provider or 'offline'
        self._refresh_planning_components()

    def _get_occupancy(self):
        """实时占用数组访问器（供在线机动生成纳入周围障碍）"""
        return self.occupancy

    def _refresh_planning_components(self):
        """按当前名称（重新）实例化可用组件"""
        self.ref_path_generator = make_ref_path_generator(self.ref_path_generator_name)
        if self.graph is not None:
            self.route_planner = make_route_planner(self.route_planner_name, self.graph)
        if self.offline_maneuver is not None:
            self.maneuver_provider = make_maneuver_provider(
                self.maneuver_provider_name,
                offline_maneuver=self.offline_maneuver,
                parking_spaces=self.parking_spaces,
                spot_y_offset=self.spot_y_offset,
                spot_waypoints=getattr(self, 'spot_waypoints', None),
                spot_headings=getattr(self, 'spot_headings', None),
                occupancy_getter=self._get_occupancy,
            )

    def load_intent_model(self, model_path: str):
        """
        load_graph must be called before load_intent_model.
        """
        self.intent_predictor.load_model(waypoints=self.graph, model_path=model_path)
    
    @staticmethod
    def _densify_vertices(verts, step=1.0, thresh=1.2):
        """路径顶点加密：相邻顶点 > thresh 的段按 ~step 插值。
        spline 对稀疏长段（如 6-14m 只有两端点）会病态——产生 1m+ 的大弧/横漂，
        表现为「不符合交通规则的轨迹」；线性加密不改变路径几何，仅稳定样条。"""
        if not verts:
            return verts
        out = [verts[0]]
        for v in verts[1:]:
            a = out[-1].coords
            b = v.coords
            d = float(np.linalg.norm(b - a))
            if d > thresh:
                n = int(np.ceil(d / step))
                for k in range(1, n):
                    out.append(Vertex(a + (b - a) * (k / float(n))))
            out.append(v)
        return out

    def compute_ref_path(self, graph_sol: AStarGraph, offset: float = None, spot_index: int = None):
        if not offset:
            # directed 车道偏移模式：顶点已烘焙到车道线，不再叠加全局横向偏移
            if getattr(self, 'lane_offset_mode', 'global') == 'directed':
                offset = 0.0
            else:
                offset = self.vehicle_config.offset

        # 防御：去除路径中重合/近重合的连续顶点（spline 对零长段会除零产生 NaN）
        if graph_sol is not None and getattr(graph_sol, 'vertices', None):
            vs = graph_sol.vertices
            dedup = [vs[0]]
            for v in vs[1:]:
                if float(np.linalg.norm(v.coords - dedup[-1].coords)) > 0.05:
                    dedup.append(v)
            graph_sol.vertices = dedup
            if hasattr(graph_sol, 'edges') and graph_sol.edges:
                graph_sol.edges = [e for e in graph_sol.edges
                                   if e.c > 0.05]

        if spot_index is None:
            # exiting
            x_ref, y_ref, yaw_ref = self.ref_path_generator.generate(graph_sol.vertices, offset)
        else:
            # parking
            last_edge = graph_sol.edges[-1]
            pointed_right = last_edge.v2.coords[0] - last_edge.v1.coords[0] > 0

            last_x, last_y = last_edge.v2.coords

            # 该车位是否已有地图逐车位标定的车道点（spot_waypoints）。
            # 存在时路径终点即为该精确点，跳过 DJI 语义的 ±4 m 纵向调整：
            #   1) 原位移方向硬编码沿 x 轴，而 JTH 路网 13/28 条边沿 Y 轴，
            #      会产生垂直于行驶方向的横向推出；
            #   2) JTH 车位与车道点纵向已对齐（231/269 个 |along| <= 0.5 m）。
            # DJI 地图的 spot_waypoints 为 None，走 else 分支，行为不变。
            _si_wp = abs(int(spot_index))
            _has_map_waypoint = (
                getattr(self, 'spot_waypoints', None) is not None
                and _si_wp < len(self.spot_waypoints)
                and self.spot_waypoints[_si_wp] is not None)

            if _has_map_waypoint:
                self.should_overshoot = False
            else:
                if pointed_right:
                    overshoot_ranges = self.overshoot_ranges['pointed_right']
                else:
                    overshoot_ranges = self.overshoot_ranges['pointed_left']

                self.should_overshoot = any([spot_index >= r[0] and spot_index <= r[1] for r in overshoot_ranges])

                _v2 = np.asarray(last_edge.v2.coords, dtype=float)
                _v1 = np.asarray(last_edge.v1.coords, dtype=float)
                _d = _v2 - _v1
                _n = float(np.linalg.norm(_d))
                # 位移沿最后一条边的实际行驶方向（原实现硬编码沿 x 轴）
                u_dir = (_d / _n) if _n > 1e-9 else np.array([1.0, 0.0])

                if self.should_overshoot:
                    # 越过终点 4 m，为泊车机动留出空间
                    graph_sol.vertices.append(Vertex(_v2 + u_dir * 4.0))
                else:
                    new_vertex = Vertex(_v2 - u_dir * 4.0)

                    # 找到沿行驶方向已越过 new_vertex 的顶点，截断至其处
                    last_waypoint = None
                    for i, v in enumerate(reversed(graph_sol.vertices)):
                        _proj = float((np.asarray(v.coords, dtype=float) - _v2) @ u_dir)
                        if _proj < -4.0:
                            last_waypoint = -i-1
                            break

                    if last_waypoint is None:
                        # 未找到截断点（路径极短/waypoint 已邻近）：不截断，直接追加
                        graph_sol.vertices.append(new_vertex)
                    else:
                        graph_sol.vertices = graph_sol.vertices[:last_waypoint+1]
                        graph_sol.vertices.append(new_vertex)

            x_ref, y_ref, yaw_ref = self.ref_path_generator.generate(
                self._densify_vertices(graph_sol.vertices), offset)

        return x_ref, y_ref, yaw_ref

    def cruise_planning(self, task: VehicleTask):

        assert self.parking_spaces is not None, "Please run load_parking_spaces first."
        # assert self.spot_index is not None, "Please run set_spot_idx first."
        assert self.graph is not None, "Please run load_graph first."

        self.vehicle_config.v_cruise = task.v_cruise
        # 离场任务标记：CRUISE 目标坐标与出口点重合（离场车最后一段）
        self._cruise_to_exit = False
        # 出口目标优先级：active_exit_target（多出口；其坐标命中图顶点，误差≈0）
        # > legacy exit_coords。仅改比较基准，判定逻辑（<1.0 m）不变。
        _exit_ref = getattr(self, 'active_exit_target', None)
        if _exit_ref is None:
            _exit_ref = getattr(self, 'exit_coords', None)
        if task.target_coords is not None and _exit_ref is not None:
            try:
                self._cruise_to_exit = bool(np.linalg.norm(
                    np.asarray(task.target_coords, float) - np.asarray(_exit_ref, float)) < 1.0)
            except Exception:
                self._cruise_to_exit = False
        # 地图级速度限制（只降不升；JTH 限速以保障控制器跟踪，DJI 无配置不受影响）
        _lim = getattr(self, 'map_vehicle_limits', None)
        if _lim:
            if _lim.get('v_cruise') is not None:
                self.vehicle_config.v_cruise = min(
                    float(self.vehicle_config.v_cruise), float(_lim['v_cruise']))
            if _lim.get('v_end') is not None:
                self.vehicle_config.v_end = min(
                    float(self.vehicle_config.v_end), float(_lim['v_end']))

        # 新一轮巡航规划：清除上一次的预规划机动（若本次仍去车位，将在下方重建）
        self.pending_maneuver = None

        start_coords = np.array([self.state.x.x, self.state.x.y])

        start_vertex_idx = self.graph.search(start_coords)

        if task.target_spot_index is not None:
            # Going to a spot
            
            is_north_spot = any([abs(task.target_spot_index) >= r[0] and abs(
                task.target_spot_index) <= r[1] for r in self.north_spot_idx_ranges])
            _si = abs(task.target_spot_index)
            if getattr(self, 'active_spot_target', None) is not None:
                # 建图预计算的 (泊位,入口) 目标顶点（落在该入口可达弧上）。
                # 规避烘焙后 graph.search 的病态最近邻（会同对向不可达弧互相翻转）；
                # 坐标即图顶点坐标，search 命中误差 0。
                waypoint_coords = [float(self.active_spot_target[0]),
                                   float(self.active_spot_target[1])]
            elif self.spot_waypoints is not None and _si < len(self.spot_waypoints) and \
                    self.spot_waypoints[_si] is not None:
                # 每车位真实车道航向点（避免全局 offset 落在障碍上）
                waypoint_coords = [float(self.spot_waypoints[_si][0]),
                                   float(self.spot_waypoints[_si][1])]
            else:
                y_offset = -self.spot_y_offset if is_north_spot else self.spot_y_offset
                waypoint_coords = [self.parking_spaces[_si][0],
                                   self.parking_spaces[_si][1] + y_offset]

            graph_sol = self.route_planner.plan(
                self.graph.vertices[start_vertex_idx], self.graph.vertices[self.graph.search(waypoint_coords)])

            if graph_sol is None or not getattr(graph_sol, 'edges', None):
                # 零长/退化路径（起点==终点，如入口旁车位）：两点直连，避免 spline 除零
                x_ref = np.array([float(start_coords[0]), float(waypoint_coords[0])])
                y_ref = np.array([float(start_coords[1]), float(waypoint_coords[1])])
                yaw_ref = np.array([0.0, 0.0])
            else:
                x_ref, y_ref, yaw_ref = self.compute_ref_path(
                    graph_sol=graph_sol, spot_index=task.target_spot_index)

            # 提前新建泊车起始计算节点（compute_ref_path 已在顶点序列末尾追加
            # 泊车起始点）并提前计算泊车机动：使显示路径延伸到泊位，
            # 且在线几何生成在巡航阶段即完成、并纳入周围障碍物。
            self._preplan_parking_maneuver(task.target_spot_index, x_ref, y_ref, yaw_ref)

        elif task.target_coords is not None:
            # Travel to a coordinates
            graph_sol = self.route_planner.plan(
                self.graph.vertices[start_vertex_idx], self.graph.vertices[self.graph.search(task.target_coords)])
            if graph_sol is None:
                # A* 失败（起点==终点等退化）：两点直连
                x_ref = [float(start_coords[0]), float(task.target_coords[0])]
                y_ref = [float(start_coords[1]), float(task.target_coords[1])]
                yaw_ref = [0.0, 0.0]
            elif len(graph_sol.edges) == 0: # just go to a waypoint
                x_ref = [self.state.x.x, task.target_coords[0]]
                y_ref = [self.state.x.y, task.target_coords[1]]
                yaw_ref = [self.state.e.psi, self.state.e.psi]
            else: # compute a-star path
                x_ref, y_ref, yaw_ref = self.compute_ref_path(graph_sol=graph_sol, spot_index=None)

        self.set_ref_pose(x_ref, y_ref, yaw_ref)
        self.set_ref_v(0)

    def _preplan_parking_maneuver(self, spot_index, x_ref, y_ref, yaw_ref):
        """提前（巡航规划阶段）预计算泊车机动。

        以参考路径终点（即 compute_ref_path 提前新建的泊车起始计算节点）
        作为预测起始位姿；参数与 update_state_parking 一致（pointing 随机值
        提前采样固定）。仅用于可视化与提前生成/验证，不改变实际执行流程
        （执行时仍按实际位姿重新取机动）。
        """
        self.pending_maneuver = None
        if spot_index is None or not x_ref or len(x_ref) < 2:
            return
        try:
            s_idx = int(spot_index)
            spot = 'north' if any([abs(s_idx) >= r[0] and abs(s_idx) <= r[1]
                                   for r in self.north_spot_idx_ranges]) else 'south'
            psi_end = float(yaw_ref[-1])
            direction = 'west' if psi_end > np.pi / 2 or psi_end < -np.pi / 2 else 'east'
            if self.should_overshoot:
                location = 'right' if direction == 'east' else 'left'
            else:
                location = 'left' if direction == 'east' else 'right'
            pointing = 'up' if np.random.rand() < 0.5 else 'down'
            park_start = (float(x_ref[-1]) - self.vehicle_config.offset * np.sin(psi_end),
                          float(y_ref[-1]) + self.vehicle_config.offset * np.cos(psi_end))
            xy_offset = [park_start[0] - 4 if location == 'right' else park_start[0] + 4,
                         park_start[1]]
            self.pending_maneuver = self.maneuver_provider.get_maneuver(
                xy_offset, direction, location, spot, pointing,
                task_kind='parking',
                start_pose=(float(x_ref[-1]), float(y_ref[-1]), psi_end),
                spot_index=s_idx)
        except Exception as exc:
            self.pending_maneuver = None
            print('[vehicle %d] preplan parking maneuver failed: %s' % (self.vehicle_id, exc))

    def execute_next_task(self):
        if len(self.task_profile) > 0:
            task = self.task_profile.pop(0)

            self.current_task = task.name

            if task.name == "CRUISE":
                if task.target_spot_index is not None:
                    self.spot_index = task.target_spot_index
                    
                self.cruise_planning(task=task)
            elif task.name == "PARK":
                if task.target_spot_index is not None:
                    self.spot_index = task.target_spot_index
            elif task.name == "UNPARK":
                pass

                if self.task_profile[0].name == "CRUISE":
                    # Need to try the next CRUISE task for getting the direction to unpark
                    self.cruise_planning(self.task_profile[0])
                else:
                    raise ValueError("UNPARK task should be followed with a CRUISE task.")

            elif task.name == "IDLE":
                self.idle_duration = task.duration
            else:
                raise ValueError(f'Undefined task name. {task.name} is received.')

            # State will always be assigned if given todo: commented this out
            # self.state = task.state if task.state is not None else self.state

            self.task_history.append(task)
        else:
            # Finished all tasks
            self.current_task = "END"
    
    def reached_target(self):
        dist = np.linalg.norm([self.state.x.x - self.x_ref[-1], self.state.x.y - self.y_ref[-1]])
        ang = ((np.arctan2(self.y_ref[-1] - self.state.x.y, self.x_ref[-1] - self.state.x.x) - self.state.e.psi) + (2*np.pi)) % (2*np.pi)
        reached_tgt = dist < self.vehicle_config.braking_distance/2 and ang > (np.pi / 2) and ang < (3 * np.pi / 2)
        return reached_tgt

    def reached_exit(self):
        """离场任务（CRUISE 目标 = 出口点）的完成判定：路径尽头 + 接近出口（或已停稳）。

        出口不是停车位，不需要「目标在车后方」的倒车语义（reached_target 为停车入库设计）。
        修复：限速后在终点前停住、永不满足 reached_target 导致的出口滞留。

        多出口说明：_cruise_to_exit 由 cruise_planning 按 active_exit_target（优先）或
        legacy exit_coords 计算，故本函数无需第二套判据。"""
        if not getattr(self, '_cruise_to_exit', False):
            return False
        n = self.num_waypoints()
        if n == 0:
            return False
        if self.target_idx < n - self.vehicle_config.steps_to_end:
            return False   # 参考路径尚未走完
        try:
            d = float(np.linalg.norm([self.state.x.x - self.x_ref[-1],
                                      self.state.x.y - self.y_ref[-1]]))
        except Exception:
            return False
        if d < 6.0:
            return True
        v = abs(float(getattr(self.state.v, 'v', 0.0) or 0.0))
        return d < 15.0 and v < 0.5   # 已停稳在出口附近
    
    def num_waypoints(self):
        return len(self.x_ref)

    def set_method_to_change_central_occupancy(self, method):
        self.method_to_change_central_occupancy = method

    def get_central_occupancy(self, occupancy):
        """
        Get the parking occupancy
        """
        self.occupancy = occupancy

    def change_central_occupancy(self, idx, new_value):
        """
        Request to change the occupancy
        """
        method = self.method_to_change_central_occupancy
        if callable(method):
            # Call ROS service to change occupancy
            method(idx, new_value)
        else:
            method[idx] = new_value

    def _display_path_arrays(self):
        """参考路径展示拼接：行驶段 + 机动段（终点落在泊位/出口）。

        输出做统一降采样（上限 _DISP_CAP 点）：展示路径会随 info 消息传给
        其他车辆做前瞻模拟（全网订阅），未限长会拖慢全套流程（曾导致全员假死）。
        """
        _DISP_CAP = 160

        def _cap(xs, ys, psis):
            n = len(xs)
            if n <= _DISP_CAP:
                return xs, ys, psis
            stride = (n + _DISP_CAP - 1) // _DISP_CAP
            idx = list(range(0, n, stride))
            if idx[-1] != n - 1:
                idx.append(n - 1)
            return ([xs[i] for i in idx], [ys[i] for i in idx], [psis[i] for i in idx])

        try:
            if self.current_task == "PARK" and self.parking_maneuver is not None:
                step = int(self.parking_step)
                return _cap(list(self.parking_maneuver.x[step:]),
                            list(self.parking_maneuver.y[step:]),
                            list(self.parking_maneuver.psi[step:]))
            if self.current_task == "UNPARK" and self.unparking_maneuver is not None:
                step = int(self.unparking_step)
                # 机动数组顺序为 [过道 → 停好]，回放自后向前；
                # 展示剩余段（当前位置 → 出口侧），终点即出口方向。
                return _cap(list(self.unparking_maneuver.x[:step + 1])[::-1],
                            list(self.unparking_maneuver.y[:step + 1])[::-1],
                            list(self.unparking_maneuver.psi[:step + 1])[::-1])
            if self.x_ref and len(self.x_ref) > 1:
                xs = list(self.x_ref)
                ys = list(self.y_ref)
                psis = list(self.yaw_ref)
                if self.pending_maneuver is not None:
                    xs = xs + list(self.pending_maneuver.x)
                    ys = ys + list(self.pending_maneuver.y)
                    psis = psis + list(self.pending_maneuver.psi)
                return _cap(xs, ys, psis)
        except Exception:
            pass
        return list(self.x_ref), list(self.y_ref), list(self.yaw_ref)

    def get_info(self):
        # 可视化参考路径 = 行驶参考路径 +（预规划/进行中的）机动库路径，
        # 使展示路径终点落在目标泊位/出口上（不影响控制使用的 x_ref/y_ref）。
        disp_x, disp_y, disp_psi = self._display_path_arrays()
        self.info.ref_pose.x = array.array('d', disp_x)
        self.info.ref_pose.y = array.array('d', disp_y)
        self.info.ref_pose.psi = array.array('d', disp_psi)
        self.info.ref_v = float(self.v_ref)
        self.info.target_idx = int(self.target_idx)
        self.info.priority = int(self.priority)
        self.info.task = self.current_task

        if self.is_parking():
            self.info.parking_progress = "PARKING"
        elif self.is_unparking():
            self.info.parking_progress = "UNPARKING"
        else:
            self.info.parking_progress = ""

        self.info.is_braking = self.is_braking
        self.info.parking_start_time = self.parking_start_time
        self.info.waiting_for = self.waiting_for

        self.info.disp_text = self.disp_text
        self.info.is_all_done = self.is_all_done()

        return self.info

    def get_other_info(self, active_vehicles: Dict[int, AbstractAgent]):
        """
        The equavilence of ROS subscribers
        """
        active_ids = set([id for id in active_vehicles if id != self.vehicle_id])
        
        self.other_vehicles.update(active_ids)
        ids_to_delete = self.other_vehicles - active_ids
        for _del_id in ids_to_delete:
            self.remove_other_vehicle(_del_id)

        for id in active_vehicles:
            if id == self.vehicle_id:
                continue
            
            v = active_vehicles[id]

            self.other_state[id] = v.state
            ref_pose = VehiclePrediction()
            ref_pose.x = v.x_ref
            ref_pose.y = v.y_ref
            ref_pose.psi = v.yaw_ref
            self.other_ref_pose[id] = ref_pose
            self.other_ref_v[id] = v.v_ref
            self.other_target_idx[id] = v.target_idx
            self.other_priority[id] = v.priority

            self.other_task[id] = v.current_task
            if v.is_parking():
                self.other_parking_progress[id] = "PARKING"
            elif v.is_unparking():
                self.other_parking_progress[id] = "UNPARKING"
            else:
                self.other_parking_progress[id] = ""

            self.other_is_braking[id] = v.is_braking
            self.other_parking_start_time[id] = v.parking_start_time
            self.other_waiting_for[id] = v.waiting_for
            self.other_is_all_done[id] = v.is_all_done()

    def remove_other_vehicle(self, vehicle_id: int):
        """清除某辆已离场车辆的缓存（防长时间仿真下 other_* 字典无限累积）。"""
        self.other_vehicles.discard(vehicle_id)
        self.nearby_vehicles.discard(vehicle_id)
        for _d in (self.other_state, self.other_ref_pose, self.other_ref_v,
                   self.other_target_idx, self.other_priority, self.other_task,
                   self.other_parking_progress, self.other_is_braking,
                   self.other_parking_start_time, self.other_waiting_for,
                   self.other_is_all_done):
            _d.pop(vehicle_id, None)

    def dist_from(self, other_id: int):
        """
        Compute Euclidean distance with the other vehicle
        """
        return np.linalg.norm([self.other_state[other_id].x.x - self.state.x.x, self.other_state[other_id].x.y - self.state.x.y])

    def will_crash_with(self) -> Set[int]:
        will_crash_with = set()

        # 前瞻视界内的候选车辆（碰撞只可能在视界内发生；车辆多时全量前瞻会拖爆 CPU）
        _steps = int(self.vehicle_config.look_ahead_timesteps)
        _vref = float(self.v_ref) if self.v_ref is not None else 5.0
        _horizon = max(6.0, _steps * 0.1 * max(_vref, 2.0) + 3.0)
        _cands = []
        for id in self.nearby_vehicles:
            st = self.other_state.get(id)
            if st is None:
                continue
            _cands.append((np.hypot(st.x.x - self.state.x.x, st.x.y - self.state.x.y), id))
        _cands.sort()
        _look_ids = [id for d, id in _cands if d <= _horizon][:6]

        # create states for looking ahead
        look_ahead_state = self.state.copy()
        other_look_ahead_states = [self.other_state[id].copy() for id in _look_ids]
        # 预转 numpy（避免前瞻循环内反复 asarray 大数组）
        _x_np = np.asarray(self.x_ref, dtype=float)
        _y_np = np.asarray(self.y_ref, dtype=float)
        _yaw_np = np.asarray(self.yaw_ref, dtype=float)

        # for each time step, looking ahead
        for _ in range(self.vehicle_config.look_ahead_timesteps):
            # calculate new positions
            self.motion_predictor.set_ref_pose(_x_np, _y_np, _yaw_np)
            self.motion_predictor.set_ref_v(self.v_ref)
            self.motion_predictor.set_target_idx(self.target_idx)
            ai, di, _ = self.motion_predictor.solve(look_ahead_state, self.is_braking)
            self.motion_predictor.step(look_ahead_state, ai, di)

            for id, other_look_ahead_state in zip(_look_ids, other_look_ahead_states):
                if id not in will_crash_with: # for efficiency
                    self.motion_predictor.set_ref_pose(self.other_ref_pose[id].x, self.other_ref_pose[id].y, self.other_ref_pose[id].psi)
                    self.motion_predictor.set_ref_v(self.other_ref_v[id])
                    # ref_pose 为展示拼接路径（长度可能与对方控制器路径不同），
                    # target_idx 需钳制到路径范围内（曾因越界导致车辆崩溃）
                    _oi = int(self.other_target_idx[id])
                    _on = len(self.other_ref_pose[id].x)
                    if _on > 0:
                        _oi = max(0, min(_oi, _on - 1))
                    self.motion_predictor.set_target_idx(_oi)
                    ai, di, _ = self.motion_predictor.solve(other_look_ahead_state, self.other_is_braking[id])
                    self.motion_predictor.step(other_look_ahead_state, ai, di)


            # detect crash
            for id, other_look_ahead_state in zip(_look_ids, other_look_ahead_states):
                if id not in will_crash_with: # for efficiency
                    if self.will_collide(look_ahead_state, other_look_ahead_state, self.vehicle_body):
                        # NOTE: Here we assume all other vehicles have the same vehicle body as us
                        will_crash_with.add(id)

        return will_crash_with

    def should_go_before(self, other_id):
        """
        Determines if one car should go before another. Does it based on angles: if one vehicle has gone more past the other vehicle than the other, it should go first.
        """
        this_ang = ((np.arctan2(self.other_state[other_id].x.y - self.state.x.y, self.other_state[other_id].x.x - self.state.x.x) - self.state.e.psi) + (2*np.pi)) % (2*np.pi)
        other_ang = ((np.arctan2(self.state.x.y - self.other_state[other_id].x.y, self.state.x.x - self.other_state[other_id].x.x) - self.other_state[other_id].e.psi) + (2*np.pi)) % (2*np.pi)
        this_ang_centered = this_ang if this_ang < np.pi else this_ang - 2 * np.pi
        other_ang_centered = other_ang if other_ang < np.pi else other_ang - 2 * np.pi
        return abs(this_ang_centered) > abs(other_ang_centered)

    # ==================== item2/3: 未来参考轨迹走廊重叠 + 优先级仲裁 ====================
    def _maneuvering(self, oid=None):
        """是否处于泊位机动（PARK/UNPARK 中途）。oid=None 表示本车。"""
        if oid is None:
            return self.is_parking() or self.is_unparking()
        return bool(self.other_parking_progress.get(oid))

    def _ref_window(self, oid=None, sample_len=None):
        """沿参考轨迹（本车或邻车）从目标索引向前取一窗口点列 [[x,y],...]。少于2点返回空。"""
        if sample_len is None:
            sample_len = float(self.vehicle_config.lookahead_distance)
        if oid is None:
            xs, ys, idx = self.x_ref, self.y_ref, self.target_idx
        else:
            rp = self.other_ref_pose.get(oid)
            if rp is None or getattr(rp, 'x', None) is None:
                return []
            xs, ys, idx = rp.x, rp.y, self.other_target_idx.get(oid, 0)
        n = len(xs)
        if n < 2:
            return []
        pts = []
        prev = None
        cum = 0.0
        i = max(0, idx - 1)
        while i < n:
            x = float(xs[i]); y = float(ys[i])
            if prev is not None:
                cum += float(np.hypot(x - prev[0], y - prev[1]))
            pts.append([x, y])
            prev = [x, y]
            if cum >= sample_len:
                break
            i += 1
        return pts

    def _dense_points(self, pts, step=0.5):
        out = []
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]; x1, y1 = pts[i + 1]
            seg = max(1, int(np.hypot(x1 - x0, y1 - y0) / step))
            for j in range(seg + 1):
                t = j / seg
                out.append([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t])
        return out or ([pts[0]] if pts else [])

    def _polyline_close(self, a, b, thresh):
        sa = self._dense_points(a); sb = self._dense_points(b)
        for p in sa:
            for q in sb:
                if np.hypot(p[0] - q[0], p[1] - q[1]) < thresh:
                    return True
        return False

    def _window_with_time(self, pts, speed):
        """把轨迹点列附累计弧长与到达时间：返回 (x, y, s, t) 列表。
        窗外假设速度近似不变 t = s / v；未传有效速度时降级为纯几何（t=0）。"""
        if not pts or len(pts) < 2 or not speed or speed <= 0:
            return [(float(p[0]), float(p[1]), 0.0, 0.0) for p in pts]
        v = float(speed)
        items = []
        s = 0.0
        prev = None
        for p in pts:
            x = float(p[0]); y = float(p[1])
            if prev is not None:
                s += float(np.hypot(x - prev[0], y - prev[1]))
            items.append((x, y, s, s / v))
            prev = (x, y)
        return items

    def _spacetime_overlap(self, mine_wt, other_wt, thresh, time_tol):
        """时空窗重叠：存在空间临近点对且到达时间差在宽限内 → 真冲突。
        空间近但时间错开（|Δt| > time_tol）→ 不算冲突（两车先后经过同一走廊）。

        ★ 性能（2026-09-17）：先用轴对齐包围盒剪枝。若两窗口包围盒在 x 或 y 上的
        间隙已 > thresh，则不存在任何相距 < thresh 的点对 —— 该剪枝**不改变判定结果**，
        只把常见的「空间不相邻」情形从 O(P²) 降到 O(P)。60 车全连接（每车 ~54 个
        邻近车、crash_check_radius=15m）时这里是实测量的 CPU 热点。
        """
        if not mine_wt or not other_wt:
            return False
        mx0 = my0 = float('inf')
        mx1 = my1 = float('-inf')
        for (ax, ay, _as, _at) in mine_wt:
            if ax < mx0:
                mx0 = ax
            if ax > mx1:
                mx1 = ax
            if ay < my0:
                my0 = ay
            if ay > my1:
                my1 = ay
        ox0 = oy0 = float('inf')
        ox1 = oy1 = float('-inf')
        for (bx, by, _bs, _bt) in other_wt:
            if bx < ox0:
                ox0 = bx
            if bx > ox1:
                ox1 = bx
            if by < oy0:
                oy0 = by
            if by > oy1:
                oy1 = by
        if (mx0 - ox1) >= thresh or (ox0 - mx1) >= thresh:
            return False
        if (my0 - oy1) >= thresh or (oy0 - my1) >= thresh:
            return False
        for (ax, ay, _as, at) in mine_wt:
            for (bx, by, _bs, bt) in other_wt:
                if np.hypot(ax - bx, ay - by) < thresh:
                    if time_tol <= 0 or abs(at - bt) < time_tol:
                        return True
        return False

    def _current_speed(self):
        """本车实际速度（m/s）。取不到 / 非有限 / 非正 → 0.0。

        注意：旧实现在异常时返回 1.0，会把一个凭空的巡航速度喂给 ETA 计时
        （见 _eta_at_conflict），使静止车的"到达时间"变成有限值而非 0，
        与对方恒 0 的时间戳组合后得出"对方永远先到"的伪结论。
        """
        try:
            v = float(self.state.v.v)
        except Exception:
            return 0.0
        if not np.isfinite(v) or v <= 0.0:
            return 0.0
        return v

    def _maneuver_body_intrudes(self, oid, pad):
        """机动车辆真实车身（OBB）是否侵入本车未来窗口采样点（含 pad 外扩）。
        倒库/摆尾会让车身熔出参考走廊中心线，纯走廊时空窗会漏判 → 用车身占用补回。"""
        st = self.other_state.get(oid)
        if st is None:
            return False
        cx, cy = st.x.x, st.x.y
        cp, sn = float(np.cos(st.e.psi)), float(np.sin(st.e.psi))
        hl = 0.5 * self.vehicle_body.l + pad
        hw = 0.5 * self.vehicle_body.w + pad
        for (px, py) in self._ref_window():
            dx, dy = px - cx, py - cy
            lx = dx * cp + dy * sn   # 车体纵向分量
            ly = -dx * sn + dy * cp   # 车体横向分量
            if abs(lx) <= hl and abs(ly) <= hw:
                return True
        return False


    def ref_traj_overlap_with(self):
        """item3: 返回自身未来参考轨迹会成为冲突的邻车集合。
        机制：沿两车参考轨迹向前一窗口走廊彼此接近且到达时间重合 → 判定未来会重叠。
        巡航与泊位机动车辆一律按时空窗判定，仅机动方向放宽少许容差（易探出轨道/摆动）；
        空间不重叠或时间错开即不判冲突。真实碰撞由 will_crash_with 兜底。"""
        overlap = set()
        mine = self._ref_window()
        half = 0.5 * self.vehicle_body.w
        margin = float(self.vehicle_config.trajectory_corridor_margin)
        thresh = 2.0 * half + margin
        if self.USE_SPACETIME_WINDOW:
            _tol = max(0.6, (4.6 + margin) / max(self._current_speed(), 1.0))
            mine_wt = self._window_with_time(mine, self._current_speed())
        else:
            _tol = 0.0
            mine_wt = None
        for oid in self.nearby_vehicles:
            if oid in overlap:
                continue
            other = self._ref_window(oid)
            if not (other and len(other) >= 2 and mine and len(mine) >= 2):
                continue
            _is_mv = self._maneuvering(oid)
            # 机动车辆：真实车身侵入本车窗口（倒库/摆尾探出车身）即判冲突，补回纯中心线走廊的漏判
            if _is_mv and self._maneuver_body_intrudes(oid, half + margin):
                overlap.add(oid)
                continue
            # 机动容差：泊位机动易探出走廊/摆动，空间与时间容差各放宽少许
            _th = thresh + (0.5 if _is_mv else 0.0)
            _tt = (_tol + (0.6 if _is_mv else 0.0)) if self.USE_SPACETIME_WINDOW else 0.0
            if self.USE_SPACETIME_WINDOW:
                other_wt = self._window_with_time(other, self.other_ref_v.get(oid))
                if self._spacetime_overlap(mine_wt, other_wt, _th, _tt):
                    overlap.add(oid)
            elif self._polyline_close(mine, other, _th):
                overlap.add(oid)
        return overlap

    def _eta_at_conflict(self, oid):
        """估算本车与 oid 到达彼此冲突区的 ETA（复用时空窗的时间戳）。
        返回 (my_eta, other_eta)；ETA 不可信或找不到空间临近点对时返回 None。

        ★ 修复（2026-09-17 链式死锁根因）：
        两侧到达时间必须用「同一量纲」的速度计算才可比 —— 本车只有实际速度，
        对方只有参考速度 other_ref_v。制动中的车 ref_v ≡ 0，若据此计时，对方
        所有点的到达时间戳会塌缩为 0（被当成"已停在冲突点"）；而本车残余速度
        可能是 3.3e-17 这类次正规浮点（能通过 `speed > 0` 检查），于是时间戳
        = 距离/3.3e-17 变成天文数字。两者组合后 abs(eta[0]-eta[1]) > 0.3 成立，
        而比较 `eta[0] < eta[1]` 恒为 False ⇒ 每辆车都判"对方先到"而无限让行，
        且互等分支双方同时如此 ⇒ 永久互锁（实测 v20<->v22 100% 不解）。

        因此：任一侧速度不可用（<= 0.05 m/s：停定，或 ref_v 为 0）即视为 ETA
        无意义，返回 None，交由调用方的确定性全序（vehicle_id 全序，无环）兜底。
        车辆正常行驶时两侧速度均有效，ETA 优先级判据不受影响。
        """
        EPS_V = 0.05
        v_self = self._current_speed()
        if v_self <= EPS_V:
            return None
        v_other = self.other_ref_v.get(oid)
        if v_other is None:
            return None
        try:
            v_other = float(v_other)
        except Exception:
            return None
        if not np.isfinite(v_other) or v_other <= EPS_V:
            return None
        mine = self._ref_window()
        other = self._ref_window(oid)
        if not (mine and len(mine) >= 2 and other and len(other) >= 2):
            return None
        mw = self._window_with_time(mine, v_self)
        ow = self._window_with_time(other, v_other)
        half = 0.5 * self.vehicle_body.w
        margin = float(self.vehicle_config.trajectory_corridor_margin)
        thresh = 2.0 * half + margin
        best = None  # (my_t, other_t) 且时间差最小（最可能的真实冲突时刻）
        for (ax, ay, _as, at) in mw:
            for (bx, by, _bs, bt) in ow:
                if np.hypot(ax - bx, ay - by) < thresh:
                    if best is None or abs(at - bt) < abs(best[0] - best[1]):
                        best = (at, bt)
        return best

    def _park_timed_blocked(self):
        """P4 泊位放行时序守卫：若任一巡航对向车将在 park_clear_time 内到达本车走廊冲突区，则暂不放行。
        仅新增「阻止过早放行」的安全条件，不强制提前放行。"""
        if not self.USE_SPARK_TIMED:
            return False
        finish = float(getattr(self.vehicle_config, 'park_clear_time', 6.0))
        for oid in self.nearby_vehicles:
            if self.other_task.get(oid) in ("PARK", "UNPARK"):
                continue
            if self.has_passed(other_id=oid):
                continue  # 已在后方，不阻断
            eta = self._eta_at_conflict(oid)
            if eta is not None and eta[1] < finish:
                return True  # 对向车过早到达 → 暂缓
        return False

    def _self_goes_first(self, oid):
        """本车相对 oid 是否有优先通行权（对方需停）：对方泊位机动→我停；我泊位机动→我走；同巡航→领先者先走（P2 用 ETA）。"""
        if self._maneuvering() and not self._maneuvering(oid):
            return True
        if self._maneuvering(oid) and not self._maneuvering():
            return False
        if self.USE_ETA_PRIORITY:
            eta = self._eta_at_conflict(oid)
            if eta is not None:
                # 先到冲突区者先走；时间差极小时用旧判据平局
                if abs(eta[0] - eta[1]) > 0.3:
                    return eta[0] < eta[1]
        # 确定性兜底：ETA 不可用/退化（双方停定，速度≈0 → ETA 全为 0）时，
        # 用全局全序（更小编号先走）取代几何启发 should_go_before。
        # should_go_before 非反对称，可构成优先级环（A让B、B让C、C让A），
        # 是链式制动死锁的根源；vehicle_id 全序保证无环、必有唯一最小者先走。
        return self.vehicle_id < oid

    def _should_brake_for(self, conflicts):
        """按优先级判定：存在本车需让路的重叠车则刹车。"""
        return any((not self._self_goes_first(o)) for o in conflicts)

    def _pick_yield_target(self, conflicts):
        outs = [o for o in conflicts if not self._self_goes_first(o)]
        if not outs:
            return None
        # 优先让给泊位机动者；否则让给 v_id 更小者（确定性）
        outs.sort(key=lambda o: (0 if self._maneuvering(o) else 1, int(o)))
        return outs[0]

    def _should_unbrake(self, conflicts):
        """解除制动：不再需让行则走；互等死锁用 ETA/更小编号结算；超时可放行。"""
        if self.waiting_for == 0:
            self._wait_iters = 0
            return True
        if self.waiting_for not in conflicts:
            self._wait_iters = 0
            return True
        # ★ 幽灵目标：等待对象已离场或已完成 → 该等待永远无法被满足，立即放行。
        # （实测 stale 边：v7->v6、v16/v19->v10、v56->v41，目标车早已不在场景中，
        #  但等待一直保留，把整条下游队列钉死。）
        _wf_probe = self.waiting_for
        if _wf_probe not in self.other_state or self.other_is_all_done.get(_wf_probe):
            self._wait_iters = 0
            return True
        if not self._self_goes_first(self.waiting_for):
            wf = self.waiting_for
            if self.USE_ETA_DEADLOCK:
                self._wait_iters += 1
                # 互等死锁：先到冲突区者（ETA 更小）先走
                if self.other_waiting_for.get(wf) == self.vehicle_id:
                    self._wait_iters = 0
                    eta = self._eta_at_conflict(wf)
                    if eta is not None and abs(eta[0] - eta[1]) > 0.2:
                        return eta[0] < eta[1]
                    return self.vehicle_id < wf  # 无 ETA / 平局 → 兜底更小编号（全序，无环）
                # watchdog：等待同一目标过久 → 链式死锁（A让B、B让C、C让A）。
                # ★ 修复（2026-09-17）：旧实现的两个条件都失效，使本 watchdog 成为死代码 ——
                #   ① 进入本分支的前提是 not _self_goes_first(wf)；在静止态 ETA 退化、
                #      优先级退回 vehicle_id 全序（见 _self_goes_first 末行），该前提等价于
                #      `vehicle_id >= wf`。而旧的放行条件要求 `vehicle_id < wf`，
                #      两者互为否定 ⇒ 放行条件永不可达（实测 v8->v7：8<7 为假，永久制动）。
                #   ② `other_task` 记录的是车辆的**任务类型**，在本场景整个生命周期都是
                #      "PARK"（并非只在泊位机动期间），故 `not in ("PARK","UNPARK")`
                #      恒为 False，即使去掉 ① 也无法放行。
                # 现改为：等待超过阈值、且目标未处于泊位/出库机动（机动时长有界，值得等）
                # → 无条件放行。以 `_maneuvering` 表达「不抢占正在机动的车」这一本意。
                _max_i = int(getattr(self.vehicle_config, 'max_deadlock_iters', 120))
                if self._wait_iters > _max_i and not self._maneuvering(wf):
                    self._wait_iters = 0
                    return True
                return False
            # legacy：互等死锁时更小编号放行
            if self.other_waiting_for.get(wf) == self.vehicle_id \
                    and self.vehicle_id < wf:
                return True
            return False
        return True

    def has_passed(self, this_id: int=None, other_id: int=None, parking_dist_away=None):
        """
        If the rear corners of this vehicle have passed the front corners of the other vehicle, we say this vehicle has passed the other vehicle.
        parking_dist_away: additional check, if this_id's x-coordinate is parking_dist_away past other_id's x-coordinate 
        """
        if this_id is None or this_id == self.vehicle_id:
            this_corners = self.get_corners()
            this_state = self.state
            this_psi = this_state.e.psi
        else:
            this_corners = self.get_corners(self.other_state[this_id])
            this_state = self.other_state[this_id]
            this_psi = this_state.e.psi
        
        if other_id is None or other_id == self.vehicle_id:
            other_corners = self.get_corners()
            other_state = self.state
        else:
            other_corners = self.get_corners(self.other_state[other_id]) # NOTE: For now, assume the other vehicle has the same vehicle body
            other_state = self.other_state[other_id]

        for this_corner in [this_corners[0], this_corners[1]]:
            for other_corner in [other_corners[2], other_corners[3]]:
                ang = ((np.arctan2(other_corner[1] - this_corner[1], other_corner[0] - this_corner[0]) - this_psi) + (2*np.pi)) % (2*np.pi)
                if ang < (np.pi/2) or ang > (3*np.pi)/2:
                    return False
        if parking_dist_away is not None:
            if this_psi > np.pi / 2 and this_psi < np.pi * 3 / 2: # facing west
                if this_state.x.x - other_state.x.x > -parking_dist_away:
                    return False
            else:
                if this_state.x.x - other_state.x.x < parking_dist_away:
                    return False
        return True

    def other_within_parking_box(self, other_id):
        ang = ((np.arctan2(self.other_state[other_id].x.y - self.state.x.y, self.other_state[other_id].x.x - self.state.x.x) - self.state.e.psi) + (2*np.pi)) % (2*np.pi)
        dist = self.dist_from(other_id)
        if ang < self.vehicle_config.parking_ahead_angle or ang > 2 * np.pi - self.vehicle_config.parking_ahead_angle:
            return dist < 2*self.vehicle_config.parking_radius
        else:
            return dist < self.vehicle_config.parking_radius

    def update_state(self):
        self.controller.set_ref_pose(self.x_ref, self.y_ref, self.yaw_ref)
        self.controller.set_ref_v(self.v_ref)
        self.controller.set_target_idx(self.target_idx)
        # get acceleration toward target speed (ai), amount we should turn (di), and next target (target_idx)
        ai, di, self.target_idx = self.controller.solve(self.state, self.is_braking)
        # advance state of vehicle (updates x, y, yaw, velocity)
        self.controller.step(self.state, ai, di)
            
    def update_state_parking(self, advance=True):
        if self.parking_maneuver is None: # start parking
            # get parking parameters
            direction = 'west' if self.state.e.psi > np.pi / 2 or self.state.e.psi < -np.pi / 2 else 'east'
            if self.should_overshoot:
                location = 'right' if (direction == 'east') else 'left' # we are designed to overshoot the spot
            else:
                location = 'left' if (direction == 'east') else 'right' # we are designed to undershoot the spot
            pointing = 'up' if np.random.rand() < 0.5 else 'down' # random for diversity
            spot = 'north' if any([self.spot_index >= r[0] and self.spot_index <= r[1] for r in self.north_spot_idx_ranges]) else 'south'
            
            # get parking maneuver
            # 逐泊位绝对轨迹表（provides_absolute）**不能**再套 DJI 时代的
            # 「park_start_coords ± 4」全局平移，否则整条轨迹会被二次平移出去。
            if getattr(self.maneuver_provider, 'provides_absolute', False):
                xy_offset = [0.0, 0.0]
            else:
                xy_offset = [self.park_start_coords[0] - 4 if location == 'right' else self.park_start_coords[0] + 4,
                             self.park_start_coords[1]]
            offline_maneuver = self.maneuver_provider.get_maneuver(
                xy_offset,
                direction, location, spot, pointing,
                task_kind='parking',
                start_pose=(self.state.x.x, self.state.x.y, self.state.e.psi),
                spot_index=self.spot_index)

            time_seq = np.arange(start=offline_maneuver.t[0], stop=offline_maneuver.t[-1], step=self.controller.dt)
            
            self.parking_maneuver = interpolate_states_inputs(offline_maneuver, time_seq)

            self.parking_start_time = time.time()
            
            
        step = self.parking_step
        # set state
        self.state.x.x = self.parking_maneuver.x[step]
        self.state.x.y = self.parking_maneuver.y[step]
        self.state.e.psi = self.parking_maneuver.psi[step]
        self.state.v.v = self.parking_maneuver.v[step]

        self.state.u.u_a = self.parking_maneuver.u_a[step]
        self.state.u.u_steer = self.parking_maneuver.u_steer[step]
        
        if self.parking_step >= len(self.parking_maneuver.x) - 1:
            # done parking — 抵达泊位：同步补发占用（可视化障碍物随车辆到达出现）
            self.change_central_occupancy(self.spot_index, True)
            self.reset_parking_related()

            self.execute_next_task()
        else:
            # update parking step if advancing
            self.parking_step += 1 if advance else 0
        
    def update_state_unparking(self, advance=True):
        if self.unparking_maneuver is None: # start unparking
            # get unparking parameters
            direction = 'west' if self.x_ref[0] > self.x_ref[1] else 'east' # if first direction of travel is left, face west
            location = 'right' if np.random.rand() < 0.5 else 'left' # random for diversity
            pointing = 'up' if self.state.e.psi > 0 else 'down' # determine from state
            spot = 'north' if any([abs(self.spot_index) >= r[0] and abs(self.spot_index) <= r[1] for r in self.north_spot_idx_ranges]) else 'south'
            
            # get parking maneuver
            # 同上：逐泊位绝对轨迹表不做全局平移。
            if getattr(self.maneuver_provider, 'provides_absolute', False):
                xy_offset = [0.0, 0.0]
            else:
                xy_offset = [self.state.x.x if location == 'right' else self.state.x.x,
                             self.state.x.y - 6.25 if spot == 'north' else self.state.x.y + 6.25]
            offline_maneuver = self.maneuver_provider.get_maneuver(
                xy_offset,
                direction, location, spot, pointing,
                task_kind='unparking',
                start_pose=(self.state.x.x, self.state.x.y, self.state.e.psi),
                spot_index=self.spot_index)

            # 出库回放加速（3×）：机动采样步长放大，缩短车辆离位时间，
            # 使车位占用更早释放（原回放约 6-8s → 约 2-3s）
            time_seq = np.arange(start=offline_maneuver.t[0], stop=offline_maneuver.t[-1],
                                 step=self.controller.dt * 3)
            
            self.unparking_maneuver = interpolate_states_inputs(offline_maneuver, time_seq)
            
            # set initial unparking state
            self.unparking_step = len(self.unparking_maneuver.x) - 1

            self.parking_start_time = time.time()
            
        # get step
        step = self.unparking_step
            
        # set state
        self.state.x.x = self.unparking_maneuver.x[step]
        self.state.x.y = self.unparking_maneuver.y[step]
        self.state.e.psi = self.unparking_maneuver.psi[step]
        self.state.v.v = self.unparking_maneuver.v[step]

        self.state.u.u_a = self.unparking_maneuver.u_a[step]
        self.state.u.u_steer = self.unparking_maneuver.u_steer[step]
        
        if self.unparking_step == 0: # done unparking
            self.change_central_occupancy(self.spot_index, False)
            
            self.reset_parking_related()
            self.execute_next_task()
        else:
            # update parking step if advancing
            self.unparking_step -= 1 if advance else 0

    def reset_parking_related(self):
        # inf means haven't start parking or unparking. Anything above 0 is parking
        self.parking_start_time = float('inf')

        self.parking_maneuver = None
        self.parking_step = 0

        # unparking stuff
        self.unparking_maneuver = None
        self.unparking_step = -1

        self.park_start_coords = None
        self.pending_maneuver = None

    def get_corners(self, state: VehicleState=None, vehicle_body: VehicleBody=None):
        """
        state, vehicle_body: If computing the state of vehicle itself, leave this optional. If computing another vehicle, fill in the corresponding property
        """
        if state is None:
            state = self.state

        if vehicle_body is None:
            vehicle_body = self.vehicle_body

        return get_vehicle_corners(state=state, vehicle_body=vehicle_body)

    def brake(self):
        """
        Set target speed to 0 and turn on brakes, which make deceleration faster
        """
        self._pre_brake_target_speed = self.v_ref
        self.v_ref = 0
        self.is_braking = True

    def unbrake(self):
        """
        Set target speed back to what it was. Only does something if braking
        """
        if self.is_braking:
            self.v_ref = self._pre_brake_target_speed
            self.is_braking = False
            self.priority = 0
            self.waiting_for = 0
    
    def is_parking(self):
        """
        Are we in the middle of a parking manuever? If this is False, traffic should have the right of way, else this vehicle should have the right of way
        """
        return self.current_task == "PARK" and self.parking_maneuver is not None and self.parking_step > 0 and self.parking_step < len(self.parking_maneuver.x) - 1

    def is_unparking(self):
        return (self.current_task == "UNPARK" and self.unparking_maneuver is not None and self.unparking_step < len(self.unparking_maneuver.x) - 1 and self.unparking_step > 0) and not (self.current_task == "UNPARK" and self.unparking_step == -1)
    
    def is_all_done(self):
        """
        Have we finished the parking maneuver or we have reached the exit?
        """
        
        return self.current_task == "END"
    
    def update_nearby_vehicles(self, radius=None):
        """
        radius: (Optional) vehicles inside this radius are considered as "nearby". If left empty, we will use vehicle_config.crash_check_radius
        """
        if not radius:
            radius = self.vehicle_config.crash_check_radius

        self.nearby_vehicles.clear()

        for id in self.other_vehicles:
            if self.other_is_all_done[id]:
                continue
            
            dist = self.dist_from(id)

            if dist < radius:
                self.nearby_vehicles.add(id)
            

    def solve(self, time=None):
        """
        Having other_vehicle_objects here is just to mimic the ROS service to change values of the other vehicle. Should use this to acquire information
        """
        if self.current_task == "END":
            return

        # Firstly update the set of nearby_vehicles
        self.update_nearby_vehicles()

        # driving control

        if self.current_task in ["PARK", "UNPARK"]:
            pass
        elif self.current_task == "IDLE":
            if self.idle_start_time is None:
                self.idle_start_time = time
            
            if time - self.idle_start_time >= self.idle_duration:
                self.idle_start_time = None
                self.idle_duration = None
                self.execute_next_task()
        elif not (self.reached_target() or self.reached_exit()):
            # normal driving (haven't reached pre-parking point)

            # braking controller
            if not self.is_braking:
                # 目标速度（巡航）：仅非制动时设为目标巡航速；制动时保持 v_ref=0 以刹停
                if self.target_idx < self.num_waypoints() - self.vehicle_config.steps_to_end:
                    self.set_ref_v(self.vehicle_config.v_cruise)
                else:
                    self.set_ref_v(self.vehicle_config.v_end)

                conflicts = self.ref_traj_overlap_with() | self.will_crash_with()

                if self._should_brake_for(conflicts):
                    y = self._pick_yield_target(conflicts)
                    if y is not None:
                        self.brake()
                        self.waiting_for = y
                        self.priority = self.other_priority.get(y, 0) - 1
                        if self._maneuvering(y):
                            self.waiting_for_unparker = (self.other_task.get(y) == 'UNPARK')

            else:  # already braking / waiting
                conflicts = self.ref_traj_overlap_with() | self.will_crash_with()
                if self._should_unbrake(conflicts):
                    self.unbrake()
                    self.waiting_for = 0
                    self.waiting_for_unparker = False

        else:
            # if reached target (pre-parking point), start parking
            self.set_ref_v(0)
            self.execute_next_task()

        if self.current_task == "IDLE":
            pass
        elif self.current_task == "PARK":
            # wait for coast to be clear, then start parking
            # everyone within range should be braking or parking or unparking

            # ★ 修复（2026-09-17 PARK 门互锁）：
            # 条件 1「邻车在巡航且本车已越过它」+ 条件 4「距离 ≥ 2*parking_radius(14m)」
            # 无法覆盖一种常见情形：本车停在预泊位点、身后紧跟着一队「因本车处于 PARK
            # 而制动让行」的巡航车（实测 v12 身后 0.4–2m 处有 v26/v27/v29/v31/v35/v43）。
            # 本车不可能"越过"身后的车，距离又不足 14m，其余分支只对 UNPARK/PARK 生效
            # ⇒ all() 恒 False ⇒ 本车永不开始泊车；而身后那些车正交由本车（互等）⇒ 死锁。
            # 新增分支：仅当该邻车**正在等待本车**（other_waiting_for[id] == 本车）时，
            # 用确定性全序（更小 id 先开始泊位）打破互等。判据非对称 ⇒ 必有唯一放行者、
            # 无环；仅对「可证明互等」的邻车生效，不影响正常车流与泊位门前方的运动车辆。
            should_go = (self.parking_maneuver is not None and self.parking_step > 0) \
                or (all([
                    (self.other_task[id] not in ["UNPARK", "PARK"] and self.has_passed(other_id=id)) 
                    or (self.other_task[id] == "UNPARK" and self.other_parking_progress[id] == "")
                    or (self.other_task[id] == "PARK" and self.other_parking_start_time[id] > self.parking_start_time)
                    or self.dist_from(id) >= 2*self.vehicle_config.parking_radius
                    or (self.other_waiting_for.get(id) == self.vehicle_id and self.vehicle_id < int(id))
                    for id in self.nearby_vehicles
                    ]) and not self._park_timed_blocked())

            if self.park_start_coords is None:
                self.park_start_coords = (self.state.x.x - self.vehicle_config.offset * np.sin(self.state.e.psi), self.state.x.y + self.vehicle_config.offset * np.cos(self.state.e.psi))
            self.update_state_parking(should_go)
        elif self.current_task == "UNPARK": # wait for coast to be clear, then start unparking
            # to start unparking, everyone within range should be (normal driving and far past us) or (waiting to unpark and spawned after us)
            # always yields to parkers in the area

            unparking_nearby_vehicles = [id for id in self.nearby_vehicles if np.abs(self.other_state[id].x.y - self.y_ref[0]) < self.vehicle_config.parking_radius]
            # old version
            # unparking_nearby_vehicles = [id for id in self.nearby_vehicles if self.dist_from(id) >= 2*self.vehicle_config.parking_radius]
            should_go = (self.unparking_maneuver is not None and self.unparking_step < len(self.unparking_maneuver.x) - 1) \
                or (all([
                    (self.other_task[id] not in ["PARK", "UNPARK"] and self.has_passed(this_id=id, parking_dist_away=7)) 
                    or (self.other_task[id] == "UNPARK" and self.other_parking_start_time[id] > self.parking_start_time)
                    for id in unparking_nearby_vehicles
                    ]) and not self._park_timed_blocked())
            """
            and all([self.other_parking_progress[id] != "UNPARKING" or np.linalg.norm([self.x_ref[0] - self.other_ref_pose[id].x[0], self.y_ref[0] - self.other_ref_pose[id].y[0]]) > 10 for id in self.other_vehicles]))
            """

            self.update_state_unparking(should_go)
        else: 
            self.update_state()

        self.state_hist.append(self.state.copy())
        self.logger.append(f't = {time}: x = {self.state.x.x:.2f}, y = {self.state.x.y:.2f}')

    def predict_intent(self, vehicle_id, history):
        """
        predict the intent of the specific vehicle
        """
        img = self.inst_centric_generator.inst_centric(vehicle_id, history)
        return self.intent_predictor.predict(img, np.array([self.state.x.x, self.state.x.y]), self.state.e.psi, self.state.v.v, 1.0)

    def get_state_dict(self):
        state_dict = {}
        state_dict['center-x'] = self.state.x.x
        state_dict['center-y'] = self.state.x.y
        state_dict['heading'] = self.state.e.psi
        state_dict['corners'] = self.vehicle_body.V
        return state_dict

    def get_other_vehicles(self):
        other_states = {i : self.other_state[i] for i in self.other_state}
        return other_states
