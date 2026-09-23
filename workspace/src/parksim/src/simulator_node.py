#!/usr/bin/env python3
import rclpy

import subprocess
import signal
import numpy as np

from pathlib import Path
import glob
import time
import os

import traceback
import json

import pickle
import yaml

from dlp.dataset import Dataset
from dlp.visualizer import Visualizer as DlpVisualizer

from std_msgs.msg import Int16MultiArray, Bool, Float32, String
from parksim.msg import VehicleStateMsg
from parksim.srv import OccupancySrv
from parksim.base_node import MPClabNode, parksim_path
from parksim.allocation import (
    make_allocator, available_allocators, load_through_map, ThroughSpotLock,
    pick_random_occupancy,
)
from parksim.scenario import load_scenario, effective_mode, load_map, ScenarioError, MapError, VALID_MODES
from parksim.pytypes import VehicleState, NodeParamTemplate

class SimulatorNodeParams(NodeParamTemplate):
    """
    template that stores all parameters needed for the node as well as default values
    """
    def __init__(self):
        self.dlp_path = parksim_path('python', 'parksim', 'priorFiles', 'data', 'DJI_0012')
        self.timer_period = 0.1
        self.random_seed = 0

        self.blocked_spots = []

        # Spot allocation（可插拔泊位分配方法）
        self.allocation_method = 'random'   # random | nearest_entrance | graph_cost | balanced_rows | manual
        self.allocation_manual_spots = []   # 仅 manual：按序消费的车位列表（用尽后随机回退）

        # 场景初始化（scenario）：random | replay | custom
        self.scenario_file = ''             # '' = 默认 config/scenario.yaml；'<name>' = config/scenarios/<name>.yaml；或文件路径
        # 地图（map）：config/maps/<name>.yaml；决定 dlp 数据集与关联数据文件
        self.map = 'DJI_0012'
        # 初始化模式覆盖（'' = 用场景文件里的 mode）
        self.init_mode = ''
        # 传递给每辆车的规划组件覆盖（'' = 使用 vehicle.yaml 值）
        self.route_planner = ''
        self.ref_path_generator = ''
        self.maneuver_provider = ''
        # launch 参数覆盖（json 字符串；优先级最高，由 launch 传入）
        self.launch_overrides = ''

        self.spawn_entering = 3
        self.spawn_exiting = 3
        self.y_bound_to_resume_spawning = 70
        self.spawn_interval_mean = 5 # (s)

        # 多出入口（schema v2 有 portals）选口策略：
        #   'random' = 按 random_seed 从可行口里随机；或直接写 portal id（P1/P4 入场，P1/P2/P3 离场）
        self.entrance_choice = 'random'
        self.exit_choice = 'random'

        self.spots_data_path = ''
        self.agents_data_path = parksim_path('python', 'parksim', 'priorFiles', 'agents_data_0012.pickle')

        self.use_existing_agents = True

        self.write_log = True
        self.log_path = parksim_path('vehicle_log')

def load_waypoints_graph_data():
    """
    Load the waypoint graph and entrance coordinates (used by pluggable allocators).
    """
    with open(parksim_path('python', 'parksim', 'priorFiles', 'waypoints_graph.pickle'), 'rb') as f:
        data = pickle.load(f)
    return data['graph'], data['entrance_coords']


class SimulatorNode(MPClabNode):
    """
    Node class for simulation
    """
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('Initializing Simulator Node')
        namespace = self.get_namespace()

        param_template = SimulatorNodeParams()
        self.autodeclare_parameters(param_template, namespace)
        self.autoload_parameters(param_template, namespace)

        # ======== 场景配置与 launch 覆盖（优先级：文件 < 场景预设 < launch 参数）
        launch_overrides = {}
        if self.launch_overrides:
            try:
                launch_overrides = json.loads(self.launch_overrides)
            except Exception as exc:
                self.get_logger().error('Invalid launch_overrides JSON (%s); ignored.' % exc)
                launch_overrides = {}

        scenario_name = launch_overrides.get('scenario', '') or ''
        config_dir = parksim_path('workspace', 'src', 'parksim', 'config')
        scenario, scenario_path = load_scenario(scenario_name, config_dir)
        self.scenario = scenario
        self.scenario_path = scenario_path

        # 地图（map）：解析地图预设并覆盖数据集/数据文件路径
        map_name = launch_overrides.get('map') or self.map or 'DJI_0012'
        try:
            map_cfg = load_map(map_name, config_dir)
        except MapError as exc:
            self.get_logger().error('Map error: %s' % exc)
            raise
        if map_cfg.get('dlp_path'):
            self.dlp_path = map_cfg['dlp_path']
        if map_cfg.get('agents_data_path'):
            self.agents_data_path = map_cfg['agents_data_path']
        # 新地图（地图目录提供 layout/spots_data）：车位几何直接来自地图数据，
        # 而不依赖 DLP 数据集；可覆盖封锁位（blocked_spots）。
        self.map_spots_data_path = ''
        if map_cfg.get('layout') and map_cfg.get('spots_data_path'):
            self.map_spots_data_path = map_cfg['spots_data_path']
        if map_cfg.get('blocked_spots') is not None:
            self.blocked_spots = [int(i) for i in (map_cfg.get('blocked_spots') or [])]
        # 多出入口数据（schema v2）；DJI/legacy 无 portals → 全部置空，走 legacy 行为
        self._load_portal_data(map_cfg)
        self.map = map_name
        self.get_logger().info('Map: %s (dataset=%s)' % (map_name, self.dlp_path))

        # 初始化模式：launch 参数 > 场景文件 mode > 旧参数映射
        requested_mode = launch_overrides.get('init_mode') or self.init_mode or ''
        if requested_mode and requested_mode not in VALID_MODES:
            self.get_logger().warn('Unknown init_mode %r; 回退到场景文件 mode' % requested_mode)
            requested_mode = ''
        self.scenario_mode = requested_mode or effective_mode(scenario, self.use_existing_agents)
        self.get_logger().info(
            'Scenario: mode=%s file=%s' % (self.scenario_mode, scenario_path or '(legacy)'))

        # 分配方法与规划组件（文件 < 预设 < launch）
        for key, default in (('allocation_method', 'random'), ('route_planner', ''),
                             ('ref_path_generator', ''), ('maneuver_provider', '')):
            value = getattr(self, key) or default
            if scenario.get(key):
                value = scenario[key]
            if launch_overrides.get(key):
                value = launch_overrides[key]
            setattr(self, key, value)

        # random 模式的生成参数与随机种子
        rand = scenario.get('random') or {}
        if 'seed' in rand:
            self.random_seed = int(rand['seed'])
        np.random.seed(self.random_seed)
        if 'entering' in rand:
            self.spawn_entering = int(rand['entering'])
        if 'exiting' in rand:
            self.spawn_exiting = int(rand['exiting'])
        if 'interval_mean' in rand:
            self.spawn_interval_mean = float(rand['interval_mean'])
        if 'y_bound' in rand:
            self.y_bound_to_resume_spawning = float(rand['y_bound'])
        if (scenario.get('replay') or {}).get('agents_data_path'):
            self.agents_data_path = str(scenario['replay']['agents_data_path'])

        self.entering_spot_pool = set(int(i) for i in (rand.get('entering_spot_pool') or []))
        self.custom_schedule = []

        # Check whether there are unterminated vehicle processes
        all_nodes_names = [x[0] for x in self.get_node_names_and_namespaces()]
        print(all_nodes_names)
        if 'vehicle' in all_nodes_names:
            self.get_logger().error("Some vehicle nodes are not shut down cleanly. Please kill those processes first.")
            raise KeyboardInterrupt()

        # Clean up the log folder if needed
        if self.write_log:
            # yccc7: path changed
            # log_dir_path = str(Path.home()) + self.log_path
            log_dir_path = self.log_path
            if not os.path.exists(log_dir_path):
                os.mkdir(log_dir_path)
            log_files = glob.glob(log_dir_path+'/*.log')
            for f in log_files:
                os.remove(f)
            self.get_logger().info("Logs will be saved in %s. Old logs are cleared." % log_dir_path)

        # DLP
        home_path = str(Path.home())
        self.get_logger().info('Loading Dataset...')
        ds = Dataset()
        # ds.load(home_path + self.dlp_path)
        # yccc7: path changed
        ds.load(self.dlp_path)
        self.dlpvis = DlpVisualizer(ds)

        # Parking Spaces
        self.parking_spaces, self.occupied = self._gen_occupancy()

        # 串联泊位（tandem）互斥占用锁：数据驱动，来自地图 spot_access /
        # layout spots[].access.through_spot_ids。无 through 关系的地图（DJI 系列）
        # 得到空锁，后续所有调用均为 no-op，行为与改前完全一致。
        self.through_lock = self._build_through_lock(map_cfg, len(self.occupied))

        # 车位认领与封锁记录（出库语义 / 防重复分配）
        self.entering_claimed = set()   # 入库车辆认领的车位（停车后长期占用）
        self.exiting_claimed = set()    # 出库车辆已使用过的车位（不再作为出发源）
        self.unavailable_spots = set()  # 封锁车位（不可入库、不可出发）

        # 出库车辆占用释放保障：(Popen, vehicle_id, spot)；
        # 车辆进程若未发送释放就结束（崩溃/被杀），由看门狗代发释放，避免幽灵占用。
        self.exit_procs = []
        self._gate_closed_at = 0
        # 入库车辆抵达保障（镜像）：(Popen, vehicle_id, spot)；
        # 车辆未抵达即结束（崩溃/被杀）时释放认领，避免车位被无效锁定。
        self.enter_procs = []
        self._last_release_guard = 0.0
        _n_spots = len(self.occupied)
        for idx in self.blocked_spots:
            idx = int(idx)
            if not (0 <= idx < _n_spots):
                self.get_logger().warn(
                    'blocked_spots 索引 %d 越界（地图 %s 共 %d 个车位），已忽略' % (idx, self.map, _n_spots))
                continue
            self.occupied[idx] = True
            self._mark_unavailable(idx)

        # 随机生成模式：车位占用随机化（经验初始化保持数据集真实占用）
        if self.scenario_mode == 'random':
            rand_occ = rand.get('occupancy_random') or {}
            if rand_occ.get('enable', True):
                self.occupied = self._randomize_occupancy(self.occupied, rand, rand_occ)

        # 场景附加占用（blocked/occupied 附加项；custom 直接定义时亦生效）
        occ = scenario.get('occupancy') or rand.get('occupancy') or {}
        for idx in (occ.get('blocked') or []):
            idx = int(idx)
            if not (0 <= idx < len(self.occupied)):
                self.get_logger().warn(
                    '场景 occupancy.blocked 索引 %d 越界（共 %d 个车位），已忽略' % (idx, len(self.occupied)))
                continue
            self.occupied[idx] = True
            self._mark_unavailable(idx)
        for idx in (occ.get('occupied') or []):
            idx = int(idx)
            if not (0 <= idx < len(self.occupied)):
                self.get_logger().warn(
                    '场景 occupancy.occupied 索引 %d 越界（共 %d 个车位），已忽略' % (idx, len(self.occupied)))
                continue
            self.occupied[idx] = True

        # custom 模式：载入显式时间表
        if self.scenario_mode == 'custom':
            custom = scenario.get('custom') or {}
            custom_file = custom.get('file') or ''
            if custom_file:
                custom_path = custom_file if os.path.isabs(custom_file) else os.path.join(config_dir, custom_file)
                with open(custom_path, 'r') as f:
                    custom_data = yaml.safe_load(f) or {}
                self.custom_schedule = list(custom_data.get('vehicles') or [])
                custom_occ = custom_data.get('occupancy') or {}
                for idx in (custom_occ.get('blocked') or []):
                    idx = int(idx)
                    if not (0 <= idx < len(self.occupied)):
                        self.get_logger().warn(
                            'custom occupancy.blocked 索引 %d 越界（共 %d 个车位），已忽略' % (idx, len(self.occupied)))
                        continue
                    self.occupied[idx] = True
                    self._mark_unavailable(idx)
                for idx in (custom_occ.get('occupied') or []):
                    idx = int(idx)
                    if not (0 <= idx < len(self.occupied)):
                        self.get_logger().warn(
                            'custom occupancy.occupied 索引 %d 越界（共 %d 个车位），已忽略' % (idx, len(self.occupied)))
                        continue
                    self.occupied[idx] = True
            else:
                self.custom_schedule = list(scenario.get('vehicles') or [])
            self.custom_schedule.sort(key=lambda e: float(e['t']))
            self.get_logger().info('Custom schedule: %d vehicles' % len(self.custom_schedule))

        # 串联泊位：初始化完成后的最终自检（覆盖场景 / custom 显式指定的占用）。
        # 仅告警不修正——显式配置冲突应由使用者处理；这些泊位将被出库候选池过滤掉。
        conflicts = self.through_lock.conflicts(self.occupied, self.unavailable_spots)
        if conflicts:
            self.get_logger().warn(
                '[tandem] 初始化后存在串联冲突泊位 %s（配对双方同时占用；'
                '将不可作为出库出发位）' % conflicts)

        # Agents
        self._gen_agents()

        # Spot allocation（可插拔泊位分配器）
        self.allocator = make_allocator(
            self.allocation_method,
            {
                'parking_spaces': self.parking_spaces,
                'graph_loader': load_waypoints_graph_data,
                'manual_spots': self.allocation_manual_spots,
            },
        )
        self.get_logger().info(
            "Spot allocation method: %s (available: %s)" % (
                self.allocator.name, ", ".join(available_allocators())))

        # Spawning
        # 生成智能体，生成时间服从指数分布
        self.spawn_entering_time = list(np.random.exponential(self.spawn_interval_mean, self.spawn_entering))

        self.spawn_exiting_time = list(np.random.exponential(self.spawn_interval_mean, self.spawn_exiting))

        self.last_enter_id = None
        self.last_enter_sub = None
        self.last_enter_state = VehicleState()
        self.keep_spawn_entering = True

        self.start_time = self.get_ros_time()

        # 暂停期间冻结仿真时钟：累计已暂停时长（秒）与本次暂停起点
        self._paused_total = 0.0
        self._pause_t0 = None

        # 生成节拍基准统一改用「仿真时钟」（0 起点），
        # 否则暂停后墙钟继续走，恢复瞬间会一次性放行大量车辆。
        self.last_enter_time = 0.0
        self.last_exit_time = 0.0

        self.vehicles = []
        self.num_vehicles = 0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Publish the simulation time
        self.sim_time_pub = self.create_publisher(Float32, '/sim_time', 10)

        # Visualizer publish this status since the button is on GUI
        self.sim_status_sub = self.create_subscription(Bool, '/sim_status', self.sim_status_cb, 10)
        self.sim_is_running = True

        self.occupancy_pub = self.create_publisher(Int16MultiArray, 'occupancy', 10)
        # 即将驶出车位（出库车已生成、车辆仍在位）：供可视化区分显示
        self.departing_pub = self.create_publisher(Int16MultiArray, '/departing_spots', 10)

        # 发车队列剩余量（JSON）：供 webviz 判定本轮发车是否已全部完成。
        # 1 Hz 定时发布 + 每次从任一队列 pop 后立即发布。
        self.spawn_status_pub = self.create_publisher(String, '/webviz/spawn_status', 10)
        self.spawn_status_timer = self.create_timer(1.0, self.publish_spawn_status)

        self.occupancy_srv = self.create_service(OccupancySrv, 'occupancy', self.occupancy_srv_callback)

        self.occupancy_cli = self.create_client(OccupancySrv, '/occupancy')

    def sim_now(self):
        """仿真时钟（秒，0 起点）：暂停期间冻结，恢复后连续。"""
        now = self.get_ros_time()
        paused = self._paused_total
        if self._pause_t0 is not None:
            paused += max(0.0, now - self._pause_t0)
        return now - self.start_time - paused

    def publish_spawn_status(self):
        """上报两个 spawn 队列各自剩余未 pop 的元素个数（JSON 字符串）。

        无 spawn 队列的模式（custom / replay）下队列保持初始长度，
        照常发布实际长度即可（通常为 0）。
        """
        msg = String()
        msg.data = json.dumps({
            'entering_remaining': len(self.spawn_entering_time),
            'exiting_remaining': len(self.spawn_exiting_time),
        })
        self.spawn_status_pub.publish(msg)

    def sim_status_cb(self, msg: Bool):
        running = bool(msg.data)
        if running == bool(self.sim_is_running):
            return
        now = self.get_ros_time()
        if running:
            # 恢复：把本次暂停时长并入累计
            if self._pause_t0 is not None:
                self._paused_total += max(0.0, now - self._pause_t0)
                self._pause_t0 = None
        else:
            # 暂停：记下起点
            self._pause_t0 = now
        self.sim_is_running = running

    def occupancy_srv_callback(self, request, response):
        vehicle_id = request.vehicle_id
        idx = request.idx
        new_value = request.new_value

        if idx < 0:   # 防御：负索引在 numpy 中会回绕到错误车位
            self.get_logger().warn('[occupancy] negative idx %d from vehicle %d; using abs' % (idx, vehicle_id))
            idx = abs(idx)
        if new_value == 0 and self.occupied[idx] == 0:
            self.get_logger().warn('[occupancy] duplicate release for spot %d from vehicle %d' % (idx, vehicle_id))

        self.occupied[idx] = new_value

        response.status = True

        self.get_logger().info("Vehicle %d changed the occupancy at %d to be %r" % (vehicle_id, idx, new_value))

        # 串联泊位：锁状态由占用表实时推导，占用位清零即等于解锁配对泊位，
        # 无需（也无法遗漏）额外的解锁动作；此处只做可观测性记录。
        partners = self.through_lock.partners(idx)
        if partners:
            if new_value == 0:
                self.get_logger().info(
                    '[tandem] spot %d released; partner(s) %s unlocked' % (idx, partners))
            else:
                self.get_logger().info(
                    '[tandem] spot %d occupied; partner(s) %s locked' % (idx, partners))

        return response

    # --- 串联泊位（tandem）互斥占用 ---
    def _build_through_lock(self, map_cfg, num_spots):
        """构建串联泊位互斥锁：数据驱动，读不到 through 关系则返回空锁。

        数据源优先级：spots_data.pickle 的 spot_access → layout_rotated.json 的
        spots[].access.through_spot_ids。两者都读不到（DJI 系列）→ 空锁，
        所有判定返回 False，行为与改前完全一致。
        """
        spots_data_path = str((map_cfg or {}).get('spots_data_path') or '')
        layout_path = str((map_cfg or {}).get('layout') or '')
        through_map = load_through_map(
            spots_data_path=spots_data_path, layout_path=layout_path,
            num_spots=int(num_spots or 0), log=self.get_logger())
        lock = ThroughSpotLock(through_map, num_spots=int(num_spots or 0))
        if lock.enabled:
            self.get_logger().info(
                '[tandem] 串联泊位互斥锁已启用：%d 组 %s（入库轨迹穿过前排泊位，'
                '配对泊位互斥占用）' % (len(lock.pairs()), lock.summary()))
        else:
            self.get_logger().info(
                '[tandem] 地图 %s 未提供 through 关系；互斥锁未启用（行为不变）' % self.map)
        return lock

    def _mark_unavailable(self, idx):
        """封锁车位：其串联配对泊位同步封锁（前排被永久占用 → 内层永久不可用）。"""
        idx = int(idx)
        self.unavailable_spots.add(idx)
        for partner in self.through_lock.partners(idx):
            if 0 <= partner < len(self.occupied) and partner not in self.unavailable_spots:
                self.unavailable_spots.add(partner)
                self.get_logger().info(
                    '[tandem] 封锁位 %d 的串联配对泊位 %d 同步封锁（永久不可用）'
                    % (idx, partner))
    # --- end 串联泊位互斥占用 ---

    def _randomize_occupancy(self, occupied, rand, rand_occ):
        """随机生成：重掷车位占用（封锁位保持占用；默认与数据集占用同规模）。

        count/ratio 表示「占用总数目标」；封锁位（unavailable）始终计入并保持占用，
        其余从非封锁车位中无放回抽取，保证封锁语义在随机模式下不变。

        启用串联锁时，每抽中一个泊位就把其配对泊位从剩余池中移除（仍为无放回），
        保证初始占用也不出现「前排与内层同时停放」。
        """
        n_total = len(occupied)
        base_count = int(sum(int(o) for o in occupied))
        unavailable = set(int(i) for i in (self.unavailable_spots or []))
        pool = [i for i in range(n_total) if i not in unavailable]
        count = int(rand_occ.get('count') or 0)
        ratio = float(rand_occ.get('ratio') or 0.0)
        if count <= 0:
            if ratio > 0:
                count = int(round(ratio * n_total))
            else:
                count = base_count
                # 无数据集占用的新地图（如 jth_b1）：给合理默认（~40%），避免空场
                if count <= 0 and len(pool) > 0:
                    count = max(1, int(round(0.4 * len(pool))))
        random_count = max(0, min(count - len(unavailable), len(pool)))
        seed = int(rand.get('seed', self.random_seed)) + 917
        picked, excluded_by_tandem = pick_random_occupancy(
            pool, random_count, seed, self.through_lock)
        chosen = set(int(i) for i in picked)
        chosen |= unavailable
        new_occupied = [1 if i in chosen else 0 for i in range(n_total)]
        self.get_logger().info(
            'Random occupancy: %d/%d spots occupied (dataset: %d, blocked: %d, seed=%d)'
            % (len(chosen), n_total, base_count, len(unavailable), seed))
        if self.through_lock.enabled:
            self.get_logger().info(
                '[tandem] 随机占用联动排除 %d 个串联配对泊位（目标 %d，实占 %d）'
                % (excluded_by_tandem, random_count, len(picked)))
        # 防御性自检：初始占用不得出现「前排与内层同时停放」
        violations = self.through_lock.conflicts(new_occupied, self.unavailable_spots)
        if violations:
            self.get_logger().warn(
                '[tandem] 初始占用存在串联冲突泊位 %s（配对双方同时占用，不应发生）'
                % violations)
        return new_occupied

    def _gen_occupancy(self):
        # 新地图：地图目录自带 spots_data.pickle → 直接使用其中的车位中心
        # （与车辆侧 load_parking_spaces 完全一致，不依赖 DLP 数据集）
        if getattr(self, 'map_spots_data_path', '') and os.path.isfile(self.map_spots_data_path):
            try:
                with open(self.map_spots_data_path, 'rb') as f:
                    data = pickle.load(f)
                parking_spaces = np.asarray(data['parking_spaces'], dtype=float)
                occupied = [0] * len(parking_spaces)
                self.get_logger().info(
                    'Occupancy source: map spots_data (%d spots) — %s'
                    % (len(parking_spaces), self.map_spots_data_path))
                return parking_spaces, occupied
            except Exception as exc:
                self.get_logger().warn(
                    'map spots_data load failed (%s); fallback to DLP' % exc)

        # 默认（DJI 等）：从 DLP 数据集推导
        # get parking spaces
        arr = self.dlpvis.parking_spaces.to_numpy()
        # array of tuples of x-y coords of centers of spots
        parking_spaces = np.array([[round((arr[i][2] + arr[i][4]) / 2, 3), round((arr[i][3] + arr[i][9]) / 2, 3)] for i in range(len(arr))])

        scene = self.dlpvis.dataset.get('scene', self.dlpvis.dataset.list_scenes()[0])

        # figure out which parking spaces are occupied
        car_coords = [self.dlpvis.dataset.get('obstacle', o)['coords'] for o in scene['obstacles']]
        # 1D array of booleans — are the centers of any of the cars contained within this spot's boundaries?
        occupied = [any([c[0] > arr[i][2] and c[0] < arr[i][4] and c[1] < arr[i][3] and c[1] > arr[i][9] for c in car_coords]) for i in range(len(arr))]

        return parking_spaces, list(map(int, occupied))

    def _gen_agents(self):
        # home_path = str(Path.home())
        # with open(home_path + self.agents_data_path, 'rb') as f:
        #     self.agents_dict = pickle.load(f)

        # yccc7: path changed
        try:
            with open(self.agents_data_path, 'rb') as f:
                self.agents_dict = pickle.load(f)
        except (FileNotFoundError, OSError):
            self.get_logger().warn(
                'Agents data not found: %s（replay 将无车辆）' % self.agents_data_path)
            self.agents_dict = {}

        # 场景 replay 限制：max_agents（按 init_time 取前 N）
        max_agents = int((self.scenario.get('replay') or {}).get('max_agents', 0) or 0)
        if max_agents > 0 and len(self.agents_dict) > max_agents:
            order = sorted(self.agents_dict, key=lambda k: self.agents_dict[k]['init_time'])
            keep = set(order[:max_agents])
            self.agents_dict = {k: v for k, v in self.agents_dict.items() if k in keep}

    # --- 多出入口（schema v2 有 portals）支持；无 portals → '' 走 legacy ---
    def _load_portal_data(self, map_cfg):
        """从地图的 waypoints_graph.pickle 读多出入口数据。

        旧 pickle 无 portals → 全部置空，选口返回 ''，vehicle_node 走 legacy 门顶推导。
        """
        self.portals = []
        self.spot_entry_portals = {}
        self.spot_targets = {}
        self.entrance_portal_ids = []
        self.exit_portal_ids = []
        self._graph = None
        self._spot_exits = {}
        self._exit_reach_ready = False
        self._portal_rng = np.random.default_rng(int(getattr(self, 'random_seed', 0) or 0))
        wgp = (map_cfg or {}).get('waypoints_graph_path') or ''
        if not wgp or not os.path.isfile(wgp):
            return
        try:
            with open(wgp, 'rb') as f:
                data = pickle.load(f)
        except Exception as exc:
            self.get_logger().warn('portal 数据读取失败（%s）；走 legacy' % exc)
            return
        if not isinstance(data, dict) or not data.get('portals'):
            return
        self.portals = data['portals'] or []
        self._graph = data.get('graph')
        self.spot_entry_portals = data.get('spot_entry_portals') or {}
        self.spot_targets = data.get('spot_targets') or {}
        self.entrance_portal_ids = data.get('entrance_portal_ids') or []
        self.exit_portal_ids = data.get('exit_portal_ids') or []
        self.get_logger().info(
            '[portals] %d loaded: %s | entrance=%s exit=%s'
            % (len(self.portals),
               ', '.join('%s(%s)' % (p.get('id'), p.get('flow')) for p in self.portals),
               self.entrance_portal_ids, self.exit_portal_ids))

    def _exit_portal_choices(self):
        """离场可选的物理 portal id（由 exit_portal_ids 的图节点 id 映射回物理 id）。"""
        out = []
        for nid in (self.exit_portal_ids or []):
            pid = None
            for p in self.portals:
                if p.get('graph_node') == nid or p.get('id') == nid:
                    if 'exit' in (p.get('role_for_b1') or []):
                        pid = p.get('id')
            if pid and pid not in out:
                out.append(pid)
        return out

    def _choose_entry_portal(self, spot_index):
        """按「(泊位, 入口) 可行集」选入口；无 portals → ''（legacy）。"""
        if not getattr(self, 'portals', None):
            return ''
        choice = (getattr(self, 'entrance_choice', 'random') or 'random').strip()
        if choice and choice != 'random':
            return choice
        by_index = (self.spot_targets or {}).get('by_index') or []
        try:
            si = abs(int(spot_index))
        except (TypeError, ValueError):
            si = -1
        feasible = []
        if 0 <= si < len(by_index):
            feasible = list((by_index[si].get('by_entry') or {}).keys())
        if not feasible:
            # 回退：不区分泊位的入口集合
            feasible = [p.get('id') for p in self.portals
                        if 'entrance' in (p.get('role_for_b1') or [])]
        if not feasible:
            return ''
        if len(feasible) == 1:
            return feasible[0]
        return feasible[int(self._portal_rng.integers(0, len(feasible)))]

    def _ensure_exit_reach(self):
        """构建「每个泊位 → 真正可达的出口」映射（有向图反向 BFS，只做一次）。

        与入场侧的「入口↔泊位联合约束」对称：出口选口同样必须受泊位约束。
        实测：P2（B1→B2）对所有泊位可达，但 P1 / P3 各有 53 个泊位不可达
        （样例 164、185-268），即约 20% 泊位只能从 P2 出场。若随机在 P1/P2/P3
        里挑，这些车里约 2/3 会拿到走不通的出口，A* 直接抛
        'Path is not found'（a_star.py:108）。
        """
        if getattr(self, '_exit_reach_ready', False):
            return
        self._exit_reach_ready = True
        self._spot_exits = {}
        graph = getattr(self, '_graph', None)
        portals = getattr(self, 'portals', None)
        # 离场起点与运行时一致：parking_spaces[spot_index]（见 rule_based_stanley_vehicle
        # 的 set_vehicle_state(spot_index=...)）
        ps = getattr(self, 'parking_spaces', None)
        if graph is None or not portals or ps is None:
            self.get_logger().warn(
                '[portals] 出口可达性未构建（graph/portals/parking_spaces 缺一）；退化为不约束')
            return

        V = graph.vertices
        n = len(V)
        id_to_idx = {id(v): i for i, v in enumerate(V)}
        adj = [[] for _ in range(n)]
        for i, v in enumerate(V):
            for c, _e in zip(*v.get_children()):   # get_children() 返回 (children, edges)
                j = id_to_idx.get(id(c))
                if j is not None:
                    adj[i].append(j)
        radj = [[] for _ in range(n)]
        for u in range(n):
            for w in adj[u]:
                radj[w].append(u)

        def _can_reach(tgt):
            seen = [False] * n
            stack = [tgt]
            seen[tgt] = True
            while stack:
                u = stack.pop()
                for w in radj[u]:
                    if not seen[w]:
                        seen[w] = True
                        stack.append(w)
            return seen

        can = {}
        for p in portals:
            if 'exit' not in (p.get('role_for_b1') or []):
                continue
            t = p.get('exit_target_rotated_m')
            if not t:
                continue
            best, bd = None, None
            for i, v in enumerate(V):
                d = (float(v.coords[0]) - t[0]) ** 2 + (float(v.coords[1]) - t[1]) ** 2
                if bd is None or d < bd:
                    best, bd = i, d
            if best is not None:
                can[p.get('id')] = _can_reach(best)

        allowed = self._exit_portal_choices()
        for i in range(min(len(ps), 269)):
            try:
                sv = graph.search([float(ps[i][0]), float(ps[i][1])])
            except Exception:
                continue
            ok = [pid for pid in allowed if pid in can and sv < len(can[pid]) and can[pid][sv]]
            self._spot_exits[i] = ok or list(allowed)
        restricted = sum(1 for v in self._spot_exits.values() if len(v) < len(allowed))
        self.get_logger().info('[portals] exit reach built: %d/%d spots restricted to subset'
                               % (restricted, len(self._spot_exits)))

    def _choose_exit_portal(self, spot_index):
        """在该泊位**真能到达**的出口里选一个；无 portals → ''（legacy）。"""
        if not getattr(self, 'portals', None):
            return ''
        self._ensure_exit_reach()
        choices = self._exit_portal_choices()
        choice = (getattr(self, 'exit_choice', 'random') or 'random').strip()
        if choice and choice != 'random':
            return choice if choice in choices else (choices[0] if choices else '')
        if not choices:
            return ''
        try:
            si = abs(int(spot_index))
        except (TypeError, ValueError):
            si = -1
        feasible = (getattr(self, '_spot_exits', {}) or {}).get(si) or choices
        if not feasible:
            return ''
        if len(feasible) == 1:
            return feasible[0]
        return feasible[int(self._portal_rng.integers(0, len(feasible)))]
    # --- end 多出入口支持 ---

    def add_vehicle(self, spot_index: int, entry_portal: str = '', exit_portal: str = ''):
        self.num_vehicles += 1
        extra = []
        if entry_portal:
            extra.append('entry_portal:=%s' % entry_portal)
        if exit_portal:
            extra.append('exit_portal:=%s' % exit_portal)
        proc = subprocess.Popen(
            ["ros2", "launch", "parksim", "vehicle.launch.py", "vehicle_id:=%d" % self.num_vehicles,
             "spot_index:=%d" % spot_index, "use_existing:=0"] + self._vehicle_launch_args() + extra,
            start_new_session=True,
        )
        self.vehicles.append(proc)
        self.get_logger().info("A vehicle with id = %d is added with spot_index = %d"
                               % (self.num_vehicles, spot_index))
        return proc
    def add_existing_vehicle(self, vehicle_id: int):

        self.num_vehicles += 1

        self.vehicles.append(
            subprocess.Popen(
                ["ros2", "launch", "parksim", "vehicle.launch.py", "vehicle_id:=%d" % vehicle_id,
                 "spot_index:=%d" % 0, "use_existing:=1"] + self._vehicle_launch_args(),
                start_new_session=True,
            )
        )

        self.get_logger().info("An existing vehicle with id = %d is added" % vehicle_id)
    def _vehicle_launch_args(self):
        """地图与规划组件覆盖参数（非空才传递；供 vehicle.launch.py 解析）"""
        extra = []
        if self.map:
            extra.append('map:=%s' % self.map)
        for key in ('route_planner', 'ref_path_generator', 'maneuver_provider'):
            value = getattr(self, key, '') or ''
            if value:
                extra.append('%s:=%s' % (key, value))
        return extra

    def try_spawn_custom(self):
        """custom 模式：按时间表生成车辆"""
        current_time = self.sim_now()
        remaining = []
        for entry in self.custom_schedule:
            if float(entry.get('t', 0.0)) > current_time:
                remaining.append(entry)
                continue
            spot = int(entry.get('spot'))
            kind = str(entry.get('kind', 'entering'))
            if not (0 <= spot < len(self.occupied)):
                self.get_logger().warn('custom: spot %d out of range, skipped' % spot)
                continue
            if self.occupied[spot] or spot in self.entering_claimed:
                self.get_logger().warn('custom: spot %d already occupied/claimed, skipped' % spot)
                continue
            if self.through_lock.is_locked(
                    spot, self.occupied, self.unavailable_spots,
                    self.entering_claimed | self.exiting_claimed):
                self.get_logger().warn(
                    'custom: spot %d is tandem-locked (partner %s occupied); skipped'
                    % (spot, self.through_lock.partners(spot)))
                continue
            proc = self.add_vehicle(spot if kind == 'entering' else -spot)
            if kind == 'entering':
                # 占用由车辆抵达泊位时补发（同随机模式语义）
                self.entering_claimed.add(spot)
                self.enter_procs.append((proc, self.num_vehicles, int(spot)))
            else:
                self.occupied[spot] = True
                self.exit_procs.append((proc, self.num_vehicles, int(spot)))
        self.custom_schedule = remaining

    def shutdown_vehicles(self):
        for vehicle in self.vehicles:
            if vehicle.poll() is not None:
                continue

            try:
                vehicle.send_signal(signal.SIGINT)
            except ProcessLookupError:
                continue

        for vehicle in self.vehicles:
            if vehicle.poll() is not None:
                continue

            try:
                vehicle.wait(timeout=5)
                continue
            except subprocess.TimeoutExpired:
                pass

            try:
                os.killpg(vehicle.pid, signal.SIGTERM)
            except ProcessLookupError:
                continue

            try:
                vehicle.wait(timeout=2)
                continue
            except subprocess.TimeoutExpired:
                pass

            try:
                os.killpg(vehicle.pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
            vehicle.wait(timeout=2)

        print("Vehicle nodes are down")

    def last_enter_cb(self, msg):
        self.unpack_msg(msg, self.last_enter_state)

        # If vehicle left entrance area, start spawning another one
        # 判据1（旧场地）：y 边界；判据2（地图规则）：距 spawn 点 > 30m
        if getattr(self, '_enter_spawn_pos', None) is None:
            self._enter_spawn_pos = (self.last_enter_state.x.x, self.last_enter_state.x.y)
        _left = self.last_enter_state.x.y < self.y_bound_to_resume_spawning
        if not _left and self._enter_spawn_pos is not None:
            _dx = self.last_enter_state.x.x - self._enter_spawn_pos[0]
            _dy = self.last_enter_state.x.y - self._enter_spawn_pos[1]
            _left = (_dx * _dx + _dy * _dy) ** 0.5 > 30.0
        if _left:
            self.keep_spawn_entering = True
            self.get_logger().info("Vehicle %d left the entrance area." % self.last_enter_id)

    def try_spawn_entering(self):
        current_time = self.sim_now()

        if self.spawn_entering_time and current_time - self.last_enter_time > self.spawn_entering_time[0]:
            # 入库车辆从空位中选择（>0：0 作为「无车位」约定值不用于入库目标；
            # 已认领未抵达的也不可选，避免重复分配）
            claimed = self.entering_claimed | self.exiting_claimed
            empty_spots = [i for i in range(len(self.occupied))
                           if not self.occupied[i] and i > 0 and i not in claimed
                           and i not in self.unavailable_spots]
            if self.entering_spot_pool:
                empty_spots = [i for i in empty_spots if i in self.entering_spot_pool]
            # 串联互斥：入库轨迹要穿过的配对泊位若已被占用 / 已被认领（在途），
            # 该泊位不可分配（否则抵达时会整车穿过已停放车辆）。
            tandem_locked = 0
            if self.through_lock.enabled:
                before = len(empty_spots)
                empty_spots = self.through_lock.filter_available(
                    empty_spots, self.occupied, self.unavailable_spots, claimed)
                tandem_locked = before - len(empty_spots)
            if not empty_spots:
                self.get_logger().warn(
                    'No free spot for entering vehicle (tandem-locked: %d); retry next interval'
                    % tandem_locked)
                self.last_enter_time = current_time
                return
            chosen_spot = self.allocator.choose('entering', self.occupied, empty_spots)
            # 先定泊位 → 再从该泊位的可行入口集里选口（入口↔泊位联合约束）
            _entry = self._choose_entry_portal(chosen_spot)
            proc = self.add_vehicle(chosen_spot, entry_portal=_entry) # pick from empty spots (pluggable allocation)
            # 占用（可视化障碍物）由车辆在 PARK 完成（抵达泊位）时补发：
            # change_central_occupancy(spot, True)，与到达时刻同步。
            self.entering_claimed.add(chosen_spot)
            self.enter_procs.append((proc, self.num_vehicles, int(chosen_spot)))
            self.spawn_entering_time.pop(0)
            self.publish_spawn_status()

            self.last_enter_time = current_time
            self.last_enter_id = self.num_vehicles
            self.last_enter_sub = self.create_subscription(VehicleStateMsg, '/vehicle_%d/state' % self.last_enter_id, self.last_enter_cb, 10)
            self.keep_spawn_entering = False
            self._enter_spawn_pos = None   # 新入场车：重置 spawn 位置基准
            self._gate_closed_at = time.time()

    def try_spawn_exiting(self):
        current_time = self.sim_now()

        if self.spawn_exiting_time and current_time - self.last_exit_time > self.spawn_exiting_time[0]:
            # 出库车辆必须从「当前有虚拟停放车」的车位出发：
            # 已占用 且 未被在位车辆认领（入库车目标 / 已出库车位 / 封锁位均排除）。
            # 车位由车辆在 UNPARK 完成时释放（change_central_occupancy）。
            claimed = self.entering_claimed | self.exiting_claimed
            candidates = [i for i in range(len(self.occupied))
                          if self.occupied[i] and i not in claimed
                          and i not in self.unavailable_spots]
            # 串联互斥：出库机动同样要穿过配对泊位。锁生效时配对泊位必为空（初始
            # 占用与入库分配都保证互斥），故此过滤在正常运行时是 no-op；仅用于兜住
            # 场景文件显式指定的冲突占用，避免出现穿车出库轨迹。
            tandem_locked = 0
            if self.through_lock.enabled:
                before = len(candidates)
                candidates = self.through_lock.filter_available(
                    candidates, self.occupied, self.unavailable_spots, claimed)
                tandem_locked = before - len(candidates)
            if not candidates:
                self.get_logger().warn(
                    'No departable spot (occupied & unclaimed) for exiting vehicle '
                    '(tandem-locked: %d); entry skipped' % tandem_locked)
                self.spawn_exiting_time.pop(0)
                self.publish_spawn_status()
            else:
                chosen_spot = int(self.allocator.choose('exiting', self.occupied, candidates))
                _exit = self._choose_exit_portal(chosen_spot)
                proc = self.add_vehicle(-1 * chosen_spot, exit_portal=_exit)
                self.occupied[chosen_spot] = True   # 车辆仍在位；UNPARK 完成后由车辆侧释放
                self.exiting_claimed.add(chosen_spot)
                # 释放保障：记录 (进程, 车辆号, 车位)；若车辆未发释放即退出，由看门狗代发
                self.exit_procs.append((proc, self.num_vehicles, chosen_spot))
                self.spawn_exiting_time.pop(0)
                self.publish_spawn_status()

            self.last_exit_time = current_time

    def _release_guard(self):
        """占用同步保障（出库释放 / 入库抵达）：

        正常路径：
        - 出库车在 UNPARK 完成时释放占用（change_central_occupancy False）；
        - 入库车在 PARK 完成（抵达泊位）时补发占用（change_central_occupancy True）。
        本保障兜底异常路径：
        - 出库车未发释放即退出 → 代发释放，防幽灵占用；
        - 入库车未抵达即退出 → 释放认领（车位保持空），防车位被无效锁定。
        """
        if not self.exit_procs and not self.enter_procs:
            return
        remaining = []
        for proc, vid, spot in self.exit_procs:
            if proc.poll() is None:
                remaining.append((proc, vid, spot))
                continue
            # 车辆进程已结束：若车位仍未释放且未被入库车辆认领，则代发释放
            if (0 <= spot < len(self.occupied)
                    and self.occupied[spot]
                    and spot not in self.entering_claimed):
                self.occupied[spot] = 0
                self.get_logger().warn(
                    '[occupancy] exit vehicle %d ended without release; spot %d released by guard'
                    % (vid, spot))
            else:
                self.get_logger().info(
                    '[occupancy] exit vehicle %d ended; spot %d already released' % (vid, spot))
        self.exit_procs = remaining

        remaining_in = []
        for proc, vid, spot in self.enter_procs:
            if proc.poll() is None:
                remaining_in.append((proc, vid, spot))
                continue
            # 入库车进程已结束：未抵达（占用仍为 0）→ 释放认领；已抵达 → 占用保持
            if (0 <= spot < len(self.occupied)
                    and not self.occupied[spot]
                    and spot in self.entering_claimed):
                self.entering_claimed.discard(spot)
                self.get_logger().warn(
                    '[occupancy] parking vehicle %d ended without arrival; spot %d claim freed'
                    % (vid, spot))
            else:
                self.get_logger().info(
                    '[occupancy] parking vehicle %d ended; spot %d kept occupied' % (vid, spot))
        self.enter_procs = remaining_in

    def try_spawn_existing(self):
        time_scale = float((self.scenario.get('replay') or {}).get('time_scale', 1.0) or 1.0)
        current_time = self.sim_now() / time_scale
        added_vehicles = []

        for agent in self.agents_dict:
            if self.agents_dict[agent]["init_time"] < current_time:
                self.add_existing_vehicle(agent)
                added_vehicles.append(agent)

        for added in added_vehicles:
            del self.agents_dict[added]

    def timer_callback(self):

        if self.sim_is_running:
            if self.scenario_mode == 'custom':
                self.try_spawn_custom()
            elif self.scenario_mode == 'replay':
                self.try_spawn_existing()
            else:
        
                if self.keep_spawn_entering:
                    if self.last_enter_sub:
                        self.destroy_subscription(self.last_enter_sub)
                        self.last_enter_sub = None

                    self.try_spawn_entering()
                elif getattr(self, '_gate_closed_at', 0) and time.time() - self._gate_closed_at > 45.0:
                    # 放行门超时兜底：等待「车辆离开入口区域」超时（车辆异常/位置偏远）则强制放行
                    self.get_logger().warn(
                        'entrance gate timeout (%.0fs); allow next entering spawn' % (time.time() - self._gate_closed_at))
                    self._gate_closed_at = 0
                    if self.last_enter_sub:
                        self.destroy_subscription(self.last_enter_sub)
                        self.last_enter_sub = None
                    self.keep_spawn_entering = True

                self.try_spawn_exiting()

        # 出库占用释放保障（每 ~2 秒；代价极小）
        _now = self.get_ros_time()
        if _now - self._last_release_guard > 2.0:
            self._last_release_guard = _now
            self._release_guard()

        # Publish current simulation time
        time_msg = Float32()
        time_msg.data = self.sim_now()
        self.sim_time_pub.publish(time_msg)

        occupancy_msg = Int16MultiArray()
        occupancy_msg.data = self.occupied
        self.occupancy_pub.publish(occupancy_msg)

        departing_msg = Int16MultiArray()
        departing_msg.data = [
            int(spot) for _proc, _vid, spot in self.exit_procs
            if _proc.poll() is None
            and 0 <= spot < len(self.occupied)
            and self.occupied[spot]]
        self.departing_pub.publish(departing_msg)


def main(args=None):
    # ===== 临时诊断（定位异常退出根因；稳定后移除）=====
    try:
        import faulthandler
        faulthandler.enable()
    except Exception:
        pass

    def _diag_signal(signum, frame):
        try:
            print('[DIAG] signal %d received' % signum, flush=True)
            import faulthandler as _fh
            _fh.dump_traceback()
        except Exception:
            pass
        try:
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)
        except Exception:
            os._exit(1)

    for _sig_name in ('SIGHUP', 'SIGTERM', 'SIGQUIT', 'SIGABRT', 'SIGSEGV', 'SIGBUS'):
        _sig = getattr(signal, _sig_name, None)
        if _sig is not None:
            try:
                signal.signal(_sig, _diag_signal)
            except Exception:
                pass
    # ===== 诊断结束 =====

    rclpy.init(args=args)

    simulator = SimulatorNode()

    try:
        rclpy.spin(simulator)
        print('[DIAG] spin returned normally', flush=True)
    except KeyboardInterrupt:
        print('Simulation is terminated', flush=True)
        traceback.print_exc()
    except BaseException:
        print('Unknown exception', flush=True)
        traceback.print_exc()
    finally:
        print('[DIAG] finally: shutdown_vehicles ...', flush=True)
        simulator.shutdown_vehicles()
        print('[DIAG] finally: destroy_node ...', flush=True)
        simulator.destroy_node()
        print('Simulation stopped cleanly', flush=True)

        rclpy.shutdown()

if __name__ == "__main__":
    main()
