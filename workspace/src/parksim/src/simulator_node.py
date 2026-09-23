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

        # 外来串扰节点检测「最小观察窗」的计时起点：钉在第一个检查点（startup）。
        # 为什么需要：发布镜像把 frames.json / instances.json 换成 {}（1.54G→1.27G 瘦身）后，
        # dlp.Dataset.load() 近乎瞬时完成，使 startup 与 post-dataset-load 两个检查点的真实时间
        # 间隔从「宿主约 6 秒」塌缩到「容器约 0.25 秒」，DDS 发现来不及收敛 → 守卫漏检（QA 残留风险）。
        # 故在非首个检查点强制一段最小观察窗（见 _check_foreign_vehicles）；宿主上数据集加载本就
        # 超过该窗口，故不引入额外延迟。
        self._foreign_guard_t0 = None

        # 晚期串扰「一次性可见化」状态（不杀进程；见 _maybe_report_late_cross_talk）：
        # 两个启动检查点之后才被本域发现的外来节点，当前实现会完全静默（与 N7「静默降级」同类）——
        # 仿真继续跑、操作者不知情。故在 1 Hz spawn_status 路径低频补扫一次做可见化。
        # 观察窗起点复用 self._foreign_guard_t0（= 首个检查点/init 时刻）。
        self._late_cross_talk_warned = False       # 一次性：命中后不再打印（防刷屏）
        self._late_cross_talk_last_scan = None     # 单调时钟：上次扫描时刻（5 s 节流）

        # 外来串扰节点检测 checkpoint #1（startup）：多轮复核（vehicle + simulator），防 DDS 延迟竞态误判。
        # 注意：此处本 participant 刚创建约百毫秒，DDS 尚未收敛，可能看不到已在运行的外来节点
        # （假阴性，实测约 62% 看不到）；故此处仅作「尽早拦截」，真正的收敛后确认见 checkpoint #2。
        # startup 是「首个检查点」：不施加最小观察窗，保持「首轮干净立即放行」（宿主启动零额外延迟）。
        self._check_foreign_vehicles('startup')

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

        # 外来串扰节点检测 checkpoint #2（post-dataset-load）：
        # 此时本 participant 已存活约「数据集加载耗时」，DDS 发现通常已收敛，对已在运行的外部
        # vehicle/simulator 节点可见性高（摆脱启动瞬间的未收敛窗口）。
        # 位置确在「首批车 spawn 之前」——车辆仅在 timer_callback 中生成，__init__ 内不 spawn。
        # 失败语义与 checkpoint #1 完全一致（error + 抛 KeyboardInterrupt → 非 0 退出）。
        # 作为「非首个检查点」，此处会强制一段最小观察窗（_FOREIGN_GUARD_MIN_WINDOW）：容器瘦身
        # 后本检查点距 startup 仅 ~0.25s，需主动补足观察时间，否则守卫形同虚设；宿主上两检查点
        # 间隔本就 > 该窗口，故增量延迟为 0。
        self._check_foreign_vehicles('post-dataset-load')

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

    # ======== 外来串扰节点检测（vehicle + simulator；多轮复核；防 DDS 延迟误判 / 问题 N6） ========
    # 背景：DDS 节点发现存在收敛延迟——新建 participant 在 0~百毫秒内尚看不到已运行的对端
    # （实测「立即读」约 62% 看不到，「3s 后读」100% 看得到）。一次性检查因此成为竞态：
    # 容器表现为有时 exit=0 有时 exit=1。改为两处 checkpoint 复用同一多轮复核方法：
    #   #1 startup          ：participant 刚建，仅尽早拦截（可能假阴性，日志会标注）
    #   #2 post-dataset-load：数据集加载后，participant 已收敛，可见性 100%，稳定可判
    # 监听两类外来节点：外来 vehicle（他实例已 spawn 的车）+ 外来 simulator（他实例本身，
    # 覆盖「两实例同时冷启动、双方都还没 spawn vehicle」的路径）；自身节点按同名计数排除。
    # 两处均为「连续 _FOREIGN_CHECK_CONFIRM 轮命中才判失败；首轮干净立即放行（正常单实例零额外延迟）」。
    # 另提供显式逃生开关 PARKSIM_ALLOW_FOREIGN_VEHICLES，便于多实例联调（两处均尊重，同时跳过两类）。
    # 最小观察窗：发布镜像瘦身（frames/instances → {}）后 Dataset.load() 近瞬时完成，两检查点间隔
    # 从宿主 ~6s 塌缩到容器 ~0.25s，DDS 来不及收敛 → 守卫漏检。故从「第二个检查点起」，在两个检查点
    # 起点之间强制至少 _FOREIGN_GUARD_MIN_WINDOW 秒的观察时间（宿主本已满足，零额外延迟）。
    _FOREIGN_CHECK_INTERVAL = 1.0    # 轮询间隔（秒）
    _FOREIGN_CHECK_MAX_WAIT = 10.0   # 最长观察窗口（秒）
    _FOREIGN_CHECK_CONFIRM = 2       # 连续命中轮数达到该值才判定失败
    _FOREIGN_GUARD_MIN_WINDOW = 3.0  # 非首个检查点的最小观察窗（秒）；PARKSIM_FOREIGN_GUARD_WINDOW 覆盖，取 0 关闭
    _FOREIGN_ALLOW_TRUTHY = ('1', 'true', 'yes', 'on')   # PARKSIM_ALLOW_FOREIGN_VEHICLES 真值
    # 晚期串扰「一次性可见化」（不杀进程；见 _maybe_report_late_cross_talk）：
    _LATE_CROSS_TALK_INTERVAL = 5.0   # 晚期串扰扫描周期（秒）
    _LATE_CROSS_TALK_WINDOW = 60.0    # 晚期串扰观察窗（自 _foreign_guard_t0 起；之后停止扫描，避免长期开销/噪声）

    def _foreign_vehicles_allowed(self) -> bool:
        """逃生开关：环境变量 PARKSIM_ALLOW_FOREIGN_VEHICLES 为真时跳过外来节点检查。

        大小写不敏感，接受 1/true/yes/on（含首尾空白）。
        """
        raw = os.environ.get('PARKSIM_ALLOW_FOREIGN_VEHICLES', '')
        return str(raw).strip().lower() in self._FOREIGN_ALLOW_TRUTHY

    def _foreign_guard_min_window(self) -> float:
        """非首个检查点的最小观察窗秒数。

        环境变量 PARKSIM_FOREIGN_GUARD_WINDOW 覆盖默认值；解析风格与
        _foreign_vehicles_allowed 一致（去首尾空白；无法解析或为负数时回落默认值）。
        取 0 表示关闭该机制（回到「首轮干净立即放行」的旧行为）。
        """
        default = self._FOREIGN_GUARD_MIN_WINDOW
        raw = os.environ.get('PARKSIM_FOREIGN_GUARD_WINDOW', '')
        try:
            value = float(str(raw).strip())
        except (TypeError, ValueError):
            return default
        return value if value >= 0.0 else default

    def _scan_foreign_nodes(self, exclude_own_vehicles: bool = False):
        """扫描当前 ROS 图，判定「外来」串扰节点，返回 (全部节点标识, 外来vehicle, 外来simulator)。

        判定口径（均排除自身）：
          - 外来 vehicle   ：name == 'vehicle'（命名空间 /vehicle_<id>），与改前口径一致。
            注：exclude_own_vehicles=True 时额外排除**本实例自己已 spawn 的车**
            （命名空间 /vehicle_<id>，id ∈ [1, self.num_vehicles]，id 单调递增）。
            启动期（两个检查点）本实例尚未 spawn 任何车（num_vehicles=0），且默认 False，
            故启动口径**逐字节不变**；运行期晚期扫描必须传 True，否则会把自己的车误报为外来。
          - 外来 simulator ：name == 自身节点名（'simulator'）。自身与其它 simulator 的
            (name, namespace) 完全相同（均为 ('simulator','/')），且 ROS2 图 API 不去重
            （实测同名同命名空间会返回多条），故以「同名同命名空间出现次数 - 1（自身）」判定。
          覆盖「两实例同时冷启动」路径：此刻双方都还没 spawn vehicle，但各自都有 simulator 节点。
        无关节点（_ros2cli_daemon_*、webviz_bridge、visualizer 等）名字不匹配，天然排除。
        """
        pairs = [(str(n[0]), str(n[1])) for n in self.get_node_names_and_namespaces()]

        def norm_ns(ns):
            return '' if ns in ('', '/') else ns.rstrip('/')

        def label(name, ns):
            return ('/' + name) if norm_ns(ns) == '' else (norm_ns(ns) + '/' + name)

        labels = [label(name, ns) for name, ns in pairs]

        own_vehicle_ns = set()
        if exclude_own_vehicles:
            own_vehicle_ns = {'/vehicle_%d' % i for i in range(1, int(self.num_vehicles) + 1)}

        def _is_foreign_vehicle(name, ns):
            if name != 'vehicle':
                return False
            return not (exclude_own_vehicles and norm_ns(ns) in own_vehicle_ns)

        foreign_vehicle = [name for name, ns in pairs if _is_foreign_vehicle(name, ns)]

        self_name = str(self.get_name())
        self_ns = norm_ns(str(self.get_namespace()))
        same_as_self = sum(1 for name, ns in pairs if name == self_name and norm_ns(ns) == self_ns)
        foreign_simulator = [self_name] * max(0, same_as_self - 1)
        return labels, foreign_vehicle, foreign_simulator

    def _check_foreign_vehicles(self, checkpoint: str = 'startup') -> None:
        """多轮复核是否存在未清理的外来串扰节点（外来 vehicle 或外来 simulator），必要时终止启动。

        checkpoint 区分调用位置并写入日志：
          - 'startup'           启动早期（participant 刚创建，DDS 可能未收敛，仅尽早拦截）
          - 'post-dataset-load' 数据集加载后；两处复用同一方法。

        - 环境变量 PARKSIM_ALLOW_FOREIGN_VEHICLES（1/true/yes/on，大小写不敏感）为真时
          完全跳过检查并打印 WARN（多实例联调）；两个 checkpoint 均尊重该开关。
        - 否则轮询节点名列表：连续 _FOREIGN_CHECK_CONFIRM 轮都命中（外来 vehicle 或外来
          simulator）才判定失败；失败时保留原有错误语义（抛 KeyboardInterrupt → 非 0 退出）。
        - 首个检查点（startup）：首轮干净立即放行（不给宿主启动引入额外延迟）。
        - 非首个检查点（post-dataset-load）：首轮干净时，若距首个检查点起点不足
          _FOREIGN_GUARD_MIN_WINDOW 秒则继续观察、补足窗口后才放行（循环以单调递增的
          elapsed_total 判界，必然终止）。原因：发布镜像瘦身后 Dataset.load() 近瞬时完成，
          两检查点间隔由宿主 ~6s 塌缩到容器 ~0.25s，DDS 发现来不及收敛 -> 守卫在
          「显式关隔离 + 存在外来 simulator」时会漏检；宿主上间隔本就 > 窗口，故零额外延迟。
        - 最长观察 _FOREIGN_CHECK_MAX_WAIT 秒上限、连续确认要求均保持不变。
        - 每轮打印 checkpoint / 轮次 / 看到的全部节点 / 连续命中计数 / 耗时与最终判定，便于复盘。
        """
        if self._foreign_vehicles_allowed():
            self.get_logger().warn(
                'PARKSIM_ALLOW_FOREIGN_VEHICLES=%r is set; skipping foreign node check (vehicle + simulator) '
                '(checkpoint=%s). Multiple simulator instances may interfere with each other (allow-listed).'
                % (os.environ.get('PARKSIM_ALLOW_FOREIGN_VEHICLES'), checkpoint))
            return

        # 最小观察窗：计时起点钉在「第一个检查点」；仅「非首个检查点」受窗口约束（startup 行为完全不变）。
        is_first_checkpoint = self._foreign_guard_t0 is None
        if is_first_checkpoint:
            self._foreign_guard_t0 = time.monotonic()
        min_window = self._foreign_guard_min_window()

        interval = self._FOREIGN_CHECK_INTERVAL
        max_wait = self._FOREIGN_CHECK_MAX_WAIT
        confirm = self._FOREIGN_CHECK_CONFIRM
        hits = 0
        round_idx = 0
        start = time.monotonic()
        while True:
            round_idx += 1
            labels, foreign_vehicle, foreign_simulator = self._scan_foreign_nodes()
            foreign = bool(foreign_vehicle or foreign_simulator)
            if foreign:
                hits += 1
                self.get_logger().warn(
                    'Foreign node check [%s] round %d: found%s%s (consecutive %d/%d, elapsed %.1fs); all nodes=%s'
                    % (checkpoint, round_idx,
                       (' foreign vehicle=%s' % foreign_vehicle) if foreign_vehicle else '',
                       (' foreign simulator=%s' % foreign_simulator) if foreign_simulator else '',
                       hits, confirm, time.monotonic() - start, labels))
            else:
                hits = 0
                self.get_logger().info(
                    'Foreign node check [%s] round %d: no foreign node; all nodes=%s'
                    % (checkpoint, round_idx, labels))
            # 连续确认：达到阈值 -> 判定失败，保留原有错误语义（并区分 vehicle / simulator）
            if hits >= confirm:
                if foreign_vehicle and foreign_simulator:
                    what = 'Some vehicle nodes are not shut down cleanly, and another simulator is running'
                elif foreign_vehicle:
                    what = 'Some vehicle nodes are not shut down cleanly'
                else:
                    what = 'Another simulator is running (cross-talk)'
                self.get_logger().error(
                    '%s (checkpoint=%s, confirmed over %d consecutive rounds, elapsed %.1fs). '
                    'Please kill those processes first.' % (what, checkpoint, hits, time.monotonic() - start))
                raise KeyboardInterrupt()
            # 本轮干净：非首个检查点须先满足最小观察窗，否则继续观察（不 return）
            if not foreign:
                elapsed_total = time.monotonic() - self._foreign_guard_t0
                if (not is_first_checkpoint) and elapsed_total < min_window:
                    self.get_logger().info(
                        'Foreign node check [%s] round %d: clean, but min observation window not met '
                        '(elapsed_total=%.2fs < %.2fs, need %.2fs more); keep observing.'
                        % (checkpoint, round_idx, elapsed_total, min_window, min_window - elapsed_total))
                    time.sleep(interval)   # elapsed_total 单调递增 -> 循环必然终止
                    continue
                self.get_logger().info(
                    'Foreign node check [%s] passed (clean on round %d, elapsed %.1fs, window_total %.1fs).'
                    % (checkpoint, round_idx, time.monotonic() - start, elapsed_total))
                return
            # 命中但未被连续确认且已达观察上限 -> 放行，避免 DDS 抖动误杀
            if time.monotonic() - start >= max_wait:
                self.get_logger().warn(
                    'Foreign node check [%s] reached max wait %.1fs with only %d consecutive hit(s); '
                    'allowing startup without confirmation.' % (checkpoint, max_wait, hits))
                return
            time.sleep(interval)

    def _maybe_report_late_cross_talk(self) -> None:
        """晚期串扰「一次性可见化」：启动两检查点之后才被本域发现的外来节点不再静默。

        为什么需要：startup / post-dataset-load 两个检查点只在**启动窗口内**做判断。若外来实例
        晚于该窗口才被本域发现（或发现耗时 > 最小观察窗），现有实现会**完全静默** —— 仿真继续
        运行、操作者不知情（与 N7「静默降级」同类问题，故仅做可见化，不硬杀）。

        设计约束（保持最小、加法式、健康路径零额外开销）：
        - 复用**既有** 1 Hz 派生状态定时器路径（publish_spawn_status），不新建线程、不新建 timer
          对象，避免与已有执行器交互出事；用单调时钟累积达到 _LATE_CROSS_TALK_INTERVAL 才扫。
        - 仅在 self.sim_is_running 为真时扫。
        - 观察窗 _LATE_CROSS_TALK_WINDOW = 60 s，**起点复用 self._foreign_guard_t0**
          （= 首个检查点/init 时刻，已在 __init__ 钉住）；超过窗口即停止扫描，避免长期开销与噪声。
        - 命中外来 vehicle/simulator（复用 _scan_foreign_nodes()，判定口径不改）时**只打一条**
          WARN（实例标志位 _late_cross_talk_warned 保证一次性、防刷屏）；**不 raise、不杀进程**。
        - 逃生开关 PARKSIM_ALLOW_FOREIGN_VEHICLES 为真时**整路跳过**（与两个启动检查点一致），
          且**不打印** skip WARN（避免与启动期那两条 skip WARN 重复刷屏）。
        - 与 PARKSIM_FOREIGN_GUARD_WINDOW **相互独立**：后者只关「最小观察窗（启动期）」，
          不关本路晚期扫描（两者语义不同：一个是启动期防漏检，一个是运行期可见化）。
        """
        if self._late_cross_talk_warned:
            return
        if self._foreign_vehicles_allowed():     # 逃生开关：整路跳过，且不打印 skip WARN
            return
        if not self.sim_is_running:
            return
        t0 = self._foreign_guard_t0
        if t0 is None:                            # 无起点（异常/未设）则不做，避免误判
            return
        now = time.monotonic()
        if now - t0 > self._LATE_CROSS_TALK_WINDOW:   # 超出观察窗：停止扫描
            return
        last = self._late_cross_talk_last_scan
        if last is not None and (now - last) < self._LATE_CROSS_TALK_INTERVAL:
            return                                # 5 s 节流：低频扫描
        self._late_cross_talk_last_scan = now
        # 运行期扫描必须排除本实例自己已 spawn 的车（/vehicle_<id>），否则会把自己刚发的车
        # 误报为「外来 vehicle」（启动期两检查点无此问题，那时还没 spawn）。
        _labels, foreign_vehicle, foreign_simulator = self._scan_foreign_nodes(
            exclude_own_vehicles=True)
        if not (foreign_vehicle or foreign_simulator):
            return
        self._late_cross_talk_warned = True       # 一次性：命中即置位，后续不再打印
        detail = []
        if foreign_vehicle:
            detail.append('foreign vehicle=%s' % sorted(set(foreign_vehicle)))
        if foreign_simulator:
            detail.append('%d foreign simulator node(s)' % len(foreign_simulator))
        self.get_logger().warn(
            'Late cross-talk detected: foreign node(s) [%s] appeared after startup checkpoints; '
            'this instance and another are sharing one DDS domain. '
            'Isolate with ROS_LOCALHOST_ONLY=1 or set PARKSIM_ALLOW_FOREIGN_VEHICLES=1 to silence.'
            % '; '.join(detail))

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
        # 复用既有 1 Hz 派生状态路径做低频（5 s）外来节点扫描：晚期串扰一次性可见化，
        # 不新建线程/timer、不杀进程（见 _maybe_report_late_cross_talk）。
        self._maybe_report_late_cross_talk()

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
