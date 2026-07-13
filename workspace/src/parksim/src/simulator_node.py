#!/usr/bin/env python3
import rclpy

import subprocess
import signal
import numpy as np

from pathlib import Path
import glob
import os

import traceback

import pickle
import json

from dlp.dataset import Dataset
from dlp.visualizer import Visualizer as DlpVisualizer

from std_msgs.msg import Int16MultiArray, Bool, Float32, String
from parksim.msg import VehicleStateMsg
from parksim.srv import OccupancySrv
from parksim.base_node import MPClabNode, parksim_path
from parksim.pytypes import VehicleState, NodeParamTemplate
from parksim.vla.vehicle_ids import VehicleIdAllocator, replay_vehicle_ids

VLA_AGENT_TYPES = {
    'qwen_vla', 'mllm_direct', 'mllm_self_reflect', 'mllm_external_feedback', 'fleet_min_cost',
    'greedy_nearest', 'greedy_shortest_path', 'risk_aware_rule', 'bundle_risk_aware',
    'conflict_aware_bundle', 'min_bundle_cost', 'reservation_bundle', 'rolling_horizon_bundle',
    'centralized_min_cost', 'oracle_intent_bundle', 'vla_baseline',
}


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def _traffic_vehicle_role(event_type, agent_type):
    event_type = str(event_type).lower()
    agent_type = str(agent_type).lower()
    if agent_type in VLA_AGENT_TYPES:
        prefix = 'av'
    elif agent_type == 'rule_based':
        prefix = 'human_rule'
    else:
        prefix = 'human'
    if event_type == 'exiting':
        return prefix + '_exiting'
    return prefix + '_entering'


class SimulatorNodeParams(NodeParamTemplate):
    """
    template that stores all parameters needed for the node as well as default values
    """
    def __init__(self):
        self.dlp_path = parksim_path('python', 'parksim', 'priorFiles', 'data', 'DJI_0012')
        self.timer_period = 0.1
        self.random_seed = 0

        self.blocked_spots = []

        self.spawn_entering = 3
        self.spawn_exiting = 3
        self.av_spawn_entering = 0
        self.av_spawn_exiting = 0
        self.av_entry_vehicle_agent_type = 'qwen_vla'
        self.av_exit_vehicle_agent_type = 'qwen_vla'
        self.av_entry_vehicle_role = 'av_entering'
        self.av_exit_vehicle_role = 'av_exiting'
        self.y_bound_to_resume_spawning = 70
        self.spawn_interval_mean = 5 # (s)

        self.spots_data_path = ''
        self.agents_data_path = parksim_path('python', 'parksim', 'priorFiles', 'agents_data_0012.pickle')

        self.use_existing_agents = True
        self.background_mode = 'replay'  # replay, rule_random, mixed, or human_mixed

        # Long-horizon human-machine mixed traffic generator. The legacy path is
        # unchanged unless this mode is explicitly enabled by an experiment script.
        self.traffic_flow_mode = 'legacy'  # legacy or human_mixed_long_horizon
        self.traffic_arrival_process = 'nhpp_piecewise'
        self.traffic_rate_profile = 'parking_diurnal'
        self.traffic_rate_window_seconds = 300.0
        self.long_horizon_duration = 600.0
        self.hard_stop_at_long_horizon = False
        self.restore_obstacles_as_exit_vehicles = False
        self.static_obstacle_exit_fraction = 0.35
        self.static_obstacle_exit_max = 40
        self.static_obstacle_exit_start_time = 5.0
        self.long_horizon_enter_interval_mean = 10.0
        self.long_horizon_exit_interval_mean = 14.0
        # Optional actor-class-specific Poisson streams. Non-positive values
        # preserve compatibility by falling back to the legacy aggregate mean.
        self.long_horizon_human_enter_interval_mean = -1.0
        self.long_horizon_human_exit_interval_mean = -1.0
        self.long_horizon_av_enter_interval_mean = -1.0
        self.long_horizon_av_exit_interval_mean = -1.0
        self.human_intent_hidden_fraction = 0.75
        self.max_concurrent_background_vehicles = 80
        self.delayed_spawn_retry_seconds = 2.0
        self.exit_spot_reuse_delay = 20.0
        self.entry_vehicle_agent_type = 'rule_based'
        self.exit_vehicle_agent_type = 'rule_based'
        self.entry_vehicle_role = ''
        self.exit_vehicle_role = ''

        self.spawn_qwen_ego = False
        self.qwen_ego_spawn_time = 0.5
        self.qwen_ego_spot_index = 1
        self.spawn_controlled_ego = False
        self.controlled_ego_spawn_time = 0.5
        self.controlled_ego_spot_index = 1
        self.controlled_ego_agent_type = 'qwen_vla'
        self.controlled_ego_blocks_entrance = True
        self.qwen_endpoint = ''
        self.qwen_model = 'Qwen2.5-VL-7B-Instruct'
        self.qwen_timeout = 15.0
        self.qwen_decision_period = 3.0
        self.qwen_max_candidate_spots = 8
        self.qwen_periodic_replan = False
        self.qwen_decision_log_path = ''
        self.vla_baseline_strategy = 'risk_aware_rule'
        self.fleet_coordinator_enabled = False
        self.fleet_run_id = ''

        self.write_log = True
        self.log_path = parksim_path('vehicle_log')

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

        np.random.seed(self.random_seed)

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
        for idx in self.blocked_spots:
            self.occupied[idx] = True

        # Agents
        self._gen_agents()
        self.replay_vehicle_ids = replay_vehicle_ids(self.agents_dict.keys(), self.background_mode)
        self.vehicle_id_allocator = VehicleIdAllocator(self.replay_vehicle_ids)

        self.last_enter_id = None
        self.last_enter_sub = None
        self.last_enter_state = VehicleState()
        self.keep_spawn_entering = True

        self.start_time = self.get_ros_time()
        self.sim_time = 0.0

        self.last_enter_time = 0.0
        self.last_exit_time = 0.0

        self.vehicles = []
        # Kept as the most recently allocated dynamic ID for legacy callers.
        self.num_vehicles = 0
        self.qwen_ego_spawned = False
        self.exit_reserved_spots = {}
        self.traffic_schedule = []
        self.next_traffic_event_idx = 0
        self.traffic_events_path = os.path.join(self.log_path, 'traffic_events.jsonl')
        self.traffic_schedule_path = os.path.join(self.log_path, 'traffic_schedule.json')

        # Spawning
        # Legacy mode keeps the original fixed-count exponential traffic process.
        if self._uses_scheduled_traffic():
            self.spawn_entering_time = []
            self.spawn_exiting_time = []
            self._init_scheduled_traffic()
        else:
            self.spawn_entering_time = list(np.random.exponential(self.spawn_interval_mean, self.spawn_entering))
            self.spawn_exiting_time = list(np.random.exponential(self.spawn_interval_mean, self.spawn_exiting))

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Publish the simulation time
        self.sim_time_pub = self.create_publisher(Float32, '/sim_time', 10)

        # Visualizer publish this status since the button is on GUI
        self.sim_status_sub = self.create_subscription(Bool, '/sim_status', self.sim_status_cb, 10)
        self.sim_is_running = True
        self.fleet_paused = False
        self.fleet_pause_sub = self.create_subscription(Bool, '/vla/fleet_pause', self.fleet_pause_cb, 10)
        self.fleet_pause_ack_pub = self.create_publisher(String, '/vla/fleet_pause_ack', 10)

        self.occupancy_pub = self.create_publisher(Int16MultiArray, 'occupancy', 10)

        self.occupancy_srv = self.create_service(OccupancySrv, 'occupancy', self.occupancy_srv_callback)

        self.occupancy_cli = self.create_client(OccupancySrv, '/occupancy')

    def sim_status_cb(self, msg: Bool):
        self.sim_is_running = msg.data

    def fleet_pause_cb(self, msg: Bool):
        self.fleet_paused = bool(msg.data)
        ack = String()
        ack.data = json.dumps({
            'run_id': str(self.fleet_run_id),
            'paused': bool(self.fleet_paused),
            'sim_time': float(self.sim_time),
        })
        self.fleet_pause_ack_pub.publish(ack)

    def occupancy_srv_callback(self, request, response):
        vehicle_id = request.vehicle_id
        idx = request.idx
        new_value = request.new_value

        self.occupied[idx] = new_value

        response.status = True

        self.get_logger().info("Vehicle %d changed the occupancy at %d to be %r" % (vehicle_id, idx, new_value))
        
        return response

    def _gen_occupancy(self):
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
        with open(self.agents_data_path, 'rb') as f:
            self.agents_dict = pickle.load(f)

    def add_vehicle(
            self,
            spot_index: int,
            agent_type: str = 'rule_based',
            is_controlled_ego: bool = False,
            vehicle_role: str = 'background',
            intent_observable: bool = True,
            intent_label: str = '',
            spawn_event_id: str = ''):

        vehicle_id = self.vehicle_id_allocator.allocate()
        self.num_vehicles = vehicle_id

        command = [
            "ros2", "launch", "parksim", "vehicle.launch.py",
            "vehicle_id:=%d" % vehicle_id,
            "spot_index:=%d" % spot_index,
            "agent_type:=%s" % agent_type,
            "is_controlled_ego:=%s" % str(bool(is_controlled_ego)).lower(),
            "vehicle_role:=%s" % vehicle_role,
            "intent_observable:=%s" % str(bool(intent_observable)).lower(),
            "intent_label:=%s" % intent_label,
            "spawn_event_id:=%s" % spawn_event_id,
            "log_path:=%s" % self.log_path,
        ]
        if agent_type in VLA_AGENT_TYPES:
            command.extend([
                "qwen_endpoint:=%s" % self.qwen_endpoint,
                "qwen_model:=%s" % self.qwen_model,
                "qwen_timeout:=%s" % self.qwen_timeout,
                "qwen_decision_period:=%s" % self.qwen_decision_period,
                "qwen_max_candidate_spots:=%s" % self.qwen_max_candidate_spots,
                "qwen_periodic_replan:=%s" % str(self.qwen_periodic_replan).lower(),
                "qwen_decision_log_path:=%s" % (self.qwen_decision_log_path or os.path.join(self.log_path, "qwen_vla_decisions.jsonl")),
                "vla_baseline_strategy:=%s" % self.vla_baseline_strategy,
                "reveal_background_intents_to_vla:=false",
                "fleet_coordinator_enabled:=%s" % str(bool(self.fleet_coordinator_enabled)).lower(),
                "fleet_run_id:=%s" % self.fleet_run_id,
            ])

        self.vehicles.append(
            subprocess.Popen(command, start_new_session=True)
        )

        self.get_logger().info(
            "A %s vehicle with id = %d is added with spot_index = %d role=%s intent_observable=%r controlled_ego=%r"
            % (agent_type, vehicle_id, spot_index, vehicle_role, bool(intent_observable), bool(is_controlled_ego)))
        return vehicle_id

    def _replay_declared_operation(self, vehicle_id: int) -> str:
        agent = self.agents_dict.get(vehicle_id, self.agents_dict.get(str(vehicle_id), {})) or {}
        tasks = list(agent.get('task_profile') or [])
        task_names = [str(task.get('name', '')).upper() for task in tasks]
        if 'PARK' in task_names:
            return 'entering'
        if 'UNPARK' in task_names:
            return 'exiting'

        initial = np.asarray(agent.get('init_coords') or [], dtype=float)
        target = np.asarray([], dtype=float)
        for task in reversed(tasks):
            if task.get('target_coords') is not None:
                target = np.asarray(task.get('target_coords'), dtype=float)
                break
            spot = task.get('target_spot_index')
            if spot is not None and 0 <= int(spot) < len(self.parking_spaces):
                target = np.asarray(self.parking_spaces[int(spot)], dtype=float)
                break
        if initial.size >= 2 and target.size >= 2 and len(self.parking_spaces):
            spaces = np.asarray(self.parking_spaces, dtype=float)
            initial_distance = float(np.min(np.linalg.norm(spaces - initial[:2], axis=1)))
            target_distance = float(np.min(np.linalg.norm(spaces - target[:2], axis=1)))
            return 'entering' if target_distance < initial_distance else 'exiting'
        return 'exiting'

    def add_existing_vehicle(self, vehicle_id: int):
        try:
            vehicle_id = self.vehicle_id_allocator.claim_reserved(vehicle_id)
        except ValueError as exc:
            self.get_logger().error("Replay vehicle launch rejected: %s" % exc)
            return None

        declared_operation = self._replay_declared_operation(vehicle_id)
        self.vehicles.append(
            subprocess.Popen(
                [
                    "ros2", "launch", "parksim", "vehicle.launch.py",
                    "vehicle_id:=%d" % vehicle_id,
                    "spot_index:=%d" % 0,
                    "use_existing:=1",
                    "vehicle_role:=replay_%s" % declared_operation,
                    "intent_observable:=false",
                    "intent_label:=replay_hidden",
                    "spawn_event_id:=replay_%d" % vehicle_id,
                    "log_path:=%s" % self.log_path,
                ],
                start_new_session=True,
            )
        )

        self.get_logger().info("An existing replay vehicle with id = %d is added" % vehicle_id)
        return vehicle_id

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


    def _uses_scheduled_traffic(self):
        mode = str(self.traffic_flow_mode).lower()
        background_mode = str(self.background_mode).lower()
        return mode in ('human_mixed', 'human_mixed_long_horizon', 'long_horizon', 'long_horizon_mixed') or background_mode in ('human_mixed', 'human_mixed_long_horizon')

    def _active_vehicle_count(self):
        return sum(1 for vehicle in self.vehicles if vehicle.poll() is None)

    def _hidden_intent(self):
        return bool(np.random.random() < max(0.0, min(1.0, _safe_float(self.human_intent_hidden_fraction, 0.75))))

    def _traffic_time(self):
        return float(self.sim_time)

    def _append_traffic_event(self, event, status, reason='', vehicle_id=None, spot_index=None):
        if not self.write_log:
            return
        os.makedirs(self.log_path, exist_ok=True)
        payload = dict(event)
        payload.update({
            'sim_time': self._traffic_time(),
            'status': status,
            'reason': reason,
            'vehicle_id': vehicle_id,
            'spot_index': spot_index,
            'active_vehicle_count': self._active_vehicle_count(),
        })
        with open(self.traffic_events_path, 'a') as f:
            f.write(json.dumps(payload) + '\n')

    def _write_traffic_schedule(self):
        if not self.write_log:
            return
        os.makedirs(self.log_path, exist_ok=True)
        payload = {
            'traffic_flow_mode': str(self.traffic_flow_mode),
            'traffic_arrival_process': str(self.traffic_arrival_process),
            'traffic_rate_profile': str(self.traffic_rate_profile),
            'traffic_rate_window_seconds': _safe_float(self.traffic_rate_window_seconds, 300.0),
            'background_mode': str(self.background_mode),
            'long_horizon_duration': _safe_float(self.long_horizon_duration, 0.0),
            'spawn_entering': _safe_int(self.spawn_entering, 0),
            'spawn_exiting': _safe_int(self.spawn_exiting, 0),
            'av_spawn_entering': _safe_int(self.av_spawn_entering, 0),
            'av_spawn_exiting': _safe_int(self.av_spawn_exiting, 0),
            'av_entry_vehicle_agent_type': str(self.av_entry_vehicle_agent_type),
            'av_exit_vehicle_agent_type': str(self.av_exit_vehicle_agent_type),
            'av_entry_vehicle_role': str(self.av_entry_vehicle_role),
            'av_exit_vehicle_role': str(self.av_exit_vehicle_role),
            'restore_obstacles_as_exit_vehicles': _as_bool(self.restore_obstacles_as_exit_vehicles),
            'entry_vehicle_agent_type': str(self.entry_vehicle_agent_type),
            'exit_vehicle_agent_type': str(self.exit_vehicle_agent_type),
            'entry_vehicle_role': str(self.entry_vehicle_role),
            'exit_vehicle_role': str(self.exit_vehicle_role),
            'human_intent_hidden_fraction': _safe_float(self.human_intent_hidden_fraction, 0.0),
            'long_horizon_enter_interval_mean': _safe_float(self.long_horizon_enter_interval_mean, 0.0),
            'long_horizon_exit_interval_mean': _safe_float(self.long_horizon_exit_interval_mean, 0.0),
            'long_horizon_human_enter_interval_mean': self._flow_interval_mean('human', 'entering'),
            'long_horizon_human_exit_interval_mean': self._flow_interval_mean('human', 'exiting'),
            'long_horizon_av_enter_interval_mean': self._flow_interval_mean('av', 'entering'),
            'long_horizon_av_exit_interval_mean': self._flow_interval_mean('av', 'exiting'),
            'events': self.traffic_schedule,
            'hidden_event_count': sum(1 for event in self.traffic_schedule if not event.get('intent_observable', True)),
        }
        with open(self.traffic_schedule_path, 'w') as f:
            json.dump(payload, f, indent=2)

    def _cumulative_event_times(self, count, interval_mean, start=0.0, actor_class='human', event_type='entering'):
        count = max(0, _safe_int(count, 0))
        mean = max(0.1, _safe_float(interval_mean, 1.0))
        horizon = max(0.0, _safe_float(self.long_horizon_duration, 0.0))
        times = []
        current = float(start)
        process = str(self.traffic_arrival_process or 'homogeneous_poisson').lower()
        profile = self._traffic_rate_multipliers(actor_class, event_type)
        maximum = max(profile) if process == 'nhpp_piecewise' else 1.0
        attempts = 0
        while len(times) < count:
            attempts += 1
            if attempts > max(1000, count * 100):
                break
            current += float(np.random.exponential(mean / max(0.05, maximum)))
            if horizon > 0.0 and current > horizon:
                break
            if process == 'nhpp_piecewise':
                multiplier = self._traffic_rate_multiplier(profile, current)
                if float(np.random.random()) > multiplier / max(0.05, maximum):
                    continue
            times.append(round(current, 3))
        return times

    def _traffic_rate_multipliers(self, actor_class, event_type):
        if str(self.traffic_rate_profile).lower() != 'parking_diurnal':
            return [1.0]
        entering = [0.70, 0.85, 1.05, 1.30, 1.55, 1.45, 1.25, 1.05, 0.90, 0.80, 0.70, 0.60]
        exiting = [0.60, 0.70, 0.80, 0.90, 1.05, 1.25, 1.45, 1.60, 1.45, 1.20, 0.90, 0.70]
        values = entering if str(event_type).lower() == 'entering' else exiting
        if str(actor_class).lower() == 'av':
            values = [0.9 + 0.1 * value for value in values]
        return values

    def _traffic_rate_multiplier(self, profile, sim_time):
        if not profile:
            return 1.0
        window = max(1.0, _safe_float(self.traffic_rate_window_seconds, 300.0))
        index = min(len(profile) - 1, max(0, int(float(sim_time) // window)))
        return max(0.05, _safe_float(profile[index], 1.0))

    def _flow_interval_mean(self, actor_class, event_type):
        legacy = (self.long_horizon_enter_interval_mean
                  if event_type == 'entering'
                  else self.long_horizon_exit_interval_mean)
        attr = 'long_horizon_%s_%s_interval_mean' % (
            actor_class, 'enter' if event_type == 'entering' else 'exit')
        configured = _safe_float(getattr(self, attr, -1.0), -1.0)
        return configured if configured > 0.0 else max(0.1, _safe_float(legacy, 1.0))

    def _initial_departable_spots(self):
        blocked = set(int(idx) for idx in list(self.blocked_spots))
        return [idx for idx in range(1, len(self.occupied)) if bool(self.occupied[idx]) and idx not in blocked]

    def _init_scheduled_traffic(self):
        events = []
        horizon = max(0.0, _safe_float(self.long_horizon_duration, 0.0))
        if _as_bool(self.restore_obstacles_as_exit_vehicles):
            departable = self._initial_departable_spots()
            np.random.shuffle(departable)
            fraction = max(0.0, min(1.0, _safe_float(self.static_obstacle_exit_fraction, 0.35)))
            max_count = max(0, _safe_int(self.static_obstacle_exit_max, len(departable)))
            count = min(max_count, int(round(len(departable) * fraction)))
            start = _safe_float(self.static_obstacle_exit_start_time, 5.0)
            times = self._cumulative_event_times(
                count, self._flow_interval_mean('human', 'exiting'), start=start,
                actor_class='human', event_type='exiting')
            for seq, (spot, event_time) in enumerate(zip(departable[:len(times)], times)):
                hidden = self._hidden_intent()
                events.append({
                    'event_id': 'restore_exit_%03d' % seq,
                    'time': float(event_time),
                    'event_type': 'exiting',
                    'source': 'static_obstacle_restore',
                    'spot_index': int(spot),
                    'intent_observable': not hidden,
                    'ground_truth_intent': 'exit_from_spot_%d' % int(spot),
                    'preference_seed': int(np.random.randint(0, 2 ** 31 - 1)),
                    'attempts': 0,
                })
        human_enter_mean = self._flow_interval_mean('human', 'entering')
        human_exit_mean = self._flow_interval_mean('human', 'exiting')
        for seq, event_time in enumerate(self._cumulative_event_times(
                self.spawn_entering, human_enter_mean, start=0.0,
                actor_class='human', event_type='entering')):
            hidden = self._hidden_intent()
            events.append({
                'event_id': 'enter_%03d' % seq,
                'time': float(event_time),
                'event_type': 'entering',
                'source': 'long_horizon_flow',
                'spot_index': None,
                'intent_observable': not hidden,
                'ground_truth_intent': 'enter_and_park',
                'preference_seed': int(np.random.randint(0, 2 ** 31 - 1)),
                'attempts': 0,
            })
        for seq, event_time in enumerate(self._cumulative_event_times(
                self.spawn_exiting, human_exit_mean, start=0.0,
                actor_class='human', event_type='exiting')):
            hidden = self._hidden_intent()
            events.append({
                'event_id': 'exit_%03d' % seq,
                'time': float(event_time),
                'event_type': 'exiting',
                'source': 'long_horizon_flow',
                'spot_index': None,
                'intent_observable': not hidden,
                'ground_truth_intent': 'leave_from_occupied_spot',
                'preference_seed': int(np.random.randint(0, 2 ** 31 - 1)),
                'attempts': 0,
            })
        self._append_av_demand_events(events)
        events.sort(key=lambda row: (float(row.get('time', 0.0)), str(row.get('event_id', ''))))
        if horizon > 0.0:
            events = [event for event in events if float(event.get('time', 0.0)) <= horizon]
        self.traffic_schedule = events
        self.next_traffic_event_idx = 0
        self._write_traffic_schedule()
        self.get_logger().info('Scheduled human-mixed traffic events=%d hidden_intents=%d' % (
            len(events), sum(1 for event in events if not event.get('intent_observable', True))))

    def _append_av_demand_events(self, events):
        flows = (
            ('entering', self.av_spawn_entering, self._flow_interval_mean('av', 'entering'), 'av_enter', self.av_entry_vehicle_agent_type, self.av_entry_vehicle_role),
            ('exiting', self.av_spawn_exiting, self._flow_interval_mean('av', 'exiting'), 'av_exit', self.av_exit_vehicle_agent_type, self.av_exit_vehicle_role),
        )
        for event_type, count, interval, prefix, agent_type, vehicle_role in flows:
            for seq, event_time in enumerate(self._cumulative_event_times(
                    count, interval, start=0.0,
                    actor_class='av', event_type=event_type)):
                events.append({
                    'event_id': '%s_%03d' % (prefix, seq),
                    'time': float(event_time),
                    'event_type': event_type,
                    'source': 'automated_vehicle_demand',
                    'actor_class': 'av',
                    'agent_type': str(agent_type),
                    'vehicle_role': str(vehicle_role),
                    'spot_index': None,
                    'intent_observable': True,
                    'ground_truth_intent': 'av_%s_task' % event_type,
                    'preference_seed': int(np.random.randint(0, 2 ** 31 - 1)),
                    'attempts': 0,
                })

    def _expire_exit_reservations(self, current_time):
        expired = []
        for spot, release_time in list(self.exit_reserved_spots.items()):
            if current_time >= release_time and (spot >= len(self.occupied) or not self.occupied[spot]):
                expired.append(spot)
        for spot in expired:
            del self.exit_reserved_spots[spot]

    def _empty_spots_for_entry(self):
        blocked = set(int(idx) for idx in list(self.blocked_spots))
        reserved = set(int(idx) for idx in self.exit_reserved_spots.keys())
        return [idx for idx in range(1, len(self.occupied)) if not self.occupied[idx] and idx not in blocked and idx not in reserved]

    def _departable_spots_for_exit(self):
        blocked = set(int(idx) for idx in list(self.blocked_spots))
        reserved = set(int(idx) for idx in self.exit_reserved_spots.keys())
        return [idx for idx in range(1, len(self.occupied)) if self.occupied[idx] and idx not in blocked and idx not in reserved]

    @staticmethod
    def _event_preferred_spot(candidates, event):
        candidates = sorted(int(value) for value in candidates)
        if not candidates:
            return None
        seed = _safe_int(event.get('preference_seed'), 0)
        generator = np.random.RandomState(seed)
        ordering = generator.permutation(np.asarray(candidates, dtype=int))
        return int(ordering[0])

    def _delay_traffic_event(self, event, reason):
        event['_done'] = True
        delayed = dict(event)
        delayed.pop('_done', None)
        delayed['attempts'] = _safe_int(delayed.get('attempts'), 0) + 1
        delayed['time'] = round(self._traffic_time() + max(0.1, _safe_float(self.delayed_spawn_retry_seconds, 2.0)), 3)
        self._append_traffic_event(delayed, 'delayed', reason=reason)
        self.traffic_schedule.append(delayed)
        self.traffic_schedule.sort(key=lambda row: (float(row.get('time', 0.0)), str(row.get('event_id', ''))))

    def _spawn_scheduled_entering(self, event, current_time):
        if not self.keep_spawn_entering:
            self._delay_traffic_event(event, 'entrance_occupied')
            return
        empty_spots = self._empty_spots_for_entry()
        if not empty_spots:
            event['_done'] = True
            self._append_traffic_event(event, 'skipped', reason='no_selectable_empty_spot')
            return
        chosen_spot = self._event_preferred_spot(empty_spots, event)
        vehicle_id = self.add_vehicle(
            chosen_spot,
            agent_type=str(event.get('agent_type') or self.entry_vehicle_agent_type),
            vehicle_role=str(event.get('vehicle_role') or self.entry_vehicle_role) or _traffic_vehicle_role('entering', event.get('agent_type') or self.entry_vehicle_agent_type),
            intent_observable=bool(event.get('intent_observable', True)),
            intent_label='enter:spot_%d' % chosen_spot,
            spawn_event_id=str(event.get('event_id', '')),
        )
        self.occupied[chosen_spot] = True
        self._track_entrance_vehicle(current_time, vehicle_id)
        event['_done'] = True
        self._append_traffic_event(event, 'spawned', vehicle_id=vehicle_id, spot_index=chosen_spot)

    def _spawn_scheduled_exiting(self, event, current_time):
        requested = event.get('spot_index')
        chosen_spot = None
        if requested is not None:
            requested = int(requested)
            if 0 <= requested < len(self.occupied) and self.occupied[requested] and requested not in self.exit_reserved_spots and requested not in set(self.blocked_spots):
                chosen_spot = requested
        if chosen_spot is None:
            departable = self._departable_spots_for_exit()
            if not departable:
                event['_done'] = True
                self._append_traffic_event(event, 'skipped', reason='no_departable_occupied_spot')
                return
            chosen_spot = self._event_preferred_spot(departable, event)
        self.exit_reserved_spots[chosen_spot] = current_time + max(0.0, _safe_float(self.exit_spot_reuse_delay, 20.0))
        vehicle_id = self.add_vehicle(
            -1 * chosen_spot,
            agent_type=str(event.get('agent_type') or self.exit_vehicle_agent_type),
            vehicle_role=str(event.get('vehicle_role') or self.exit_vehicle_role) or _traffic_vehicle_role('exiting', event.get('agent_type') or self.exit_vehicle_agent_type),
            intent_observable=bool(event.get('intent_observable', True)),
            intent_label='exit:spot_%d' % chosen_spot,
            spawn_event_id=str(event.get('event_id', '')),
        )
        self.last_exit_time = current_time
        event['_done'] = True
        self._append_traffic_event(event, 'spawned', vehicle_id=vehicle_id, spot_index=chosen_spot)

    def try_spawn_scheduled_traffic(self):
        current_time = self._traffic_time()
        self._expire_exit_reservations(current_time)
        ready = []
        while self.next_traffic_event_idx < len(self.traffic_schedule):
            event = self.traffic_schedule[self.next_traffic_event_idx]
            if event.get('_done'):
                self.next_traffic_event_idx += 1
                continue
            if float(event.get('time', 0.0)) > current_time:
                break
            ready.append(event)
            self.next_traffic_event_idx += 1
        if not ready:
            return
        max_active = max(1, _safe_int(self.max_concurrent_background_vehicles, 80))
        for event in ready:
            if self._active_vehicle_count() >= max_active:
                self._delay_traffic_event(event, 'max_concurrent_background_vehicles')
                continue
            event_type = str(event.get('event_type', '')).lower()
            if event_type == 'entering':
                self._spawn_scheduled_entering(event, current_time)
            elif event_type == 'exiting':
                self._spawn_scheduled_exiting(event, current_time)
            else:
                event['_done'] = True
                self._append_traffic_event(event, 'skipped', reason='unknown_event_type')

    def last_enter_cb(self, msg):
        self.unpack_msg(msg, self.last_enter_state)

        # If vehicle left entrance area, start spawning another one
        if self.last_enter_state.x.y < self.y_bound_to_resume_spawning:
            self.keep_spawn_entering = True
            self.get_logger().info("Vehicle %d left the entrance area." % self.last_enter_id)

    def _track_entrance_vehicle(self, current_time, vehicle_id=None):
        self.last_enter_time = current_time
        self.last_enter_id = self.num_vehicles if vehicle_id is None else int(vehicle_id)
        self.last_enter_sub = self.create_subscription(VehicleStateMsg, '/vehicle_%d/state' % self.last_enter_id, self.last_enter_cb, 10)
        self.keep_spawn_entering = False

    def try_spawn_entering(self):
        current_time = self._traffic_time()

        if self.spawn_entering_time and current_time - self.last_enter_time > self.spawn_entering_time[0]:
            empty_spots = [i for i in range(len(self.occupied)) if not self.occupied[i]]
            chosen_spot = np.random.choice(empty_spots)
            vehicle_id = self.add_vehicle(chosen_spot, vehicle_role='rule_entering', intent_observable=True, intent_label='legacy_enter:spot_%d' % chosen_spot) # pick from empty spots randomly
            self.occupied[chosen_spot] = True
            self.spawn_entering_time.pop(0)

            self._track_entrance_vehicle(current_time, vehicle_id)

    def try_spawn_exiting(self):
        current_time = self._traffic_time()

        if self.spawn_exiting_time and current_time - self.last_exit_time > self.spawn_exiting_time[0]:
            departable_spots = [i for i in range(1, len(self.occupied)) if self.occupied[i] and i not in self.blocked_spots]
            if not departable_spots:
                return
            chosen_spot = int(np.random.choice(departable_spots))
            self.add_vehicle(-1 * chosen_spot, vehicle_role='rule_exiting', intent_observable=True, intent_label='legacy_exit:spot_%d' % chosen_spot)
            self.spawn_exiting_time.pop(0)

            self.last_exit_time = current_time

    def try_spawn_existing(self):
        current_time = self._traffic_time()
        added_vehicles = []

        for agent in self.agents_dict:
            if self.agents_dict[agent]["init_time"] < current_time:
                self.add_existing_vehicle(agent)
                added_vehicles.append(agent)

        for added in added_vehicles:
            del self.agents_dict[added]

    def try_spawn_qwen_ego(self):
        current_time = self._traffic_time()
        legacy_qwen = bool(self.spawn_qwen_ego)
        controlled = bool(self.spawn_controlled_ego)
        if not (legacy_qwen or controlled) or self.qwen_ego_spawned:
            return
        spawn_time = float(self.controlled_ego_spawn_time if controlled else self.qwen_ego_spawn_time)
        if current_time < spawn_time:
            return
        spot_index = int(self.controlled_ego_spot_index if controlled else self.qwen_ego_spot_index)
        agent_type = str(self.controlled_ego_agent_type if controlled else 'qwen_vla').lower()
        vehicle_id = self.add_vehicle(spot_index, agent_type=agent_type, is_controlled_ego=bool(legacy_qwen or controlled), vehicle_role='controlled_ego', intent_observable=True, intent_label='controlled:spot_%d' % spot_index, spawn_event_id='controlled_ego')
        if spot_index > 0 and bool(self.controlled_ego_blocks_entrance):
            self._track_entrance_vehicle(current_time, vehicle_id)
        self.qwen_ego_spawned = True

    def timer_callback(self):

        if self.sim_is_running and not self.fleet_paused:
            self.sim_time = round(float(self.sim_time) + float(self.timer_period), 9)
            self.try_spawn_qwen_ego()
            mode = str(self.background_mode).lower()
            if mode not in ['replay', 'rule_random', 'mixed', 'human_mixed', 'human_mixed_long_horizon']:
                mode = 'replay' if self.use_existing_agents else 'rule_random'
            if self._uses_scheduled_traffic():
                if self.last_enter_sub and self.keep_spawn_entering:
                    self.destroy_subscription(self.last_enter_sub)
                    self.last_enter_sub = None
                self.try_spawn_scheduled_traffic()
            elif mode in ['rule_random', 'mixed']:
                if self.keep_spawn_entering:
                    if self.last_enter_sub:
                        self.destroy_subscription(self.last_enter_sub)
                        self.last_enter_sub = None

                    self.try_spawn_entering()

                self.try_spawn_exiting()
            if mode in ['replay', 'mixed', 'human_mixed', 'human_mixed_long_horizon']:
                self.try_spawn_existing()

        # Publish current simulation time
        time_msg = Float32()
        time_msg.data = float(self.sim_time)
        self.sim_time_pub.publish(time_msg)

        occupancy_msg = Int16MultiArray()
        occupancy_msg.data = self.occupied
        self.occupancy_pub.publish(occupancy_msg)

        if (
            _as_bool(self.hard_stop_at_long_horizon)
            and self._uses_scheduled_traffic()
            and float(self.long_horizon_duration) > 0.0
            and float(self.sim_time) + 1e-9 >= float(self.long_horizon_duration)
        ):
            self.get_logger().info(
                'Hard simulation-time horizon reached at %.3f s' % float(self.sim_time))
            raise KeyboardInterrupt()


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt()


def main(args=None):
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    rclpy.init(args=args)

    simulator = SimulatorNode()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        print('Simulation is terminated')
    except:
        print('Unknown exception')
        traceback.print_exc()
    finally:
        simulator.shutdown_vehicles()
        simulator.destroy_node()
        print('Simulation stopped cleanly')

        rclpy.shutdown()

if __name__ == "__main__":
    main()
