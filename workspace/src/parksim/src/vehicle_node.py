#!/usr/bin/env python3

import json
import re
import traceback
from parksim.controller.stanley_controller import StanleyController

from parksim.controller_types import StanleyParams

import rclpy
from rclpy.handle import InvalidHandle

from pathlib import Path
import os
import numpy as np
import pickle
from std_msgs.msg import Int16MultiArray, Bool, Float32, String
from parksim.msg import VehicleStateMsg, VehicleInfoMsg
from parksim.srv import OccupancySrv
from parksim.pytypes import VehicleState, NodeParamTemplate
from parksim.vehicle_types import VehicleBody, VehicleConfig, VehicleInfo, VehicleTask
from parksim.base_node import MPClabNode, parksim_path
from parksim.agents.rule_based_stanley_vehicle import RuleBasedStanleyVehicle

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)


VLA_BASELINE_AGENT_TYPES = ('greedy_nearest', 'greedy_shortest_path', 'risk_aware_rule', 'bundle_risk_aware', 'conflict_aware_bundle', 'min_bundle_cost', 'reservation_bundle', 'rolling_horizon_bundle', 'centralized_min_cost', 'oracle_intent_bundle', 'vla_baseline')
MLLM_FLEET_AGENT_TYPES = ('qwen_vla', 'mllm_direct', 'mllm_self_reflect', 'mllm_external_feedback', 'fleet_min_cost')


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _as_value(value, default):
    if value is None:
        return default
    if isinstance(value, str) and value == '':
        return default
    return value


class VehicleNodeParams(NodeParamTemplate):
    """
    template that stores all parameters needed for the node as well as default values
    """
    def __init__(self):
        self.timer_period = 0.1
        self.warm_start_time = 0.2

        self.random_seed =0

        self.entrance_coords = [14.38, 76.21]

        self.spots_data_path = parksim_path('python', 'parksim', 'priorFiles', 'spots_data.pickle')
        self.offline_maneuver_path = parksim_path('python', 'parksim', 'priorFiles', 'parking_maneuvers.pickle')
        self.waypoints_graph_path = parksim_path('python', 'parksim', 'priorFiles', 'waypoints_graph.pickle')
        self.intent_model_path = parksim_path('python', 'parksim', 'priorFiles', 'model', 'smallRegularizedCNN_L0.068_01-29-2022_19-50-35.pth')

        self.use_existing_agents = False
        self.agents_data_path = parksim_path('python', 'parksim', 'priorFiles', 'agents_data_0012.pickle')

        self.agent_type = 'rule_based'
        self.is_controlled_ego = False
        self.vehicle_role = 'background'
        self.intent_observable = True
        self.intent_label = ''
        self.spawn_event_id = ''
        self.reveal_background_intents_to_vla = False
        self.rl_policy_path = ''
        self.rl_max_steps = 1000

        self.qwen_endpoint = ''
        self.qwen_model = 'Qwen2.5-VL-7B-Instruct'
        self.qwen_timeout = 15.0
        self.qwen_decision_period = 3.0
        self.qwen_max_candidate_spots = 8
        self.qwen_periodic_replan = False
        self.qwen_decision_log_path = parksim_path('vehicle_log', 'qwen_vla_decisions.jsonl')
        self.vla_baseline_strategy = 'risk_aware_rule'
        self.fleet_coordinator_enabled = False
        self.fleet_run_id = ''

        self.write_log = True
        self.log_path = parksim_path('vehicle_log')
        self.trace_log_enabled = True
        self.trace_log_path = ''
        self.summary_log_path = ''

class VehicleNode(MPClabNode):
    """
    Node for rule based stanley vehicle
    """
    def __init__(self):
        """
        init
        """
        super().__init__('vehicle')
        self.get_logger().info('Initializing Vehicle...')
        namespace = self.get_namespace()

        # ======== Parameters
        param_template = VehicleNodeParams()
        self.autodeclare_parameters(param_template, namespace)
        self.autoload_parameters(param_template, namespace)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.declare_parameter('vehicle_id', 0)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

        self.declare_parameter('spot_index', 0)
        self.spot_index = self.get_parameter('spot_index').get_parameter_value().integer_value

        self._load_launch_overrides()

        self.get_logger().info("Spot Index: " + str(self.spot_index))
        self.get_logger().info("Agent Type: " + str(self.agent_type))

        # ======== Publishers, Subscribers, Services
        self.state_pub = self.create_publisher(VehicleStateMsg, 'state', 10)
        self.info_pub = self.create_publisher(VehicleInfoMsg, 'info', 10)
        self.operation_pub = self.create_publisher(String, 'declared_operation', 10)
        self.sim_time = 0.0
        self.have_sim_time = False
        self.last_sim_time = None
        self.fleet_paused = False
        self._fleet_mode_active = False
        self.fleet_registry_pub = self.create_publisher(String, '/vla/fleet_registry', 10)
        self.fleet_context_pub = self.create_publisher(String, '/vla/fleet_context', 10)
        self.fleet_decision_ack_pub = self.create_publisher(String, '/vla/fleet_decision_ack', 10)
        self.sim_time_sub = self.create_subscription(Float32, '/sim_time', self.sim_time_cb, 10)
        self.fleet_pause_sub = self.create_subscription(Bool, '/vla/fleet_pause', self.fleet_pause_cb, 10)
        self.fleet_epoch_sub = self.create_subscription(String, '/vla/fleet_epoch', self.fleet_epoch_cb, 10)
        self.fleet_decisions_sub = self.create_subscription(String, '/vla/fleet_decisions', self.fleet_decisions_cb, 10)

        self.sim_status_sub = self.create_subscription(Bool, '/sim_status', self.sim_status_cb, 10)
        self.sim_is_running = True

        self.state_subs = {}
        self.info_subs = {}
        self.operation_subs = {}
        self.occupancy_sub = self.create_subscription(Int16MultiArray, '/occupancy', self.occupancy_cb, 10)

        self.occupancy_cli = self.create_client(OccupancySrv, '/occupancy')
        while not self.occupancy_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service not available, waiting again...')

        vehicle_body = VehicleBody()

        if self.use_existing_agents:
            # agents = pickle.load(open(str(Path.home()) + self.agents_data_path, "rb"))

            # Yccc7: path changed
            agents = pickle.load(open(self.agents_data_path, "rb"))

            agent_dict = agents[self.vehicle_id]

            vehicle_body.w = agent_dict["width"]
            vehicle_body.l = agent_dict["length"]

        vehicle_config = VehicleConfig()

        agent_type = str(self.agent_type).lower()
        self._fleet_mode_active = _as_bool(self.fleet_coordinator_enabled) and agent_type in MLLM_FLEET_AGENT_TYPES
        if agent_type in MLLM_FLEET_AGENT_TYPES:
            from parksim.vla.agent import QwenVLAVehicle

            self.vehicle = QwenVLAVehicle(
                vehicle_id=self.vehicle_id,
                vehicle_body=vehicle_body,
                vehicle_config=vehicle_config,
                qwen_endpoint=str(self.qwen_endpoint),
                qwen_model=str(self.qwen_model),
                qwen_timeout=float(self.qwen_timeout),
                decision_period=float(self.qwen_decision_period),
                max_candidate_spots=int(self.qwen_max_candidate_spots),
                entrance_coords=np.array(self.entrance_coords),
                fallback_spot_index=self.spot_index if self.spot_index > 0 else None,
                periodic_replan=_as_bool(self.qwen_periodic_replan),
                decision_log_path=str(self.qwen_decision_log_path),
                reveal_background_intents_to_vla=_as_bool(self.reveal_background_intents_to_vla),
                fleet_coordinator_enabled=self._fleet_mode_active,
            )
        elif agent_type in VLA_BASELINE_AGENT_TYPES:
            from parksim.vla.agent import BaselineVLAVehicle

            baseline_strategy = agent_type if agent_type != 'vla_baseline' else str(self.vla_baseline_strategy).lower()
            self.vehicle = BaselineVLAVehicle(
                vehicle_id=self.vehicle_id,
                vehicle_body=vehicle_body,
                vehicle_config=vehicle_config,
                baseline_strategy=baseline_strategy,
                decision_period=float(self.qwen_decision_period),
                max_candidate_spots=int(self.qwen_max_candidate_spots),
                entrance_coords=np.array(self.entrance_coords),
                fallback_spot_index=self.spot_index if self.spot_index > 0 else None,
                periodic_replan=_as_bool(self.qwen_periodic_replan),
                decision_log_path=str(self.qwen_decision_log_path),
                reveal_background_intents_to_vla=_as_bool(self.reveal_background_intents_to_vla),
            )
        elif agent_type == 'rl_policy':
            from parksim.rl.agents import RLPolicyAgent

            policy_path = self.rl_policy_path or os.environ.get('PARKSIM_POLICY_PATH', '')
            self.vehicle = RLPolicyAgent(
                vehicle_id=self.vehicle_id,
                vehicle_body=vehicle_body,
                vehicle_config=vehicle_config,
                policy_path=policy_path,
                max_steps=int(self.rl_max_steps),
            )
        elif agent_type == 'rule_based':
            controller_params = StanleyParams(dt=self.timer_period)
            controller = StanleyController(control_params=controller_params, vehicle_body=vehicle_body, vehicle_config=vehicle_config)
            motion_predictor = StanleyController(control_params=controller_params, vehicle_body=vehicle_body, vehicle_config=vehicle_config)

            self.vehicle = RuleBasedStanleyVehicle(
                vehicle_id=self.vehicle_id,
                vehicle_body=vehicle_body,
                vehicle_config=vehicle_config,
                controller=controller,
                motion_predictor=motion_predictor,
                inst_centric_generator=None,
                intent_predictor=None
                )
        else:
            raise ValueError("Unsupported agent_type '%s'. Use a supported rule, MLLM fleet, or VLA-compatible baseline agent type." % self.agent_type)

        self.vehicle.vehicle_role = str(self.vehicle_role)
        self.vehicle.intent_observable = _as_bool(self.intent_observable)
        self.vehicle.intent_label = str(self.intent_label)
        self.vehicle.spawn_event_id = str(self.spawn_event_id)

        self.vehicle.set_printer(self.get_logger().info)
        self.vehicle.load_parking_spaces(spots_data_path=self.spots_data_path)
        self.vehicle.load_graph(waypoints_graph_path=self.waypoints_graph_path)
        self.vehicle.load_maneuver(offline_maneuver_path=self.offline_maneuver_path)

        self.vehicle.set_method_to_change_central_occupancy(self.change_occupancy)
        task_profile = []

        if not self.use_existing_agents:
            if self.spot_index > 0:
                if agent_type in MLLM_FLEET_AGENT_TYPES or agent_type in VLA_BASELINE_AGENT_TYPES:
                    task_profile = []
                else:
                    cruise_task = VehicleTask(
                        name="CRUISE", v_cruise=5, target_spot_index=self.spot_index)
                    park_task = VehicleTask(name="PARK", target_spot_index=self.spot_index)
                    task_profile = [cruise_task, park_task]

                state = VehicleState()
                state.x.x = self.entrance_coords[0] - vehicle_config.offset
                state.x.y = self.entrance_coords[1]
                state.e.psi = - np.pi/2

                self.vehicle.set_vehicle_state(state=state)
            else:
                unpark_task = VehicleTask(name="UNPARK")
                cruise_task = VehicleTask(
                    name="CRUISE", v_cruise=5, target_coords=np.array(self.entrance_coords))
                task_profile = [unpark_task, cruise_task]

                self.vehicle.set_vehicle_state(spot_index=abs(self.spot_index))
        else:
            raw_tp = agent_dict["task_profile"]
            for task in raw_tp:
                if task["name"] == "IDLE":
                    task_profile.append(VehicleTask(name="IDLE", duration=task["duration"]))
                elif task["name"] == "PARK":
                    task_profile.append(VehicleTask(name="PARK", target_spot_index=task["target_spot_index"]))
                elif task["name"] == "UNPARK":
                    task_profile.append(VehicleTask(name="UNPARK", target_spot_index=task["target_spot_index"]))
                elif task["name"] == "CRUISE":
                    if "target_coords" in task:
                        task_profile.append(VehicleTask(name="CRUISE", v_cruise=task["v_cruise"], target_coords=task["target_coords"]))
                    else:
                        task_profile.append(VehicleTask(name="CRUISE", v_cruise=task["v_cruise"], target_spot_index=task["target_spot_index"]))

            if "init_spot" in agent_dict:
                init_spot = agent_dict["init_spot"]
                init_heading = agent_dict["init_heading"]
                self.vehicle.set_vehicle_state(spot_index=init_spot, heading=init_heading)
            else:
                agent_state = VehicleState()
                agent_state.x.x = agent_dict["init_coords"][0]
                agent_state.x.y = agent_dict["init_coords"][1]
                agent_state.e.psi = agent_dict["init_heading"]
                agent_state.v.v = agent_dict["init_v"]
                self.vehicle.set_vehicle_state(state=agent_state)

        self.vehicle.set_task_profile(task_profile=task_profile)

        role = str(self.vehicle_role or '').lower()
        task_names = [str(getattr(task, 'name', task)).upper() for task in task_profile]
        if 'enter' in role or 'PARK' in task_names:
            self.declared_operation = 'entering'
        elif 'exit' in role or 'depart' in role or 'UNPARK' in task_names:
            self.declared_operation = 'exiting'
        else:
            self.declared_operation = 'unknown'

        self.vehicle.execute_next_task()

        self.start_time = self.get_ros_time()
        self.start_solving = False
        self.last_sim_time = None
        self.total_non_idle_time = 0
        if self._fleet_mode_active:
            self._publish_fleet_registry('register')

    def _get_plain_launch_parameter(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return _as_value(self.get_parameter(name).value, default)

    def _load_launch_overrides(self):
        # The node is launched under /vehicle_<id>. Template parameters are
        # declared with that namespace, while launch arguments arrive as plain
        # node parameters. Read the plain overrides explicitly so agent_type and
        # Qwen settings are not silently left at template defaults.
        self.use_existing_agents = _as_bool(self._get_plain_launch_parameter('use_existing_agents', self.use_existing_agents))
        self.use_existing_agents = _as_bool(self._get_plain_launch_parameter('use_existing', self.use_existing_agents))
        for name in (
            'agent_type',
            'is_controlled_ego',
            'vehicle_role',
            'intent_observable',
            'intent_label',
            'spawn_event_id',
            'reveal_background_intents_to_vla',
            'rl_policy_path',
            'rl_max_steps',
            'qwen_endpoint',
            'qwen_model',
            'qwen_timeout',
            'qwen_decision_period',
            'qwen_max_candidate_spots',
            'qwen_periodic_replan',
            'qwen_decision_log_path',
            'vla_baseline_strategy',
            'fleet_coordinator_enabled',
            'fleet_run_id',
            'log_path',
            'trace_log_enabled',
            'trace_log_path',
            'summary_log_path',
        ):
            object.__setattr__(self, name, self._get_plain_launch_parameter(name, getattr(self, name)))
        self.trace_log_enabled = _as_bool(self.trace_log_enabled)
        self.is_controlled_ego = _as_bool(self.is_controlled_ego)
        self.intent_observable = _as_bool(self.intent_observable)
        self.reveal_background_intents_to_vla = _as_bool(self.reveal_background_intents_to_vla)
        self.fleet_coordinator_enabled = _as_bool(self.fleet_coordinator_enabled)

    def _default_trace_log_path(self):
        return os.path.join(self.log_path, "vehicle_%d_trace.jsonl" % self.vehicle_id)

    def _default_summary_log_path(self):
        return os.path.join(self.log_path, "vehicle_%d_summary.json" % self.vehicle_id)

    def _append_trace_record(self, sim_time, wall_time=None, final=False, censored=False):
        wall_time = self.get_ros_time() if wall_time is None else float(wall_time)
        if not self.write_log or not self.trace_log_enabled:
            return
        log_dir_path = self.log_path
        if not os.path.exists(log_dir_path):
            os.makedirs(log_dir_path, exist_ok=True)
        state = self.vehicle.state
        record = {
            "sim_time": float(sim_time),
            "time": float(sim_time),
            "wall_time": float(wall_time),
            "vehicle_id": int(self.vehicle_id),
            "agent_type": str(self.agent_type),
            "is_controlled_ego": bool(self.is_controlled_ego),
            "vehicle_role": str(self.vehicle_role),
            "intent_observable": bool(self.intent_observable),
            "intent_label": str(self.intent_label),
            "spawn_event_id": str(self.spawn_event_id),
            "spot_index": int(self.spot_index),
            "task": self.vehicle.current_task,
            "is_final": bool(final),
            "censored": bool(censored),
            "total_non_idle_time": float(self.total_non_idle_time),
            "x": float(state.x.x),
            "y": float(state.x.y),
            "yaw": float(state.e.psi),
            "speed": float(state.v.v),
            "acceleration": float(state.u.u_a),
            "steering": float(state.u.u_steer),
            "is_braking": bool(getattr(self.vehicle, "is_braking", False)),
            "waiting_for": int(getattr(self.vehicle, "waiting_for", 0) or 0),
            "deadlock_release_count": int(getattr(self.vehicle, "deadlock_release_count", 0) or 0),
            "target_idx": int(getattr(self.vehicle, "target_idx", 0) or 0),
            "vehicle_spot_index": int(getattr(self.vehicle, "spot_index", 0) or 0),
        }
        trace_path = self.trace_log_path or self._default_trace_log_path()
        os.makedirs(os.path.dirname(trace_path), exist_ok=True)
        with open(trace_path, 'a') as f:
            f.write(json.dumps(record) + "\n")

    def _write_summary_record(self, sim_time, censor_reason=""):
        if not self.write_log:
            return
        log_dir_path = self.log_path
        if not os.path.exists(log_dir_path):
            os.makedirs(log_dir_path, exist_ok=True)
        state = self.vehicle.state
        completed = bool(self.vehicle.is_all_done())
        summary = {
            "vehicle_id": int(self.vehicle_id),
            "agent_type": str(self.agent_type),
            "is_controlled_ego": bool(self.is_controlled_ego),
            "vehicle_role": str(self.vehicle_role),
            "intent_observable": bool(self.intent_observable),
            "intent_label": str(self.intent_label),
            "spawn_event_id": str(self.spawn_event_id),
            "spot_index": int(self.spot_index),
            "vehicle_spot_index": int(getattr(self.vehicle, "spot_index", 0) or 0),
            "completed": completed,
            "censored": bool(censor_reason and not completed),
            "censor_reason": str(censor_reason if not completed else ""),
            "total_time": float(sim_time),
            "total_non_idle_time": float(self.total_non_idle_time),
            "deadlock_release_count": int(getattr(self.vehicle, "deadlock_release_count", 0) or 0),
            "final_task": self.vehicle.current_task,
            "final_state": {
                "x": float(state.x.x),
                "y": float(state.x.y),
                "yaw": float(state.e.psi),
                "speed": float(state.v.v),
            },
        }
        summary_path = self.summary_log_path or self._default_summary_log_path()
        os.makedirs(os.path.dirname(summary_path), exist_ok=True)
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)

    def sim_status_cb(self, msg: Bool):
        self.sim_is_running = msg.data

    def sim_time_cb(self, msg):
        self.sim_time = float(msg.data)
        self.have_sim_time = True

    def fleet_pause_cb(self, msg):
        self.fleet_paused = bool(msg.data)

    def _publish_fleet_registry(self, event):
        if not self._fleet_mode_active:
            return
        message = String()
        message.data = json.dumps({
            'run_id': str(self.fleet_run_id),
            'event': str(event),
            'vehicle_id': int(self.vehicle_id),
        })
        self.fleet_registry_pub.publish(message)

    def fleet_epoch_cb(self, msg):
        if not self._fleet_mode_active:
            return
        try:
            payload = json.loads(msg.data)
            epoch_id = int(payload.get('epoch_id'))
            sim_time = float(payload.get('sim_time'))
        except (TypeError, ValueError):
            return
        run_id = str(payload.get('run_id', ''))
        if run_id and self.fleet_run_id and run_id != str(self.fleet_run_id):
            return
        expected = [int(item) for item in payload.get('expected_vehicle_ids', [])]
        if expected and int(self.vehicle_id) not in expected:
            return
        context = self.vehicle.build_fleet_epoch_context(epoch_id, sim_time)
        message = String()
        if context is None:
            message.data = json.dumps({
                'run_id': str(self.fleet_run_id),
                'epoch_id': epoch_id,
                'sim_time': sim_time,
                'vehicle_id': int(self.vehicle_id),
                'ready': False,
                'defer_reason': str(getattr(self.vehicle, '_fleet_defer_reason', '') or 'not_ready'),
            })
        else:
            message.data = json.dumps({
                'run_id': str(self.fleet_run_id),
                'epoch_id': epoch_id,
                'sim_time': sim_time,
                'vehicle_id': int(self.vehicle_id),
                'ready': True,
                'context': context.to_dict(),
            })
        self.fleet_context_pub.publish(message)

    def fleet_decisions_cb(self, msg):
        if not self._fleet_mode_active:
            return
        try:
            payload = json.loads(msg.data)
            epoch_id = int(payload.get('epoch_id'))
            sim_time = float(payload.get('sim_time', self.sim_time))
        except (TypeError, ValueError):
            return
        run_id = str(payload.get('run_id', ''))
        if run_id and self.fleet_run_id and run_id != str(self.fleet_run_id):
            return
        decision = next((
            item for item in payload.get('fleet_decisions', []) or []
            if int(item.get('vehicle_id', -1)) == int(self.vehicle_id)
        ), None)
        if decision is None:
            return
        applied = False
        error = ''
        try:
            applied = bool(self.vehicle.apply_fleet_epoch_decision(epoch_id, decision, sim_time))
        except Exception as exc:
            error = '%s: %s' % (type(exc).__name__, exc)
        ack = String()
        ack.data = json.dumps({
            'run_id': str(self.fleet_run_id),
            'epoch_id': int(epoch_id),
            'vehicle_id': int(self.vehicle_id),
            'decision_sim_time': float(sim_time),
            'vehicle_sim_time': float(self.sim_time),
            'action_id': str(decision.get('action_id', '')),
            'applied': bool(applied),
            'error': error,
        })
        self.fleet_decision_ack_pub.publish(ack)

    def vehicle_state_cb(self, vehicle_id):
        def callback(msg):
            state = VehicleState()
            self.unpack_msg(msg, state)
            self.vehicle.other_state[vehicle_id] = state

            if vehicle_id in self.vehicle.other_is_braking:
                # Add vehicle only when we both have state and info available
                self.vehicle.other_vehicles.add(vehicle_id)

        return callback

    def vehicle_info_cb(self, vehicle_id):
        def callback(msg):
            info = VehicleInfo()
            self.unpack_msg(msg, info)

            self.vehicle.other_ref_pose[vehicle_id] = info.ref_pose
            self.vehicle.other_ref_v[vehicle_id] = info.ref_v
            self.vehicle.other_target_idx[vehicle_id] = info.target_idx
            self.vehicle.other_priority[vehicle_id] = info.priority
            self.vehicle.other_task[vehicle_id] = info.task
            self.vehicle.other_parking_progress[vehicle_id] = info.parking_progress
            self.vehicle.other_is_braking[vehicle_id] = info.is_braking
            self.vehicle.other_parking_start_time[vehicle_id] = info.parking_start_time
            self.vehicle.other_waiting_for[vehicle_id] = info.waiting_for
            self.vehicle.other_is_all_done[vehicle_id] = info.is_all_done

            if vehicle_id in self.vehicle.other_state:
                # Add vehicle only when we both have state and info available
                self.vehicle.other_vehicles.add(vehicle_id)

        return callback

    def declared_operation_cb(self, vehicle_id):
        def callback(msg):
            operation = str(msg.data or 'unknown').lower()
            if operation in ('entering', 'exiting'):
                self.vehicle.other_declared_operation[vehicle_id] = operation
            else:
                self.vehicle.other_declared_operation.setdefault(vehicle_id, 'unknown')

        return callback

    def _declared_operation(self):
        declared = str(getattr(self, 'declared_operation', 'unknown') or 'unknown').lower()
        if declared in ('entering', 'exiting'):
            return declared
        role = str(self.vehicle_role or '').lower()
        if 'enter' in role:
            return 'entering'
        if 'exit' in role or 'depart' in role:
            return 'exiting'
        tasks = [str(getattr(task, 'name', task)).upper() for task in getattr(self.vehicle, 'task_profile', [])]
        current_task = str(getattr(self.vehicle, 'current_task', '') or '').upper()
        if 'PARK' in tasks or current_task == 'PARK':
            return 'entering'
        if 'UNPARK' in tasks or current_task == 'UNPARK':
            return 'exiting'
        return 'unknown'

    def occupancy_cb(self, msg):
        self.vehicle.occupancy = msg.data

    def change_occupancy(self, idx, new_value):
        def response_cb(future):
            res = future.result()
            if res.status:
                self.get_logger().info("Service request from vehicle %d to change occupancy is successful" % self.vehicle_id)

        req = OccupancySrv.Request()
        req.vehicle_id = self.vehicle_id
        req.idx = int(idx)
        req.new_value = int(new_value)

        future = self.occupancy_cli.call_async(req)
        future.add_done_callback(response_cb)


    def update_subs(self):
        topic_list_types = self.get_topic_names_and_types()

        for topic_name, _ in topic_list_types:
            state_name_pattern = re.match("/vehicle_([1-9][0-9]*)/state", topic_name)
            info_name_pattern = re.match("/vehicle_([1-9][0-9]*)/info", topic_name)
            operation_name_pattern = re.match("/vehicle_([1-9][0-9]*)/declared_operation", topic_name)

            if state_name_pattern:
                vehicle_id = int(state_name_pattern.group(1))
                if vehicle_id == self.vehicle_id:
                    continue

                publisher = self.get_publishers_info_by_topic(topic_name=topic_name)

                if vehicle_id not in self.state_subs and publisher:
                    # If there is publisher, but we haven't subscribed to it
                    self.state_subs[vehicle_id] = self.create_subscription(VehicleStateMsg, topic_name, self.vehicle_state_cb(vehicle_id), 10)
                    # self.get_logger().info("State subscriber to vehicle %d is built." % vehicle_id)
                elif vehicle_id in self.state_subs and not publisher:
                    # If we have subscribed to it, but there is no publisher anymore
                    self.destroy_subscription(self.state_subs[vehicle_id])
                    self.state_subs.pop(vehicle_id)
                    # self.get_logger().info("Vehicle %d is not publishing anymore. State subscriber is destroyed." % vehicle_id)

                    self.vehicle.other_vehicles.discard(vehicle_id)


            elif info_name_pattern:
                vehicle_id = int(info_name_pattern.group(1))
                if vehicle_id == self.vehicle_id:
                    continue

                publisher = self.get_publishers_info_by_topic(topic_name=topic_name)

                if vehicle_id not in self.info_subs and publisher:
                    # If there is publisher, but we haven't subscribed to it
                    self.info_subs[vehicle_id] = self.create_subscription(VehicleInfoMsg, topic_name, self.vehicle_info_cb(vehicle_id), 10)
                    # self.get_logger().info("Info subscriber to vehicle %d is built." % vehicle_id)
                elif vehicle_id in self.info_subs and not publisher:
                    # If we have subscribed to it, but there is no publisher anymore
                    self.destroy_subscription(self.info_subs[vehicle_id])
                    self.info_subs.pop(vehicle_id)
                    # self.get_logger().info("Vehicle %d is not publishing anymore. Info ubscriber is destroyed." % vehicle_id)

                    self.vehicle.other_vehicles.discard(vehicle_id)

            elif operation_name_pattern:
                vehicle_id = int(operation_name_pattern.group(1))
                if vehicle_id == self.vehicle_id:
                    continue
                publisher = self.get_publishers_info_by_topic(topic_name=topic_name)
                if vehicle_id not in self.operation_subs and publisher:
                    self.operation_subs[vehicle_id] = self.create_subscription(
                        String, topic_name, self.declared_operation_cb(vehicle_id), 10)
                elif vehicle_id in self.operation_subs and not publisher:
                    self.destroy_subscription(self.operation_subs[vehicle_id])
                    self.operation_subs.pop(vehicle_id)

            else:
                continue

    def timer_callback(self):
        wall_time = self.get_ros_time()
        sim_time = float(self.sim_time)
        if self.vehicle.is_all_done():
            self.get_logger().info("Vehicle %d is done. Destroying node." % self.vehicle_id)
            if self._fleet_mode_active:
                self._publish_fleet_registry('unregister')
            log_dir_path = self.log_path
            if not os.path.exists(log_dir_path):
                os.makedirs(log_dir_path, exist_ok=True)
            self._append_trace_record(sim_time, wall_time=wall_time, final=True)
            self._write_summary_record(sim_time)
            with open(log_dir_path + "/vehicle_%d.log" % self.vehicle_id, 'a') as f:
                f.writelines(str(self.total_non_idle_time))
                self.vehicle.logger.clear()
            self.destroy_node()
            return

        self.update_subs()
        advanced = self.have_sim_time and (
            self.last_sim_time is None or sim_time > float(self.last_sim_time) + 1e-9
        )
        if advanced:
            previous_sim_time = sim_time if self.last_sim_time is None else float(self.last_sim_time)
            elapsed = max(0.0, sim_time - previous_sim_time)
            if self.vehicle.current_task != "IDLE":
                self.total_non_idle_time += elapsed
            if sim_time >= float(self.warm_start_time):
                self.start_solving = True
            if self._fleet_mode_active:
                self._publish_fleet_registry('register')
            if self.sim_is_running and not self.fleet_paused:
                if self.start_solving:
                    self.vehicle.solve(time=sim_time)
                self._append_trace_record(sim_time, wall_time=wall_time)
            self.last_sim_time = sim_time
        elif not self.sim_is_running and self.write_log and len(self.vehicle.logger) > 0:
            log_dir_path = self.log_path
            if not os.path.exists(log_dir_path):
                os.mkdir(log_dir_path)
            with open(log_dir_path + "/vehicle_%d.log" % self.vehicle_id, 'a') as f:
                f.writelines('\n'.join(self.vehicle.logger))
                self.vehicle.logger.clear()

        state_msg = VehicleStateMsg()
        self.populate_msg(state_msg, self.vehicle.state)
        self.state_pub.publish(state_msg)
        info_msg = VehicleInfoMsg()
        self.populate_msg(info_msg, self.vehicle.get_info())
        self.info_pub.publish(info_msg)
        operation_msg = String()
        operation_msg.data = self._declared_operation()
        self.operation_pub.publish(operation_msg)


def main(args=None):
    rclpy.init(args=args)

    vehicle = VehicleNode()

    try:
        rclpy.spin(vehicle)
    except KeyboardInterrupt:
        print("Vehicle %d is terminated." % vehicle.vehicle_id)
    except InvalidHandle as e:
        print(e)
        print("Vehicle %d node is destroyed cleanly." % vehicle.vehicle_id)
    except Exception:
        traceback.print_exc()
        print("Unknown exception")
    finally:
        try:
            summary_path = vehicle.summary_log_path or vehicle._default_summary_log_path()
            if vehicle.write_log and not os.path.exists(summary_path):
                sim_time = float(vehicle.sim_time)
                vehicle._append_trace_record(
                    sim_time,
                    wall_time=vehicle.get_ros_time(),
                    final=False,
                    censored=True,
                )
                vehicle._write_summary_record(sim_time, censor_reason="simulation_horizon_or_shutdown")
            if vehicle._fleet_mode_active:
                vehicle._publish_fleet_registry('unregister')
        except Exception:
            traceback.print_exc()
        try:
            vehicle.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()
