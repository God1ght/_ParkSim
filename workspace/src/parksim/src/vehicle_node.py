#!/usr/bin/env python3

import re
import time
from parksim.controller.stanley_controller import StanleyController

from parksim.controller_types import StanleyParams

import rclpy
from rclpy.handle import InvalidHandle

from pathlib import Path
import os
import numpy as np
import pickle
from std_msgs.msg import Int16MultiArray, Bool, Int16
from parksim.msg import VehicleStateMsg, VehicleInfoMsg
from parksim.srv import OccupancySrv
from parksim.pytypes import VehicleState, NodeParamTemplate
from parksim.vehicle_types import VehicleBody, VehicleConfig, VehicleInfo, VehicleTask
from parksim.base_node import MPClabNode, parksim_path
from parksim.scenario import load_map, MapError
from parksim.agents.rule_based_stanley_vehicle import RuleBasedStanleyVehicle


# --- 门顶行驶方向右偏移推导（2026-09-14，入场/离场终点不再硬编码坐标） ---
_GATE_HEAD_COORD = [14.38, 76.21]     # 门顶顶点254（门洞上端锚点）
_GATE_ENTER_DIR  = (0.0, -1.0)        # 入场行驶方向：门顶254 -> 枢纽255 向场内（-y）


def _right_normal(dx, dy):
    """行驶方向 (dx, dy) 的右侧法向（坐标约定：x右、y上，(0,-1) 为向场内下行）。"""
    return np.array([dy, -dx])


def _gate_derived_enter_exit(gate_head, offset, in_dir=_GATE_ENTER_DIR, out_dir=None):
    """由门顶锚点 + 行驶方向右侧偏移 offset，推导入场起点与离场 CRUISE 终点。

    入场点 = gate_head + offset * right(in_dir)  （入场靠行驶方向右侧）
    离场点 = gate_head + offset * right(out_dir)（反向，近似对称另一侧）
    返回 (enter_pt, exit_pt) 均 [x, y]。
    """
    g = np.asarray(gate_head, float)
    din = np.asarray(in_dir, float)
    din = din / np.linalg.norm(din)
    enter_pt = (g + offset * _right_normal(*din)).tolist()
    dout = -din if out_dir is None else np.asarray(out_dir, float)
    dout = dout / np.linalg.norm(dout)
    exit_pt = (g + offset * _right_normal(*dout)).tolist()
    return enter_pt, exit_pt
# --- end 门顶偏移推导 ---

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

class VehicleNodeParams(NodeParamTemplate):
    """
    template that stores all parameters needed for the node as well as default values
    """
    def __init__(self):
        self.timer_period = 0.1
        self.warm_start_time = 0.2

        self.random_seed =0

        self.entrance_coords = [14.38, 76.21]

        # 多出入口地图（schema v2，有 portals）的指派口；空串 = legacy 门顶推导（DJI）
        self.entry_portal = ''   # P1 | P4
        self.exit_portal = ''    # P1 | P2 | P3

        self.map = 'DJI_0012'    # 地图预设名（config/maps/<name>.yaml）
        self.spots_data_path = parksim_path('python', 'parksim', 'priorFiles', 'spots_data.pickle')
        self.offline_maneuver_path = parksim_path('python', 'parksim', 'priorFiles', 'parking_maneuvers.pickle')
        # 逐泊位绝对轨迹表（方案 D）；留空时由 spots_data_path 同目录推导
        self.per_spot_maneuver_path = ''
        self.waypoints_graph_path = parksim_path('python', 'parksim', 'priorFiles', 'waypoints_graph.pickle')
        self.intent_model_path = parksim_path('python', 'parksim', 'priorFiles', 'model', 'smallRegularizedCNN_L0.068_01-29-2022_19-50-35.pth')

        self.use_existing_agents = False
        self.agents_data_path = parksim_path('python', 'parksim', 'priorFiles', 'agents_data_0012.pickle')

        # 可插拔规划组件（默认 = 现状；可被 launch/scenario 覆盖）
        self.route_planner = 'astar'            # astar | dijkstra | via
        self.ref_path_generator = 'spline'      # spline | linear
        # 'per_spot' = 逐泊位带避障绝对轨迹表（方案 D，地图无该表时显式告警后回退 offline）
        self.maneuver_provider = 'per_spot'     # per_spot | offline | online_rs

        self.write_log = True
        self.log_path = parksim_path('vehicle_log')

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

        self.get_logger().info("Spot Index: " + str(self.spot_index))

        # 地图（map）解析：覆盖数据文件路径（spots / graph / maneuvers / agents）
        try:
            map_cfg = load_map(self.map, parksim_path('workspace', 'src', 'parksim', 'config'))
            if map_cfg.get('spots_data_path'):
                self.spots_data_path = map_cfg['spots_data_path']
            if map_cfg.get('waypoints_graph_path'):
                self.waypoints_graph_path = map_cfg['waypoints_graph_path']
            if map_cfg.get('parking_maneuvers_path'):
                self.offline_maneuver_path = map_cfg['parking_maneuvers_path']
            if map_cfg.get('agents_data_path'):
                self.agents_data_path = map_cfg['agents_data_path']
            if map_cfg.get('per_spot_maneuvers_path'):
                self.per_spot_maneuver_path = map_cfg['per_spot_maneuvers_path']
            self.get_logger().info('Map: %s' % map_cfg.get('name', self.map))
        except MapError as exc:
            self.get_logger().warn('Map 解析失败（%s）；使用 vehicle.yaml 路径' % exc)

        # 逐泊位机动表：默认与 spots_data.pickle 同目录（jth_b1 下有，DJI 下没有 -> 回退并告警）
        if not self.per_spot_maneuver_path:
            self.per_spot_maneuver_path = os.path.join(
                os.path.dirname(self.spots_data_path), 'parking_maneuvers_per_spot.pickle')
        if os.path.exists(self.per_spot_maneuver_path):
            self.get_logger().info('per-spot maneuver table: %s' % self.per_spot_maneuver_path)
        else:
            self.get_logger().warn(
                '[per_spot] 逐泊位机动表不存在：%s —— 将回退 legacy 16 组合离线表'
                '（无避障，JTH 上机动终点会偏离泊位中位数 4.5 m）。'
                % self.per_spot_maneuver_path)

        # ======== Publishers, Subscribers, Services
        self.state_pub = self.create_publisher(VehicleStateMsg, 'state', 10)
        self.info_pub = self.create_publisher(VehicleInfoMsg, 'info', 10)
        self.spot_pub = self.create_publisher(Int16, 'spot', 10)   # 目标车位（+入库 / -出库），供可视化

        self.sim_status_sub = self.create_subscription(Bool, '/sim_status', self.sim_status_cb, 10)
        self.sim_is_running = True

        self.state_subs = {}
        self.info_subs = {}
        self.occupancy_sub = self.create_subscription(Int16MultiArray, '/occupancy', self.occupancy_cb, 10)

        self.occupancy_cli = self.create_client(OccupancySrv, '/occupancy')
        # 有界等待：高负载下 DDS 服务发现可能持续抖动；超过 20s 带病启动，
        # 后续占用写入进入重试队列（不再无限阻塞节点启动）。
        _occ_deadline = time.time() + 20.0
        while not self.occupancy_cli.wait_for_service(timeout_sec=1.0):
            if time.time() > _occ_deadline:
                self.get_logger().warning('occupancy service not discovered in 20s; start anyway (writes will retry)')
                break
            self.get_logger().warning('service not available, waiting again...')
        # 占用写入重试队列（发现抖动/服务未就绪时）；见 _flush_occupancy_retries
        self._occ_retries = []
        self._occ_retry_timer = self.create_timer(0.5, self._flush_occupancy_retries)

        vehicle_body = VehicleBody()

        agent_dict = None
        if self.use_existing_agents:
            # agents = pickle.load(open(str(Path.home()) + self.agents_data_path, "rb"))

            # Yccc7: path changed
            try:
                agents = pickle.load(open(self.agents_data_path, "rb"))
                agent_dict = agents[self.vehicle_id]
                vehicle_body.w = agent_dict["width"]
                vehicle_body.l = agent_dict["length"]
            except (FileNotFoundError, KeyError) as exc:
                self.get_logger().error(
                    'Agent profile unavailable (%s: %s)；车辆将空转' % (type(exc).__name__, exc))

        vehicle_config = VehicleConfig()

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

        self.vehicle.set_printer(self.get_logger().info)
        self.vehicle.set_planning_profile(
            route_planner=self.route_planner,
            ref_path_generator=self.ref_path_generator,
            maneuver_provider=self.maneuver_provider,
        )
        self.vehicle.load_parking_spaces(spots_data_path=self.spots_data_path)
        self.vehicle.load_graph(waypoints_graph_path=self.waypoints_graph_path)
        self.vehicle.load_maneuver(offline_maneuver_path=self.offline_maneuver_path,
                                   per_spot_maneuver_path=self.per_spot_maneuver_path)
        # 入口坐标以地图（graph pickle）为准（否则沿用默认场地的写死值）
        try:
            ec = np.asarray(self.vehicle.entrance_coords, dtype=float)
            self.entrance_coords = [float(ec[0]), float(ec[1])]
            self.get_logger().info('Entrance coords (from map): (%.1f, %.1f)' % (self.entrance_coords[0], self.entrance_coords[1]))
        except Exception as exc:
            self.get_logger().warn('entrance_coords 读取失败（%s）；沿用默认' % exc)

        self.vehicle.set_method_to_change_central_occupancy(self.change_occupancy)
        task_profile = []

        if not self.use_existing_agents:
            if self.spot_index > 0:
                cruise_task = VehicleTask(
                    name="CRUISE", v_cruise=5, target_spot_index=self.spot_index)
                park_task = VehicleTask(name="PARK", target_spot_index=self.spot_index)
                task_profile = [cruise_task, park_task]

                state = VehicleState()
                _p = self._portal_entry()
                if _p is not None:
                    # 真出入口：spawn 取该入口弧链首顶点（车道线已在建图时烘焙，误差 0）
                    state.x.x = float(_p['spawn_pose_rotated_m'][0])
                    state.x.y = float(_p['spawn_pose_rotated_m'][1])
                    state.e.psi = float(_p['heading_rad'])
                    self.get_logger().info(
                        'entry_portal=%s spawn=(%.3f, %.3f) psi=%.4f' % (
                            self.entry_portal, state.x.x, state.x.y, state.e.psi))
                else:
                    # legacy：门顶行驶方向右偏移（DJI / 未指派口）
                    _enter_pt, _exit_pt = _gate_derived_enter_exit(
                        _GATE_HEAD_COORD, vehicle_config.offset)
                    state.x.x = _enter_pt[0]
                    state.x.y = _enter_pt[1]
                    eh = getattr(self.vehicle, 'entrance_heading', None)
                    state.e.psi = float(eh) if eh is not None else - np.pi / 2

                self.vehicle.set_vehicle_state(state=state)
                # (泊位, 入口) 目标顶点：规避烘焙后 graph.search 的病态最近邻
                self._apply_spot_target(self.spot_index)
            else:
                unpark_task = VehicleTask(name="UNPARK")
                _xp = self._portal_exit()
                if _xp is not None:
                    # 真出入口：离场目标取该出口弧链末顶点（命中图顶点，误差 0）
                    _tgt = np.array([float(_xp['exit_target_rotated_m'][0]),
                                     float(_xp['exit_target_rotated_m'][1])])
                    self.get_logger().info(
                        'exit_portal=%s target=(%.3f, %.3f)' % (
                            self.exit_portal, _tgt[0], _tgt[1]))
                    self.vehicle.set_active_exit_target(_tgt)
                else:
                    # legacy：门顶推导的离场点
                    _enter_pt, _exit_pt = _gate_derived_enter_exit(
                        _GATE_HEAD_COORD, vehicle_config.offset)
                    _tgt = np.array(_exit_pt)
                cruise_task = VehicleTask(
                    name="CRUISE", v_cruise=5, target_coords=_tgt)
                task_profile = [unpark_task, cruise_task]

                self.vehicle.set_vehicle_state(spot_index=abs(self.spot_index))
        elif agent_dict is not None:
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

        self.vehicle.execute_next_task()

        self.start_time = self.get_ros_time()
        self.start_solving = False
        self.last_time = self.start_time
        self.total_non_idle_time = 0

    # --- 多出入口（schema v2 有 portals）支持；无 portals / 未指派 → None 走 legacy ---
    def _portal_by_id(self, pid):
        """按物理 portal id（P1/P2/P3/P4）取 portal 字典。"""
        if not pid:
            return None
        for p in (getattr(self.vehicle, 'portals', None) or []):
            if p.get('id') == pid:
                return p
        return None

    def _portal_entry(self):
        """入场 portal（须有 spawn_pose_rotated_m）；不可用 → None。"""
        p = self._portal_by_id(getattr(self, 'entry_portal', ''))
        if p is None or p.get('spawn_pose_rotated_m') is None:
            return None
        return p

    def _portal_exit(self):
        """离场 portal（须有 exit_target_rotated_m）；不可用 → None。"""
        p = self._portal_by_id(getattr(self, 'exit_portal', ''))
        if p is None or p.get('exit_target_rotated_m') is None:
            return None
        return p

    def _apply_spot_target(self, spot_index):
        """把建图预计算的 (泊位, 入口) 目标顶点交给控制器。

        坐标落在「该入口可达」的弧上，避免 A* 目标落在 R11_F/R12_F/R13_F/R23_R
        等入口不可达弧上而抛 Path is not found。未命中则保持 legacy
        spot_waypoints 路径（控制器侧 active_spot_target 为 None）。
        """
        pid = getattr(self, 'entry_portal', '')
        st = getattr(self.vehicle, 'spot_targets', None)
        if not pid or not isinstance(st, dict):
            return
        by_index = st.get('by_index') or []
        try:
            si = abs(int(spot_index))
        except (TypeError, ValueError):
            return
        if si >= len(by_index):
            return
        tgt = (by_index[si].get('by_entry') or {}).get(pid)
        if tgt and tgt.get('coords') is not None:
            self.vehicle.set_active_spot_target(tgt['coords'])
            self.get_logger().info(
                'spot_target: spot_index=%d entry=%s arc=%s vertex=%s' % (
                    si, pid, tgt.get('arc_id'), tgt.get('vertex_id')))
    # --- end 多出入口支持 ---

    def sim_status_cb(self, msg: Bool):
        self.sim_is_running = msg.data

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

    def occupancy_cb(self, msg):
        self.vehicle.occupancy = msg.data

    def change_occupancy(self, idx, new_value):
        """占用变更（服务调用）；服务未就绪时进入重试队列（避免发现抖动导致丢失）。"""
        if not self.occupancy_cli.service_is_ready():
            self.get_logger().warning(
                'occupancy service not ready; queued write idx=%d val=%d' % (int(idx), int(new_value)))
            self._occ_retries.append([int(idx), int(new_value), 0])
            return
        self._send_occupancy(int(idx), int(new_value))

    def _send_occupancy(self, idx, new_value, retry_on_fail=True):
        def response_cb(future):
            try:
                res = future.result()
            except Exception:
                res = None
            if res is not None and res.status:
                self.get_logger().info("Service request from vehicle %d to change occupancy is successful" % self.vehicle_id)
            elif retry_on_fail and len(self._occ_retries) < 50:
                self.get_logger().warning(
                    'occupancy write failed; queued retry idx=%d val=%d' % (idx, new_value))
                self._occ_retries.append([idx, new_value, 0])

        req = OccupancySrv.Request()
        req.vehicle_id = self.vehicle_id
        req.idx = int(idx)
        req.new_value = int(new_value)

        future = self.occupancy_cli.call_async(req)
        future.add_done_callback(response_cb)

    def _flush_occupancy_retries(self):
        """重试队列落盘（由 0.5s 定时器驱动）"""
        if not self._occ_retries:
            return
        pending = []
        for item in self._occ_retries:
            idx, val, tries = item
            if self.occupancy_cli.service_is_ready():
                self._send_occupancy(idx, val, retry_on_fail=False)
            elif tries < 40:
                item[2] = tries + 1
                pending.append(item)
            else:
                self.get_logger().error(
                    'occupancy write dropped after retries: idx=%d val=%d' % (idx, val))
        self._occ_retries = pending


    def update_subs(self):
        topic_list_types = self.get_topic_names_and_types()

        for topic_name, _ in topic_list_types:
            state_name_pattern = re.match("/vehicle_([1-9][0-9]*)/state", topic_name)
            info_name_pattern = re.match("/vehicle_([1-9][0-9]*)/info", topic_name)

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

                    self.vehicle.remove_other_vehicle(vehicle_id)


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

                    self.vehicle.remove_other_vehicle(vehicle_id)

            else:
                continue

    def timer_callback(self):
        if self.vehicle.is_all_done():
            self.get_logger().info("Vehicle %d is done. Destroying node." % self.vehicle_id)

            # write logs
            log_dir_path = self.log_path
            if not os.path.exists(log_dir_path):
                os.mkdir(log_dir_path)

            with open(log_dir_path + "/vehicle_%d.log" % self.vehicle_id, 'a') as f:
                f.writelines(str(self.total_non_idle_time))
                self.vehicle.logger.clear()

            self.destroy_node()

        # 订阅管理（含 DDS 图查询 get_publishers_info_by_topic）代价高，限频 2Hz
        self._sub_tick = getattr(self, '_sub_tick', 0) + 1
        if self._sub_tick % 5 == 0:
            self.update_subs()

        current_time = self.get_ros_time()
        if self.vehicle.current_task != "IDLE":
            self.total_non_idle_time += current_time - self.last_time
        self.last_time = current_time

        if self.get_ros_time() - self.start_time > self.warm_start_time:
            self.start_solving = True

        if self.sim_is_running:
            if self.start_solving:
                self.vehicle.solve(time=self.get_ros_time())
        elif self.write_log and len(self.vehicle.logger) > 0:
            # write logs
            log_dir_path = self.log_path
            if not os.path.exists(log_dir_path):
                os.mkdir(log_dir_path)

            with open(log_dir_path + "/vehicle_%d.log" % self.vehicle_id, 'a') as f:
                f.writelines('\n'.join(self.vehicle.logger))
                self.vehicle.logger.clear()

        state_msg = VehicleStateMsg()
        if self.vehicle.state.v is not None:
            # webviz: 同步纵向速度（v -> v_long），供可视化读取
            self.vehicle.state.v.v_long = self.vehicle.state.v.v
        self.populate_msg(state_msg, self.vehicle.state)
        self.state_pub.publish(state_msg)

        info_msg = VehicleInfoMsg()
        self.populate_msg(info_msg, self.vehicle.get_info())
        self.info_pub.publish(info_msg)

        spot_msg = Int16()
        spot_msg.data = int(self.spot_index)
        self.spot_pub.publish(spot_msg)


def main(args=None):
    # 诊断：faulthandler（卡死时可用 SIGUSR1 向日志转储全部线程堆栈）
    try:
        import faulthandler
        import signal as _signal
        faulthandler.enable()
        faulthandler.register(_signal.SIGUSR1, all_threads=True, chain=False)
    except Exception:
        pass

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
        print("Unknown exception")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
