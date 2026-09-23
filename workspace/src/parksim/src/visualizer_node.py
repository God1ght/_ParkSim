#!/usr/bin/env python3

from typing import Dict
from collections import defaultdict

import rclpy
import re

from pathlib import Path

import numpy as np

from dlp.dataset import Dataset

import rclpy.logging
from std_msgs.msg import Bool, Float32
from parksim.msg import VehicleStateMsg, VehicleInfoMsg
from parksim.pytypes import VehicleState, NodeParamTemplate
from parksim.vehicle_types import VehicleBody, VehicleInfo
from parksim.base_node import MPClabNode

from parksim.visualizer.realtime_visualizer import RealtimeVisualizer
from parksim.base_node import parksim_path

import json
import os
import pickle


class VisualizerNodeParams(NodeParamTemplate):
    """
    template that stores all parameters needed for the node as well as default values
    """
    def __init__(self):
        self.dlp_path = None
        self.timer_period = 0.05

        # 地图布局注入（新地图，如 jth_b1）：为空时画面全部来自 DLP 数据集（DJI legacy）
        self.map = ''              # 地图名 → 自动找 priorFiles/maps/<map>/layout_rotated.json
        self.map_layout_path = ''  # 直接给出布局 JSON 路径（优先级高于 map）
        # 与 simulator 同款：launch 侧 JSON 覆盖（{"map": "jth_b1"} 等）
        self.launch_overrides = ''

        self.use_existing_agents = False
        self.dlp_time_offset = -1

        self.driving_color = [0, 255, 0, 255]
        self.parking_color = [255, 128, 0, 255]
        self.braking_color = [255, 0, 0, 255]
        self.alldone_color = [0, 0, 0, 255]

        self.dlp_color = [255, 255, 0, 128]

        self.disp_text_offset = [-2, 2]
        self.disp_text_size = 25

class VisualizerNode(MPClabNode):
    """
    Node class for visualizing everything
    """
    def __init__(self):
        super().__init__('visualizer')
        self.get_logger().info('Initializing Visualization Node')
        namespace = self.get_namespace()

        param_template = VisualizerNodeParams()
        self.autodeclare_parameters(param_template, namespace)
        self.autoload_parameters(param_template, namespace)

        self.state_subs = {}
        self.info_subs = {}
        self.states: Dict[int, VehicleState] = defaultdict(lambda: None)
        self.infos: Dict[int, VehicleInfo] = defaultdict(lambda: None)

        # Load dataset
        ds = Dataset()
        home_path = str(Path.home())
        # ds.load(home_path + self.dlp_path)

        # Yccc7: path changed
        ds.load(self.dlp_path)

        # Load Vehicle Body
        vehicle_body = VehicleBody()

        # 地图布局注入：新地图（jth_b1 等）自己带 layout_rotated.json，与仿真/车辆
        # 同一套 rotated 米制坐标；没有就退回 DLP 数据集（DJI legacy 行为不变）。
        map_size, parking_lines, waypoints, obstacle_polygons = self._load_map_layout()

        self.vis = RealtimeVisualizer(ds, vehicle_body,
                                      map_size=map_size,
                                      parking_lines=parking_lines,
                                      waypoints=waypoints,
                                      obstacle_polygons=obstacle_polygons)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.sim_status_pub = self.create_publisher(Bool, '/sim_status', 10)

        self.sim_time = 0.
        self.sim_time_sub = self.create_subscription(Float32, '/sim_time', self.sim_time_cb, 10)

    def _load_graph_vertices(self, layout_path):
        """从地图目录的 ``waypoints_graph.pickle`` 取**全部顶点坐标** (N,2)。

        这是仿真车辆真正走的那张图（rotated 米制，与泊位 quad 同坐标系），
        顶点稠密；布局 JSON 里的 ``graph.nodes`` 只有骨架（服务器版仅 24 个），
        只作为兜底。任何异常都返回空数组，由调用方回退。
        """
        candidates = [os.path.join(os.path.dirname(layout_path),
                                   'waypoints_graph.pickle')]
        name = (self._effective_map_name() or '').strip()
        if name:
            candidates.append(os.path.join(
                parksim_path('python', 'parksim', 'priorFiles', 'maps', name),
                'waypoints_graph.pickle'))
        for path in candidates:
            if not os.path.isfile(path):
                continue
            try:
                with open(path, 'rb') as fh:
                    data = pickle.load(fh)
                verts = None
                if isinstance(data, dict):
                    graph = data.get('graph')
                    verts = getattr(graph, 'vertices', None)
                    if verts is None:
                        verts = data.get('vertices')
                else:
                    verts = getattr(data, 'vertices', None)
                if verts is None:
                    continue
                if isinstance(verts, np.ndarray):
                    xy = np.asarray(verts, dtype=float)
                else:
                    xy = np.asarray([np.asarray(v.coords, dtype=float)
                                     for v in verts], dtype=float)
                xy = xy.reshape(-1, 2)
                if not len(xy):
                    continue
                self.get_logger().info('waypoints from %s：%d 顶点' % (path, len(xy)))
                return xy
            except Exception as exc:  # 绝不让可视化节点起不来
                self.get_logger().warn(
                    '读取 %s 失败（%r），尝试下一个候选/回退布局节点' % (path, exc))
        return np.zeros((0, 2), dtype=float)

    def _effective_map_name(self):
        """当前生效的地图名（launch 覆盖 > map 参数）。"""
        name = (self.map or '').strip()
        raw_overrides = (self.launch_overrides or '').strip()
        if raw_overrides:
            try:
                overrides = json.loads(raw_overrides)
                if isinstance(overrides, dict) and overrides.get('map'):
                    name = str(overrides['map']).strip()
            except Exception:
                pass
        return name

    def _resolve_map_layout_path(self):
        """确定地图布局 JSON 路径；返回 '' 表示不注入（回退 DLP 数据集）。"""
        # launch 侧 JSON 覆盖（与 simulator 的 launch_overrides 约定一致）
        name = self._effective_map_name()
        raw = (self.map_layout_path or '').strip()
        if raw:
            path = os.path.expandvars(raw)
            if os.path.isfile(path):
                return path
            self.get_logger().warn(
                'map_layout_path 不存在，回退 DLP 数据集：%s' % path)
            return ''
        if not name:
            return ''
        path = os.path.join(
            parksim_path('python', 'parksim', 'priorFiles', 'maps', name),
            'layout_rotated.json')
        # DJI 系列没有布局文件 → 静默回退（保持 legacy 行为）
        return path if os.path.isfile(path) else ''

    def _load_obstacle_polygons(self, layout_path):
        """从地图目录的 ``obstacles.json`` 取硬障碍物顶点环列表。

        返回 ``[(N, 2) ndarray, ...]``，rotated 米制，与 ``spots[*].quad`` 同坐标系。
        朝向编码在顶点里，绘制端必须按顶点原样画成多边形，不能退化成轴对齐矩形。

        文件缺失（如 DJI 系列、或尚未部署 obstacles.json 的地图目录）或任何异常
        都返回空列表 —— 只是不画障碍物，其余图层与 legacy 行为完全不变。
        """
        candidates = [os.path.join(os.path.dirname(layout_path), 'obstacles.json')]
        name = (self._effective_map_name() or '').strip()
        if name:
            candidates.append(os.path.join(
                parksim_path('python', 'parksim', 'priorFiles', 'maps', name),
                'obstacles.json'))
        for path in candidates:
            if not os.path.isfile(path):
                continue
            try:
                with open(path, 'r', encoding='utf-8') as fh:
                    doc = json.load(fh)
                if not isinstance(doc, dict):
                    raise TypeError('obstacles.json 根对象不是 dict：%s'
                                    % type(doc).__name__)
                polys = []
                for ob in (doc.get('obstacles') or []):
                    poly = (ob or {}).get('polygon') or []
                    if len(poly) >= 3:
                        polys.append(np.asarray(poly, dtype=float).reshape(-1, 2))
                self.get_logger().info(
                    'obstacles from %s：%d 个多边形' % (path, len(polys)))
                return polys
            except Exception as exc:  # 障碍物缺失/损坏不得影响其它图层
                self.get_logger().warn('读取障碍物失败（%s）：%r' % (path, exc))
        return []

    def _load_map_layout(self):
        """读 layout_rotated.json，返回 (map_size, parking_lines, waypoints, obstacle_polygons)。

        坐标来源（全部 rotated 米制，与车辆/仿真同一套）：
          - parking_lines     : 布局里的 spots[*].quad
          - waypoints         : 优先同目录 ``waypoints_graph.pickle`` 的**全部顶点**
                                （仿真真实路网，上千个点）；失败则回退
                                ``graph.nodes`` 骨架；再失败则 None（不画路网点）。
          - obstacle_polygons : 同目录 ``obstacles.json`` 的 obstacles[*].polygon；
                                缺失则为空列表（不画障碍物）。
          - map_size          : 泊位角点 ∪ 路网顶点 的包围盒右上角。
                                （刻意不含障碍物：障碍含外围边界环，会把画布撑大
                                且负坐标不可见，保持与既有行为一致。）

        任一步失败都返回 (None, None, None, [])，让可视化器退回 DLP 数据集，
        保证 DJI legacy 行为零变化。
        """
        path = self._resolve_map_layout_path()
        if not path:
            return None, None, None, []
        try:
            with open(path, 'r', encoding='utf-8') as fh:
                lay = json.load(fh)
            if not isinstance(lay, dict):
                raise TypeError('布局根对象不是 dict：%s' % type(lay).__name__)

            quads = []
            for s in (lay.get('spots') or []):
                if isinstance(s, dict) and s.get('quad'):
                    quads.append(
                        np.asarray(s['quad'], dtype=float).reshape(4, 2))

            # 兜底骨架节点（服务器版布局只有 24 个，Mac 版 740 个）
            skeleton = np.asarray(
                (lay.get('graph') or {}).get('nodes') or [],
                dtype=float).reshape(-1, 2)

            # 真实路网：优先 waypoints_graph.pickle（与仿真同图）
            verts = self._load_graph_vertices(path)
            if not len(verts):
                verts = skeleton
                if len(verts):
                    self.get_logger().warn(
                        '未取到 waypoints_graph.pickle，回退布局骨架节点 %d 个' % len(verts))

            xs, ys = [], []
            for q in quads:
                xs.extend(q[:, 0].tolist())
                ys.extend(q[:, 1].tolist())
            if len(verts):
                xs.extend(verts[:, 0].tolist())
                ys.extend(verts[:, 1].tolist())
            if not xs:
                self.get_logger().warn('布局 %s 无可用坐标，回退 DLP 数据集' % path)
                return None, None, None, []

            map_size = {'x': float(max(xs)), 'y': float(max(ys))}
            waypoints = {'map': verts} if len(verts) else None
            obstacle_polygons = self._load_obstacle_polygons(path)
            self.get_logger().info(
                'map layout: %s（%d spots / %d waypoints / %d obstacles / size x=%.2f y=%.2f）'
                % (path, len(quads), len(verts), len(obstacle_polygons),
                   map_size['x'], map_size['y']))
            return map_size, quads, waypoints, obstacle_polygons
        except Exception as exc:  # 任何异常都不许影响建图/legacy
            self.get_logger().warn(
                '读取地图布局失败（%s），回退 DLP 数据集：%r' % (path, exc))
            return None, None, None, []

    def sim_time_cb(self, msg: Float32):
        self.sim_time = msg.data

    def vehicle_state_cb(self, vehicle_id):
        def callback(msg):
            state = VehicleState()
            self.unpack_msg(msg, state)
            self.states[vehicle_id] = state

        return callback

    def vehicle_info_cb(self, vehicle_id):
        def callback(msg):
            info = VehicleInfo()
            self.unpack_msg(msg, info)
            self.infos[vehicle_id] = info

        return callback

    def update_subs(self):
        topic_list_types = self.get_topic_names_and_types()

        for topic_name, _ in topic_list_types:
            state_name_pattern = re.match("/vehicle_([1-9][0-9]*)/state", topic_name)
            info_name_pattern = re.match("/vehicle_([1-9][0-9]*)/info", topic_name)

            if state_name_pattern:
                vehicle_id = int(state_name_pattern.group(1))

                publisher = self.get_publishers_info_by_topic(topic_name=topic_name)

                if vehicle_id not in self.state_subs and publisher:
                    # If there is publisher, but we haven't subscribed to it
                    self.state_subs[vehicle_id] = self.create_subscription(VehicleStateMsg, topic_name, self.vehicle_state_cb(vehicle_id), 10)
                    self.get_logger().info("State subscriber to vehicle %d is built." % vehicle_id)
                elif vehicle_id in self.state_subs and not publisher:
                    # If we have subscribed to it, but there is no publisher anymore
                    self.destroy_subscription(self.state_subs[vehicle_id])
                    self.state_subs.pop(vehicle_id)
                    self.get_logger().info("Vehicle %d is not publishing anymore. State subscriber is destroyed." % vehicle_id)
                    # We don't delete state storage since we want the vehicle to remain in the visualizer

            elif info_name_pattern:
                vehicle_id = int(info_name_pattern.group(1))

                publisher = self.get_publishers_info_by_topic(topic_name=topic_name)

                if vehicle_id not in self.info_subs and publisher:
                    # If there is publisher, but we haven't subscribed to it
                    self.info_subs[vehicle_id] = self.create_subscription(VehicleInfoMsg, topic_name, self.vehicle_info_cb(vehicle_id), 10)
                    self.get_logger().info("Info subscriber to vehicle %d is built." % vehicle_id)
                elif vehicle_id in self.info_subs and not publisher:
                    # If we have subscribed to it, but there is no publisher anymore
                    self.destroy_subscription(self.info_subs[vehicle_id])
                    self.info_subs.pop(vehicle_id)
                    if vehicle_id in self.infos:
                        self.infos.pop(vehicle_id)
                    self.get_logger().info("Vehicle %d is not publishing anymore. Info ubscriber is destroyed." % vehicle_id)

            else:
                continue

    def timer_callback(self):
        """
        update the list of subscribers and plot
        """
        self.update_subs()

        self.vis.clear_frame()

        if self.use_existing_agents:
            scene_token = self.vis.dlpvis.dataset.list_scenes()[0]
            agent_token_list = self.vis.dlpvis.dataset.get('scene', scene_token)['agents']
            frame = self.vis.dlpvis.dataset.get_frame_at_time(
                scene_token=scene_token, timestamp=max(self.sim_time + self.dlp_time_offset, 0))
            inst_tokens = frame['instances']
            for inst_token in inst_tokens:
                instance = self.vis.dlpvis.dataset.get('instance', inst_token)
                agent = self.vis.dlpvis.dataset.get('agent', instance['agent_token'])

                vehicle_id = agent_token_list.index(instance['agent_token'])
                if agent['type'] in {'Pedestrian', 'Undefined', 'Bicycle'}:
                    continue
                state = VehicleState()
                state.x.x = instance['coords'][0]
                state.x.y = instance['coords'][1]
                state.e.psi = instance['heading']

                self.vis.draw_vehicle(state, fill=self.dlp_color)
                self.vis.draw_text([state.x.x + self.disp_text_offset[0], state.x.y +
                                   self.disp_text_offset[1]], str(vehicle_id), size=self.disp_text_size)

        for vehicle_id in self.states:
            state = self.states[vehicle_id]
            info = self.infos[vehicle_id]

            if not info or info.is_all_done:
                color = self.alldone_color
            elif info.is_braking:
                color = self.braking_color
            elif info.task in ["PARK", "UNPARK"]:
                color = self.parking_color
            else:
                color = self.driving_color

            self.vis.draw_vehicle(state, fill=color)
            if info and info.disp_text:
                self.vis.draw_text([state.x.x + self.disp_text_offset[0], state.x.y + self.disp_text_offset[1]], info.disp_text, size=self.disp_text_size)

        self.vis.render()

        sim_status_msg = Bool()
        sim_status_msg.data = self.vis.is_running()
        self.sim_status_pub.publish(sim_status_msg)

def main(args=None):
    rclpy.init(args=args)

    visualizer = VisualizerNode()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print('Visualization is terminated')
    finally:
        visualizer.destroy_node()
        print('Visualization stopped cleanly')

        rclpy.shutdown()

if __name__ == "__main__":
    main()
