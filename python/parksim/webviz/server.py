#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ParkSim webviz — 浏览器实时可视化桥（独立进程）。

功能
----
  · rclpy 节点动态订阅 /vehicle_*/state、/vehicle_*/info 与 /sim_time
  · 读取 DLP 地图数据（车位 / waypoints / 障碍物）与数据集"现有智能体"帧（幽灵层）
  · aiohttp 提供静态页面与 WebSocket 帧流（init 一次性 + ~15Hz frame）
  · --control 时发布 /sim_status（网页可暂停/恢复仿真）

用法
----
    source <部署>/env_ros.sh
    python <repo>/python/parksim/webviz/server.py --port 8099 [--control] [--record]

注意
----
  为避免与 ROS 工作区安装的 parksim 包互相遮蔽，本脚本不导入 parksim 包；
  消息类型由 rosidl_runtime_py 动态解析，路径/配置工具在本文件内实现。
"""

import argparse
import asyncio
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


# --------------------------------------------------------------------------
# 轻量工具
# --------------------------------------------------------------------------

def find_repo_root():
    """优先 PARKSIM_ROOT；否则从本文件向上查找仓库根。"""
    env_root = os.environ.get('PARKSIM_ROOT')
    if env_root and os.path.isdir(env_root):
        return str(Path(env_root).expanduser().resolve())
    p = Path(__file__).resolve()
    for cand in p.parents:
        if (cand / 'python' / 'parksim').is_dir() and (cand / 'workspace' / 'src' / 'parksim').is_dir():
            return str(cand)
    raise RuntimeError('无法定位 ParkSim 仓库根，请设置 PARKSIM_ROOT')


def expand_config_value(value, root):
    """展开配置中的 ${PARKSIM_ROOT} / ${PARKSIM_PRIORFILES}（等价 parksim.base_node 的工具）。"""
    if isinstance(value, str):
        priorfiles = os.path.join(root, 'python', 'parksim', 'priorFiles')
        v = value.replace('${PARKSIM_ROOT}', root).replace('$PARKSIM_ROOT', root)
        v = v.replace('${PARKSIM_PRIORFILES}', priorfiles).replace('$PARKSIM_PRIORFILES', priorfiles)
        return os.path.expanduser(os.path.expandvars(v))
    if isinstance(value, list):
        return [expand_config_value(item, root) for item in value]
    if isinstance(value, dict):
        return {k: expand_config_value(item, root) for k, item in value.items()}
    return value


def rounded(v, n=3):
    try:
        return round(float(v), n)
    except (TypeError, ValueError):
        return v


# --------------------------------------------------------------------------
# 依赖（需要 ROS 环境已 source）
# --------------------------------------------------------------------------
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int16, Int16MultiArray, String
from aiohttp import web, WSMsgType
import io as _pio
from PIL import Image as _PILImage, ImageOps as _PILOps
import dlp
from dlp.dataset import Dataset
from dlp.visualizer import Visualizer as DlpVisualizer
from rosidl_runtime_py.utilities import get_message

COLOR_KEYS = ['driving', 'parking', 'braking', 'alldone', 'dlp']
DEFAULT_COLORS = {
    'driving': [0, 255, 0, 255],
    'parking': [255, 128, 0, 255],
    'braking': [255, 0, 0, 255],
    'alldone': [0, 0, 0, 255],
    'dlp': [0, 0, 0, 48],
}

STATE_MSG = get_message('parksim/msg/VehicleStateMsg')
INFO_MSG = get_message('parksim/msg/VehicleInfoMsg')


# --------------------------------------------------------------------------
# 鲁棒性：告警阈值 / 文案 与 启动期关键资产自检
# --------------------------------------------------------------------------
# 发车异常：running 且 /webviz/spawn_status 的 entering/exiting 余量都 > 0，
#           但当前帧车辆数连续 SPAWN_STUCK_ALERT_S 秒为 0
#           → 判为「车辆节点未起来」（N7：关键资产缺失使 vehicle_node 启动即崩溃）。
SPAWN_STUCK_ALERT_S = 60.0
# 停滞兜底：running 且 spawn 两队列余量都为 0 且当前帧车辆数 > 0，
#           但仿真时间连续 STALL_ALERT_S 秒无推进
#           → 判为「仿真停滞」（某车节点死锁导致 never END、finished 无兜底）。
#           状态**保持 running**，不伪造 finished。
STALL_ALERT_S = 120.0

SPAWN_STUCK_ALERT_MSG = '车辆节点未起来（疑似关键资产缺失或节点崩溃），请检查服务端日志'
STALL_ALERT_MSG = '仿真停滞：车辆长时间无进展（可能某车节点死锁）'

# 关键资产清单：相对「资产根」的路径。资产根默认 <repo>/python/parksim，
# 可用环境变量 PARKSIM_ASSET_ROOT 覆盖（便于指向空目录做降级验证）。
REQUIRED_ASSETS_MAP_FILES = (
    'waypoints_graph.pickle',
    'spots_data.pickle',
    'parking_maneuvers_per_spot.pickle',
    'layout_rotated.json',
)
REQUIRED_ASSETS_TOP_FILES = (
    'parking_maneuvers.pickle',
    'spots_data.pickle',
    'waypoints_graph.pickle',
)


def resolve_asset_root(root):
    """关键资产根目录。

    优先环境变量 PARKSIM_ASSET_ROOT（便于指向空目录做降级验证），
    否则取 <repo>/python/parksim。
    """
    env_root = os.environ.get('PARKSIM_ASSET_ROOT')
    if env_root:
        return os.path.abspath(os.path.expanduser(env_root))
    return os.path.join(root, 'python', 'parksim')


def check_required_assets(root, map_name, asset_root=None):
    """启动期关键资产自检（只做存在性检查，缺失**不**阻断启动）。

    参数
    ----
    root        : 仓库根（PARKSIM_ROOT）
    map_name    : 当前地图名（地图包资产位于 priorFiles/maps/<map_name>/）
    asset_root  : 资产根；缺省 resolve_asset_root(root)

    返回
    ----
    (asset_root, missing)：missing 为缺失项的**绝对路径**列表，顺序与清单一致。
    """
    if asset_root is None:
        asset_root = resolve_asset_root(root)
    rels = [os.path.join('priorFiles', 'maps', map_name, n)
            for n in REQUIRED_ASSETS_MAP_FILES]
    rels += [os.path.join('priorFiles', n) for n in REQUIRED_ASSETS_TOP_FILES]
    missing = []
    for rel in rels:
        full = os.path.join(asset_root, rel)
        if not os.path.isfile(full):
            missing.append(full)
    return asset_root, missing


def update_asset_status(shared, payload=None, log=False):
    """重算关键资产自检并写入 shared['degraded'] / ['assets_missing'] / ['asset_root']。

    如提供 payload（init 字典），同时把 degraded/assets_missing 字段注入其中。
    返回缺失路径列表。启动时与本图切换后各调用一次，保证页面横幅与实际情况一致。
    """
    root = shared.get('root')
    map_name = str((shared.get('opts') or {}).get('map') or '')
    asset_root, missing = check_required_assets(root, map_name)
    shared['asset_root'] = asset_root
    shared['degraded'] = bool(missing)
    shared['assets_missing'] = list(missing)
    if payload is not None:
        payload['degraded'] = bool(missing)
        payload['assets_missing'] = list(missing)
    if log:
        if missing:
            print('[webviz] ERROR 关键资产缺失（桥侧资产自检未通过；若为 PARKSIM_ASSET_ROOT 覆盖所致，'
                  '仿真节点仍读真实资产、不受影响），共 %d 项：' % len(missing))
            for _p in missing:
                print('[webviz] ERROR   缺失: %s' % _p)
            print('[webviz] ERROR 资产根=%s（可用 PARKSIM_ASSET_ROOT 覆盖）' % asset_root)
        else:
            print('[webviz] asset check: OK（map=%s，资产根=%s）' % (map_name, asset_root))
    return missing


# --------------------------------------------------------------------------
# 运行状态机（stopped / starting / running / finished）——模块级单一事实来源
# --------------------------------------------------------------------------

class SimStateTracker(object):
    """仿真运行状态机。

    状态语义：
      stopped   —— 无仿真进程（未启动 / 已停止 / 意外退出）
      starting  —— restart/启动已触发，等待首帧 sim_time 前进
                   （restart 期间前端沿用现有 'restarting' 广播，视同 starting）
      running   —— 启动后收到 sim_time 前进的帧
      finished  —— running 且「spawn 两队列余量均为 0 且当前帧车辆数为 0」
                   持续 30 秒（只报一次；新车辆出现或 restart 重置）。
                   从未收到 /webviz/spawn_status 时退化为
                   「已进入过 running 且车辆数为 0 持续 30 秒」。

    告警（只报一次；条件恢复时发一条 active=False 复位）：
      spawn_stuck        —— running 且 spawn 两队列余量都 > 0，但车辆数连续
                            SPAWN_STUCK_ALERT_S 秒为 0（车辆节点未起来，N7）。
      stalled_no_progress—— running 且 spawn 两队列余量都为 0、车辆数 > 0，
                            但 sim_time 连续 STALL_ALERT_S 秒无推进（车节点死锁，
                            finished 无兜底的缓解）。状态保持 running，不伪造 finished。
                            若既有 stalled（数据流停滞，见 note_dataflow_stall）已生效，
                            本条被抑制，避免重复上报。

    线程安全：ROS spin 线程（帧/spawn 回调）、aiohttp 事件循环（watchdog）、
    WS 处理线程（stop/restart）都会访问，内部用一把锁保护。
    note_frame() 返回**待广播事件列表**，由调用方（事件循环内）负责广播。
    """

    FINISH_GRACE_S = 30.0  # 零车 + 队列空 持续时长阈值

    def __init__(self):
        self._lock = threading.Lock()
        self.state = 'stopped'
        self.spawn = None            # {'entering_remaining':int,'exiting_remaining':int} 或 None
        self._last_t = None          # starting 期间已见帧的 sim_time（用于判定“前进”）
        self._ever_running = False   # 本轮是否进入过 running（finished 退化判定用）
        self._zero_since = None      # 进入「零车 + 队列空」条件的起始墙钟时刻
        # 告警（只报一次；条件恢复时发 active=False 复位）
        self._spawn_stuck_since = None     # 「spawn 余量 > 0 但零车」起始墙钟时刻
        self._spawn_stuck_alerted = False
        self._stall_since = None           # 「余量空 + 有车 但 sim_time 不推进」起始时刻
        self._stall_alerted = False
        # 既有「数据流停滞」(sim_watchdog 的红色 stalled) 是否生效；生效期间抑制
        # stalled_no_progress，避免与既有告警重复上报（既有 stalled 更严重、优先）。
        self._dataflow_stalled = False

    # ---------- 外部事件 ----------
    def _reset_alerts_locked(self):
        """清空两条告警的全部内部状态（新一轮仿真 / 停止时调用）。"""
        self._spawn_stuck_since = None
        self._spawn_stuck_alerted = False
        self._stall_since = None
        self._stall_alerted = False
        self._dataflow_stalled = False

    def note_dataflow_stall(self, active):
        """由 sim_watchdog 告知「既有 stalled（数据流停滞）是否生效」。

        生效期间抑制 stalled_no_progress —— 保留既有 stalled（红色、带重启按钮，
        更严重），避免两者对同一「sim_time 不推进」现象重复上报。
        """
        with self._lock:
            changed = (bool(active) != self._dataflow_stalled)
            self._dataflow_stalled = bool(active)
            if changed:
                print('[webviz] stalled_no_progress %s（既有 stalled %s）' % (
                    '已抑制' if self._dataflow_stalled else '恢复评估',
                    '生效' if self._dataflow_stalled else '解除'))

    def set_starting(self):
        """启动/restart 已触发（广播 'restarting' 由现有路径负责）。"""
        with self._lock:
            self.state = 'starting'
            self.spawn = None   # 上一轮 spawn 余量失效；新仿真的 spawn_status 会重建
            self._last_t = None
            self._ever_running = False
            self._zero_since = None
            self._reset_alerts_locked()

    def set_stopped(self):
        """stop 完成 / restart 失败 / 仿真意外退出。"""
        with self._lock:
            self.state = 'stopped'
            self.spawn = None   # 上一轮 spawn 余量随仿真停止失效；下轮由新话题消息重建
            self._last_t = None
            self._ever_running = False
            self._zero_since = None
            self._reset_alerts_locked()

    def note_spawn(self, entering_remaining, exiting_remaining):
        """缓存最新 /webviz/spawn_status（随 init 下发为 spawn 字段）。"""
        try:
            entry = {'entering_remaining': int(entering_remaining),
                     'exiting_remaining': int(exiting_remaining)}
        except (TypeError, ValueError):
            return
        with self._lock:
            self.spawn = entry

    def snapshot(self):
        """(state, spawn) —— 供 init 载荷使用。"""
        with self._lock:
            return self.state, (dict(self.spawn) if self.spawn is not None else None)

    def active_alerts(self):
        """当前仍生效的告警事件列表（供新 WS 连接回放，保证横幅跨重连保持）。"""
        out = []
        with self._lock:
            if self._spawn_stuck_alerted:
                out.append({'type': 'alert', 'code': 'spawn_stuck', 'active': True,
                            'level': 'warn', 'message': SPAWN_STUCK_ALERT_MSG})
            if self._stall_alerted:
                out.append({'type': 'alert', 'code': 'stalled_no_progress', 'active': True,
                            'level': 'warn', 'message': STALL_ALERT_MSG})
        return out

    # ---------- 帧驱动的状态迁移 ----------
    def _queues_empty_locked(self):
        """spawn 余量均为 0；从未收到 spawn_status 时退化为「视为空」。"""
        if self.spawn is None:
            return True
        return (self.spawn.get('entering_remaining') == 0
                and self.spawn.get('exiting_remaining') == 0)

    def _eval_alerts_locked(self, sim_t, prev_t, n_vehicles, now, events):
        """评估两条告警（在 running/finished 分支内调用，调用方已持锁）。

        两条告警都「只报一次」，条件恢复时追加一条 ``active=False`` 复位事件：
          ① spawn_stuck         —— running 且 spawn 两余量都 > 0，但车辆数恒为 0；
          ② stalled_no_progress —— running 且 spawn 两余量都为 0、车辆数 > 0，
                                   但 sim_time 不推进（某车节点死锁）。
        余量为 0 的 stall 判定要求**真实收到过** spawn_status（spawn 非 None），
        避免与 finished 的退化语义混淆、也不误报。
        """
        run = (self.state == 'running')
        spawn = self.spawn
        ent = spawn.get('entering_remaining') if spawn else None
        ext = spawn.get('exiting_remaining') if spawn else None

        # ---------- ① 发车异常：余量 > 0 却零车 ----------
        stuck = bool(run and ent is not None and ext is not None
                     and ent > 0 and ext > 0 and n_vehicles == 0)
        if stuck:
            if self._spawn_stuck_since is None:
                self._spawn_stuck_since = now
            elif (not self._spawn_stuck_alerted
                  and (now - self._spawn_stuck_since) >= SPAWN_STUCK_ALERT_S):
                self._spawn_stuck_alerted = True
                events.append({'type': 'alert', 'code': 'spawn_stuck',
                               'active': True, 'level': 'warn',
                               'message': SPAWN_STUCK_ALERT_MSG})
        else:
            self._spawn_stuck_since = None
            if self._spawn_stuck_alerted:
                self._spawn_stuck_alerted = False
                events.append({'type': 'alert', 'code': 'spawn_stuck',
                               'active': False})

        # ---------- ② 停滞兜底：余量空 + 有车，但 sim_time 不推进 ----------
        # 若既有 stalled（数据流停滞）已生效 → 抑制本条，避免重复上报（既有更严重）。
        stall_base = bool(run and ent == 0 and ext == 0 and n_vehicles > 0)
        if stall_base and not self._dataflow_stalled:
            if prev_t is None or sim_t != prev_t:
                # sim_time 在推进 → 条件恢复（条件之一：仿真时间恢复推进）
                self._stall_since = None
                if self._stall_alerted:
                    self._stall_alerted = False
                    events.append({'type': 'alert', 'code': 'stalled_no_progress',
                                   'active': False})
            elif self._stall_since is None:
                self._stall_since = now
            elif (not self._stall_alerted
                  and (now - self._stall_since) >= STALL_ALERT_S):
                self._stall_alerted = True
                events.append({'type': 'alert', 'code': 'stalled_no_progress',
                               'active': True, 'level': 'warn',
                               'message': STALL_ALERT_MSG})
        else:
            self._stall_since = None
            if self._stall_alerted:
                self._stall_alerted = False
                events.append({'type': 'alert', 'code': 'stalled_no_progress',
                               'active': False})

    def note_frame(self, sim_t, n_vehicles, now=None):
        """每收到一帧调用（watchdog 轮询）。返回待广播事件列表。"""
        if now is None:
            now = time.time()
        events = []
        with self._lock:
            if self.state == 'starting':
                # 首帧仅记录基准；sim_time 相对上一帧前进 → running
                if self._last_t is None:
                    self._last_t = sim_t
                elif sim_t != self._last_t:
                    self._last_t = sim_t
                    self.state = 'running'
                    self._ever_running = True
                    self._zero_since = None
                    events.append({'type': 'status', 'value': 'running'})
            elif self.state in ('running', 'finished'):
                prev_t = self._last_t
                self._last_t = sim_t
                # 告警评估（只报一次 / 恢复复位），在 finished 迁移前完成
                self._eval_alerts_locked(sim_t, prev_t, n_vehicles, now, events)
                if n_vehicles > 0:
                    # 新车辆出现：finished 闩锁重置，回到 running
                    self._zero_since = None
                    if self.state == 'finished':
                        self.state = 'running'
                        events.append({'type': 'status', 'value': 'running'})
                elif self._ever_running and self._queues_empty_locked():
                    if self._zero_since is None:
                        self._zero_since = now
                    elif (now - self._zero_since) >= self.FINISH_GRACE_S \
                            and self.state != 'finished':
                        self.state = 'finished'
                        events.append({'type': 'status', 'value': 'finished'})
                else:
                    self._zero_since = None
            # stopped：忽略帧（桥进程常驻，仿真停止后仍在出帧）
        return events


# 模块级单一事实来源
sim_state_tracker = SimStateTracker()


# --------------------------------------------------------------------------
# 桥接节点
# --------------------------------------------------------------------------

class BridgeNode(Node):
    """订阅仿真状态并维护共享快照（供 WebSocket 推送）。"""

    def __init__(self, shared, control=False):
        super().__init__('webviz_bridge')
        self.shared = shared
        self.control = control
        self.state_subs = {}
        self.info_subs = {}
        self.states = {}   # vid -> (x, y, psi)
        self.infos = {}    # vid -> dict(task, is_braking, is_all_done, disp_text)
        self.sim_time = 0.0
        self.paused = False
        self._seq = 0
        self.focus = 0            # 网页聚焦车辆 id（0=无），用于按需下发参考路径
        self.spot_subs = {}
        self.spots = {}           # vid -> 目标车位（+ 入库 / - 出库 / 0 无）
        self._pending_retire = {} # vid -> 发布者消失后的退役判定截止时间
        self._spot_centers = self._compute_spot_centers()
        # 泊位几何（quad + 朝向 + 尺寸），用于把退役车辆**吸附**到泊位：
        # 索引与 occupancy / spots_data 一致；为空时退回旧的「车辆自身位姿」行为。
        self._spot_geoms = self._load_spot_geometry()
        # 显式障碍物序列（静态停放车辆车库）：spot -> {x,y,psi,w,l,t_arrive,vehicle_id}
        # 由 /occupancy 生命周期驱动（到达=占用1→建立；离场释放=占用0→移除）
        self.static_garage = {}
        # 静态停放车辆的近似车身尺寸（DH 数据无单辆尺寸；采用 DJI 标准小汽车）
        self._obstacle_w = 1.85
        self._obstacle_l = 4.6

        self.create_subscription(Float32, '/sim_time', self._sim_time_cb, 10)
        self.create_subscription(Int16MultiArray, '/occupancy', self._occupancy_cb, 10)
        self.create_subscription(Int16MultiArray, '/departing_spots', self._departing_cb, 10)
        # 生成队列余量（simulator_node 以 1 Hz 发布 JSON：
        # {"entering_remaining":int,"exiting_remaining":int}），供 finished 判定与 init 下发
        self.create_subscription(String, '/webviz/spawn_status', self._spawn_status_cb, 10)
        self.status_pub = self.create_publisher(Bool, '/sim_status', 10) if control else None

        self._ghosts = []
        self._ghosts_t = None
        self.create_timer(0.05, self._tick)
        self.get_logger().info('webviz bridge started (control=%s)' % control)

    # ---------- 订阅回调 ----------
    def _sim_time_cb(self, msg):
        self.sim_time = float(msg.data)

    def _occupancy_cb(self, msg):
        """车位占用更新（随机生成模式下网页据此绘制停放车辆）。"""
        data = [int(v) for v in msg.data]
        with self.shared['lock']:
            if data != self.shared.get('occupancy'):
                old = self.shared.get('occupancy') or []
                self.shared['occupancy'] = data
                self.shared['occupancy_seq'] = self.shared.get('occupancy_seq', 0) + 1
                # 生命周期同步：某车位占用 1->0（离场释放）→ 从静态车库移除该障碍物
                if old:
                    n = min(len(old), len(data))
                    for i in range(n):
                        if old[i] and not data[i]:
                            self.static_garage.pop(i, None)

    def _departing_cb(self, msg):
        """即将驶出车位更新（出库车已生成、车辆仍在位）。"""
        data = [int(v) for v in msg.data]
        with self.shared['lock']:
            if data != self.shared.get('departing'):
                self.shared['departing'] = data
                self.shared['departing_seq'] = self.shared.get('departing_seq', 0) + 1

    def _spawn_status_cb(self, msg):
        """生成队列余量更新（std_msgs/String，JSON）。"""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        sim_state_tracker.note_spawn(data.get('entering_remaining'),
                                     data.get('exiting_remaining'))

    def _state_cb(self, vid):
        def cb(msg):
            try:
                speed = round(float(msg.v.v_long), 2)
            except Exception:
                speed = None
            self.states[vid] = (rounded(msg.x.x), rounded(msg.x.y), rounded(msg.e.psi), speed)
        return cb

    def _info_cb(self, vid):
        def cb(msg):
            entry = {
                'task': str(msg.task),
                'is_braking': bool(msg.is_braking),
                'is_all_done': bool(msg.is_all_done),
                'disp_text': str(msg.disp_text),
            }
            # 参考路径降采样缓存（供网页“路径图层”使用，聚焦车辆时随帧下发）
            try:
                xs = msg.ref_pose.x
                ys = msg.ref_pose.y
                if xs is not None and ys is not None and len(xs) == len(ys) and len(xs) > 1:
                    step = max(1, len(xs) // 64)
                    entry['path'] = [[rounded(xs[i], 2), rounded(ys[i], 2)]
                                     for i in range(0, len(xs), step)]
            except Exception:
                pass
            self.infos[vid] = entry
        return cb

    def _spot_cb(self, vid):
        def cb(msg):
            self.spots[vid] = int(msg.data)
        return cb

    # ---------- 车辆退役（发布者消失）：泊入保留 / 离场移除 ----------
    def _compute_spot_centers(self):
        try:
            dlpvis = self.shared.get('dlpvis')
            if dlpvis is None:
                return []
            arr = dlpvis.parking_spaces.to_numpy()
            return [((arr[i][2] + arr[i][4]) / 2.0, (arr[i][3] + arr[i][9]) / 2.0)
                    for i in range(len(arr))]
        except Exception:
            return []

    # ---------- 泊位几何：把「退役车辆」吸附成停在泊位里的静态障碍 ----------
    #
    # 旧逻辑只判断「3.5 m 内有泊位中心」就通过，实测 13.6% 的过道采样点会误判为已泊入，
    # 且记下的位姿/尺寸是车辆原始位姿 + 硬编码 4.6×1.85 ⇒ 画出来的障碍与泊位对不上。
    # 这里改成：点在 quad 内优先，其次到泊位中心 < SPOT_SNAP_RADIUS_M。
    SPOT_SNAP_RADIUS_M = 1.5  # ≈ 泊位中位半宽 1.20 m + 余量

    @staticmethod
    def _quad_metrics(quad):
        """(cx, cy, long_len, short_len, axis_rad)。

        quad 为 4 个角点；axis_rad 是**长边**方向（atan2，(-π, π]）。
        """
        xs = [float(p[0]) for p in quad]
        ys = [float(p[1]) for p in quad]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        edges = []
        for i in range(len(quad)):
            ax, ay = float(quad[i][0]), float(quad[i][1])
            bx, by = float(quad[(i + 1) % len(quad)][0]), float(quad[(i + 1) % len(quad)][1])
            edges.append((math.hypot(bx - ax, by - ay), math.atan2(by - ay, bx - ax)))
        # 矩形的 4 条边循环排列为 L, W, L, W，降序排序后是 [L, L, W, W]，
        # 因此短边必须取 edges[-1]，取 edges[1] 会又拿到 L。
        edges.sort(key=lambda e: -e[0])
        return cx, cy, edges[0][0], edges[-1][0], edges[0][1]

    @staticmethod
    def _point_in_quad(x, y, quad):
        """凸四边形包含测试（叉积同号）；四边形顶点顺/逆时针均可。"""
        sign = 0
        n = len(quad)
        for i in range(n):
            ax, ay = float(quad[i][0]), float(quad[i][1])
            bx, by = float(quad[(i + 1) % n][0]), float(quad[(i + 1) % n][1])
            cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
            if cross == 0.0:
                continue
            s = 1 if cross > 0 else -1
            if sign == 0:
                sign = s
            elif s != sign:
                return False
        return True

    @staticmethod
    def _ang_dist_pi(a, b):
        """两个方向角在 mod π 下的最小夹角（矩形对 180° 对称），返回 [0, π/2]。"""
        d = abs((a - b) % math.pi)
        return min(d, math.pi - d)

    def _load_spot_geometry(self):
        """返回 [(quad, cx, cy, psi, axis, long, short), ...]，索引与 occupancy 一致。

        优先用地图包 ``layout_rotated.json`` 的 ``spots[*].quad`` +
        ``direction.entry_heading_rad``（与仿真/车辆同一套 rotated 米制，且
        entry_heading 就是仿真实际泊入朝向）；没有地图布局时回退 DLP 数据集的泊位表
        （无 entry_heading，朝向取 quad 长边方向）。

        任何异常都返回空列表 —— 调用方会退回「用车辆自身位姿 + 车身尺寸」的旧行为，
        不会因为缺地图数据而把车辆全部丢弃。
        """
        geoms = []
        try:
            opts = self.shared.get('opts') or {}
            root = self.shared.get('root') or ''
            map_name = str(opts.get('map') or '')
            path = os.path.join(root, 'python', 'parksim', 'priorFiles', 'maps',
                                map_name, 'layout_rotated.json')
            if map_name and os.path.isfile(path):
                with open(path) as fh:
                    lay = json.load(fh)
                for s in (lay.get('spots') or []):
                    quad = [[float(p[0]), float(p[1])] for p in (s.get('quad') or [])]
                    if len(quad) < 3:
                        continue
                    cx, cy, long_len, short_len, axis = self._quad_metrics(quad)
                    heading = (s.get('direction') or {}).get('entry_heading_rad')
                    psi = float(heading) if heading is not None else axis
                    geoms.append(dict(quad=quad, cx=cx, cy=cy, psi=psi,
                                      axis=axis, long=long_len, short=short_len))
                if geoms:
                    self.get_logger().info(
                        'spot geometry from %s：%d 泊位' % (path, len(geoms)))
                    return geoms
        except Exception as exc:
            geoms = []
            self.get_logger().warn('读取泊位几何失败（%s）：%r' % (path, exc))
        # 回退：DLP 数据集的泊位表（DJI legacy）
        try:
            dlpvis = self.shared.get('dlpvis')
            if dlpvis is None:
                return []
            arr = dlpvis.parking_spaces.iloc[:, 2:10].to_numpy()
            for row in arr:
                quad = [[float(row[i]), float(row[i + 1])] for i in (0, 2, 4, 6)]
                cx, cy, long_len, short_len, axis = self._quad_metrics(quad)
                geoms.append(dict(quad=quad, cx=cx, cy=cy, psi=axis,
                                  axis=axis, long=long_len, short=short_len))
        except Exception:
            return []
        return geoms

    def _match_spot(self, x, y):
        """确定车辆停在哪个泊位：**点在 quad 内**优先，其次「到泊位中心 < 1.5 m」。

        返回泊位索引；都不满足返回 -1（视作过道/离场，不记为静态停放）。
        """
        best, best_d = -1, self.SPOT_SNAP_RADIUS_M
        for i, g in enumerate(self._spot_geoms):
            if self._point_in_quad(x, y, g['quad']):
                return i
            d = math.hypot(x - g['cx'], y - g['cy'])
            if d < best_d:
                best, best_d = i, d
        return best

    def _spot_pose(self, idx):
        """(x, y, psi, length, width)：命中泊位时静态障碍应采用的位姿与尺寸。

        - 位置吸附到泊位中心、朝向取泊位朝向（entry_heading_rad），
          保证「门 / 车位 / 障碍」三者朝向一致；
        - 尺寸取泊位 quad 自身的长/短边，按「哪条边更接近 psi」分配 L/W
          （不能用 quad 在 psi 方向的投影跨度，那会把斜置矩形放大成 AABB）。
        """
        g = self._spot_geoms[idx]
        psi = g['psi']
        if self._ang_dist_pi(psi, g['axis']) <= math.pi / 4:
            length, width = g['long'], g['short']
        else:
            length, width = g['short'], g['long']
        return g['cx'], g['cy'], psi, length, width

    def _near_any_spot(self, x, y, radius=3.5):
        r2 = radius * radius
        for cx, cy in self._spot_centers:
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2:
                return True
        return False

    def _retire_vehicle(self, vid):
        """发布者消失后判定：位于车位内（泊入完成）保留为静态停车；否则（离场/异常）移除。

        保留时把该车辆写成显式障碍物序列 static_garage 的一项
        （vehicle -> obstacle 的一环）。**命中的泊位决定其位置、朝向与尺寸**：
          - 位置吸附到泊位中心、朝向取泊位朝向、尺寸取泊位实际长宽；
          - 三者都保留角度语义，避免画成与倾斜泊位对不上的轴对齐/错位矩形。
        没有泊位几何（legacy / 缺地图数据）或没匹配到泊位时，退回旧的
        「车辆自身位姿 + 车身尺寸」行为，不抛异常。
        """
        st = self.states.get(vid)
        if st is None:
            self._drop_vehicle(vid)
            return

        geom_ok = bool(self._spot_geoms)
        idx = self._match_spot(st[0], st[1]) if geom_ok else -1
        if geom_ok:
            if idx < 0:
                self._drop_vehicle(vid)
                return
        elif not self._near_any_spot(st[0], st[1]):
            self._drop_vehicle(vid)
            return

        spot = int(self.spots.get(vid) or 0)
        # 仅把「正车位（入库抵达）」记为静态障碍物；出库/异常一律不记
        if spot > 0:
            occ = self.shared.get('occupancy') or []
            if 0 <= spot < len(occ) and occ[spot]:
                if idx >= 0:
                    # 位姿来自几何命中的泊位；能用车辆自报的 spot 就用它，
                    # 保证与 static_garage 的键（由 /occupancy 生命周期释放）一致
                    pose_idx = spot if 0 <= spot < len(self._spot_geoms) else idx
                    x, y, psi, length, width = self._spot_pose(pose_idx)
                    self.static_garage[spot] = {
                        'spot': spot,
                        'x': rounded(x),
                        'y': rounded(y),
                        'psi': round(float(psi), 6),
                        'w': round(float(width), 3),
                        'l': round(float(length), 3),
                        't_arrive': round(self.sim_time, 2),
                        'vehicle_id': vid,
                    }
                else:
                    # legacy 回退：车辆自身位姿 + 标准车身尺寸
                    self.static_garage[spot] = {
                        'spot': spot,
                        'x': rounded(st[0]),
                        'y': rounded(st[1]),
                        'psi': st[2],
                        'w': self._obstacle_w,
                        'l': self._obstacle_l,
                        't_arrive': round(self.sim_time, 2),
                        'vehicle_id': vid,
                    }
        # 到达车辆从活动车辆列表移除（避免重复绘制），仅以静态障碍物呈现
        self._drop_vehicle(vid)

    def _drop_vehicle(self, vid):
        self.states.pop(vid, None)
        self.infos.pop(vid, None)
        self.spots.pop(vid, None)
        if self.focus == vid:
            self.focus = 0

    def _update_subs(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception:
            return
        for topic_name, _types in topics:
            m = re.match(r'^/vehicle_([1-9][0-9]*)/state$', topic_name)
            if m:
                vid = int(m.group(1))
                has_pub = bool(self.get_publishers_info_by_topic(topic_name))
                if vid not in self.state_subs and has_pub:
                    self.state_subs[vid] = self.create_subscription(
                        STATE_MSG, topic_name, self._state_cb(vid), 10)
                    self._pending_retire.pop(vid, None)
                    self.get_logger().info('subscribed vehicle %d state' % vid)
                elif vid in self.state_subs and not has_pub:
                    self.destroy_subscription(self.state_subs.pop(vid))
                    # 延迟 1 秒再判定（防发现抖动；离场车届时移除，泊入车保留）
                    self._pending_retire[vid] = time.time() + 0.3
                continue
            m = re.match(r'^/vehicle_([1-9][0-9]*)/info$', topic_name)
            if m:
                vid = int(m.group(1))
                has_pub = bool(self.get_publishers_info_by_topic(topic_name))
                if vid not in self.info_subs and has_pub:
                    self.info_subs[vid] = self.create_subscription(
                        INFO_MSG, topic_name, self._info_cb(vid), 10)
                elif vid in self.info_subs and not has_pub:
                    self.destroy_subscription(self.info_subs.pop(vid))
                continue
            m = re.match(r'^/vehicle_([1-9][0-9]*)/spot$', topic_name)
            if m:
                vid = int(m.group(1))
                has_pub = bool(self.get_publishers_info_by_topic(topic_name))
                if vid not in self.spot_subs and has_pub:
                    self.spot_subs[vid] = self.create_subscription(
                        Int16, topic_name, self._spot_cb(vid), 10)
                elif vid in self.spot_subs and not has_pub:
                    self.destroy_subscription(self.spot_subs.pop(vid))
                continue

    # ---------- 幽灵层（数据集现有智能体回放） ----------
    def _build_ghosts(self):
        opts = self.shared['opts']
        ds = self.shared.get('ds')
        if ds is None or not opts['use_existing_agents']:
            return self._ghosts
        t = max(self.sim_time + opts['dlp_time_offset'], 0.0)
        if self._ghosts_t is not None and abs(t - self._ghosts_t) < 0.05:
            return self._ghosts
        self._ghosts_t = t
        try:
            frame = ds.get_frame_at_time(scene_token=self.shared['scene_token'], timestamp=t)
        except Exception:
            return self._ghosts
        ghosts = []
        for inst_token in frame.get('instances', []):
            inst = ds.get('instance', inst_token)
            agent = ds.get('agent', inst.get('agent_token'))
            if agent is None or agent.get('type') in ('Pedestrian', 'Undefined', 'Bicycle'):
                continue
            size = agent.get('size') or [4.6, 1.85]
            ghosts.append([rounded(inst['coords'][0]), rounded(inst['coords'][1]),
                           rounded(inst['heading']), rounded(size[0]), rounded(size[1])])
        self._ghosts = ghosts
        return ghosts

    # ---------- 主 tick ----------
    def _color_index(self, vid):
        info = self.infos.get(vid)
        if not info or info['is_all_done']:
            return 3
        if info['is_braking']:
            return 2
        if info['task'] in ('PARK', 'UNPARK'):
            return 1
        return 0

    def _tick(self):
        self._update_subs()
        # 处理延迟退役（发布者消失的车辆）
        now = time.time()
        for vid, due in list(self._pending_retire.items()):
            if now >= due:
                self._pending_retire.pop(vid, None)
                self._retire_vehicle(vid)
        vehicles = []
        for vid in sorted(set(list(self.states.keys()) + list(self.infos.keys()))):
            st = self.states.get(vid)
            if st is None:
                continue
            info = self.infos.get(vid) or {}
            vehicles.append({
                'id': vid, 'x': st[0], 'y': st[1], 'psi': st[2],
                'v': st[3] if len(st) > 3 else None,
                'c': self._color_index(vid),
                'label': info.get('disp_text', ''),
                'spot': self.spots.get(vid),
            })
        self._seq += 1
        fpath = None
        if self.focus:
            fpath = (self.infos.get(self.focus) or {}).get('path')
        frame = {
            'type': 'frame',
            'seq': self._seq,
            't': rounded(self.sim_time),
            'vehicles': vehicles,
            'ghosts': self._build_ghosts(),
            'static_obstacles': [dict(self.static_garage[k]) for k in sorted(self.static_garage)],
            'fpath': fpath,
        }
        with self.shared['lock']:
            self.shared['frame'] = frame
            self.shared['running'] = (not self.paused)
            # 暂停标记（与 running 区分）：running 在停止/结束态同样为 False，
            # 不能拿来判暂停；sim_watchdog 用 sim_paused 区分
            # 「时钟被设计成冻结」与真的停滞。
            self.shared['sim_paused'] = bool(self.paused)
        if self.status_pub is not None:
            msg = Bool()
            msg.data = not self.paused
            self.status_pub.publish(msg)
        rec = self.shared.get('recorder')
        if rec is not None:
            rec.write(frame)

    # ---------- 控制 ----------
    def set_paused(self, value):
        self.paused = bool(value)
        # 立即同步到 shared（不必等下一帧发布），供 sim_watchdog 判暂停
        try:
            with self.shared['lock']:
                self.shared['sim_paused'] = bool(self.paused)
        except Exception:
            pass

    def clear_vehicles(self):
        """重启前清空车辆缓存与聚焦（桥进程存活，等待新仿真重新发布）。"""
        self.states.clear()
        self.infos.clear()
        self.spots.clear()
        self._pending_retire.clear()
        self.static_garage.clear()
        self.focus = 0
        self.sim_time = 0.0
        self.paused = False
        try:
            with self.shared['lock']:
                self.shared['sim_paused'] = False
        except Exception:
            pass
        self._ghosts = []
        self._ghosts_t = None


class FrameRecorder(object):
    """将帧流写入 JSONL（每个仿真时刻一条）。"""

    def __init__(self, path):
        self.path = path
        self.f = open(path, 'a', buffering=1)
        self.last_t = None

    def write(self, frame):
        t = frame.get('t')
        if t == self.last_t:
            return
        self.last_t = t
        self.f.write(json.dumps(frame, separators=(',', ':')) + '\n')

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# 静态载荷与配置
# --------------------------------------------------------------------------

def _norm_gate(v):
    """标准化出入口数据 → {'x','y','heading'}（兼容 dict(coords/heading) 与 [x,y]）。"""
    try:
        if isinstance(v, dict):
            c = v.get('coords') or [v.get('x'), v.get('y')]
            h = v.get('heading')
            return {'x': round(float(c[0]), 2), 'y': round(float(c[1]), 2),
                    'heading': (round(float(h), 4) if h is not None else None)}
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            return {'x': round(float(v[0]), 2), 'y': round(float(v[1]), 2), 'heading': None}
    except Exception:
        pass
    return None


def _front_gate_gates(head, off, in_dir):
    """由门顶锚点 + 行驶方向右侧偏移推导前端入口/出口标记。"""
    import math as _m
    dx, dy = in_dir
    L = (dx * dx + dy * dy) ** 0.5 or 1.0
    u = (dx / L, dy / L)
    rx, ry = u[1], -u[0]
    ent = {'x': round(head[0] + off * rx, 2), 'y': round(head[1] + off * ry, 2),
           'heading': round(_m.atan2(dy, dx), 4)}
    ux, uy = -u[0], -u[1]
    rxx, ryy = uy, -ux
    ext = {'x': round(head[0] + off * rxx, 2), 'y': round(head[1] + off * ryy, 2),
           'heading': round(_m.atan2(-dy, -dx), 4)}
    return ent, ext


def _gates_from_map_yaml(root, map_name):
    """从 config/maps/<name>.yaml 读 entrance/exit（DJI 等非地图包结构）。
    当该地图无独立 yaml（如合并后的共享底图 DJI_XXXX 占用数据集）时，
    回退到共享权威源 waypoints_graph.pickle 的 entrance_coords/exit_coords，
    保证所有 DJI 占用方案都显示一致的入口/出口（与共享几何一致）。"""
    # DJI 占用数据集共享同一套门区几何；前端标记与 vehicle_node 使用同一推导。
    if str(map_name).startswith('DJI'):
        return _front_gate_gates([14.38, 76.21], 1.75, (0.0, -1.0))

    if root:
        try:
            p = os.path.join(root, 'workspace', 'src', 'parksim', 'config', 'maps', '%s.yaml' % map_name)
            if os.path.isfile(p):
                with open(p) as f:
                    mc = yaml.safe_load(f) or {}
                if mc.get('entrance') is not None or mc.get('exit') is not None:
                    return _norm_gate(mc.get('entrance')), _norm_gate(mc.get('exit'))
        except Exception:
            pass
        # 回退：共享路网权威出入口（DJI 系共享几何；占用数据集间出入口一致）
        try:
            import pickle as _pickle
            wpp = os.path.join(root, 'python', 'parksim', 'priorFiles', 'waypoints_graph.pickle')
            if os.path.isfile(wpp):
                _wp = _pickle.load(open(wpp, 'rb'))
                # DJI 共享几何：入口主轴 255 (14.38, 70.9) 为两条双车道弧的公共起始顶点。
                #   入口起始点 = 枢纽 255（右弧起始端）
                #   出口驶离点 = 枢纽 255（左弧起始端）
                #   （双向双车道同一分叉点起止）
                return _front_gate_gates([14.38, 76.21], 1.75, (0.0, -1.0))
        except Exception:
            pass
    return None, None


# --------------------------------------------------------------------------
# 多出入口（portal）→ 扁长「门」矩形
# --------------------------------------------------------------------------
# 数据优先级：
#   ① <地图目录>/waypoints_graph.pickle（schema_version >= 2 且含 portals）
#   ② outputs/jth_b1_refined/directed_centerline_graph.json（CAD 交付，含门几何）
#   ③ 都没有 → 返回 []，前端回退到旧的单点 entrance/exit 画法
#
# 每个 gate：{id, x, y, heading, long, thick, roles, entry_side}
#   x,y     = 门中心（rotated 米制）
#   heading = 门**法向**（过门行驶方向）；门长轴角 = heading + 90°
#   long    = 门长（沿长轴）  thick = 门厚（沿法向）

_GATE_THICK_M = 2.0            # 门厚（米）
_GATE_LONG_FALLBACK_M = 3.5    # 门长兜底 = 2 × lane_half_width_m

# CAD 门宽兜底表：**最后一道**防线。
# 权威门宽来自地图包（portal 的 width_m / section_rotated_m），其次 CAD 交付 JSON
# （outputs/…_refined/）。本表只在两者都拿不到时生效（例如部署环境的 pickle 还是
# 没写 width_m 的旧快照、且仓库里没有 outputs/）。地图包一旦补齐即可删除本表。
_GATE_LONG_FALLBACK_BY_ID = {
    'P1': 11.40,
    'P2': 6.60,
    'P3': 8.153,
    'P4': 7.764,
}


def _portal_width(portal, pid, cad_long, long_fb):
    """门宽（沿长轴）取值优先级：

    ① portal['width_m']            —— 地图包权威值（推荐，建图侧已 baking）
    ② portal['section_rotated_m']  —— 地图包门洞端点距（rotated 米制，直接用，不自算）
    ③ CAD 交付 JSON 的 section_drawing_units 端点距
    ④ _GATE_LONG_FALLBACK_BY_ID
    ⑤ 2 × lane_half_width_m
    """
    w = portal.get('width_m')
    if w:
        return float(w)
    sec = portal.get('section_rotated_m') or []
    if len(sec) >= 2:
        return ((float(sec[-1][0]) - float(sec[0][0])) ** 2
                + (float(sec[-1][1]) - float(sec[0][1])) ** 2) ** 0.5
    if cad_long.get(str(pid)):
        return float(cad_long[str(pid)])
    if _GATE_LONG_FALLBACK_BY_ID.get(str(pid)):
        return float(_GATE_LONG_FALLBACK_BY_ID[str(pid)])
    return float(long_fb)


def _delivery_json_path(root, map_name):
    """CAD 交付的 directed_centerline_graph.json（优先 <map>_refined）。"""
    cands = []
    if map_name:
        cands.append(os.path.join(root or '', 'outputs', '%s_refined' % map_name,
                                  'directed_centerline_graph.json'))
    cands.append(os.path.join(root or '', 'outputs', 'jth_b1_refined',
                              'directed_centerline_graph.json'))
    for p in cands:
        if root and os.path.isfile(p):
            return p
    return None


def _rotated_from_metric(mt):
    """由 metric_transform 生成 (x_cad, y_cad) -> rotated 米制 的闭包。

    公式（与 jth_loader.CoordinateSystems 一致）：
        x_m = x_cad * unit_to_meter ; y_m = y_cad * unit_to_meter
        xr  = y_m - y_min_m         ; yr  = x_max_m - x_m
    """
    try:
        s = float(mt['unit_to_meter'])
        xmax = float(mt['x_max_m'])
        ymin = float(mt['y_min_m'])
    except Exception:
        return None

    def _to_rot(p):
        return (float(p[1]) * s - ymin, xmax - float(p[0]) * s)

    return _to_rot


def _cad_gate_lengths(root, map_name):
    """从 CAD 交付 JSON 取 {portal_id: 门宽(米)} = section 端点距（rotated 米制）。

    门宽只是观感参数：解析失败返回空 dict，由调用方用车道宽度兜底。
    """
    out = {}
    path = _delivery_json_path(root, map_name)
    if not path:
        return out
    try:
        with open(path) as f:
            d = json.load(f)
        to_rot = _rotated_from_metric(d.get('metric_transform') or {})
        if to_rot is None:
            return out
        for p in (d.get('portals') or []):
            pid = p.get('id')
            sect = p.get('section_drawing_units') or []
            if not pid or len(sect) < 2:
                continue
            a = to_rot(sect[0])
            b = to_rot(sect[-1])
            out[str(pid)] = round(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5, 3)
    except Exception as exc:
        print('[webviz] gates: CAD 门宽解析失败（%s）' % exc)
    return out


def _entry_side(anchor, heading, spawn, exit_target):
    """判定双向门「进/出」各占哪半边：沿门长轴的符号（+1 / -1）。

    spawn（入场车道端点）所在侧为 entry；只有 exit_target 时取其反侧。
    """
    import math as _m
    tx = _m.cos(heading + _m.pi / 2)
    ty = _m.sin(heading + _m.pi / 2)
    ax = float(anchor[0])
    ay = float(anchor[1])
    if spawn:
        d = (float(spawn[0]) - ax) * tx + (float(spawn[1]) - ay) * ty
        if abs(d) > 1e-6:
            return 1 if d > 0 else -1
    if exit_target:
        d = (float(exit_target[0]) - ax) * tx + (float(exit_target[1]) - ay) * ty
        if abs(d) > 1e-6:
            return -1 if d > 0 else 1
    return -1


def _mk_gate(pid, x, y, heading, long_m, roles, entry_side, section=None):
    """统一构造一个 gate 记录。

    ``section`` 是权威「门洞截面」两端点（rotated 米制，来自 portal 的
    ``section_rotated_m``）。有它时前端直接拿这两个点当长边，不再按 heading
    反推长轴 —— 因为门的物理朝向由截面决定，而 heading（过门行驶方向）未必
    与截面严格垂直（P2/P3 实测差 ~3.7°）。
    """
    gate = {
        'id': str(pid),
        'x': round(float(x), 3),
        'y': round(float(y), 3),
        'heading': round(float(heading), 5),
        'long': round(float(long_m), 3),
        'thick': _GATE_THICK_M,
        'roles': list(roles),
        'entry_side': int(entry_side),
    }
    if section and len(section) >= 2:
        try:
            gate['section'] = [[round(float(section[0][0]), 3), round(float(section[0][1]), 3)],
                               [round(float(section[-1][0]), 3), round(float(section[-1][1]), 3)]]
        except Exception:
            pass
    return gate


def _gates_from_pickle(root, map_name, cad_long):
    """① waypoints_graph.pickle：仅 schema_version >= 2 且含 portals 时使用。"""
    gates = []
    if not (root and map_name):
        return gates
    path = os.path.join(root, 'python', 'parksim', 'priorFiles', 'maps',
                        map_name, 'waypoints_graph.pickle')
    if not os.path.isfile(path):
        return gates
    import pickle as _pickle
    try:
        with open(path, 'rb') as f:
            data = _pickle.load(f)
    except Exception as exc:
        print('[webviz] gates: pickle 不可读（%s）；回退 CAD 交付 JSON' % exc)
        return gates
    if not isinstance(data, dict):
        return gates
    if int(data.get('schema_version') or 0) < 2 or not data.get('portals'):
        return gates
    ent = set(str(i) for i in (data.get('entrance_portal_ids') or []))
    ext = set(str(i) for i in (data.get('exit_portal_ids') or []))
    half = float(data.get('lane_half_width_m') or 1.75)
    long_fb = round(2.0 * half, 3)
    for p in data['portals']:
        pid = p.get('id')
        anchor = p.get('anchor_rotated_m')
        if not pid or not anchor:
            continue
        roles = []
        if str(pid) in ent:
            roles.append('entrance')
        if str(pid) in ext:
            roles.append('exit')
        if not roles:
            continue
        heading = float(p.get('heading_rad') or 0.0)
        long_m = _portal_width(p, pid, cad_long, long_fb)
        section = p.get('section_rotated_m')
        ax, ay = float(anchor[0]), float(anchor[1])
        # ENTRY_STUB 对齐：进场弧的出生位沿**来向上游**推移了 entry_stub_m
        #   spawn = anchor + lane_half_width·right − heading·entry_stub_m
        # 门洞截面是 CAD 权威位置（对账用，不因建图改动），但**入口的可视化位置
        # 必须与出生点一致**——否则车出生在门线上游 2.36 m 处、门却画在门线，
        # 观感上「车不是从门里出来的」。故这里把门框沿 −heading 平移同样的距离，
        # 使门线与 spawn 落在同一条横线上；横向仍保持门洞中心（不随车道偏移）。
        stub = 0.0
        if 'entrance' in roles:
            try:
                stub = float(p.get('entry_stub_m') or 0.0)
            except (TypeError, ValueError):
                stub = 0.0
        if stub > 1e-9:
            ux = math.cos(heading) * stub
            uy = math.sin(heading) * stub
            ax -= ux
            ay -= uy
            if section and len(section) >= 2:
                section = [[float(section[0][0]) - ux, float(section[0][1]) - uy],
                           [float(section[-1][0]) - ux, float(section[-1][1]) - uy]]
        gates.append(_mk_gate(pid, ax, ay, heading, long_m, roles,
                              _entry_side((ax, ay), heading,
                                          p.get('spawn_pose_rotated_m'),
                                          p.get('exit_target_rotated_m')),
                              section))
    return gates


# ---------------------------------------------------------------------------
# 已知口径差异（不是 bug，别再当缺陷查）：
# P3 的门长轴在两条数据路径下差 3.82°——路径① 93.82°，路径② 90.00°。
#   路径① 用 pickle 的 heading_rad = 0.0666：建图侧对「纯 exit」portal 取的是
#          弧的**弦向** atan2，不是首段切向。
#   路径② 用 CAD 交付 JSON 弧末端位姿的朝向：R30_F 末两段位姿的 heading 都是 0.0。
# 两个值都对，只是建图口径不同；差 3.82° 对可视化无影响，画法（长轴 = heading+90°，
# P1/P4 沿 X、P2/P3 沿 Y）保持不变。线上（服务器）走路径①，取 93.82°。
# ---------------------------------------------------------------------------


def _portal_node_id(pid, node_ids):
    """portal 物理 id（P2）→ 图节点 id（P2_OUT）。"""
    for n in node_ids:
        if str(n) == str(pid):
            return str(n)
    for n in node_ids:
        if str(n).startswith('%s_' % pid):
            return str(n)
    return None


def _portal_heading(arcs, node_id, roles, portal, to_rot):
    """门法向 = portal 节点所在有向弧在该端的行驶朝向。

    - 入口门：取「从该节点出发」的弧，用首点朝向（进场方向）
    - 出口门：取「到达该节点」的弧，用末点朝向（离场方向）
    - 取不到：退化为 CAD 截面长轴 + 90°
    """
    import math as _m
    want_out = 'entrance' in (roles or [])
    if node_id:
        ordered = []
        for a in arcs:
            poses = a.get('waypoint_poses_rotated_m') or []
            if not poses:
                continue
            if want_out and a.get('from') == node_id and len(poses[0]) >= 3:
                ordered.append((0, float(poses[0][2])))
            elif (not want_out) and a.get('to') == node_id and len(poses[-1]) >= 3:
                ordered.append((0, float(poses[-1][2])))
        for _, h in ordered:
            return h
        for a in arcs:                                    # 反向兜底
            poses = a.get('waypoint_poses_rotated_m') or []
            if not poses:
                continue
            if a.get('from') == node_id and len(poses[0]) >= 3:
                return float(poses[0][2])
            if a.get('to') == node_id and len(poses[-1]) >= 3:
                return float(poses[-1][2])
    if to_rot is not None:
        sect = portal.get('section_drawing_units') or []
        if len(sect) >= 2:
            a = to_rot(sect[0])
            b = to_rot(sect[-1])
            return _m.atan2(b[1] - a[1], b[0] - a[0]) + _m.pi / 2
    return 0.0


def _gates_from_delivery_json(root, map_name, cad_long):
    """② CAD 交付 JSON 的 portals（Mac 本地 pickle 为旧 schema 时走这条）。"""
    gates = []
    path = _delivery_json_path(root, map_name)
    if not path:
        return gates
    try:
        with open(path) as f:
            d = json.load(f)
        to_rot = _rotated_from_metric(d.get('metric_transform') or {})
        if to_rot is None:
            return gates
        nodes = dict((n.get('id'), n) for n in (d.get('nodes') or []))
        arcs = list(d.get('directed_arcs') or [])
        ent_ids = [str(x) for x in (d.get('entrance_nodes') or [])]
        ext_ids = [str(x) for x in (d.get('exit_nodes') or [])]
        for p in (d.get('portals') or []):
            pid = str(p.get('id') or '')
            if not pid:
                continue
            roles = []
            if _portal_node_id(pid, ent_ids):
                roles.append('entrance')
            if _portal_node_id(pid, ext_ids):
                roles.append('exit')
            if not roles:
                continue
            centre = p.get('centre_drawing_units')
            node_id = _portal_node_id(pid, ent_ids + ext_ids)
            if centre:
                cx, cy = to_rot(centre)
            elif node_id and (nodes.get(node_id) or {}).get('rotated_m'):
                rm = nodes[node_id]['rotated_m']
                cx, cy = float(rm[0]), float(rm[1])
            else:
                continue
            heading = _portal_heading(arcs, node_id, roles, p, to_rot)
            long_m = _portal_width(p, pid, cad_long, _GATE_LONG_FALLBACK_M)
            # CAD 交付没有现成的 rotated 截面，用与门宽同一套变换换算
            section = None
            sect_raw = p.get('section_drawing_units') or []
            if len(sect_raw) >= 2:
                try:
                    section = [list(to_rot(sect_raw[0])), list(to_rot(sect_raw[-1]))]
                except Exception:
                    section = None
            gates.append(_mk_gate(pid, cx, cy, heading, long_m, roles, -1, section))
    except Exception as exc:
        print('[webviz] gates: CAD 交付 JSON 解析失败（%s）' % exc)
        return []
    return gates


def _load_map_gates(root, map_name):
    """地图出入口「门」列表；任何异常都不能让 webviz 起不来。"""
    try:
        cad_long = _cad_gate_lengths(root, map_name)
        gates = _gates_from_pickle(root, map_name, cad_long)
        if gates:
            print('[webviz] gates: %d from waypoints_graph.pickle %s'
                  % (len(gates), cad_long))
            return gates
        gates = _gates_from_delivery_json(root, map_name, cad_long)
        if gates:
            print('[webviz] gates: %d from CAD delivery JSON %s'
                  % (len(gates), cad_long))
            return gates
    except Exception as exc:
        print('[webviz] gates: 解析失败（%s）；回退单点 entrance/exit' % exc)
        return []
    print('[webviz] gates: 无 portal 数据；回退单点 entrance/exit')
    return []


def build_static_payload(ds, dlpvis, opts, root=None):
    # 新地图：地图目录带 layout_rotated.json 时，静态层直接采用其车位/路网
    # （与仿真/车辆使用同一套「旋转后」坐标系；不依赖 DLP 的 DJI 车位表）
    map_name = str(opts.get('map') or '').strip()
    if root and map_name:
        lay_path = os.path.join(root, 'python', 'parksim', 'priorFiles', 'maps',
                                map_name, 'layout_rotated.json')
        if os.path.isfile(lay_path):
            try:
                with open(lay_path) as _f:
                    lay = json.load(_f)
                spots = []
                for s in lay['spots']:
                    flat = []
                    for pt in s['quad']:
                        flat.extend([round(float(pt[0]), 3), round(float(pt[1]), 3)])
                    spots.append(flat)
                xs = [p[0] for p in lay['graph']['nodes']]
                ys = [p[1] for p in lay['graph']['nodes']]
                wp_pts = []
                for e in lay['graph']['edges']:
                    for pt in e['pts'][::3]:
                        wp_pts.append([round(float(pt[0]), 3), round(float(pt[1]), 3)])
                ramps = []
                _lvl = 'B1'
                _entry = None
                _exit = None
                rules_path = os.path.join(os.path.dirname(lay_path), 'map_rules.json')
                if os.path.isfile(rules_path):
                    try:
                        _r = json.load(open(rules_path))
                        ramps = _r.get('ramps') or []
                        _lvl = _r.get('level') or 'B1'
                        _entry = _norm_gate(_r.get('entrance'))
                        _exit = _norm_gate(_r.get('exit'))
                    except Exception:
                        pass
                # 多出入口「门」（portal）：pickle(schema2) → CAD 交付 JSON → 单点回退
                _gates = _load_map_gates(root, map_name)
                # 硬障碍物（obstacles.json，整图重建包提供）
                obstacles = []
                obs_path = os.path.join(os.path.dirname(lay_path), 'obstacles.json')
                if os.path.isfile(obs_path):
                    try:
                        _o = json.load(open(obs_path))
                        for ob in (_o.get('obstacles') or []):
                            poly = ob.get('polygon') or []
                            if len(poly) >= 3:
                                obstacles.append([list(p) for p in poly])
                    except Exception:
                        pass
                # 底图（map_overlay.png 等）
                _base_url = ''
                _bgt = None
                for _cand in ('base_map_clean.png', 'map_overlay.png', 'jth_base_map.png'):
                    if os.path.isfile(os.path.join(os.path.dirname(lay_path), _cand)):
                        _base_url = '/maps/%s/%s' % (map_name, _cand)
                        break
                # 底图变换（PNG 像素 → 旋转世界坐标）：
                #   世界→PNG：px = (sx/u2m)*(x_max - yr) + tx ; py = ty - (sy/u2m)*(xr + y_min)
                #   反解：xr = -kx*py + cx ; yr = -ky*px + cy
                if _base_url.endswith(('map_overlay.png', 'base_map_clean.png')):
                    try:
                        _meta = lay.get('meta') or {}
                        _tr = _meta.get('transform') or {}
                        _u2m = float(_meta.get('unit_to_meter') or 0.2)
                        _xmax = float(_tr.get('x_max_m'))
                        _ymin = float(_tr.get('y_min_m'))
                        _ext_p = os.path.join(os.path.dirname(lay_path), 'tools', 'parking_extraction.json')
                        if os.path.isfile(_ext_p):
                            _ex = json.load(open(_ext_p))
                            _it = ((_ex.get('metadata') or {}).get('image_transform') or {})
                            _sx = float(_it['sx']); _sy = float(_it['sy'])
                            _tx = float(_it['tx']); _ty = float(_it['ty'])
                            _bgt = {
                                'kx': _u2m / _sy, 'cx': _ty * _u2m / _sy - _ymin,
                                'ky': _u2m / _sx, 'cy': _xmax + _tx * _u2m / _sx,
                            }
                            print('[webviz] base map transform: kx=%.6f cx=%.3f ky=%.6f cy=%.3f'
                                  % (_bgt['kx'], _bgt['cx'], _bgt['ky'], _bgt['cy']))
                    except Exception as _exc:
                        print('[webviz] base map transform calc failed: %s' % _exc)
                # 分区概览图（可选图层）：sidecar zone_overview.json 与对应 PNG 均在时才下发
                _zone_url = ''
                _zone_t = None
                _zo_path = os.path.join(os.path.dirname(lay_path), 'zone_overview.json')
                if os.path.isfile(_zo_path):
                    try:
                        _zo = json.load(open(_zo_path))
                        _zo_img = str(_zo.get('image') or 'zone_overview.png')
                        if os.path.isfile(os.path.join(os.path.dirname(lay_path), _zo_img)):
                            _zone_url = '/maps/%s/%s' % (map_name, _zo_img)
                            _zone_t = {
                                'kx': float(_zo['kx']), 'cx': float(_zo['cx']),
                                'ky': float(_zo['ky']), 'cy': float(_zo['cy']),
                            }
                            # 完整仿射优先：该图带约 -0.1 度的微小旋转，
                            # 只用 kx/cx/ky/cy 会丢掉交叉项，实测中位误差 0.279 m（超 0.3 m 的 p95）
                            _aff = _zo.get('affine')
                            if isinstance(_aff, dict) and all(k in _aff for k in ('a', 'b', 'c', 'd', 'e', 'f')):
                                _zone_t['affine'] = {k: float(_aff[k]) for k in ('a', 'b', 'c', 'd', 'e', 'f')}
                            print('[webviz] zone overview: %s kx=%.6f cx=%.3f ky=%.6f cy=%.3f'
                                  % (_zone_url, _zone_t['kx'], _zone_t['cx'],
                                     _zone_t['ky'], _zone_t['cy']))
                    except Exception as _exc:
                        print('[webviz] zone overview sidecar failed: %s' % _exc)
                        _zone_url = ''
                        _zone_t = None
                # 全内容边界（spots + graph + obstacles + entrance/exit）+ 边距
                _cx = []; _cy = []
                for _s in lay['spots']:
                    for _pt in _s['quad']:
                        _cx.append(float(_pt[0])); _cy.append(float(_pt[1]))
                for _p in lay['graph']['nodes']:
                    _cx.append(float(_p[0])); _cy.append(float(_p[1]))
                for _ob in obstacles:
                    for _pt in _ob:
                        _cx.append(float(_pt[0])); _cy.append(float(_pt[1]))
                if _entry: _cx.append(_entry['x']); _cy.append(_entry['y'])
                if _exit: _cx.append(_exit['x']); _cy.append(_exit['y'])
                _M = 8.0
                _cbounds = [min(_cx) - _M, min(_cy) - _M, max(_cx) + _M, max(_cy) + _M] if _cx else None
                print('[webviz] static from map layout: %s (%d spots, %d ramps, %d obstacles)' % (
                    map_name, len(spots), len(ramps), len(obstacles)))
                if _cbounds:
                    print('[webviz] content bounds: x[%.1f..%.1f] y[%.1f..%.1f]' % (
                        _cbounds[0], _cbounds[2], _cbounds[1], _cbounds[3]))
                return {
                    'type': 'init',
                    'map_size': {'x': round(max(xs) + 6) if xs else 200,
                                 'y': round(max(ys) + 6) if ys else 100},
                    'scale': 10,
                    'spots': spots,
                    'waypoints': {'jth': wp_pts},
                    'ramps': ramps,
                    'level': _lvl,
                    'obstacles': obstacles,
                    'entrance': _entry,
                    'exit': _exit,
                    'gates': _gates,
                    'base_map_transform': _bgt,
                    'row_fill': False,
                    'content_bounds': _cbounds,
                    'colors': opts['colors'],
                    'options': {
                        'control': opts['control'],
                        'dlp_time_offset': opts['dlp_time_offset'],
                        'use_existing_agents': opts['use_existing_agents'],
                        'base_map_url': _base_url,
                        'zone_overview_url': _zone_url,
                        'zone_overview_transform': _zone_t,
                        'manage_sim': opts.get('manage_sim', False),
                        'schemes': opts.get('schemes', {}),
                        'maps': opts.get('maps', []),
                        'occupancy_datasets': opts.get('occupancy_datasets', []),
                        'map': opts.get('map', ''),
                        'init_defaults': opts.get('init_defaults', {}),
                        'obstacle_mode': opts.get('obstacle_mode', 'dataset'),
                        'has_experience': opts.get('has_experience', True),
                        'current': opts.get('current', {}),
                    },
                    'scene_token': 'jth',
                }
            except Exception as exc:
                print('[webviz] map layout payload failed (%s); fallback to DLP dataset' % exc)

    spots = np.round(dlpvis.parking_spaces.iloc[:, 2:10].to_numpy(), 3).tolist()
    waypoints = {k: np.round(np.asarray(arr), 3).tolist() for k, arr in dlpvis.waypoints.items()}
    scene_token = ds.list_scenes()[0]
    scene = ds.get('scene', scene_token)
    obstacles = []
    for o_token in scene['obstacles']:
        o = ds.get('obstacle', o_token)
        corners = dlpvis._get_corners(o['coords'], o['size'], o['heading'])
        obstacles.append(np.round(corners, 3).tolist())
    base_map = os.path.join(os.path.dirname(dlp.__file__), 'base_map.png')
    # 出入口（config/maps/<name>.yaml 的 entrance/exit；DJI 由 yaml 配置）
    _d_entry, _d_exit = _gates_from_map_yaml(root, map_name)
    return {
        'type': 'init',
        'map_size': dlpvis.map_size,
        'scale': 10,
        'spots': spots,
        'waypoints': waypoints,
        'ramps': [],
        'level': '',
        'row_fill': False,
        'obstacles': obstacles,
        'entrance': _d_entry,
        'exit': _d_exit,
        'gates': [],          # DLP/DJI 无 portal 数据 → 前端走单点 entrance/exit
        'colors': opts['colors'],
        'options': {
            'control': opts['control'],
            'dlp_time_offset': opts['dlp_time_offset'],
            'use_existing_agents': opts['use_existing_agents'],
            'base_map_url': ('/base_map.png?v=flip2' if os.path.exists(base_map) else ''),
            'manage_sim': opts.get('manage_sim', False),
            'has_experience': opts.get('has_experience', True),
            'schemes': opts.get('schemes', {}),
            'maps': opts.get('maps', []),
            'occupancy_datasets': opts.get('occupancy_datasets', []),
            'map': opts.get('map', ''),
            'init_defaults': opts.get('init_defaults', {}),
            'obstacle_mode': opts.get('obstacle_mode', 'dataset'),
            'current': opts.get('current', {}),
        },
        'scene_token': scene_token,
    }


def load_configs(root):
    cfg_dir = os.path.join(root, 'workspace', 'src', 'parksim', 'config')
    with open(os.path.join(cfg_dir, 'global_params.yaml')) as f:
        gp = expand_config_value(yaml.safe_load(f), root)
    with open(os.path.join(cfg_dir, 'visualization.yaml')) as f:
        vis = expand_config_value(yaml.safe_load(f), root)
    vp = {}
    if isinstance(vis, dict):
        vp = vis.get('visualizer', {}).get('ros__parameters', {}) or {}
    colors = {k: list(vp.get(k + '_color', DEFAULT_COLORS[k])) for k in COLOR_KEYS}

    # 场景模式（scenario.yaml 优先；缺省回退旧参数 use_existing_agents）
    scenario_mode = 'replay' if bool(gp.get('use_existing_agents', False)) else 'random'
    try:
        scen_path = os.path.join(cfg_dir, 'scenario.yaml')
        if os.path.isfile(scen_path):
            with open(scen_path) as f:
                scen = yaml.safe_load(f) or {}
            scenario_mode = str((scen.get('mode') if isinstance(scen, dict) else None) or scenario_mode)
    except Exception:
        pass
    return gp, vp, colors, scenario_mode


def collect_init_defaults(root):
    """默认场景文件（scenario.yaml）的初始化默认值：mode + random/replay 块 + custom_file。"""
    cfg_dir = os.path.join(root, 'workspace', 'src', 'parksim', 'config')
    out = {'mode': 'random', 'random': {}, 'replay': {}, 'custom_file': ''}
    try:
        with open(os.path.join(cfg_dir, 'scenario.yaml')) as f:
            data = yaml.safe_load(f) or {}
        if isinstance(data, dict):
            out['mode'] = str(data.get('mode') or 'random')
            if isinstance(data.get('random'), dict):
                out['random'] = data['random']
            if isinstance(data.get('replay'), dict):
                out['replay'] = data['replay']
            custom = data.get('custom') or {}
            if isinstance(custom, dict):
                out['custom_file'] = str(custom.get('file') or '')
    except Exception:
        pass
    return out


def list_maps(root):
    """扫描可用地图（底图）：DJI 系列共享同一底图，合并为单一 'DJI'；
    另有 maps/ 目录的地图包（ready: true）。
    DJI_XXXX 的差异仅在于泊位初始占用状态（经验初始化数据集），不入底图列表。"""
    names = []
    try:
        data_dir = os.path.join(root, 'python', 'parksim', 'priorFiles', 'data')
        has_dji = any(f.startswith('DJI_') and f.endswith('_scene.json')
                      for f in os.listdir(data_dir))
        if has_dji:
            names.append('DJI')
    except Exception:
        pass
    try:
        maps_dir = os.path.join(root, 'python', 'parksim', 'priorFiles', 'maps')
        for dn in sorted(os.listdir(maps_dir)):
            yf = os.path.join(maps_dir, dn, 'map.yaml')
            if not os.path.isfile(yf):
                continue
            try:
                txt = open(yf, encoding='utf-8', errors='replace').read()
                if 'ready: true' in txt.replace(' ', '') or 'ready:true' in txt.replace(' ', ''):
                    names.append(dn)
            except Exception:
                pass
    except Exception:
        pass
    return sorted(set(names))


def list_occupancy_datasets(root):
    """枚举 DJI 系列的占用数据集（底图 DJI 下的经验初始化选择）。
    DJI_XXXX = 不同的泊位初始占用状态（agents_data_XXXX.pickle + 障碍车辆）。"""
    out = []
    try:
        data_dir = os.path.join(root, 'python', 'parksim', 'priorFiles', 'data')
        for fn in sorted(os.listdir(data_dir)):
            if fn.startswith('DJI_') and fn.endswith('_scene.json'):
                out.append('DJI_' + fn[len('DJI_'):-len('_scene.json')])
    except Exception:
        pass
    return out


def map_has_experience(root, map_name):
    """地图是否具备场景（经验）数据——决定「经验初始化」选项是否可用。
    DJI 系列共享地图数据、各场景有 agents_data；JTH 无场景数据 → 仅随机/自定义。"""
    if map_name.startswith('DJI'):
        # 共享底图：DJI（及 DJI_XXXX 占用数据集）均有经验/占用数据集
        return bool(list_occupancy_datasets(root))
    candidates = []
    preset = os.path.join(root, 'workspace', 'src', 'parksim', 'config', 'maps', map_name + '.yaml')
    if os.path.isfile(preset):
        try:
            _d = yaml.safe_load(open(preset, encoding='utf-8', errors='replace')) or {}
            if isinstance(_d, dict) and _d.get('agents_data_path'):
                candidates.append(expand_config_value(_d['agents_data_path'], root))
        except Exception:
            pass
    pkg = os.path.join(root, 'python', 'parksim', 'priorFiles', 'maps', map_name, 'map.yaml')
    if os.path.isfile(pkg):
        try:
            _d = yaml.safe_load(open(pkg, encoding='utf-8', errors='replace')) or {}
            if isinstance(_d, dict) and _d.get('agents_data_path'):
                candidates.append(expand_config_value(_d['agents_data_path'], root))
        except Exception:
            pass
    candidates.append(os.path.join(root, 'python', 'parksim', 'priorFiles',
                                   'agents_data_%s.pickle' % map_name))
    for p in candidates:
        if not isinstance(p, str) or 'none' in os.path.basename(p).lower():
            continue
        if os.path.isfile(p):
            return True
    return False


def resolve_dlp_prefix(root, map_name):
    """解析数据场景对应的 DLP 前缀路径（DJI_XXXX → priorFiles/data/DJI_XXXX）。"""
    prior = os.path.join(root, 'python', 'parksim', 'priorFiles')
    preset = os.path.join(root, 'workspace', 'src', 'parksim', 'config', 'maps', map_name + '.yaml')
    if os.path.isfile(preset):
        try:
            with open(preset) as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data, dict) and data.get('dlp_path'):
                return expand_config_value(data['dlp_path'], root)
        except Exception:
            pass
    return os.path.join(prior, 'data', map_name)


# --------------------------------------------------------------------------
# 仿真进程托管（--manage-sim：由桥负责启动 / 停止 / 重启仿真）
# --------------------------------------------------------------------------

class SimManager(object):
    """托管 ros2 launch 仿真进程，支持按方案参数重启。

    仅当桥以 --manage-sim 启动时启用；桥退出（Ctrl+C）时一并停止仿真。
    """

    KEYS = ('scenario', 'map', 'init_mode', 'allocation_method', 'route_planner', 'ref_path_generator', 'maneuver_provider')
    SWEEP_PATTERNS = ('ros2 launch parksim', 'parksim.*simulator_node', 'parksim.*vehicle_node')

    def __init__(self, root, initial=None):
        self.root = root
        self.lock = threading.Lock()
        self.proc = None
        self.config = self._clean(initial or {})
        self.busy = False

    # ---------- 配置清洗（仅保留白名单字段并做类型归一） ----------
    @staticmethod
    def _num(value):
        if isinstance(value, bool):
            return None
        if isinstance(value, (int, float)):
            return value
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return None
            try:
                num = float(text)
                return int(num) if num.is_integer() else num
            except ValueError:
                return None
        return None

    @staticmethod
    def _int_list(value):
        if value is None:
            return None
        if isinstance(value, str):
            value = [p for p in re.split(r'[,\s;]+', value) if p]
        if not isinstance(value, (list, tuple)):
            return None
        out = []
        for item in value:
            num = SimManager._num(item)
            if num is not None:
                out.append(int(num))
        return out

    @classmethod
    def _clean(cls, config):
        out = {}
        for key in cls.KEYS:
            value = config.get(key)
            if value:
                out[key] = str(value)
        params = config.get('params') if isinstance(config, dict) else None
        if isinstance(params, dict):
            cleaned = {}
            rand = params.get('random')
            if isinstance(rand, dict):
                vals = {}
                for key in ('seed', 'entering', 'exiting'):
                    if key in rand:
                        num = cls._num(rand[key])
                        if num is not None:
                            vals[key] = int(num)
                for key in ('interval_mean', 'y_bound'):
                    if key in rand:
                        num = cls._num(rand[key])
                        if num is not None:
                            vals[key] = float(num)
                if 'entering_spot_pool' in rand:
                    lst = cls._int_list(rand['entering_spot_pool'])
                    if lst is not None:
                        vals['entering_spot_pool'] = lst
                occ_in = rand.get('occupancy')
                if isinstance(occ_in, dict):
                    occ = {}
                    for okey in ('blocked', 'occupied'):
                        lst = cls._int_list(occ_in.get(okey))
                        if lst is not None:
                            occ[okey] = lst
                    if occ:
                        vals['occupancy'] = occ
                occr_in = rand.get('occupancy_random')
                if isinstance(occr_in, dict):
                    occr = {}
                    if 'enable' in occr_in:
                        occr['enable'] = bool(occr_in['enable'])
                    if 'count' in occr_in:
                        num = cls._num(occr_in['count'])
                        if num is not None:
                            occr['count'] = int(num)
                    if 'ratio' in occr_in:
                        num = cls._num(occr_in['ratio'])
                        if num is not None:
                            occr['ratio'] = float(num)
                    if occr:
                        vals['occupancy_random'] = occr
                if vals:
                    cleaned['random'] = vals
            rep = params.get('replay')
            if isinstance(rep, dict):
                vals = {}
                for key, as_float in (('time_scale', True), ('max_agents', False)):
                    if key in rep:
                        num = cls._num(rep[key])
                        if num is not None:
                            vals[key] = float(num) if as_float else int(num)
                if vals:
                    cleaned['replay'] = vals
            if cleaned:
                out['params'] = cleaned
        return out

    def _write_runtime_scenario(self):
        """以「预设/默认场景文件」为基座，叠加网页细参数，生成运行时场景文件。"""
        cfg_dir = os.path.join(self.root, 'workspace', 'src', 'parksim', 'config')
        name = self.config.get('scenario', '') or ''
        if name in ('', 'default'):
            base_path = os.path.join(cfg_dir, 'scenario.yaml')
        elif os.path.isabs(name) or name.endswith('.yaml') or os.sep in name or '/' in name:
            base_path = name if os.path.isabs(name) else os.path.join(cfg_dir, name)
        else:
            base_path = os.path.join(cfg_dir, 'scenarios', name + '.yaml')

        data = {}
        try:
            if os.path.isfile(base_path):
                with open(base_path) as f:
                    loaded = yaml.safe_load(f) or {}
                if isinstance(loaded, dict):
                    data = loaded
        except Exception as exc:
            print('[webviz] warning: failed reading base scenario %s (%s)' % (base_path, exc))

        init_mode = self.config.get('init_mode')
        if init_mode in ('random', 'replay', 'custom'):
            data['mode'] = init_mode

        for key in ('allocation_method', 'route_planner', 'ref_path_generator', 'maneuver_provider'):
            value = self.config.get(key)
            if value:
                data[key] = value

        params = self.config.get('params') or {}
        for block in ('random', 'replay'):
            block_vals = params.get(block)
            if isinstance(block_vals, dict) and block_vals:
                merged = dict(data.get(block) or {})
                merged.update(block_vals)
                data[block] = merged

        runtime_path = os.path.join(self.root, 'webviz_runtime.yaml')
        with open(runtime_path, 'w') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        return runtime_path, data

    def _sync_global_params(self, data):
        """把运行方案的随机种子/模式回写进参与仿真启动的 global_params.yaml。

        原因：parksim.launch.py 只把 simulator.yaml + global_params.yaml 作为
        simulator 节点 ROS 参数，webviz_runtime.yaml 并不被节点读取。因此要在每次
        启动时把方案里的 random.seed 与 use_existing_agents 同步到 global_params，
        使 simulator_node 的 np.random.seed(self.random_seed) 真正采用网页种子，
        从而不同随机实验可显式控制、同种子可复现。
        """
        gpath = os.path.join(self.root, 'workspace', 'src', 'parksim', 'config', 'global_params.yaml')
        if not os.path.isfile(gpath):
            return
        try:
            with open(gpath) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            return
        # 种子：方案 random.seed（缺省 0）
        try:
            seed = int((data.get('random') or {}).get('seed') or 0)
        except (TypeError, ValueError):
            seed = 0
        mode = str(data.get('mode') or 'random')
        cfg['random_seed'] = seed
        cfg['use_existing_agents'] = (mode == 'replay')
        try:
            with open(gpath, 'w') as f:
                yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
            print('[webviz] synced global_params.yaml: random_seed=%d, use_existing_agents=%r (mode=%s)' % (
                seed, cfg['use_existing_agents'], mode))
        except Exception as exc:
            print('[webviz] warning: failed syncing global_params.yaml (%s)' % exc)

    def _build_cmd(self):
        runtime_path, data = self._write_runtime_scenario()
        self._sync_global_params(data)
        cmd = ['ros2', 'launch', 'parksim', 'parksim.launch.py', 'gui:=false',
               'scenario:=%s' % runtime_path]
        if self.config.get('map'):
            cmd.append('map:=%s' % self.config['map'])
        return cmd

    def start(self, config=None):
        """（重新）启动仿真；config 为 None 时沿用当前方案。"""
        with self.lock:
            if config is not None:
                self.config = self._clean(config)
            self._stop_locked()
            cmd = self._build_cmd()
            print('[webviz] starting simulator: %s' % ' '.join(cmd))
            self.proc = subprocess.Popen(cmd, cwd=os.path.join(self.root, 'python'),
                                         start_new_session=True)
            proc = self.proc
        # 启动失败快速反馈（正常仿真不会退出）
        time.sleep(6)
        if proc.poll() is not None:
            raise RuntimeError('simulator exited early (code %s)，请检查参数或日志' % proc.returncode)
        return dict(self.config)

    def restart(self, config):
        """按新方案重启仿真（阻塞：停止 → 清理 → 启动）。"""
        return self.start(config)

    def stop(self):
        with self.lock:
            self._stop_locked()

    def _stop_locked(self):
        proc = self.proc
        self.proc = None
        if proc is not None and proc.poll() is None:
            print('[webviz] stopping simulator (pid %d)...' % proc.pid)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)   # 优雅：simulator 会关闭各车辆
            except Exception:
                pass
            deadline = time.time() + 12
            while time.time() < deadline and proc.poll() is None:
                time.sleep(0.2)
            forced = False
            if proc.poll() is None:
                forced = True
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                time.sleep(2)
            if proc.poll() is None:
                forced = True
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            if forced:
                # 仅当自己的仿真被强制终止时才清扫残留（已知孤儿）；
                # 其余情况一律不清扫，避免误杀其他正在运行的新实例。
                print('[webviz] cleaning residual parksim processes...')
                for pattern in self.SWEEP_PATTERNS:
                    subprocess.call(['pkill', '-f', pattern], stderr=subprocess.DEVNULL)
                time.sleep(2)
        elif proc is not None and proc.poll() is not None:
            # 仿真异常退出（崩溃/被杀）：只清理真正的孤儿进程（ppid==1），
            # 不影响其他实例的在树进程。
            self._clean_orphans()
        # proc 为 None（从未启动）→ 不做任何清扫

    def _clean_orphans(self):
        """清理脱离父进程的仿真/车辆孤儿进程（ppid == 1），避免污染下一轮。"""
        try:
            out = subprocess.check_output(['ps', '-eo', 'pid=,ppid=,args='], text=True)
        except Exception:
            return
        killed = 0
        for line in out.splitlines():
            parts = line.strip().split(None, 2)
            if len(parts) < 3:
                continue
            pid, ppid, args = parts
            if ppid != '1':
                continue
            if any(re.search(pat, args) for pat in self.SWEEP_PATTERNS):
                try:
                    os.kill(int(pid), signal.SIGKILL)
                    killed += 1
                except Exception:
                    pass
        if killed:
            print('[webviz] cleaned %d orphan parksim process(es)' % killed)


def parse_launch_args(text):
    """解析 "k:=v k2:=v2" 形式的 launch 参数（仅保留方案相关键）。"""
    config = {}
    for token in str(text or '').split():
        if ':=' in token:
            key, _, value = token.partition(':=')
            if key in SimManager.KEYS and value:
                config[key] = value
    return config


# --------------------------------------------------------------------------
# aiohttp 应用
# --------------------------------------------------------------------------

async def broadcast_all(shared, payload):
    """向所有已连接客户端广播一条消息（供监控任务等使用）。"""
    text = json.dumps(payload, separators=(',', ':'))
    for client in list(shared['clients']):
        try:
            await client.send_str(text)
        except Exception:
            pass


def create_app(shared, node):
    here = Path(__file__).resolve().parent
    static_dir = here / 'static'

    async def index(request):
        # 首页本身 no-store，但 /static/app.js 带 ?v= 版本戳会被浏览器长期缓存。
        # 这里按 app.js / style.css 的 mtime 重写版本戳：改了前端后普通刷新即可生效，
        # 不必再手工改 index.html 里的 ?v=，避免「代码已部署但页面还在跑旧版」。
        try:
            import re as _re
            _html = (static_dir / 'index.html').read_text(encoding='utf-8')
            for _name in ('app.js', 'style.css'):
                _p = static_dir / _name
                if _p.exists():
                    _v = str(int(_p.stat().st_mtime))
                    _html = _re.sub(r'(/%s\?v=)[^"\']*' % _re.escape(_name),
                                    lambda _m, _v=_v: _m.group(1) + _v, _html)
            return web.Response(text=_html, content_type='text/html',
                                headers={'Cache-Control': 'no-store'})
        except Exception:
            return web.FileResponse(str(static_dir / 'index.html'),
                                    headers={'Cache-Control': 'no-store'})

    async def base_map(request):
        p = os.path.join(os.path.dirname(dlp.__file__), 'base_map.png')
        if os.path.exists(p):
            try:
                # DJI 底图沿 X 轴翻折（上下翻转）以匹配仿真坐标：DLP 权威即对 base_map 做 FLIP_TOP_BOTTOM
                buf = _pio.BytesIO()
                _PILOps.flip(_PILImage.open(p).convert('RGB')).save(buf, 'PNG')
                buf.seek(0)
                return web.Response(body=buf.getvalue(), content_type='image/png',
                                    headers={'Cache-Control': 'no-store'})
            except Exception:
                return web.FileResponse(p)
        raise web.HTTPNotFound()

    async def map_file(request):
        name = request.match_info['path']
        base = os.path.realpath(os.path.join(shared['root'], 'python', 'parksim',
                                             'priorFiles', 'maps'))
        full = os.path.realpath(os.path.join(base, name))
        if not full.startswith(base + os.sep) or not os.path.isfile(full):
            raise web.HTTPNotFound()
        return web.FileResponse(full)

    async def ws_handler(request):
        ws = web.WebSocketResponse(max_msg_size=2 ** 22, heartbeat=30)
        await ws.prepare(request)
        init = shared['init']
        # 新连接同步运行状态机快照：sim_state / spawn（spawn 从未收到时为 null）
        _state, _spawn = sim_state_tracker.snapshot()
        init_payload = dict(init)
        init_payload['sim_state'] = _state
        init_payload['spawn'] = _spawn
        # 启动期关键资产自检结果（degraded 标记 + 缺失清单）随 init 下发（前向兼容：
        # 字段缺失时前端不报错）
        init_payload['degraded'] = bool(shared.get('degraded'))
        init_payload['assets_missing'] = list(shared.get('assets_missing') or [])
        await ws.send_str(json.dumps(init_payload, separators=(',', ':')))
        # 回放当前仍生效的告警（跨重连保持横幅；无告警时不发任何消息）
        for _alert in sim_state_tracker.active_alerts():
            await ws.send_str(json.dumps(_alert, separators=(',', ':')))
        # 连接时同步当前仿真状态（未启动→idle；已退出→sim_dead）
        sm = shared.get('sim_manager')
        if sm is not None:
            if sm.proc is None:
                await ws.send_str(json.dumps({
                    'type': 'status', 'value': 'idle',
                    'message': '底图已加载（仿真未启动）'}, separators=(',', ':')))
            else:
                code = sm.proc.poll()
                if code is not None:
                    await ws.send_str(json.dumps({
                        'type': 'status', 'value': 'sim_dead', 'code': code,
                        'message': '仿真进程已退出（code %s），可点击重启恢复' % code},
                        separators=(',', ':')))
        shared['clients'].add(ws)
        sender = asyncio.ensure_future(_frame_sender(ws, shared))

        async def broadcast(payload):
            text = json.dumps(payload, separators=(',', ':'))
            for client in list(shared['clients']):
                try:
                    await client.send_str(text)
                except Exception:
                    pass

        def _reload_scene(scene_name, loop):
            """切换底图：DJI 场景共享同一底图（dlpvis 常驻内存，无需重载 DLP）；
            地图包走 layout_rotated 分支。只重建静态载荷并广播。
            loop 由调用方（ws_handler 协程内）传入，供跨线程广播。"""
            print('[webviz] reloading scene: %s ...' % scene_name)
            t0 = time.time()
            shared['opts']['map'] = scene_name
            shared['opts']['has_experience'] = map_has_experience(
                shared['root'], scene_name)
            # 底图即当前地图选择：同步到仿真配置，避免后续「开启仿真」沿用旧地图
            try:
                _sm = shared.get('sim_manager')
                if _sm is not None:
                    with _sm.lock:
                        _sm.config['map'] = scene_name
            except Exception as _exc:
                print('[webviz] sync scene map to sim config failed: %s' % _exc)
            payload = build_static_payload(shared['ds'], shared['dlpvis'],
                                           shared['opts'], root=shared.get('root'))
            # 切换底图后重算关键资产自检，degraded 字段随新 init 载荷广播
            update_asset_status(shared, payload=payload, log=True)
            shared['init'] = payload
            asyncio.run_coroutine_threadsafe(broadcast(payload), loop)
            print('[webviz] scene reloaded in %.1fs' % (time.time() - t0))

        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except Exception:
                    continue
                mtype = data.get('type')
                if mtype == 'pause':
                    node.set_paused(bool(data.get('value')))
                    await broadcast({'type': 'paused', 'value': bool(node.paused)})
                elif mtype == 'focus':
                    try:
                        fid = int(data.get('id') or 0)
                    except Exception:
                        fid = 0
                    node.focus = fid if fid > 0 else 0
                    await ws.send_str(json.dumps({'type': 'focus', 'id': node.focus}))
                elif mtype == 'switch_map':
                    # 仅切换底图（不重启仿真）：选择地图后自动加载底图
                    map_name = str(data.get('map') or '').strip()
                    if not map_name:
                        continue
                    loop2 = asyncio.get_event_loop()

                    def _switch_worker():
                        try:
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'map_switching',
                                'message': '正在停止仿真并加载底图：%s' % map_name}), loop2)
                            # 切断原仿真：底图切换与运行中的仿真不再匹配
                            sim_manager = shared.get('sim_manager')
                            if sim_manager is not None:
                                sim_manager.stop()
                            sim_state_tracker.set_stopped()
                            node.clear_vehicles()
                            _reload_scene(map_name, loop2)
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'map_switched',
                                'map': map_name,
                                'message': '底图已切换为 %s（原仿真已停止，可用仿真方案开启）' % map_name}),
                                loop2)
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'idle',
                                'message': '底图已加载（仿真未启动）'}), loop2)
                        except Exception as exc:
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'error',
                                'message': '底图切换失败：%s' % exc}), loop2)

                    threading.Thread(target=_switch_worker, daemon=True).start()
                elif mtype == 'restart':
                    sim_manager = shared.get('sim_manager')
                    if sim_manager is None or not node.control:
                        await ws.send_str(json.dumps({
                            'type': 'status', 'value': 'error',
                            'message': '重启功能未启用（桥需以 --control --manage-sim 启动）'}))
                        continue
                    if sim_manager.busy:
                        await ws.send_str(json.dumps({'type': 'status', 'value': 'busy'}))
                        continue
                    config = data.get('config') or {}
                    sim_manager.busy = True
                    loop = asyncio.get_event_loop()

                    def _restart_worker():
                        try:
                            req_map = str((config or {}).get('map') or '').strip()
                            loaded_map = shared.get('opts', {}).get('map')
                            init_mode = str((config or {}).get('init_mode') or '')
                            if init_mode:
                                shared['opts']['obstacle_mode'] = (
                                    'occupancy' if init_mode == 'random' else 'dataset')
                            shared['opts']['current'] = SimManager._clean(config)
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'restarting',
                                'config': SimManager._clean(config)}), loop)
                            # restart 期间沿用 'restarting' 广播，状态机视同 starting
                            sim_state_tracker.set_starting()
                            node.clear_vehicles()
                            if req_map and req_map != loaded_map:
                                _reload_scene(req_map, loop)
                            sim_manager.restart(config)
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'started',
                                'config': dict(sim_manager.config), 'paused': False}), loop)
                        except Exception as exc:
                            sim_state_tracker.set_stopped()
                            asyncio.run_coroutine_threadsafe(broadcast({
                                'type': 'status', 'value': 'error',
                                'message': str(exc)}), loop)
                        finally:
                            sim_manager.busy = False

                    threading.Thread(target=_restart_worker, daemon=True).start()
                elif mtype == 'stop':
                    sim_manager = shared.get('sim_manager')
                    if sim_manager is None or not node.control:
                        await ws.send_str(json.dumps(
                            {'type': 'stop_failed', 'message': '只读模式'},
                            separators=(',', ':')))
                        continue
                    if sim_manager.busy:
                        await ws.send_str(json.dumps(
                            {'type': 'stop_failed',
                             'message': '仿真忙（启动/重启进行中），请稍后'},
                            separators=(',', ':')))
                        continue
                    sim_manager.busy = True
                    loop = asyncio.get_event_loop()

                    def _stop_worker():
                        try:
                            # SimManager.stop() 内部持 sim.lock 做 SIGINT→SIGTERM→SIGKILL 清理
                            sim_manager.stop()
                            node.clear_vehicles()
                            sim_state_tracker.set_stopped()

                            async def _notify_stopped():
                                try:
                                    await ws.send_str(json.dumps(
                                        {'type': 'stopped'}, separators=(',', ':')))
                                except Exception:
                                    pass
                                await broadcast({'type': 'status', 'value': 'stopped'})

                            asyncio.run_coroutine_threadsafe(_notify_stopped(), loop)
                            print('[webviz] simulator stopped via WS stop request')
                        except Exception as exc:
                            sim_state_tracker.set_stopped()

                            async def _notify_failed():
                                try:
                                    await ws.send_str(json.dumps(
                                        {'type': 'stop_failed', 'message': str(exc)},
                                        separators=(',', ':')))
                                except Exception:
                                    pass

                            asyncio.run_coroutine_threadsafe(_notify_failed(), loop)
                        finally:
                            sim_manager.busy = False

                    threading.Thread(target=_stop_worker, daemon=True).start()
                elif mtype == 'ping':
                    await ws.send_str(json.dumps({'type': 'pong', 't': data.get('t')}))
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        finally:
            shared['clients'].discard(ws)
            sender.cancel()
        return ws

    async def _frame_sender(ws, shared):
        last_seq = None
        last_occ = None
        last_dep = None
        try:
            while True:
                with shared['lock']:
                    frame = shared.get('frame')
                    running = shared.get('running', True)
                    occ = shared.get('occupancy')
                    occ_seq = shared.get('occupancy_seq', 0)
                    dep = shared.get('departing')
                    dep_seq = shared.get('departing_seq', 0)
                if occ is not None and occ_seq != last_occ:
                    await ws.send_str(json.dumps({'type': 'occupancy', 'data': occ}, separators=(',', ':')))
                    last_occ = occ_seq
                if dep is not None and dep_seq != last_dep:
                    await ws.send_str(json.dumps({'type': 'departing', 'data': dep}, separators=(',', ':')))
                    last_dep = dep_seq
                if frame is not None and frame.get('seq') != last_seq:
                    payload = dict(frame)
                    payload['running'] = running
                    await ws.send_str(json.dumps(payload, separators=(',', ':')))
                    last_seq = frame.get('seq')
                await asyncio.sleep(1 / 20.0)
        except (asyncio.CancelledError, ConnectionResetError):
            return
        except Exception:
            return

    async def sim_watchdog():
        """仿真健康监控：进程退出 → sim_dead；数据流停滞 → stalled（事件式广播）。"""
        state = None
        last_t = None
        last_t_change = time.time()
        proc_seen = None
        proc_start = 0.0
        while True:
            await asyncio.sleep(2.0)
            try:
                sm = shared.get('sim_manager')
                if sm is None:
                    continue
                proc = sm.proc
                if proc is not proc_seen:
                    # 进程切换（新仿真启动 / 停止后 proc 置 None）：若上一轮残留
                    # stalled，先广播 resumed 清掉红色停滞横幅，再复位本轮基准。
                    if state == 'stalled':
                        state = ('alive',)
                        sim_state_tracker.note_dataflow_stall(False)
                        await broadcast_all(shared, {'type': 'status', 'value': 'resumed'})
                    proc_seen = proc
                    state = None
                    last_t = None
                    last_t_change = time.time()
                    proc_start = time.time()
                    sim_state_tracker.note_dataflow_stall(False)
                if proc is None:
                    # 无仿真进程（未启动 / 已停止 / restart 间隙）：不做停滞判定
                    continue
                if proc is not None:
                    code = proc.poll()
                    if code is not None:
                        # 仿真意外退出（非 stop/restart 触发：那两条路径先把 proc 置 None）
                        sim_state_tracker.set_stopped()
                        if state != ('dead', code):
                            state = ('dead', code)
                            await broadcast_all(shared, {
                                'type': 'status', 'value': 'sim_dead', 'code': code,
                                'message': '仿真进程已退出（code %s），可点击重启恢复' % code})
                            await broadcast_all(shared, {'type': 'status', 'value': 'stopped'})
                        continue
                # 暂停期间仿真时钟被**设计成**冻结：跳过帧驱动的状态机推进与
                # 停滞判定，并持续刷新 last_t_change —— 否则恢复瞬间会把暂停期间
                # 累积的时长一次算成停滞（误报）。若暂停发生时已误报 stalled，
                # 这里主动解除，避免红色横幅残留到整个暂停期间。
                paused = bool(shared.get('sim_paused'))
                if paused:
                    last_t_change = time.time()
                    if state == 'stalled':
                        state = ('alive',)
                        sim_state_tracker.note_dataflow_stall(False)
                        await broadcast_all(shared, {'type': 'status', 'value': 'resumed'})
                    continue
                frame = shared.get('frame')
                if frame is not None:
                    # 运行状态机的帧驱动迁移（starting→running→finished）
                    for ev in sim_state_tracker.note_frame(
                            frame.get('t'), len(frame.get('vehicles') or [])):
                        await broadcast_all(shared, ev)
                    t = frame.get('t')
                    if t != last_t:
                        last_t = t
                        last_t_change = time.time()
                        if state == 'stalled':
                            state = ('alive',)
                            # 既有 stalled 解除 → 允许 stalled_no_progress 重新评估
                            sim_state_tracker.note_dataflow_stall(False)
                            await broadcast_all(shared, {'type': 'status', 'value': 'resumed'})
                        elif state is None:
                            state = ('alive',)
                    elif state != 'stalled' and (time.time() - proc_start) > 45:
                        stalled = int(time.time() - last_t_change)
                        if stalled > 20:
                            state = 'stalled'
                            # 既有 stalled 生效 → 抑制 stalled_no_progress，避免重复上报
                            sim_state_tracker.note_dataflow_stall(True)
                            await broadcast_all(shared, {
                                'type': 'status', 'value': 'stalled', 'seconds': stalled,
                                'message': '仿真数据流停滞约 %d 秒' % stalled})
            except Exception:
                pass

    async def on_startup(app):
        app['sim_watchdog'] = asyncio.ensure_future(sim_watchdog())

    async def on_cleanup(app):
        task = app.get('sim_watchdog')
        if task is not None:
            task.cancel()

    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/base_map.png', base_map)
    app.router.add_get('/maps/{path:.*}', map_file)
    app.router.add_static('/static/', path=str(static_dir))
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


# --------------------------------------------------------------------------
# 入口
# --------------------------------------------------------------------------

def build_arg_parser():
    ap = argparse.ArgumentParser(
        prog='server.py', description='ParkSim webviz bridge (HTML/WebSocket)')
    ap.add_argument('--host', default='0.0.0.0', help='监听地址（默认 0.0.0.0）')
    ap.add_argument('--port', type=int, default=8099, help='监听端口（默认 8099）')
    ap.add_argument('--control', action='store_true',
                    help='发布 /sim_status（网页可暂停/恢复仿真）；默认只读观看')
    ap.add_argument('--record', action='store_true',
                    help='录制帧流到 $PARKSIM_ROOT/webviz_recordings/*.jsonl')
    ap.add_argument('--manage-sim', action='store_true',
                    help='托管仿真：由桥负责启动/停止/重启 ros2 launch（网页可换方案）')
    ap.add_argument('--launch-args', default='',
                    help='初始仿真 launch 参数（如 "allocation_method:=graph_cost route_planner:=dijkstra"）')
    ap.add_argument('--auto-start', action='store_true',
                    help='启动桥时立即启动仿真（默认否：仅加载底图，由网页「开启仿真」按钮启动）')
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    root = find_repo_root()
    os.environ.setdefault('PARKSIM_ROOT', root)
    print('[webviz] repo root: %s' % root)

    gp, vp, colors, scenario_mode = load_configs(root)

    presets = []
    try:
        scen_dir = os.path.join(root, 'workspace', 'src', 'parksim', 'config', 'scenarios')
        presets = sorted(f[:-5] for f in os.listdir(scen_dir) if f.endswith('.yaml'))
    except Exception:
        pass

    initial_config = parse_launch_args(getattr(args, 'launch_args', ''))
    # 逐泊位机动表是 JTH 的正确选择（legacy offline 表会偏离泊位约 4.5 m 且无避障）。
    # launch_args 未显式指定时默认使用它，保证仿真器崩溃自动重启后也不会回退到 offline。
    if not initial_config.get('maneuver_provider'):
        initial_config['maneuver_provider'] = 'per_spot'
    init_defaults = collect_init_defaults(root)
    map_name = str(initial_config.get('map') or 'DJI_0012')
    obstacle_mode = 'occupancy' if ((initial_config.get('init_mode') or scenario_mode) == 'random') else 'dataset'
    opts = {
        'control': bool(args.control),
        'colors': colors,
        'dlp_time_offset': float(vp.get('dlp_time_offset', -3)),
        'scenario_mode': scenario_mode,
        'use_existing_agents': scenario_mode == 'replay',
        'manage_sim': bool(getattr(args, 'manage_sim', False)),
        'has_experience': map_has_experience(root, map_name),
        'maps': list_maps(root),
        'occupancy_datasets': list_occupancy_datasets(root),
        'map': map_name,
        'init_defaults': init_defaults,
        'obstacle_mode': obstacle_mode,
        'schemes': {
            'scenario': [''] + presets,
            'allocation_method': ['random', 'nearest_entrance', 'graph_cost', 'balanced_rows', 'manual'],
            'route_planner': ['astar', 'dijkstra', 'via'],
            'ref_path_generator': ['spline', 'linear'],
            'maneuver_provider': ['offline', 'online_rs', 'per_spot'],
        },
        'current': initial_config,
    }

    # 数据场景（map）解析：桥以同一场景加载数据集（与仿真端 map 参数一致）
    dlp_path = resolve_dlp_prefix(root, map_name)
    if not os.path.exists(dlp_path + '_scene.json'):
        print('[webviz] warning: %s_scene.json 不存在，回退 global_params dlp_path' % dlp_path)
        dlp_path = gp['dlp_path']

    print('[webviz] loading DLP dataset: %s (map=%s)' % (dlp_path, map_name))
    t0 = time.time()
    ds = Dataset()
    ds.load(dlp_path)
    dlpvis = DlpVisualizer(ds)
    print('[webviz] DLP ready in %.1fs; building static layers...' % (time.time() - t0))

    shared = {
        'lock': threading.Lock(),
        'frame': None,
        'running': True,
        'opts': opts,
        'ds': ds,
        'dlpvis': dlpvis,
        'scene_token': ds.list_scenes()[0],
        'clients': set(),
        'root': root,
        'occupancy': None,
        'departing': None,
        'occupancy_seq': 0,
    }
    shared['init'] = build_static_payload(ds, dlpvis, opts, root=root)
    print('[webviz] init payload: %.1f KB' % (len(json.dumps(shared['init'])) / 1024.0))
    # 启动期关键资产自检：缺失时逐条 ERROR 打印并置 degraded，随 init 下发（不阻断启动）
    update_asset_status(shared, payload=shared['init'], log=True)

    recorder = None
    if args.record:
        rec_dir = os.path.join(root, 'webviz_recordings')
        os.makedirs(rec_dir, exist_ok=True)
        rec_path = os.path.join(rec_dir, 'run_%s.jsonl' % time.strftime('%Y%m%d_%H%M%S'))
        recorder = FrameRecorder(rec_path)
        shared['recorder'] = recorder
        recorder.write({'type': 'meta', 'scenario_mode': scenario_mode,
                        'scenario_file': os.path.join(root, 'workspace', 'src', 'parksim', 'config', 'scenario.yaml')})
        print('[webviz] recording to %s' % rec_path)

    rclpy.init()
    node = BridgeNode(shared, control=opts['control'])
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sim_manager = None
    if opts['manage_sim']:
        sim_manager = SimManager(root, initial=initial_config)
        shared['sim_manager'] = sim_manager
        if getattr(args, 'auto_start', False):
            sim_state_tracker.set_starting()
            sim_manager.start()
        else:
            print('[webviz] 底图模式：仅加载底图；仿真由网页「开启仿真」按钮启动'
                  '（如需自动启动：--auto-start）')

    app = create_app(shared, node)
    print('[webviz] serving on http://%s:%d/  (Ctrl+C to stop)' % (args.host, args.port))
    try:
        web.run_app(app, host=args.host, port=args.port)
    except KeyboardInterrupt:
        pass
    finally:
        if sim_manager is not None:
            sim_manager.stop()
        if recorder is not None:
            recorder.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print('[webviz] stopped.')


if __name__ == '__main__':
    main()
