#!/usr/bin/env python3
"""
泊车/出库机动提供者（可插拔）

统一接口：
    provider.get_maneuver(xy_offset, direction, location, spot, heading,
                          task_kind=None, start_pose=None, spot_index=None) -> VehiclePrediction
    返回车辆轨迹 VehiclePrediction（t/x/y/psi/v/u_a/u_steer 等长数组），
    由 agent 按控制周期重采样后回放。

- offline（默认）: 现有 16 组合离线表（parking_maneuvers.pickle）查表，行为与现状完全一致。
- online_rs（实验性）: 依据当前位姿与目标车位，用 Reeds-Shepp 几何在线生成
  （复用仓库内 hybrid_astar/reeds_shepp_path_planning.py）：
    * parking  : 车辆当前位姿 → 车位中心（含终段微调 PARK_END_OFFSET_Y）
    * unparking: [过道位姿 → 当前（停好）位姿]（array 顺序与停车一致，由 agent 逆序回放）
  + 走廊约束（场地边界 / 路径长度）与周围障碍物校验（邻位占用车辆，来自实时
    占用数组）；在全部候选解中取「合法且无碰撞」的最短路径；失败自动回退离线表并计数打点。

实测约定（2026-09-10，车位 208）：离线机动终点 ≈ 车位中心，Δ ≈ (0.01, -0.43)；
在线生成采用相同终点约定（北排 -0.43 / 南排 +0.43），保证两种方案停到同一位置。
"""

import numpy as np

from parksim.pytypes import VehiclePrediction
from parksim.path_planner.hybrid_astar.reeds_shepp_path_planning import calc_paths

# 车位终段偏置（北排取负；南排取对称正值）
PARK_END_OFFSET_Y = -0.43
# 过道对齐点与车位中心的距离（与 agent 巡航航点一致）
DEFAULT_SPOT_Y_OFFSET = 5.0


class _ObstacleReject(ValueError):
    """候选机动因周围障碍物（邻位占用车辆）被拒绝"""


class OfflineManeuverProvider(object):
    """离线机动表（默认，= 现有行为）"""

    name = "offline"

    def __init__(self, offline_maneuver=None, **kwargs):
        assert offline_maneuver is not None, "offline provider requires an OfflineManeuver instance"
        self.offline_maneuver = offline_maneuver

    def get_maneuver(self, xy_offset=[0, 0], direction='east', location='left',
                     spot='north', heading='up', **extras):
        # 忽略扩展参数（task_kind / start_pose / spot_index），行为与现有实现一致
        return self.offline_maneuver.get_maneuver(xy_offset, direction, location, spot, heading)


class OnlineRSManeuverProvider(object):
    """Reeds-Shepp 在线生成（实验性）+ 失败自动回退离线表"""

    name = "online_rs"

    def __init__(self, offline_maneuver=None, parking_spaces=None,
                 spot_y_offset=DEFAULT_SPOT_Y_OFFSET, spot_waypoints=None,
                 spot_headings=None,
                 v_nominal=1.0, step_size=0.1, max_curvature=0.3,
                 occupancy_getter=None, neighbor_span=2,
                 car_half_len=2.45, car_half_wid=1.0, **kwargs):
        self.offline_maneuver = offline_maneuver  # 回退目标
        self.parking_spaces = np.asarray(parking_spaces, dtype=float)
        self.spot_y_offset = float(spot_y_offset)
        self.spot_waypoints = spot_waypoints   # per-spot 车道点（地图规则）
        self.spot_headings = spot_headings     # per-spot 泊位朝向（泊位方向属性）
        self.v_nominal = float(v_nominal)
        self.step_size = float(step_size)
        self.max_curvature = float(max_curvature)

        # 走廊约束：场地包围盒（停车位范围外扩，上方包含入口区）
        self._x_min = float(self.parking_spaces[:, 0].min()) - 5.0
        self._x_max = float(self.parking_spaces[:, 0].max()) + 5.0
        self._y_min = float(self.parking_spaces[:, 1].min()) - 8.0
        self._y_max = float(self.parking_spaces[:, 1].max()) + 8.0

        # 周围障碍物（邻位占用车辆）：occupancy_getter 返回实时占用数组；
        # 为 None 时跳过障碍检查（仅几何/走廊校验）
        self.occupancy_getter = occupancy_getter
        self.neighbor_span = int(neighbor_span)
        self.car_half_len = float(car_half_len)
        self.car_half_wid = float(car_half_wid)

        # 统计打点
        self.num_online = 0
        self.num_fallback = 0
        self.num_obstacle_reject = 0

    # ------------------------------------------------------------------ #
    def get_maneuver(self, xy_offset=[0, 0], direction='east', location='left',
                     spot='north', heading='up', task_kind=None, start_pose=None, spot_index=None):
        try:
            prediction = self._generate(task_kind=task_kind, direction=direction, spot=spot,
                                        heading=heading, start_pose=start_pose, spot_index=spot_index)
            self.num_online += 1
            return prediction
        except Exception as exc:  # 任何几何/数值问题都回退，保证仿真不中断
            self.num_fallback += 1
            print('[online_rs] fallback -> offline (%s)' % exc)
            return self.offline_maneuver.get_maneuver(xy_offset, direction, location, spot, heading)

    # ------------------------------------------------------------------ #
    def _generate(self, task_kind, direction, spot, heading, start_pose, spot_index):
        if spot_index is None or start_pose is None:
            raise ValueError('online_rs requires spot_index and start_pose')

        spot_index = abs(int(spot_index))
        spot_x, spot_y = self.parking_spaces[spot_index]
        sgn = 1.0 if spot == 'north' else -1.0
        # 泊位方向属性优先：停车终点朝向对齐泊位方向（一致或 180° 反转语义）
        if (self.spot_headings is not None and spot_index < len(self.spot_headings)
                and self.spot_headings[spot_index] is not None):
            psi_parked = float(self.spot_headings[spot_index])
        else:
            psi_parked = np.pi / 2 if heading == 'up' else -np.pi / 2
        # 车道点：优先 per-spot waypoint（地图规则），否则回退全局 y 偏移
        aisle_x = None
        if (self.spot_waypoints is not None and spot_index < len(self.spot_waypoints)
                and self.spot_waypoints[spot_index] is not None):
            try:
                _wp = self.spot_waypoints[spot_index]
                aisle_x = float(_wp[0]); aisle_y = float(_wp[1])
            except Exception:
                aisle_x = None
        if aisle_x is None:
            aisle_x = spot_x
            aisle_y = spot_y - sgn * self.spot_y_offset

        if task_kind == 'parking':
            start = (float(start_pose[0]), float(start_pose[1]), float(start_pose[2]))
            goal = (spot_x, spot_y + sgn * PARK_END_OFFSET_Y, psi_parked)
        elif task_kind == 'unparking':
            # array 顺序与停车任务一致： [过道 → 停好]，agent 逆序回放即完成出库
            if aisle_x is not None and abs(aisle_x - spot_x) > 1e-6:
                psi_exit = float(np.arctan2(aisle_y - spot_y, aisle_x - spot_x))
            else:
                psi_exit = np.pi if direction == 'west' else 0.0
            start = (aisle_x, aisle_y, psi_exit)
            goal = (float(start_pose[0]), float(start_pose[1]), float(start_pose[2]))
        else:
            raise ValueError('unknown task_kind %r' % (task_kind,))

        paths = calc_paths(start[0], start[1], start[2],
                           goal[0], goal[1], goal[2],
                           self.max_curvature, self.step_size)
        if not paths:
            raise ValueError('Reeds-Shepp planner produced no path')

        # 提前（生成阶段）纳入周围障碍：在全部候选解中，
        # 取「走廊合法 + 无碰撞」的最短路径；全部失败则抛错回退离线表。
        best = None
        last_exc = None
        for path in sorted(paths, key=lambda p: abs(p.L)):
            try:
                self._check_corridor(path)
                self._check_obstacles(path, spot_index)
            except ValueError as exc:
                last_exc = exc
                continue
            best = path
            break
        if best is None:
            raise ValueError('no collision-free RS path (%s)' % (last_exc,))
        return self._to_prediction(best)

    def _check_corridor(self, path):
        xs = np.asarray(path.x, dtype=float)
        ys = np.asarray(path.y, dtype=float)
        if len(xs) < 2:
            raise ValueError('degenerate RS path')

        length = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
        if length < 0.5:
            raise ValueError('RS path too short (%.2f m)' % length)
        if length > 40.0:
            raise ValueError('RS path too long (%.2f m)' % length)
        if xs.min() < self._x_min or xs.max() > self._x_max:
            raise ValueError('RS path leaves x-bounds')
        if ys.min() < self._y_min or ys.max() > self._y_max:
            raise ValueError('RS path leaves y-bounds')

    def _check_obstacles(self, path, spot_index):
        """障碍校验：候选路径不得与邻位占用车辆（提前纳入的周围障碍）冲突。

        依据实时占用数组（/occupancy，由 agent 注入）判断目标车位 ±neighbor_span
        的同排邻位是否被占用；占用车辆近似为其车位中心处的矩形
        （横向 2*car_half_wid、纵向 2*car_half_len）。占用信息缺失时跳过检查。
        """
        if self.occupancy_getter is None:
            return
        try:
            occ = self.occupancy_getter()
        except Exception:
            return
        if occ is None or len(occ) == 0:
            return

        t = int(spot_index)
        if t < 0 or t >= len(self.parking_spaces):
            return
        tx, ty = self.parking_spaces[t]

        boxes = []
        lo = max(0, t - self.neighbor_span)
        hi = min(len(self.parking_spaces), t + self.neighbor_span + 1)
        for j in range(lo, hi):
            if j == t:
                continue
            cx, cy = self.parking_spaces[j]
            if abs(cy - ty) > 5.0:      # 仅同排邻位（跨排间距 > 9m，已超出走廊）
                continue
            try:
                occupied = bool(occ[j])
            except Exception:
                occupied = False
            if not occupied:
                continue
            boxes.append((cx - self.car_half_wid, cx + self.car_half_wid,
                          cy - self.car_half_len, cy + self.car_half_len))
        if not boxes:
            return

        xs = np.asarray(path.x, dtype=float)
        ys = np.asarray(path.y, dtype=float)
        for x, y in zip(xs, ys):
            for x0, x1, y0, y1 in boxes:
                if x0 <= x <= x1 and y0 <= y <= y1:
                    self.num_obstacle_reject += 1
                    raise _ObstacleReject('conflicts with occupied neighbor spot')

    def _to_prediction(self, path):
        xs = np.asarray(path.x, dtype=float)
        ys = np.asarray(path.y, dtype=float)
        psis = np.unwrap(np.asarray(path.yaw, dtype=float))
        directions = np.asarray(path.directions, dtype=float)

        ds = np.concatenate([[0.0], np.hypot(np.diff(xs), np.diff(ys))])
        t = np.cumsum(ds) / self.v_nominal

        res = VehiclePrediction()
        res.t = t
        res.x = xs
        res.y = ys
        res.psi = psis
        res.v = directions * self.v_nominal
        res.u_a = np.zeros_like(t)
        res.u_steer = np.zeros_like(t)
        return res


class PerSpotManeuverProvider(object):
    """逐泊位离线机动表（方案 D，默认启用）。

    轨迹由 ``parksim.map.build_parking_maneuvers`` 离线生成：
      * 起点 = 该车位的车道接入点，终点 = 泊位中心 + 泊入朝向
      * 已用 obstacles.json 的多边形做车辆矩形扫掠碰撞/间隙筛选
      * **绝对坐标** —— 运行时不得再套用 DJI 时代的 ``park_start_coords ± 4`` 平移

    表是**按 spot_index 索引**的，所以 ``direction/location/spot/heading`` 这些
    DJI 语义组合键在这里全部忽略。
    """

    name = "per_spot"

    #: 运行时据此旁路全局平移（见 RuleBasedStanleyVehicle.update_state_parking）
    provides_absolute = True

    def __init__(self, offline_maneuver=None, printer=None, **kwargs):
        assert offline_maneuver is not None, "per_spot provider requires an OfflineManeuver instance"
        self.offline_maneuver = offline_maneuver      # 同时是回退目标
        self.printer = printer
        # 统计打点
        self.num_per_spot = 0
        self.num_fallback = 0
        self.num_missing = 0
        self.num_degraded = 0
        # 退化解告警按 spot 去重（否则每帧刷屏）
        self._degraded_warned = set()

    def _warn(self, msg):
        """回退必须显式告警，不允许静默。"""
        line = '[per_spot][WARN] %s' % msg
        try:
            import logging
            logging.getLogger('parksim').warning(line)
        except Exception:
            pass
        if callable(self.printer):
            try:
                self.printer(line)
            except Exception:
                pass
        print(line)

    def get_maneuver(self, xy_offset=[0, 0], direction='east', location='left',
                     spot='north', heading='up', **extras):
        spot_index = extras.get('spot_index')
        task_kind = extras.get('task_kind')

        if spot_index is not None:
            prediction = self.offline_maneuver.get_maneuver_for_spot(spot_index)
            if prediction is not None:
                self.num_per_spot += 1
                # ---- 退化解：命中即显式告警（按 spot 去重）--------------
                # 生成器允许在无零侵入解时落地「残余侵入最小」的退化解，
                # 该轨迹带擦碰。这里必须让它可见，否则"停进去其实压着邻位/墙"
                # 在运行时是完全无声的。
                try:
                    man_meta = self.offline_maneuver.get_meta_for_spot(spot_index) or {}
                except Exception:
                    man_meta = {}
                if man_meta.get('degraded'):
                    self.num_degraded += 1
                    key = int(spot_index)
                    if key not in self._degraded_warned:
                        self._degraded_warned.add(key)
                        self._warn(
                            "退化解：spot #%s (%s) 的入库轨迹带残余侵入 depth=%.4f m"
                            "（被侵对象: %s）；该轨迹仍会使用，停车时可能与邻位/障碍擦碰。"
                            % (spot_index, man_meta.get('spot_id', '?'),
                               float(man_meta.get('depth', 0.0)),
                               man_meta.get('victim', '未知')))
                return prediction
            self.num_missing += 1
        else:
            self.num_missing += 1

        # ---- 回退：DJI / 未部署 per-spot 表的地图 ----
        # 明确告警，而不是静默退化（静默退化会让"机动乱停"问题隐形）
        self.num_fallback += 1
        self._warn(
            "per-spot 机动表不可用（spot_index=%s, task=%s, file=%s）；"
            "回退到 legacy 16 组合离线表（该表不含障碍避让，JTH 上终点会偏离泊位）。"
            % (spot_index, task_kind, getattr(self.offline_maneuver, 'per_spot_file', None)))
        return self.offline_maneuver.get_maneuver(xy_offset, direction, location, spot, heading)


_PROVIDERS = {
    OfflineManeuverProvider.name: OfflineManeuverProvider,
    OnlineRSManeuverProvider.name: OnlineRSManeuverProvider,
    PerSpotManeuverProvider.name: PerSpotManeuverProvider,
}


def make_maneuver_provider(name, **kwargs):
    """工厂：按名称实例化机动提供者"""
    key = str(name or 'offline').strip().lower().replace('-', '_')
    if key not in _PROVIDERS:
        raise ValueError("Unknown maneuver provider '%s'. Available: %s" % (
            name, ", ".join(sorted(_PROVIDERS))))
    return _PROVIDERS[key](**kwargs)
