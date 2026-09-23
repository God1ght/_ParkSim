import hashlib
import os
import pickle

import matplotlib.pyplot as plt
import random

import numpy as np

from parksim.pytypes import VehiclePrediction

random.seed(0)


#: 机动表 meta['map_fingerprint'] 覆盖的地图文件（与 build_parking_maneuvers 一致）
MAP_FINGERPRINT_FILES = ("layout_rotated.json", "spots_data.pickle",
                         "obstacles.json", "waypoints_graph.pickle")


def _file_md5(path):
    """返回文件 md5；不存在/读不到返回 None。"""
    try:
        if not path or not os.path.exists(path):
            return None
        h = hashlib.md5()
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None

class OfflineManeuver(object):
    """
    Library of offline maneuver

    Two table shapes are supported:

    * legacy  : ``{(driving_dir, x_position, spot, heading): 7xN ndarray}``
                —— 相对轨迹，运行时按 ``xy_offset`` 平移（DLP/DJI 遗产）。
    * per-spot: ``{"meta": {...}, "spots": {spot_index: {"x","y","psi","v",...}}}``
                —— 由 ``parksim.map.build_parking_maneuvers`` 生成，
                **绝对坐标**、逐泊位、已避开 obstacles.json 的障碍。
    """
    def __init__(self, pickle_file, per_spot_file=None):
        with open(pickle_file, 'rb') as handle:
            self.lib = pickle.load(handle)

        # ---- 逐泊位绝对轨迹表（方案 D；不存在时为 None，走 legacy 行为）----
        self.per_spot = None
        self.per_spot_meta = None
        self.per_spot_file = per_spot_file
        if per_spot_file:
            try:
                with open(per_spot_file, 'rb') as handle:
                    blob = pickle.load(handle)
                if isinstance(blob, dict) and 'spots' in blob:
                    self.per_spot = blob['spots']
                    self.per_spot_meta = blob.get('meta')
            except FileNotFoundError:
                # 交由调用方显式告警；这里保持 None（=legacy 行为）
                self.per_spot = None

        # ---- 机动表 ↔ 地图 指纹校验 -------------------------------------
        # 地图改了而机动表没重算，会让"逐泊位绝对轨迹"整体错位。这里必须
        # 显式告警（含预期值与实际值），绝不允许静默沿用。
        self.fingerprint_checked = False
        self.fingerprint_ok = None          # True / False / None(=无法校验)
        self.fingerprint_mismatches = []    # [(文件名, 预期md5, 实际md5), ...]
        if self.per_spot is not None:
            self.fingerprint_ok = self.verify_map_fingerprint()

    def verify_map_fingerprint(self, _warn=None):
        """比对机动表 meta['map_fingerprint'] 与磁盘上当前地图文件。

        返回 True(一致) / False(不一致) / None(无法校验：旧表缺字段或文件缺失)。
        不一致时**大声告警**（stdout + logging），并把明细留在
        ``self.fingerprint_mismatches``，但**不擅自停表**——是否停用由上层决策。
        """
        say = _warn if callable(_warn) else self._warn
        try:
            meta = self.per_spot_meta or {}
            expected = meta.get('map_fingerprint')
            if not isinstance(expected, dict) or not expected:
                say('[maneuver][WARN] 机动表缺少 map_fingerprint（%s）：'
                    '无法校验它与当前地图是否同源。若地图已变更，'
                    '请重跑 build_parking_maneuvers.py 重新生成。' % self.per_spot_file)
                return None
            # 地图目录**必须**以机动表自己记录的 map_dir 为准，不能用 pickle 所在目录：
            # 生成器支持 --out 写到任意位置（实际流程就是先产出到 /tmp 再拷进地图目录），
            # 用 dirname(per_spot_file) 会让「位置不同」被误报成「地图不一致」。
            map_dir = meta.get('map_dir') or ''
            if not (map_dir and os.path.isdir(map_dir)):
                map_dir = os.path.dirname(self.per_spot_file) if self.per_spot_file else ''
            if not map_dir:
                say('[maneuver][WARN] 无法定位地图目录（meta.map_dir=%s, per_spot_file=%s），'
                    '跳过指纹校验' % (meta.get('map_dir'), self.per_spot_file))
                return None
            self.fingerprint_mismatches = []
            for name in MAP_FINGERPRINT_FILES:
                exp = expected.get(name)
                if not isinstance(exp, dict):
                    continue
                exp_md5 = exp.get('md5')
                if not exp_md5:
                    continue
                actual = _file_md5(os.path.join(map_dir, name))
                if actual is None:
                    self.fingerprint_mismatches.append((name, exp_md5, '<missing>'))
                    continue
                if actual != exp_md5:
                    self.fingerprint_mismatches.append((name, exp_md5, actual))
            self.fingerprint_checked = True
            if self.fingerprint_mismatches:
                detail = '; '.join(
                    '%s: expected=%s actual=%s' % (n, e, a)
                    for (n, e, a) in self.fingerprint_mismatches)
                say('[maneuver][WARN] 机动表与当前地图不一致（%d 个文件），'
                    '逐泊位绝对轨迹可能整体错位，请重跑 build_parking_maneuvers.py：%s'
                    % (len(self.fingerprint_mismatches), detail))
                return False
            return True
        except Exception as exc:      # 校验本身绝不能把仿真搞崩
            try:
                say('[maneuver][WARN] 地图指纹校验异常（%r），已跳过' % (exc,))
            except Exception:
                pass
            return None

    @staticmethod
    def _warn(line):
        """显式告警：logging + stdout（不允许静默）。"""
        try:
            import logging
            logging.getLogger('parksim').warning(line)
        except Exception:
            pass
        try:
            print(line)
        except Exception:
            pass

    def get_meta_for_spot(self, spot_index):
        """取某个泊位机动的 meta（degraded / depth / victim 等）；不可用返回 {}。"""
        if not self.per_spot:
            return {}
        try:
            rec = self.per_spot.get(int(spot_index))
        except Exception:
            return {}
        if rec is None:
            return {}
        m = rec.get('meta')
        return dict(m) if isinstance(m, dict) else {}

    def has_per_spot(self, spot_index=None) -> bool:
        """逐泊位表是否可用（可选指定某个泊位）。"""
        if not self.per_spot:
            return False
        if spot_index is None:
            return True
        return int(spot_index) in self.per_spot

    def get_maneuver_for_spot(self, spot_index) -> VehiclePrediction:
        """取某个泊位的**绝对坐标**泊车机动；不可用返回 None。

        与 ``get_maneuver`` 的区别：轨迹已是绝对坐标，**不要再平移**。
        """
        if not self.per_spot:
            return None
        rec = self.per_spot.get(int(spot_index))
        if rec is None:
            return None
        res = VehiclePrediction()
        res.t = np.asarray(rec['t'], dtype=float)
        res.x = np.asarray(rec['x'], dtype=float)
        res.y = np.asarray(rec['y'], dtype=float)
        res.psi = np.asarray(rec['psi'], dtype=float)
        res.v = np.asarray(rec['v'], dtype=float)          # 带符号（RS 含倒车段）
        res.u_a = np.asarray(rec.get('u_a', np.zeros_like(res.t)), dtype=float)
        res.u_steer = np.asarray(rec.get('u_steer', np.zeros_like(res.t)), dtype=float)
        return res

    def get_maneuver(self, xy_offset=[0,0], 
                        driving_dir=random.choice(['east', 'west']), 
                        x_position=random.choice(['left', 'right']),
                        spot=random.choice(['north', 'south']),
                        heading=random.choice(['up', 'down'])) -> VehiclePrediction:
        print('Trajectory requested:', (driving_dir, x_position, spot, heading))
        
        traj = self.lib[(driving_dir, x_position, spot, heading)]

        res = VehiclePrediction()
        res.t = traj[0, :]
        res.x = traj[1, :] + xy_offset[0]
        res.y = traj[2, :] + xy_offset[1]
        res.psi = traj[3, :]
        res.v = traj[4, :]

        res.u_a = traj[5, :]
        res.u_steer = traj[6, :]

        return res

def main():
    offline_maneuver = OfflineManeuver(pickle_file='parking_maneuvers.pickle')
    state, input = offline_maneuver.get_maneuver()

    plt.figure()

    ax1 = plt.subplot(2,1,1)

    # Plot the entire trajectory
    ax1.plot(state['x'], state['y'])
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_aspect('equal')

    ax2 = plt.subplot(2,1,2)
    ax2.plot(state['t'], state['v'])
    ax2.plot(state['t'], state['yaw'])
    ax2.plot(state['t'], input['a'])
    ax2.plot(state['t'], input['steer'], ':')
    ax2.legend(('speed','angle','accel','steer'))
    ax2.set_xlabel('time')

    plt.show()

if __name__ == "__main__":
    main()