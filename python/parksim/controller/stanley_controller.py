"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
from typing import List
import numpy as np

from parksim.controller_types import StanleyParams
from parksim.pytypes import VehicleState
from parksim.vehicle_types import VehicleBody, VehicleConfig

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class StanleyController(object):
    """
    Stanley Controller
    """
    def __init__(self, control_params: StanleyParams=StanleyParams(), vehicle_body: VehicleBody=VehicleBody(), vehicle_config: VehicleConfig=VehicleConfig()):
        """Instantiate the object."""
        super().__init__()

        self.k = control_params.k
        self.Kp = control_params.Kp
        self.Kp_braking = control_params.Kp_braking
        self.dt = control_params.dt

        self.L = vehicle_body.wb

        self.max_steer = vehicle_config.delta_max

        self.x_ref = []
        self.y_ref = []
        self.yaw_ref = []
        self.v_ref = 0.0

        self.target_idx = None

    def set_ref_pose(self, x_ref: List[float], y_ref: List[float], yaw_ref: List[float]):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.yaw_ref = yaw_ref

    def set_ref_v(self, v_ref: float):
        self.v_ref = v_ref

    def set_target_idx(self, target_idx: int):
        self.target_idx = target_idx

    def calc_target_index(self, state: VehicleState):
        """
        Compute index in the trajectory list of the target.

        :param state: (VehicleState object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position, given state and L (distance b/w front and rear wheels)
        fx = state.x.x + self.L * np.cos(state.e.psi)
        fy = state.x.y + self.L * np.sin(state.e.psi)

        # Search nearest point index（numpy 向量化：路径可能上千点，
        # will_crash_with 前瞻会高频调用，列表推导会成为瓶颈）
        xs = np.asarray(self.x_ref, dtype=float)
        ys = np.asarray(self.y_ref, dtype=float)
        dx = fx - xs
        dy = fy - ys
        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))

        # Project RMS error onto front axle vector
        # this is equivalent to [sin(yaw), -cos(yaw)]
        error_front_axle = (dx[target_idx] * (-np.cos(state.e.psi + np.pi / 2))
                            + dy[target_idx] * (-np.sin(state.e.psi + np.pi / 2)))

        return target_idx, error_front_axle

    def pid_control(self, target, current, braking=False):
        """
        Proportional control for the speed. If braking, have target speed be 0

        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        # Controls acceleration — Kp is how fast we approach target speed
        if not braking:
            return self.Kp * (target - current)
        else:
            return self.Kp_braking * (target - current)


    def stanley_control(self, state: VehicleState):
        """
        Stanley steering control.

        :param state: (VehicleState object)
        :return: (float, int)
        """
        # get index of waypoint we should travel to
        current_target_idx, error_front_axle = self.calc_target_index(state)

        # if we're moving forward, cool, otherwise keep going to where we were going before
        if self.target_idx >= current_target_idx:
            current_target_idx = self.target_idx

        # 防御：索引钳制（ref 路径与 target_idx 可能来自不同长度的数据源——
        # 如其他车辆的前瞻模拟使用「展示拼接路径 + 对方 target_idx」），越界会崩溃。
        _n = len(self.yaw_ref)
        if _n == 0:
            return 0.0, 0
        if current_target_idx >= _n:
            current_target_idx = _n - 1
        elif current_target_idx < 0:
            current_target_idx = 0

        # theta_e corrects the heading error
        theta_e = normalize_angle(self.yaw_ref[current_target_idx] - state.e.psi)
        # theta_d corrects based on the cross track error (k is a gain for this)
        # Cross track error: http://www.sailtrain.co.uk/gps/functions.htm
        theta_d = np.arctan2(self.k * error_front_axle, state.v.v)
        # Steering control (sum of heading correction + getting back on path)
        delta = theta_e + theta_d

        return delta, current_target_idx

    def solve(self, state: VehicleState, braking=False):
        # 空参考路径：强制刹车停车（此前会保持 v_ref 直行漂移出场地）
        if len(getattr(self, 'yaw_ref', [])) == 0 or len(getattr(self, 'x_ref', [])) == 0:
            return -3.0, 0.0, 0
        a = self.pid_control(self.v_ref, state.v.v, braking)
        d, current_target_idx = self.stanley_control(state)

        return a, d, current_target_idx

    def step(self, state: VehicleState, acceleration: float, delta: float):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        state.u.u_a = acceleration
        state.u.u_steer = delta

        # don't turn too much
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        # advance x and y
        state.x.x += state.v.v * np.cos(state.e.psi) * self.dt
        state.x.y += state.v.v * np.sin(state.e.psi) * self.dt
        # advance yaw, scaled by velocity, tangent of angle, and inverse of wheelbase
        state.e.psi += state.v.v / self.L * np.tan(delta) * self.dt
        state.e.psi = normalize_angle(state.e.psi)
        # advance velocity
        state.v.v += acceleration * self.dt
