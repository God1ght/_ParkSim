#!/usr/bin/env python3
"""
参考路径生成器（可插拔）

把「航点序列 → 车辆参考路径（x, y, yaw）」这一步抽象为可插拔组件：
    make_ref_path_generator(name) -> generator
    generator.generate(vertices, offset=0.0) -> (x_ref, y_ref, yaw_ref)

- spline（默认）: 现有实现——三次样条加密（calc_spline_course, ds=0.1）+ 右行横向偏移；
                 与 AStarGraph.compute_ref_path 的输出完全一致。
- linear（对照）: 折线等距密集采样（0.1m）+ 分段航向 + 相同横向偏移公式。

vertices: 顶点序列（含 .coords = np.array([x, y])），如 AStarGraph.vertices / WaypointsGraph 的顶点列表。
"""

import numpy as np

from parksim.utils.spline import calc_spline_course


class SplineRefPathGenerator(object):
    """三次样条 + 横向偏移（默认，= 现有行为）"""

    name = "spline"

    def generate(self, vertices, offset=0.0):
        axs = []
        ays = []
        for v in vertices:
            axs.append(v.coords[0])
            ays.append(v.coords[1])

        cxs, cys, cyaws, _, _ = calc_spline_course(axs, ays, ds=0.1)
        cxs = [cxs[j] + offset * np.sin(cyaws[j]) for j in range(len(cxs))]
        cys = [cys[j] - offset * np.cos(cyaws[j]) for j in range(len(cys))]

        return cxs, cys, cyaws


class LinearRefPathGenerator(object):
    """折线等距插值 + 分段航向（对照实现）"""

    name = "linear"
    ds = 0.1

    def generate(self, vertices, offset=0.0):
        xs = [float(v.coords[0]) for v in vertices]
        ys = [float(v.coords[1]) for v in vertices]

        px, py, pyaw = [], [], []
        for i in range(len(xs) - 1):
            p0 = np.array([xs[i], ys[i]])
            p1 = np.array([xs[i + 1], ys[i + 1]])
            seg = p1 - p0
            length = float(np.linalg.norm(seg))
            if length < 1e-9:
                continue
            yaw = float(np.arctan2(seg[1], seg[0]))
            steps = max(int(length / self.ds), 1)
            for t in np.linspace(0.0, 1.0, steps, endpoint=False):
                q = p0 + t * seg
                px.append(float(q[0]))
                py.append(float(q[1]))
                pyaw.append(yaw)

        if not px:
            # 退化：单点/零长度路径
            px, py, pyaw = xs[:1], ys[:1], [0.0]
        else:
            px.append(xs[-1])
            py.append(ys[-1])
            pyaw.append(pyaw[-1])

        cxs = [px[j] + offset * np.sin(pyaw[j]) for j in range(len(px))]
        cys = [py[j] - offset * np.cos(pyaw[j]) for j in range(len(py))]

        return cxs, cys, pyaw


_REF_PATH_GENERATORS = {
    SplineRefPathGenerator.name: SplineRefPathGenerator,
    LinearRefPathGenerator.name: LinearRefPathGenerator,
}


def make_ref_path_generator(name):
    """工厂：按名称实例化参考路径生成器"""
    key = str(name or "spline").strip().lower().replace("-", "_")
    if key not in _REF_PATH_GENERATORS:
        raise ValueError("Unknown ref path generator '%s'. Available: %s" % (
            name, ", ".join(sorted(_REF_PATH_GENERATORS))))
    return _REF_PATH_GENERATORS[key]()
