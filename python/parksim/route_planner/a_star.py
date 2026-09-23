from itertools import count
from typing import List
from queue import PriorityQueue

import numpy as np

from parksim.route_planner.graph import Vertex, Edge, WaypointsGraph
from parksim.utils.spline import calc_spline_course

class AStarGraph(WaypointsGraph):
    """
    Graph with a* result
    """
    def __init__(self, path: List['Edge']):
        """
        docstring
        """
        self.edges = path

        if path == []:
            self.vertices = []
        else:
            self.vertices = [self.edges[0].v1]

            for e in self.edges:
                self.vertices.append(e.v2)

    def path_cost(self):
        """
        compute the cost along the planned path. Now computed as the sum of all edge costs
        """
        cost = 0
        for e in self.edges:
            cost += e.c

        return cost

    def plot(self, ax = None, plt_ops = {}):
        """
        plot the A* result
        """
        ax = super().plot(ax=ax, plt_ops=plt_ops)
        ax.plot(self.vertices[0].coords[0], self.vertices[0].coords[1], marker='s', markersize=4, mfc='none', **plt_ops)
        ax.plot(self.vertices[-1].coords[0], self.vertices[-1].coords[1], marker='x', markersize=4, mfc='none', **plt_ops)

        return ax

    def compute_ref_path(self, offset: float = 0):
        """
        Compute vehicle ref path with offset from the center line
        """
        # collect x, y, yaw from A* solution
        axs = []
        ays = []

        # calculate splines

        # generate list of x, y waypoints
        for v in self.vertices:
            axs.append(v.coords[0])
            ays.append(v.coords[1])

        cxs, cys, cyaws, _, _ = calc_spline_course(axs, ays, ds=0.1)
        cxs = [cxs[j] + offset * np.sin(cyaws[j]) for j in range(len(cxs))]
        cys = [cys[j] - offset * np.cos(cyaws[j]) for j in range(len(cys))]

        return cxs, cys, cyaws

class AStarPlanner(object):
    """
    A* planner for planning shortest path on the graph
    """
    def __init__(self, v_start: 'Vertex', v_goal: 'Vertex'):
        self.v_start = v_start
        self.v_goal = v_goal

        self.fringe = PriorityQueue()
        self.closed = set()
        
        # A counter object to prevent nodes with same cost
        self.counter = count()

        # (Vertex, Path, Cost-along-path)
        start = (self.v_start, [], 0)
        self.fringe.put((0, next(self.counter), start))

    def solve(self):
        """
        solve the path
        """
        while not self.fringe.empty():
            _, _, (v, path, cost) = self.fringe.get()

            if v == self.v_goal:
                # the returned path is a list of edges
                # print("Solved")
                return AStarGraph(path)
            
            if v not in self.closed:
                self.closed.add(v)

                for child, edge in zip(*v.get_children()):
                    new_cost = cost + edge.c
                    aStar_cost = new_cost + child.dist(self.v_goal)
                    new_node = (child, path + [edge], new_cost)
                    self.fringe.put((aStar_cost, next(self.counter), new_node))

        raise Exception('Path is not found')


class DijkstraPlanner(object):
    """
    Dijkstra planner（无启发式基线）——与 AStarPlanner 同接口。

    与 A* 同为最优路径，但无启发式引导，用于对照实验。
    """
    def __init__(self, v_start: 'Vertex', v_goal: 'Vertex'):
        self.v_start = v_start
        self.v_goal = v_goal

        self.fringe = PriorityQueue()
        self.counter = count()

        # (Vertex, Path, Cost-along-path)
        start = (self.v_start, [], 0)
        self.fringe.put((0, next(self.counter), start))

    def solve(self):
        """
        solve the path
        """
        closed = set()

        while not self.fringe.empty():
            _, _, (v, path, cost) = self.fringe.get()

            if v == self.v_goal:
                return AStarGraph(path)

            if v not in closed:
                closed.add(v)

                for child, edge in zip(*v.get_children()):
                    new_cost = cost + edge.c
                    new_node = (child, path + [edge], new_cost)
                    self.fringe.put((new_cost, next(self.counter), new_node))

        raise Exception('Path is not found')


# ============================ 可插拔路由规划器 ============================

class AStarRoutePlanner(object):
    """路由规划器插件：A*（默认）"""

    name = 'astar'

    def __init__(self, graph: 'WaypointsGraph' = None, via_coords=None):
        self.graph = graph

    def plan(self, v_start: 'Vertex', v_goal: 'Vertex') -> AStarGraph:
        return AStarPlanner(v_start, v_goal).solve()


class DijkstraRoutePlanner(AStarRoutePlanner):
    """路由规划器插件：Dijkstra（无启发式基线）"""

    name = 'dijkstra'

    def plan(self, v_start: 'Vertex', v_goal: 'Vertex') -> AStarGraph:
        return DijkstraPlanner(v_start, v_goal).solve()


class ViaRoutePlanner(AStarRoutePlanner):
    """
    路由规划器插件：途经点串联。

    外部（上层场景/决策模块）可通过 set_via_points(coords) 指定途经点，
    规划结果 = start → via1 → via2 → … → goal 的逐段 A* 串联；
    未设置途经点时退化为普通 A*。
    """

    name = 'via'

    def __init__(self, graph: 'WaypointsGraph' = None, via_coords=None):
        super(ViaRoutePlanner, self).__init__(graph)
        self.via_coords = [np.asarray(c, dtype=float) for c in (via_coords or [])]

    def set_via_points(self, coords):
        """coords: [(x, y), ...] 途经点坐标序列（按行驶顺序）"""
        self.via_coords = [np.asarray(c, dtype=float) for c in (coords or [])]

    def plan(self, v_start: 'Vertex', v_goal: 'Vertex') -> AStarGraph:
        if not self.via_coords or self.graph is None:
            return AStarPlanner(v_start, v_goal).solve()

        waypoint_vertices = [v_start]
        for coords in self.via_coords:
            waypoint_vertices.append(self.graph.vertices[self.graph.search(coords)])
        waypoint_vertices.append(v_goal)

        edges = []
        for a, b in zip(waypoint_vertices[:-1], waypoint_vertices[1:]):
            segment = AStarPlanner(a, b).solve()
            edges.extend(segment.edges)

        return AStarGraph(edges)


_ROUTE_PLANNERS = {
    AStarRoutePlanner.name: AStarRoutePlanner,
    DijkstraRoutePlanner.name: DijkstraRoutePlanner,
    ViaRoutePlanner.name: ViaRoutePlanner,
}


def available_route_planners():
    """返回已注册的路由规划器名列表"""
    return sorted(_ROUTE_PLANNERS.keys())


def make_route_planner(name, graph=None, via_coords=None):
    """工厂：按名称实例化路由规划器（大小写、连字符不敏感）"""
    key = str(name or 'astar').strip().lower().replace('-', '_')
    if key not in _ROUTE_PLANNERS:
        raise ValueError("Unknown route planner '%s'. Available: %s" % (
            name, ", ".join(available_route_planners())))
    return _ROUTE_PLANNERS[key](graph, via_coords)