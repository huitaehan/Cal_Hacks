"""Paths around the cardboard walls.

The walls are the line segments in the WiFi floor plan (wifi detection/host/floorplan.json, "walls"), so
they're written down once for both sides. A grid over the robots' area marks every cell closer than the
robot's clearance to a wall as blocked; A* finds the shortest way through the rest, and the result is
straightened into a few waypoints. The Navigator then drives a robot along those waypoints with the Driver.
"""
import heapq
import math
import time

import numpy as np

from drive import Command, Driver


def _seg_dist(px, py, a, b):
    """Distance from point(s) (px, py) to the segment a-b (numpy-friendly)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = np.clip(((px - a[0]) * dx + (py - a[1]) * dy) / l2, 0.0, 1.0) if l2 > 0 else 0.0
    return np.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))


def _crosses(p, q, a, b):
    d = lambda o, s, t: (s[0] - o[0]) * (t[1] - o[1]) - (s[1] - o[1]) * (t[0] - o[0])
    return d(p, q, a) * d(p, q, b) < 0 and d(a, b, p) * d(a, b, q) < 0


class Planner:
    def __init__(self, walls, area, clearance: float = 0.13, cell: float = 0.05):
        """walls: [[(x1, y1), (x2, y2)], ...]; area: ((x0, y0), (x1, y1)), where the robots may drive;
        clearance: robot radius plus a margin."""
        (self.x0, self.y0), (x1, y1) = area
        self.walls = [(tuple(map(float, w[0])), tuple(map(float, w[1]))) for w in walls]
        self.clear, self.cell = clearance, cell
        self.nx = int(round((x1 - self.x0) / cell)) + 1
        self.ny = int(round((y1 - self.y0) / cell)) + 1
        X, Y = np.meshgrid(self.x0 + np.arange(self.nx) * cell, self.y0 + np.arange(self.ny) * cell, indexing="ij")
        d = np.full(X.shape, np.inf)
        for a, b in self.walls:
            d = np.minimum(d, _seg_dist(X, Y, a, b))
        self.blocked = d < clearance

    def wall_dist(self, p) -> float:
        return min((float(_seg_dist(p[0], p[1], a, b)) for a, b in self.walls), default=math.inf)

    def _cell(self, p):
        return (min(max(int(round((p[0] - self.x0) / self.cell)), 0), self.nx - 1),
                min(max(int(round((p[1] - self.y0) / self.cell)), 0), self.ny - 1))

    def _pt(self, c):
        return (round(self.x0 + c[0] * self.cell, 4), round(self.y0 + c[1] * self.cell, 4))

    def clear_line(self, p, q) -> bool:
        """Straight p -> q crosses no wall and keeps clear of walls (except right at p: the robot may
        already be close to one)."""
        if any(_crosses(p, q, a, b) for a, b in self.walls):
            return False
        n = max(2, int(math.dist(p, q) / (self.cell / 2)) + 1)
        for k in range(n + 1):
            s = (p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n)
            if math.dist(s, p) > self.clear and self.wall_dist(s) < self.clear:
                return False
        return True

    def nearest_free(self, p):
        """p itself if it's clear of walls, else the nearest free grid point."""
        if self.wall_dist(p) >= self.clear:
            return tuple(p)
        free = np.argwhere(~self.blocked)
        if not len(free):
            return None
        pts = np.c_[self.x0 + free[:, 0] * self.cell, self.y0 + free[:, 1] * self.cell]
        i = int(np.argmin(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])))
        return self._pt(tuple(free[i]))

    def plan(self, start, goal):
        """Waypoints from start to goal (ending at goal, or the nearest free point to it), or None."""
        start, goal = tuple(start), self.nearest_free(goal)
        if goal is None:
            return None
        if self.clear_line(start, goal):
            return [goal]
        s, g = self._cell(start), self._cell(goal)
        if self.blocked[g]:
            return None
        came, cost = {s: None}, {s: 0.0}
        heap = [(math.dist(s, g), s)]
        while heap:
            _, c = heapq.heappop(heap)
            if c == g:
                break
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if (dx or dy) and 0 <= n[0] < self.nx and 0 <= n[1] < self.ny and not self.blocked[n]:
                        nc = cost[c] + math.hypot(dx, dy)
                        if nc < cost.get(n, math.inf):
                            cost[n], came[n] = nc, c
                            heapq.heappush(heap, (nc + math.dist(n, g), n))
        if g not in came:
            return None
        cells = [g]
        while came[cells[-1]] is not None:
            cells.append(came[cells[-1]])
        pts = [self._pt(c) for c in reversed(cells)][1:]
        pts[-1] = goal
        out, cur, i = [], start, 0
        while i < len(pts):                     # straighten: jump to the farthest point in plain view
            j = len(pts) - 1
            while j > i and not self.clear_line(cur, pts[j]):
                j -= 1
            out.append(pts[j])
            cur, i = pts[j], j + 1
        return out


class Navigator:
    """Drives robots along planned paths: steers at the next waypoint, judges progress by path length."""

    def __init__(self, driver: Driver, planner: Planner | None, replan_s: float = 1.0, reached_m: float = 0.12):
        self.driver, self.planner = driver, planner
        self.replan_s, self.reached_m = replan_s, reached_m
        self.paths: dict = {}           # robot -> (goal, waypoints, planned at)

    def step(self, robot, pose, goal, now: float | None = None) -> Command:
        if self.planner is None or not self.planner.walls:
            return self.driver.step(robot, pose, goal)
        now = time.time() if now is None else now
        goal = tuple(goal)
        here = (pose.x, pose.y)
        cached = self.paths.get(robot)
        if cached is None or cached[0] != goal or now - cached[2] >= self.replan_s:
            cached = (goal, self.planner.plan(here, goal), now)
            self.paths[robot] = cached
        path = cached[1]
        if path is None:
            return Command(0.0, 0.0, False, math.dist(here, goal), f"robot {robot}: no path to "
                           f"({goal[0]:.1f}, {goal[1]:.1f}) around the walls")
        while len(path) > 1 and math.dist(here, path[0]) < self.reached_m:
            path.pop(0)
        remaining = math.dist(here, path[0]) + sum(math.dist(p, q) for p, q in zip(path, path[1:]))
        return self.driver.step(robot, pose, path[-1], aim=path[0], remaining=remaining)

    def path(self, robot):
        cached = self.paths.get(robot)
        return cached[1] if cached else None
