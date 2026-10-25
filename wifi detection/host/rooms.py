"""
Per-room occupied / empty / unknown evidence from WiFi links.

A link senses the area inside its ellipse (points whose path via them is less
than ELLIPSE_M longer than the direct path). Evidence is kept per point of a
0.1 m grid inside each room, in two separate accumulators so quiet time can
never cancel motion:

  motion - seconds the link's detector reported motion (its debounced decision,
           so isolated spikes ESPectre rejects add nothing)
  quiet  - seconds the link was clearly quiet (ratio < 0.8, no motion), half weight

  occupied - motion > 1.5 s on at least 5 % of the room
  empty    - at least 70 % of the room covered for 30 s, average quiet > 1.5 and
             essentially no motion anywhere in it (confidence capped at 0.9)
  unknown  - otherwise (including rooms no link has crossed)

A quiet link only clears its own ellipse, so it can't vouch for parts of a room
it never crossed. A link only counts for the one room holding at least 75 % of
its ellipse: a link split across rooms can't say which one the person is in
(it still feeds the x, y estimate). Everything decays with a half-life, so
rooms can be re-checked.
"""

from __future__ import annotations

import math
import threading

import numpy as np

from floorplan import FloorPlan

OCCUPIED_AT = 1.5              # seconds of detected motion on a point
EMPTY_QUIET_AT = 1.5           # weighted quiet seconds, averaged over covered points
EMPTY_MAX_MOTION = 0.5         # any point with more detected motion blocks "empty"
EMPTY_MAX_CONFIDENCE = 0.9     # "empty" is never claimed with certainty
# "Empty" needs 30 s of listening per point: in the replays ESPectre's debounce never
# fired during quiet (0 s in 9 windows) but missed a weak 15 s stretch of a real walk,
# so one 15 s stop must never be enough to call a room clear.
MIN_COVERAGE_S = 30.0
OCCUPIED_PERCENTILE = 95       # >= 5 % of the room
EMPTY_COVERED_FRACTION = 0.7
QUIET_RATIO = 0.8
QUIET_WEIGHT = 0.5
# In a 1.7 x 1.2 m house a link across one room still clips ~12 % of the next room
# and the diagonals split ~50/50; ellipse points are counted per room.
ROOM_SHARE = 0.75


class RoomEvidence:
    def __init__(self, plan: FloorPlan, ellipse_m: float = 0.35, half_life_s: float = 120.0, grid_m: float = 0.1):
        self.plan = plan
        self.ellipse_m = ellipse_m
        self.half_life_s = half_life_s
        self.motion_half_life_s = plan.motion_half_life_s or half_life_s   # sentry plans forget motion sooner
        self.lock = threading.Lock()
        self.points = {}
        for room in plan.rooms:
            xs = [p[0] for p in room.polygon]
            ys = [p[1] for p in room.polygon]
            gx = np.arange(min(xs) + grid_m / 2, max(xs), grid_m)
            gy = np.arange(min(ys) + grid_m / 2, max(ys), grid_m)
            pts = [(x, y) for x in gx for y in gy if room.contains(x, y)]
            self.points[room.name] = np.array(pts) if pts else np.zeros((0, 2))
        self.motion = {r.name: np.zeros(len(self.points[r.name])) for r in plan.rooms}
        self.quiet = {r.name: np.zeros(len(self.points[r.name])) for r in plan.rooms}
        self.coverage_s = {r.name: np.zeros(len(self.points[r.name])) for r in plan.rooms}
        self.last_t = None

    def _decay(self, now: float):
        if self.last_t is not None and now > self.last_t:
            f = 0.5 ** ((now - self.last_t) / self.half_life_s)
            fm = 0.5 ** ((now - self.last_t) / self.motion_half_life_s)
            for k in self.motion:
                self.motion[k] *= fm
                self.quiet[k] *= f
                self.coverage_s[k] *= f
        self.last_t = now if self.last_t is None else max(self.last_t, now)

    def add(self, a: tuple, b: tuple, ratio: float, now: float, dt: float, detected: bool) -> None:
        ax, ay = a
        bx, by = b
        direct = math.hypot(bx - ax, by - ay)
        with self.lock:
            self._decay(now)
            hits = {}
            for name, pts in self.points.items():
                if not len(pts):
                    continue
                excess = (np.hypot(pts[:, 0] - ax, pts[:, 1] - ay) +
                          np.hypot(pts[:, 0] - bx, pts[:, 1] - by) - direct)
                inside = excess < self.ellipse_m
                if inside.any():
                    hits[name] = inside
            total = sum(int(m.sum()) for m in hits.values())
            if not total:
                return
            name, inside = max(hits.items(), key=lambda kv: int(kv[1].sum()))
            if inside.sum() < ROOM_SHARE * total:
                return
            self.coverage_s[name][inside] += dt
            if detected:
                self.motion[name][inside] += dt
            elif ratio < QUIET_RATIO:
                self.quiet[name][inside] += dt * QUIET_WEIGHT

    def status(self, now: float) -> list[dict]:
        with self.lock:
            self._decay(now)
            out = []
            for room in self.plan.rooms:
                mot, qui, cov = self.motion[room.name], self.quiet[room.name], self.coverage_s[room.name]
                status, confidence = "unknown", 0.0
                if len(mot):
                    hot = float(np.percentile(mot, OCCUPIED_PERCENTILE))
                    covered = cov >= MIN_COVERAGE_S
                    if hot > OCCUPIED_AT:
                        status, confidence = "occupied", min(1.0, hot / 4.0)
                    elif covered.mean() >= EMPTY_COVERED_FRACTION and float(mot.max()) < EMPTY_MAX_MOTION \
                            and float(qui[covered].mean()) > EMPTY_QUIET_AT:
                        status = "empty"
                        confidence = min(EMPTY_MAX_CONFIDENCE,
                                         covered.mean() * float(qui[covered].mean()) / 4.0)
                out.append({"name": room.name, "status": status, "confidence": round(confidence, 2),
                            "coverage_s": round(float(cov.mean()) if len(cov) else 0.0, 1)})
            return out
