"""
Robot positions from the robot team's camera tracking, and when each robot is stopped.

A robot is "stopped" once every position in the last STILL_WINDOW_S stayed within
STILL_RADIUS_M of its latest one, plus SETTLE_S more: link data is only trusted
from then on. A robot whose positions stop arriving goes stale after STALE_S and
counts as moving, so a crashed tracking script can't freeze a robot in place.
"""

from __future__ import annotations

import collections
import math
import threading


class RobotTracker:
    STILL_RADIUS_M = 0.05
    STILL_WINDOW_S = 2.0
    SETTLE_S = 1.0
    STALE_S = 1.0
    HISTORY_S = 5.0

    def __init__(self):
        self.lock = threading.Lock()
        self.tracks: dict[str, dict] = {}   # mac -> {"id", "hist": deque[(t, x, y)]}

    def update(self, robots: list[dict], now: float) -> None:
        with self.lock:
            for r in robots:
                mac = str(r.get("mac", "")).lower()
                if not mac:
                    continue
                tr = self.tracks.setdefault(mac, {"id": str(r.get("id", mac[-5:])),
                                                  "hist": collections.deque(), "rest": None, "still_since": None})
                tr["id"] = str(r.get("id", tr["id"]))
                x, y = float(r["x"]), float(r["y"])
                tr["hist"].append((now, x, y))
                while tr["hist"] and tr["hist"][0][0] < now - self.HISTORY_S:
                    tr["hist"].popleft()
                # Still-since survives the history trim: it resets only when the robot
                # leaves STILL_RADIUS_M of the spot where it came to rest.
                if tr["rest"] is None or math.hypot(x - tr["rest"][0], y - tr["rest"][1]) > self.STILL_RADIUS_M:
                    tr["rest"] = (x, y)
                    tr["still_since"] = now

    def _fresh(self, tr: dict, now: float) -> bool:
        return bool(tr["hist"]) and now - tr["hist"][-1][0] <= self.STALE_S

    def _still_since(self, tr: dict) -> float | None:
        """When the robot came to rest within STILL_RADIUS_M of its current spot."""
        return tr["still_since"] if tr["hist"] else None

    def tracked(self, mac: str) -> bool:
        with self.lock:
            return mac in self.tracks

    def position(self, mac: str, now: float) -> tuple[float, float] | None:
        with self.lock:
            tr = self.tracks.get(mac)
            if tr is None or not self._fresh(tr, now):
                return None
            _, x, y = tr["hist"][-1]
            return x, y

    def stop_started(self, mac: str, now: float) -> float | None:
        with self.lock:
            tr = self.tracks.get(mac)
            if tr is None or not self._fresh(tr, now):
                return None
            return self._still_since(tr)

    def is_stopped(self, mac: str, now: float) -> bool:
        since = self.stop_started(mac, now)
        return since is not None and now - since >= self.STILL_WINDOW_S + self.SETTLE_S

    def warnings(self, heard: set[str], fixed: set[str], now: float) -> list[str]:
        with self.lock:
            tracked = {m: tr["id"] for m, tr in self.tracks.items() if self._fresh(tr, now)}
        out = [f"robot {rid} is posted with board {mac}, but no board with that MAC is heard"
               for mac, rid in tracked.items() if mac not in heard]
        out += [f"board {mac} is heard but has no position (add it to floorplan.json boards or send it in /api/robots)"
                for mac in sorted(heard) if mac not in tracked and mac not in fixed]
        return out

    def snapshot(self, now: float) -> list[dict]:
        with self.lock:
            macs = list(self.tracks)
        out = []
        for mac in macs:
            pos = self.position(mac, now)
            since = self.stop_started(mac, now)
            with self.lock:
                rid = self.tracks[mac]["id"]
            out.append({"id": rid, "mac": mac, "x": pos[0] if pos else None, "y": pos[1] if pos else None,
                        "fresh": pos is not None, "stopped": self.is_stopped(mac, now),
                        "stop_s": round(now - since, 1) if since is not None else 0.0})
        return out
