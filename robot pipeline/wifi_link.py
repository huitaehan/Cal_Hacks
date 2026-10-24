"""Link to the WiFi sensing server (wifi detection/host/presence_server.py).

Sends the position of each robot that carries a sensing board ("sensing_mac" in config.json) to
POST /api/robots (10 Hz), so the server knows where that board is and when it has stopped, and reads the
person estimate from GET /api/state (4 Hz). With fixed boards (no "sensing_mac"), nothing is sent and the
server takes every board's position from its floor plan.
Positions are averaged over 0.5 s: the server counts a robot as moving if it wanders more than 5 cm.
"""
import collections
import json
import threading
import time
import urllib.request


def person_from_state(state: dict):
    """The server's person position (x, y), only while its detector says someone is there."""
    if not (state.get("summary") or {}).get("detected"):
        return None
    p = (state.get("localization") or {}).get("person")
    return (float(p[0]), float(p[1])) if p else None


class PoseSmoother:
    def __init__(self, window_s: float = 0.5):
        self.window_s = window_s
        self.hist: dict = collections.defaultdict(collections.deque)

    def add(self, robot, pose):
        h = self.hist[robot]
        h.append((pose.t, pose.x, pose.y))
        while h and pose.t - h[0][0] > self.window_s:
            h.popleft()

    def get(self, robot, now: float):
        pts = [(x, y) for t, x, y in self.hist.get(robot, ()) if now - t <= self.window_s]
        if not pts:
            return None
        return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


class WifiLink:
    def __init__(self, base: str, robots_cfg: dict, post_s: float = 0.1, poll_s: float = 0.25):
        self.base = base.rstrip("/")
        self.robots_cfg = robots_cfg
        self.post_s, self.poll_s = post_s, poll_s
        self.lock = threading.Lock()
        self.smoother = PoseSmoother()
        self._person = None
        self._person_t = 0.0
        self.status = "connecting"
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def add_poses(self, poses: dict):
        with self.lock:
            for r, p in poses.items():
                self.smoother.add(r, p)

    def person(self, max_age: float = 1.0):
        with self.lock:
            return self._person if time.time() - self._person_t <= max_age else None

    def _request(self, path: str, body: dict | None = None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method="GET" if body is None else "POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=0.5) as r:
            return json.loads(r.read() or b"{}")

    def _run(self):
        next_poll = 0.0
        while not self._stop:
            now = time.time()
            try:
                with self.lock:
                    robots = [{"id": r, "mac": c["sensing_mac"], "x": round(xy[0], 3), "y": round(xy[1], 3)}
                              for r, c in self.robots_cfg.items()
                              if c.get("sensing_mac") and (xy := self.smoother.get(r, now))]
                if robots:
                    self._request("/api/robots", {"robots": robots})
                if now >= next_poll:
                    next_poll = now + self.poll_s
                    p = person_from_state(self._request("/api/state"))
                    with self.lock:
                        self._person, self._person_t = p, time.time()
                self.status = "ok"
            except OSError as e:        # server down or restarting: keep trying
                self.status = f"WiFi server unreachable ({type(e).__name__})"
            except Exception as e:
                self.status = f"WiFi link error: {type(e).__name__}: {e}"
            time.sleep(self.post_s)

    def close(self):
        self._stop = True
