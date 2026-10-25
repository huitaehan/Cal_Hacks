"""FirstIn client for the vision pipeline. Standard library only (no pip install).

Drop this one file next to your detection code. It tells the FirstIn server what your camera /
drone / robot sees. When TWO DIFFERENT sources report the same victim, the server places the
phone call by itself. You never need a Cartesia key, an agent ID or a phone number.

    from firstin_client import FirstIn

    robot = FirstIn(source="robot1")          # URL and token come from FIRSTIN_URL / FIRSTIN_TOKEN
    drone = FirstIn(source="drone")

    robot.report_victim("v1", room="B", confidence=0.86, description="person lying near the door")
    drone.report_victim("v1", room="B", confidence=0.80)    # second source, same id -> call goes out

    robot.report_fire("f1", room="D", confidence=0.9, x=640, y=380, r=40)
    robot.report_blocked("b1", room="A", confidence=0.7, x=300, y=250, w=24, h=50)

Every call returns a dict and never raises by default, so a network hiccup cannot crash your
vision loop. Look at result["ok"]. Pass raise_errors=True while debugging.

Command line checks:
    python3 firstin_client.py health        # is the server up, is the token right, will a call go out?
    python3 firstin_client.py demo          # sends one victim from two sources (robot1 + drone)

Setup (environment variables; neither is a Cartesia secret):
    export FIRSTIN_URL=http://<her-laptop-ip>:3000
    export FIRSTIN_TOKEN=<only if she set a DETECTION_TOKEN; otherwise leave it unset>
"""

import json
import os
import sys
import time
import urllib.error
import urllib.request


class FirstInError(Exception):
    pass


class FirstIn:
    def __init__(self, source, url=None, token=None, timeout=3.0, raise_errors=False):
        if not source or not str(source).strip():
            raise ValueError("source is required, e.g. 'robot1', 'drone' or 'camera2'")
        self.source = str(source).strip()
        self.url = (url or os.environ.get("FIRSTIN_URL") or "http://localhost:3000").rstrip("/")
        self.token = token or os.environ.get("FIRSTIN_TOKEN") or ""
        self.timeout = timeout
        self.raise_errors = raise_errors
        self._last_warn = 0.0

    # ---- reports -------------------------------------------------------------------------
    def report_victim(self, victim_id, room, confidence, description="", x=None, y=None):
        """A person was seen. room is "A".."D". Use the SAME victim_id for the same person
        from every source: two different sources is what confirms a victim."""
        return self._detect("victim", victim_id, room, confidence, description, x=x, y=y)

    def report_fire(self, fire_id, confidence, room=None, description="", x=None, y=None, r=None):
        """Fire or heat. x, y is the centre in map units (800 x 520), r the radius."""
        return self._detect("fire", fire_id, room, confidence, description, x=x, y=y, r=r)

    def report_blocked(self, blocked_id, confidence, room=None, description="", x=None, y=None, w=None, h=None):
        """Debris or a blocked path. x, y is the top-left corner, w, h the size, in map units."""
        return self._detect("blocked", blocked_id, room, confidence, description, x=x, y=y, w=w, h=h)

    def health(self):
        """GET /health: server up? token required? will a real call be placed?"""
        return self._request("GET", "/health")

    # ---- internals -----------------------------------------------------------------------
    def _detect(self, kind, det_id, room, confidence, description, **extra):
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            return self._fail("confidence must be a number between 0 and 1")
        if not 0.0 <= confidence <= 1.0:
            return self._fail("confidence must be between 0 and 1 (not a percentage)")
        if kind == "victim" and str(room or "").upper() not in ("A", "B", "C", "D"):
            return self._fail("a victim needs room 'A', 'B', 'C' or 'D'")
        body = {"id": str(det_id), "kind": kind, "confidence": confidence, "source": self.source}
        if room:
            body["room"] = str(room).upper()
        if description:
            body["description"] = str(description)[:200]
        for k, v in extra.items():
            if v is not None:
                body[k] = v
        return self._request("POST", "/detection", body)

    def _request(self, method, path, body=None):
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["x-token"] = self.token
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.url + path, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode() or "{}")
        except urllib.error.HTTPError as e:
            try:
                msg = json.loads(e.read().decode()).get("error", "")
            except Exception:
                msg = ""
            hint = " (wrong or missing FIRSTIN_TOKEN?)" if e.code == 401 else ""
            return self._fail(f"server said {e.code}: {msg}{hint}")
        except Exception as e:  # server down, wrong IP, wifi, timeout...
            return self._fail(f"cannot reach {self.url}: {e}")

    def _fail(self, message):
        if self.raise_errors:
            raise FirstInError(message)
        now = time.time()
        if now - self._last_warn > 5:  # one warning per 5 s so a per-frame loop does not flood the terminal
            print(f"[firstin] {message}", file=sys.stderr)
            self._last_warn = now
        return {"ok": False, "error": message}


def _main(argv):
    cmd = argv[1] if len(argv) > 1 else "help"
    if cmd == "health":
        print(json.dumps(FirstIn(source="cli").health(), indent=2))
    elif cmd == "demo":
        vid = "demo-" + str(int(time.time()))[-6:]
        room = argv[2].upper() if len(argv) > 2 else "B"
        print("robot1 reports:", FirstIn(source="robot1").report_victim(vid, room, 0.86, "person lying near the door"))
        time.sleep(1.2)
        print("drone  reports:", FirstIn(source="drone").report_victim(vid, room, 0.80, "person visible from above"))
        print("If the server is live, your phone should ring in a few seconds (dry run otherwise).")
    else:
        print(__doc__)


if __name__ == "__main__":
    _main(sys.argv)
