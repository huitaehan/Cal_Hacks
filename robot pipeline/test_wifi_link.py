"""Link to the WiFi sensing server: robot positions in, person estimate out."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

from tracker import Pose
from wifi_link import PoseSmoother, WifiLink, person_from_state

ROBOTS = {"A": {"marker": 10, "sensing_mac": "d0:cf:13:e1:63:30", "sentry": [4.5, 2.0]}}


def test_person_only_when_the_server_says_detected():
    st = {"summary": {"detected": True}, "localization": {"person": [1.2, 3.4]}}
    assert person_from_state(st) == (1.2, 3.4)
    assert person_from_state({"summary": {"detected": False}, "localization": {"person": [1.2, 3.4]}}) is None
    assert person_from_state({"summary": {"detected": True}, "localization": {"person": None}}) is None
    assert person_from_state({}) is None


def test_smoother_averages_the_last_half_second():
    s = PoseSmoother(window_s=0.5)
    for i, x in enumerate([1.00, 1.04, 0.96, 1.02, 0.98]):
        s.add("A", Pose(x, 2.0, 0, 10 + i * 0.1))
    x, y = s.get("A", now=10.4)
    assert abs(x - 1.0) < 1e-9 and y == 2.0
    assert s.get("A", now=12.0) is None          # nothing recent


def _server(got):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            body = json.dumps({"summary": {"detected": True}, "localization": {"person": [2.0, 1.0]}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            got.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"{}")

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def test_robots_without_a_sensing_board_are_not_reported():
    """Fixed boards (none on the robots): the server's board positions must come from its floor plan only."""
    got = []
    srv = _server(got)
    link = WifiLink(f"http://127.0.0.1:{srv.server_port}", {"A": {"marker": 10, "sentry": [4.5, 2.0]}})
    try:
        end = time.time() + 2.0                           # positions keep arriving, as from run.py
        while time.time() < end:
            link.add_poses({"A": Pose(1.5, 2.5, 90.0, time.time())})
            time.sleep(0.05)
        assert link.status == "ok" and link.person(max_age=0.5) == (2.0, 1.0) and got == [], (link.status, got)
    finally:
        link.close()
        srv.shutdown()


def test_posts_positions_and_reads_the_person_from_a_server():
    got = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            body = json.dumps({"summary": {"detected": True}, "localization": {"person": [2.0, 1.0]}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            got.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"{}")

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    link = WifiLink(f"http://127.0.0.1:{srv.server_port}", ROBOTS)
    try:
        link.add_poses({"A": Pose(1.5, 2.5, 90.0, time.time())})
        deadline = time.time() + 3
        while time.time() < deadline and (not got or link.person() is None):
            time.sleep(0.05)
        assert link.person() == (2.0, 1.0), link.status
        assert got[0] == {"robots": [{"id": "A", "mac": "d0:cf:13:e1:63:30", "x": 1.5, "y": 2.5}]}, got[0]
        assert link.status == "ok"
    finally:
        link.close()
        srv.shutdown()


def test_server_down_gives_no_person_and_a_status_message():
    link = WifiLink("http://127.0.0.1:9", ROBOTS)       # nothing listens there
    try:
        time.sleep(1.0)
        assert link.person() is None and link.status != "ok", link.status
    finally:
        link.close()


if __name__ == "__main__":
    test_robots_without_a_sensing_board_are_not_reported()
    test_person_only_when_the_server_says_detected()
    test_smoother_averages_the_last_half_second()
    test_posts_positions_and_reads_the_person_from_a_server()
    test_server_down_gives_no_person_and_a_status_message()
    print("ok")
