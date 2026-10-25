"""End to end without hardware: a rendered overhead camera, robots that obey the real UDP commands, and a
fake WiFi server. Exercises the real chain: camera thread -> colour tracker -> mission -> path around a
wall -> driver -> RobotLink UDP -> the robot moves -> the camera sees it move."""
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

from colour_tracker import ColourTracker
from drive import RobotCal, RobotLink
from run import Pipeline
from tracker import CameraTracker
from wifi_link import WifiLink

PPM = 200                                   # rendered pixels per metre
AREA = ((0.0, 0.0), (4.5, 4.0))
WALL = [[[0.5, 2.0], [1.8, 2.0]]]          # between robot A and the person
PERSON = (2.0, 3.0)
SSL = [(0.035, 0.0548), (-0.0548, 0.035), (-0.0548, -0.035), (0.035, -0.0548)]
LOOKS = {"A": ["pink"] * 4, "B": ["pink", "green", "green", "pink"]}
BGR = {"blue": (200, 90, 30), "pink": (180, 70, 255), "green": (60, 190, 50)}
M_PER_S_AT_100 = 1.0                        # simulated robot speed per 100 % PWM


class World:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = {"A": [1.0, 1.0, 90.0], "B": [3.5, 1.0, 90.0]}
        self.vel = {r: (0.0, 0.0) for r in self.pose}
        self.last_cmd = {r: 0.0 for r in self.pose}
        self.trail = {r: [tuple(p[:2])] for r, p in self.pose.items()}
        self.stop = False

    def command(self, robot, text):
        parts = text.split()
        with self.lock:
            if parts[0] == "S":
                self.vel[robot] = (0.0, 0.0)
            elif parts[0] == "D":
                speed, direction = float(parts[2]), float(parts[3])
                a = math.radians(self.pose[robot][2] + direction)      # robot dir 0 = its heading (offset 0)
                v = speed / 100 * M_PER_S_AT_100
                self.vel[robot] = (v * math.cos(a), v * math.sin(a))
            self.last_cmd[robot] = time.time()

    def physics(self):
        t = time.time()
        while not self.stop:
            time.sleep(0.01)
            now = time.time()
            with self.lock:
                for r, p in self.pose.items():
                    if now - self.last_cmd[r] > 0.3:                    # the firmware's safety stop
                        self.vel[r] = (0.0, 0.0)
                    p[0] += self.vel[r][0] * (now - t)
                    p[1] += self.vel[r][1] * (now - t)
                    self.trail[r].append((p[0], p[1]))
            t = now

    def render(self):
        img = np.full((int(4.0 * PPM), int(4.5 * PPM), 3), (120, 125, 130), np.uint8)
        px = lambda x, y: (int(round(x * PPM)), int(round((4.0 - y) * PPM)))
        with self.lock:
            poses = {r: list(p) for r, p in self.pose.items()}
        for r, (x, y, hd) in poses.items():
            cv2.circle(img, px(x, y), int(0.025 * PPM), BGR["blue"], -1)
            h = math.radians(hd)
            for (dx, dy), col in zip(SSL, LOOKS[r]):
                cv2.circle(img, px(x + dx * math.cos(h) - dy * math.sin(h), y + dx * math.sin(h) + dy * math.cos(h)),
                           int(0.02 * PPM), BGR[col], -1)
        return img


class FakeCap:
    def __init__(self, world):
        self.world = world

    def read(self):
        time.sleep(0.02)
        return True, self.world.render()

    def release(self):
        pass


def fake_robot(world, robot, ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ip, port))
    s.settimeout(0.2)

    def loop():
        while not world.stop:
            try:
                data, _ = s.recvfrom(100)
            except (socket.timeout, OSError):
                continue
            world.command(robot, data.decode())
        s.close()
    threading.Thread(target=loop, daemon=True).start()


def fake_wifi_server(t_person):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            seen = time.time() >= t_person
            body = json.dumps({"summary": {"detected": seen},
                               "localization": {"person": list(PERSON) if seen else None}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            self.rfile.read(int(self.headers["Content-Length"]))
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"{}")

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def crosses(p, q, a, b):
    d = lambda o, s, t: (s[0] - o[0]) * (t[1] - o[1]) - (s[1] - o[1]) * (t[0] - o[0])
    return d(p, q, a) * d(p, q, b) < 0 and d(a, b, p) * d(a, b, q) < 0


def test_a_detection_behind_a_wall_sends_the_nearest_robot_around_it():
    world = World()
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    fake_robot(world, "A", "127.0.0.1", port)
    fake_robot(world, "B", "127.0.0.2", port)
    threading.Thread(target=world.physics, daemon=True).start()
    srv = fake_wifi_server(time.time() + 2.0)
    cfg = {"robots": {"A": {"pattern": {"pink": 4}, "sentry": [1.0, 1.0]},
                      "B": {"pattern": {"pink": 2, "green": 2}, "sentry": [3.5, 1.0]}},
           "area": [list(AREA[0]), list(AREA[1])],
           "mission": {"settle_s": 1.0, "detect_hold_s": 1.0}}
    H = np.array([[1 / PPM, 0, 0], [0, -1 / PPM, 4.0], [0, 0, 1]])
    ft = ColourTracker({r: c["pattern"] for r, c in cfg["robots"].items()}, H=H)
    cam = CameraTracker({"camera": {}}, cap=FakeCap(world), ft=ft)
    link = RobotLink(cmd_port=port, hello_port=0, static_ips={"A": "127.0.0.1", "B": "127.0.0.2"})
    wifi = WifiLink(f"http://127.0.0.1:{srv.server_port}", cfg["robots"])
    p = Pipeline(cfg, cam, link, wifi, {"A": RobotCal(), "B": RobotCal()}, walls=WALL)
    try:
        end = time.time() + 25
        while time.time() < end and p.mission.state not in ("verify", "found"):
            p.step(time.time())
            time.sleep(0.05)
        for _ in range(10):                   # a moment of holding still
            p.step(time.time())
            time.sleep(0.05)
        m = p.mission
        assert m.state in ("verify", "found") and m.robot == "A", (m.state, m.robot, m.events, cam.errors)
        a = world.pose["A"][:2]
        assert math.dist(a, m.target) < 0.12, (a, m.target)
        trail = world.trail["A"]
        assert not any(crosses(trail[i], trail[i + 1], *WALL[0]) for i in range(len(trail) - 1)), "drove through the wall"
        assert math.dist(world.trail["B"][0], world.pose["B"][:2]) < 0.05, "robot B should have held its spot"
        assert any("sending robot A" in e for _, e in m.events), m.events
    finally:
        p.stop_all()
        world.stop = True
        cam.close()
        wifi.close()
        link.close()
        srv.shutdown()


if __name__ == "__main__":
    test_a_detection_behind_a_wall_sends_the_nearest_robot_around_it()
    print("ok")
