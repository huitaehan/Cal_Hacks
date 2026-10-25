"""The whole pipeline, one window.

    camera -> robot positions -> (WiFi server, if the robots carry sensing boards)
    WiFi server person estimate -> mission (sentry / dispatch / verify / found) -> path around the walls
    -> drive commands -> robots

    python run.py
Start the WiFi server first (wifi detection\\run_dashboard.ps1). Window keys: p pause/resume (all stop),
r recall to spots, c clear found list, q quit. Right-click on the floor: the nearest robot drives there.
"""
import json
import math
import time
from pathlib import Path

import cv2

from drive import Driver, Gains, RobotCal, RobotLink
from mission import Mission, MissionConfig
from planner import Navigator, Planner
from tracker import CONFIG, CameraTracker, apply_h, draw
from wifi_link import WifiLink

HERE = Path(__file__).parent
CAL = HERE / "calibration.json"
WIFI_FLOORPLAN = HERE.parent / "wifi detection" / "host" / "floorplan.json"


def control_step(goals: dict, poses: dict, driver, link) -> list:
    """Send each robot toward its goal; stop it if it's there, has no goal, the camera can't see it (or
    hasn't seen which way it faces yet), or the driver refuses (not calibrated, wrong way, no path).
    `driver` is a Driver or a planner.Navigator. Returns the driver's reasons for refusing."""
    faults = []
    for robot, goal in goals.items():
        pose = poses.get(robot)
        if pose is None or goal is None or pose.heading is None:
            link.stop(robot)
            continue
        cmd = driver.step(robot, pose, goal)
        if cmd.fault:
            faults.append(cmd.fault)
        if cmd.speed > 0 and not cmd.fault:
            link.drive(robot, cmd.speed, cmd.direction)
        else:
            link.stop(robot)
    return faults


def load_cals() -> dict:
    if not CAL.exists():
        return {}
    return {r: RobotCal(**c) for r, c in json.loads(CAL.read_text()).items()}


def load_walls(cfg: dict) -> list:
    """The cardboard walls: "walls" in config.json if given, else the WiFi floor plan's walls."""
    if "walls" in cfg:
        return cfg["walls"]
    path = Path(cfg.get("wifi_floorplan", WIFI_FLOORPLAN))
    if not path.is_absolute():
        path = HERE / path
    return json.loads(path.read_text()).get("walls", []) if path.exists() else []


class Pipeline:
    """One control step every call: camera poses -> WiFi -> mission -> paths -> robot commands."""

    def __init__(self, cfg: dict, cam, link, wifi, cals: dict, walls=None):
        robots = cfg["robots"]
        area = tuple(tuple(p) for p in cfg["area"])
        walls = load_walls(cfg) if walls is None else walls
        clearance = cfg.get("robot_radius_m", 0.09) + cfg.get("wall_margin_m", 0.04)
        self.planner = Planner(walls, area, clearance=clearance) if walls else None
        self.mission = Mission(MissionConfig(sentry={r: tuple(c["sentry"]) for r, c in robots.items()},
                                             area=area, **cfg.get("mission", {})),
                               snap=self.planner.nearest_free if self.planner else None)
        self.nav = Navigator(Driver(cals, Gains(**cfg.get("drive", {}))), self.planner)
        self.cam, self.link, self.wifi = cam, link, wifi
        self.poses, self.goals, self.person = {}, {}, None
        self._reported, self._n_warn = set(), 0

    def step(self, now: float):
        poses = self.cam.poses()                     # up to 0.5 s old: for the WiFi server and the display
        fresh = self.cam.poses(max_age=0.2)          # driving never uses an older position
        self.wifi.add_poses(poses)
        person = self.wifi.person()
        goals = self.mission.tick(now, poses, person)
        for fault in control_step(goals, fresh, self.nav, self.link):
            if fault not in self._reported:
                self._reported.add(fault)
                self.mission.note(now, fault)
        for w in self.link.warnings[self._n_warn:]:
            self.mission.note(now, w)
        self._n_warn = len(self.link.warnings)
        self.poses, self.goals, self.person = poses, goals, person

    def send_nearest(self, now: float, xy):
        """Operator: the nearest robot drives to xy."""
        if self.poses:
            self.mission.goto(now, min(self.poses, key=lambda r: math.dist((self.poses[r].x, self.poses[r].y), xy)), xy)

    def stop_all(self):
        for _ in range(3):
            for r in self.mission.sentry:
                self.link.stop(r)
            time.sleep(0.05)


def overlay(frame, px, p: Pipeline, cfg):
    font = cv2.FONT_HERSHEY_SIMPLEX
    mission, poses, goals, person = p.mission, p.poses, p.goals, p.person
    for a, b in (p.planner.walls if p.planner else []):
        cv2.line(frame, *map(tuple, px([a, b])), (40, 40, 220), 4)
    for r, (x, y) in mission.sentry.items():
        cv2.circle(frame, tuple(px([(x, y)])[0]), 8, (160, 160, 160), 2)
    for r, g in goals.items():
        if g is not None and r in poses:
            pts = [(poses[r].x, poses[r].y)] + (p.nav.path(r) or [g])
            cv2.polylines(frame, [px(pts)], False, (0, 220, 255), 2)
            cv2.circle(frame, tuple(px([g])[0]), 6, (0, 220, 255), -1)
    if person is not None:
        c, e = px([person, (person[0] + 0.3, person[1])])
        cv2.circle(frame, tuple(c), max(8, int(math.dist(c, e))), (0, 0, 255), 3)
        cv2.putText(frame, "WiFi: motion", tuple(c + [12, -12]), font, 0.7, (0, 0, 255), 2)
    for _, x, y in mission.found:
        c = px([(x, y)])[0]
        cv2.drawMarker(frame, tuple(c), (0, 200, 0), cv2.MARKER_TILTED_CROSS, 40, 4)
        cv2.putText(frame, "FOUND", tuple(c + [20, 30]), font, 0.9, (0, 200, 0), 2)
    lines = [f"STATE: {mission.state.upper()}" + (f"  (robot {mission.robot})" if mission.robot and
                                                 mission.state not in ("sentry", "return", "paused") else ""),
             f"WiFi server: {p.wifi.status}" + ("" if person is None else f"   person ({person[0]:.1f}, {person[1]:.1f})"),
             "robots: " + "  ".join(f"{r} {'seen' if r in poses else 'NOT SEEN'}, "
                                     f"{'link ' + p.link.addr[r] if r in p.link.addr else 'no HELLO yet'}"
                                     for r in cfg["robots"])]
    lines += [time.strftime("%H:%M:%S ", time.localtime(t)) + e for t, e in mission.events[-6:]]
    lines.append("p pause/resume   r recall   c clear found   q quit   right-click: send nearest robot")
    y = 30
    for i, s in enumerate(lines):
        cv2.putText(frame, s, (15, y), font, 0.8 if i == 0 else 0.6, (0, 0, 0), 4)
        cv2.putText(frame, s, (15, y), font, 0.8 if i == 0 else 0.6, (255, 255, 255), 1 if i else 2)
        y += 30 if i == 0 else 24


def main():
    cfg = json.loads(CONFIG.read_text())
    robots = cfg["robots"]
    cals = load_cals()
    for r in robots:
        if r not in cals:
            print(f"robot {r} isn't calibrated yet, so it won't be driven: run  python calibrate.py {r}")
    link = RobotLink(cfg["robot_udp_port"], cfg["hello_udp_port"],
                     {r: c["ip"] for r, c in robots.items() if c.get("ip")})
    wifi = WifiLink(cfg["wifi_server"], robots)
    cam = CameraTracker(cfg)
    p = Pipeline(cfg, cam, link, wifi, cals)
    print(f"{len(p.planner.walls) if p.planner else 0} walls loaded")
    click = []
    cv2.namedWindow("mission", cv2.WINDOW_NORMAL)
    # Right-click only: the left click that focuses the window must not move a robot.
    cv2.setMouseCallback("mission", lambda ev, x, y, *_: click.append((x, y)) if ev == cv2.EVENT_RBUTTONDOWN else None)
    printed = 0
    log = open(HERE / "mission_log.txt", "a", buffering=1)     # the events, kept for looking back at a run
    log.write(time.strftime("\n%Y-%m-%d %H:%M:%S run.py started\n"))
    try:
        while True:
            now = time.time()
            if click and cam.ft.H is not None:
                p.send_nearest(now, tuple(float(v) for v in apply_h(cam.ft.H, [click.pop()])[0]))
            click.clear()
            p.step(now)
            m = p.mission
            for t, e in m.events[len(m.events) - (m.n_logged - printed):]:
                line = f"{time.strftime('%H:%M:%S', time.localtime(t))} {e}"
                print(line, flush=True)
                log.write(line + "\n")
            printed = m.n_logged

            frame, det = cam.snapshot()
            if frame is not None:
                draw(frame, cam.ft, det, p.poses, cfg["area"], extra=lambda f, px: overlay(f, px, p, cfg))
                cv2.imshow("mission", frame)
            key = cv2.waitKey(40) & 0xFF
            if key == ord("q"):
                break
            if key == ord("p"):
                m.resume(now) if m.state == "paused" else m.pause(now)
            elif key == ord("r"):
                m.recall(now)
            elif key == ord("c"):
                m.clear_found(now)
    finally:
        p.stop_all()
        cam.close()
        wifi.close()
        link.close()
        log.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
