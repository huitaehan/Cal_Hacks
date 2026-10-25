"""Find which way a robot drives relative to its marker, and save it to calibration.json.

    python calibrate.py A            # put the robot in the middle of the zone, then press SPACE in the window

The robot drives briefly in its direction 0, then direction 90, and the camera measures both moves.
Then it drives back to where it started, using the result, as a check. A window shows the camera view,
each move as an arrow and what the calibration is doing; q or closing the window stops the robot.
"""
import argparse
import json
import math
import threading
import time

import cv2

from drive import CAL_FIELDS, Driver, Gains, RobotLink, solve_calibration
from run import CAL
from setup_camera import GREEN, RED, WHITE, YELLOW, banner
from tracker import CONFIG, CameraTracker, Pose, draw

WINDOW = "calibrate"
MOVES = [("start", "after dir 0", (0, 165, 255)), ("after dir 0", "after dir 90", (255, 0, 255)),
         ("after dir 90", "back", (80, 230, 80))]


class CalibrationFailed(Exception):
    pass


class Progress:
    """What the calibration is doing, shared between its thread and the window."""

    def __init__(self, echo: bool = True):
        self.echo = echo
        self.lock = threading.Lock()
        self.lines: list = []           # (text, colour), newest last
        self.points: dict = {}          # step name -> averaged Pose
        self.done = self.failed = False
        self._cancel = threading.Event()

    def say(self, text: str, colour=WHITE):
        if self.echo:
            print(text, flush=True)
        with self.lock:
            self.lines = (self.lines + [(text, colour)])[-6:]

    def mark(self, name: str, pose: Pose):
        with self.lock:
            self.points[name] = pose

    def cancel(self):
        self._cancel.set()

    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    def check(self):
        if self.cancelled:
            raise CalibrationFailed("stopped: the window was closed")


def find(cam, robot, prog: Progress, seconds):
    """Wait for the camera (which takes a moment to start) to see the robot's pattern and heading."""
    end = time.time() + seconds
    while time.time() < end:
        prog.check()
        p = cam.poses().get(robot)
        if p and p.heading is not None:
            return
        time.sleep(0.1)
    others = sorted(r for r in cam.poses() if r != robot)
    hint = (f"; it sees robot {', '.join(others)}: if this robot wears that robot's colour top, swap the two "
            f"robots' \"pattern\" entries in config.json") if others else ""
    raise CalibrationFailed(f"the camera can't see robot {robot}'s pattern (or which way it faces){hint}")


def averaged(cam, robot, prog: Progress, seconds=0.6):
    pts, end = [], time.time() + seconds
    while time.time() < end:
        prog.check()
        p = cam.poses().get(robot)
        if p and p.heading is not None:
            pts.append(p)
        time.sleep(0.03)
    if not pts:
        raise CalibrationFailed(f"the camera can't see robot {robot}'s pattern (or which way it faces)")
    h = math.degrees(math.atan2(sum(math.sin(math.radians(p.heading)) for p in pts),
                                sum(math.cos(math.radians(p.heading)) for p in pts)))
    return Pose(sum(p.x for p in pts) / len(pts), sum(p.y for p in pts) / len(pts), h, time.time())


def push(link, robot, prog: Progress, speed, direction, seconds):
    end = time.time() + seconds
    try:
        while time.time() < end:
            prog.check()
            link.drive(robot, speed, direction)
            time.sleep(0.05)
    finally:
        for _ in range(3):
            link.stop(robot)
            time.sleep(0.05)
    time.sleep(0.6)


def calibrate(cam, link, robot: str, gains: Gains, prog: Progress, speed=40.0, seconds=0.8, cal_path=CAL,
              find_s=5.0):
    """The calibration drive; returns the robot's RobotCal, also saved to cal_path. Marks `prog` done, and
    failed (with the reason as its last line) if it raises CalibrationFailed."""
    try:
        return _calibrate(cam, link, robot, gains, prog, speed, seconds, cal_path, find_s)
    except CalibrationFailed as e:
        prog.failed = True
        prog.say(str(e), RED)
        raise
    finally:
        prog.done = True


def _calibrate(cam, link, robot, gains, prog, speed, seconds, cal_path, find_s):
    prog.say(f"waiting for robot {robot} to say HELLO over WiFi ...")
    end = time.time() + 15
    while robot not in link.addr and time.time() < end:
        prog.check()
        time.sleep(0.2)
    if robot not in link.addr:
        raise CalibrationFailed("no HELLO: is the robot on the same network as this laptop, and is Python "
                                "allowed through the Windows firewall (private networks)?")
    prog.say(f"robot {robot} at {link.addr[robot]}; looking for its pattern and measuring where it is")
    find(cam, robot, prog, find_s)
    start = averaged(cam, robot, prog)
    prog.mark("start", start)
    prog.say(f"driving in its direction 0 for {seconds:.1f} s at speed {speed:.0f}", YELLOW)
    push(link, robot, prog, speed, 0, seconds)
    p1 = averaged(cam, robot, prog)
    prog.mark("after dir 0", p1)
    prog.say(f"driving in its direction 90 for {seconds:.1f} s", YELLOW)
    push(link, robot, prog, speed, 90, seconds)
    p2 = averaged(cam, robot, prog)
    prog.mark("after dir 90", p2)
    d0, d90 = math.dist((start.x, start.y), (p1.x, p1.y)), math.dist((p1.x, p1.y), (p2.x, p2.y))
    prog.say(f"moved {d0 * 100:.0f} cm in direction 0 and {d90 * 100:.0f} cm in direction 90 "
             f"({d0 / seconds:.2f} m/s at speed {speed:.0f})")
    if min(d0, d90) < 0.05:
        raise CalibrationFailed("the robot barely moved: raise --speed or --seconds, check the battery and wheels")
    cal = solve_calibration(start, (p1.x, p1.y), (p2.x, p2.y))
    prog.say(f"direction 0 = marker heading {cal.offset:+.0f} deg, "
             f"{'clockwise' if cal.mirror else 'counter-clockwise'} direction angles", GREEN)
    allc = json.loads(cal_path.read_text()) if cal_path.exists() else {}
    allc[robot] = {k: getattr(cal, k) for k in CAL_FIELDS}
    cal_path.write_text(json.dumps(allc, indent=2))
    prog.say(f"saved to {cal_path.name}; driving back to the start as a check ...", YELLOW)
    driver = Driver({robot: cal}, gains)
    end = time.time() + 10
    try:
        while time.time() < end:
            prog.check()
            p = cam.poses(max_age=0.2).get(robot)
            if p is None or p.heading is None:
                link.stop(robot)
            else:
                cmd = driver.step(robot, p, (start.x, start.y))
                if cmd.fault:
                    raise CalibrationFailed(cmd.fault)
                if cmd.arrived:
                    break
                link.drive(robot, cmd.speed, cmd.direction)
            time.sleep(0.05)
    finally:
        link.stop(robot)
    back = averaged(cam, robot, prog)
    prog.mark("back", back)
    miss = math.dist((back.x, back.y), (start.x, start.y))
    prog.say(f"back within {miss * 100:.0f} cm of the start", GREEN if miss < 0.1 else YELLOW)
    return cal


def draw_moves(prog: Progress):
    """Floor-metre drawing for tracker.draw(): the start point and each measured move as an arrow."""
    def extra(frame, px):
        with prog.lock:
            pts = dict(prog.points)
        if "start" in pts:
            s = px([(pts["start"].x, pts["start"].y)])[0]
            cv2.circle(frame, tuple(s), 12, YELLOW, 3)
            cv2.putText(frame, "start", tuple(s + [-30, 34]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)
        for a, b, colour in MOVES:
            if a in pts and b in pts:
                pa, pb = px([(pts[a].x, pts[a].y), (pts[b].x, pts[b].y)])
                cv2.arrowedLine(frame, tuple(pa), tuple(pb), colour, 4, tipLength=0.2)
                d = math.dist((pts[a].x, pts[a].y), (pts[b].x, pts[b].y)) * 100
                cv2.putText(frame, f"{b}: {d:.0f} cm", tuple((pa + pb) // 2 + [12, 0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
    return extra


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot")
    ap.add_argument("--speed", type=float, default=40)
    ap.add_argument("--seconds", type=float, default=0.5)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    rc = cfg["robots"][args.robot]
    link = RobotLink(cfg["robot_udp_port"], cfg["hello_udp_port"], {args.robot: rc["ip"]} if rc.get("ip") else None)
    cam = CameraTracker(cfg)
    prog = Progress()
    go = threading.Event()
    prog.say(f"put robot {args.robot} in the middle of the zone with 50 cm clear, check the window shows its "
             f"arrow, then press SPACE to start", YELLOW)

    def work():
        while not go.is_set() and not prog.cancelled:
            time.sleep(0.05)
        if prog.cancelled:
            return
        try:
            calibrate(cam, link, args.robot, Gains(**cfg.get("drive", {})), prog, args.speed, args.seconds)
        except CalibrationFailed:
            pass                        # already shown in the window
        except Exception as e:          # show it in the window too, not only in the console
            prog.failed = True
            prog.say(f"error: {type(e).__name__}: {e}", RED)
        finally:
            for _ in range(3):
                link.stop(args.robot)

    worker = threading.Thread(target=work, daemon=True)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720)
    worker.start()
    try:
        while True:
            frame, det = cam.snapshot()
            if frame is not None:
                draw(frame, cam.ft, det, cam.poses(), cfg["area"], extra=draw_moves(prog))
                with prog.lock:
                    lines = list(prog.lines)
                state = ("FAILED" if prog.failed else "done") if prog.done else "running" if go.is_set() else "ready"
                foot = (f"calibrating robot {args.robot}: {state}   "
                        + ("q: close" if prog.done else "SPACE: start   q: quit" if not go.is_set()
                           else "q or close the window: stop the robot"))
                banner(frame, [(foot, RED if prog.failed else GREEN if prog.done else WHITE)] + lines)
                cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord(" "):
                go.set()
            if key == ord("q") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        prog.cancel()
        worker.join(2.0)
        for _ in range(3):
            link.stop(args.robot)
        cam.close()
        link.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
