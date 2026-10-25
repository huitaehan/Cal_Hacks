"""Drive a robot to a floor point from camera poses, and talk to the robots over UDP.

The robots are omnidirectional, so they never need to turn: each command gives a speed and a
direction in the robot's own frame (the frame of the team's dir_move(): direction 0 = its "dir 0").
"""
import math
import socket
import threading
import time
from dataclasses import dataclass


def wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


@dataclass
class RobotCal:
    offset: float = 0.0     # the robot's dir 0, in field degrees = marker heading + offset
    mirror: bool = False    # True if the robot's direction angles run clockwise seen from above


CAL_FIELDS = ("offset", "mirror")


@dataclass
class Gains:
    arrive_m: float = 0.08  # closer than this counts as arrived
    slow_m: float = 0.5     # start slowing down this far from the goal
    speed_min: float = 28   # PWM % that still moves the robot (tune on the real floor)
    speed_max: float = 55


@dataclass
class Command:
    speed: float
    direction: float        # degrees in the robot's frame
    arrived: bool
    distance: float
    fault: str | None = None    # why the robot was stopped instead of driven


def drive_command(pose, goal, cal: RobotCal, g: Gains, aim=None, dist=None) -> Command:
    """Speed and direction toward `goal`. With a planned path, steer at `aim` (the next waypoint) and slow
    down by `dist` (the path length left) instead of the straight-line distance."""
    d_goal = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
    if d_goal < g.arrive_m:
        return Command(0.0, 0.0, True, d_goal)
    d = d_goal if dist is None else dist
    speed = g.speed_min + (g.speed_max - g.speed_min) * min(1.0, d / g.slow_m)
    tx, ty = aim if aim is not None else goal
    rel = wrap(math.degrees(math.atan2(ty - pose.y, tx - pose.x)) - (pose.heading + cal.offset))
    return Command(speed, -rel if cal.mirror else rel, False, d)


def _same(g1, g2):
    return g1 is not None and math.hypot(g1[0] - g2[0], g1[1] - g2[1]) < 0.02


class Driver:
    """drive_command() per robot, with three safety rules:
    - a robot without calibration is never driven (it could go any direction);
    - a robot that ends up WRONG_WAY_M farther from its goal than its best is stopped until it gets a new goal;
    - once a robot arrives it stays stopped until it's pushed more than `rearm_m` away or gets a new goal:
      camera jitter must not make a listening robot twitch."""

    WRONG_WAY_M = 0.25

    def __init__(self, cals: dict, gains: Gains, rearm_m: float = 0.15):
        self.cals, self.gains, self.rearm_m = cals, gains, rearm_m
        self._held: dict = {}       # robot -> the goal it arrived at
        self._best: dict = {}       # robot -> (goal, closest distance so far)
        self._fault: dict = {}      # robot -> (goal, reason): stopped until the goal changes

    def step(self, robot, pose, goal, aim=None, remaining=None) -> Command:
        """`aim` and `remaining` come from a planned path (planner.Navigator): steer at the next waypoint,
        and judge progress (wrong-way guard, slowing down) by the path length left."""
        goal = tuple(goal)
        d_goal = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
        d = d_goal if remaining is None else remaining
        if robot not in self.cals:
            return Command(0.0, 0.0, False, d, f"robot {robot} not calibrated: run  python calibrate.py {robot}")
        fault = self._fault.get(robot)
        if fault and _same(fault[0], goal):
            return Command(0.0, 0.0, False, d, fault[1])
        self._fault.pop(robot, None)
        best = self._best.get(robot)
        best_d = min(best[1], d) if best and _same(best[0], goal) else d
        self._best[robot] = (goal, best_d)
        if d > best_d + self.WRONG_WAY_M:
            reason = f"robot {robot} drove the wrong way and was stopped: recalibrate (python calibrate.py {robot})"
            self._fault[robot] = (goal, reason)
            return Command(0.0, 0.0, False, d, reason)
        if _same(self._held.get(robot), goal) and d_goal < self.rearm_m:
            return Command(0.0, 0.0, True, d_goal)
        self._held.pop(robot, None)
        cmd = drive_command(pose, goal, self.cals[robot], self.gains, aim, remaining)
        if cmd.arrived:
            self._held[robot] = tuple(goal)
        return cmd


def solve_calibration(start, after_dir0, after_dir90) -> RobotCal:
    """From a short drive in direction 0 then one in direction 90, recover the robot's frame."""
    phi0 = math.degrees(math.atan2(after_dir0[1] - start.y, after_dir0[0] - start.x))
    phi90 = math.degrees(math.atan2(after_dir90[1] - after_dir0[1], after_dir90[0] - after_dir0[0]))
    return RobotCal(offset=wrap(phi0 - start.heading), mirror=wrap(phi90 - phi0) < 0)


class RobotLink:
    """UDP text commands to the robots; robots announce themselves with 'HELLO <id> <ip>'."""

    def __init__(self, cmd_port: int = 4210, hello_port: int = 4211, static_ips: dict | None = None):
        self.cmd_port = cmd_port
        self.addr: dict[str, str] = dict(static_ips or {})
        self.last_hello: dict[str, float] = {}
        self.warnings: list[str] = []
        self.seq = 0
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = None
        self._stop = False
        if hello_port:
            self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.rx.bind(("", hello_port))
            self.rx.settimeout(0.5)
            threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        while not self._stop:
            try:
                data, _ = self.rx.recvfrom(200)
            except (socket.timeout, OSError):
                continue
            self.on_hello(data, time.time())

    def on_hello(self, data: bytes, now: float):
        parts = data.decode(errors="replace").split()
        if len(parts) == 3 and parts[0] == "HELLO":
            old = self.addr.get(parts[1])
            if old and old != parts[2]:
                self.warnings.append(f"robot {parts[1]} now answers from {parts[2]} (was {old}): "
                                     f"are two robots flashed with the same ID?")
                print(self.warnings[-1], flush=True)
            self.addr[parts[1]] = parts[2]
            self.last_hello[parts[1]] = now

    def _send(self, robot: str, text: str):
        ip = self.addr.get(robot)
        if ip:
            try:
                self.tx.sendto(text.encode(), (ip, self.cmd_port))
            except OSError:
                pass

    def drive(self, robot: str, speed: float, direction: float):
        self.seq += 1
        self._send(robot, f"D {self.seq} {speed:.0f} {direction:.1f} 0\n")

    def stop(self, robot: str):
        self._send(robot, "S\n")

    def close(self):
        self._stop = True
        self.tx.close()
        if self.rx:
            self.rx.close()


if __name__ == "__main__":
    # Bench test without the camera:  python drive.py A 40 0 1.0   (robot, speed, direction, seconds)
    import json
    import sys
    from pathlib import Path

    cfg = json.loads((Path(__file__).parent / "config.json").read_text())
    robot, speed, direction, seconds = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    ip = cfg["robots"][robot].get("ip")
    link = RobotLink(cfg["robot_udp_port"], cfg["hello_udp_port"], {robot: ip} if ip else None)
    end = time.time() + 15
    while robot not in link.addr and time.time() < end:
        time.sleep(0.2)
    if robot not in link.addr:
        raise SystemExit(f"no HELLO from robot {robot}: same network? Python allowed through the firewall?")
    print(f"robot {robot} at {link.addr[robot]}: driving")
    end = time.time() + seconds
    while time.time() < end:
        link.drive(robot, speed, direction)
        time.sleep(0.05)
    for _ in range(3):
        link.stop(robot)
        time.sleep(0.05)
    link.close()
