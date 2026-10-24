"""Find which way a robot drives relative to its marker, and save it to calibration.json.

    python calibrate.py A            # the robot needs about 0.5 m of free floor around it

The robot drives briefly in its direction 0, then direction 90, and the camera measures both moves.
Then it drives back to where it started, using the result, as a check.
"""
import argparse
import json
import math
import time

from drive import CAL_FIELDS, Driver, Gains, RobotLink, solve_calibration
from run import CAL
from tracker import CONFIG, CameraTracker, Pose


def averaged(cam, robot, seconds=0.6):
    pts, end = [], time.time() + seconds
    while time.time() < end:
        p = cam.poses().get(robot)
        if p and p.heading is not None:
            pts.append(p)
        time.sleep(0.03)
    if not pts:
        raise SystemExit(f"the camera can't see robot {robot}'s pattern (or which way it faces)")
    h = math.degrees(math.atan2(sum(math.sin(math.radians(p.heading)) for p in pts),
                                sum(math.cos(math.radians(p.heading)) for p in pts)))
    return Pose(sum(p.x for p in pts) / len(pts), sum(p.y for p in pts) / len(pts), h, time.time())


def push(link, robot, speed, direction, seconds):
    end = time.time() + seconds
    while time.time() < end:
        link.drive(robot, speed, direction)
        time.sleep(0.05)
    for _ in range(3):
        link.stop(robot)
        time.sleep(0.05)
    time.sleep(0.6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot")
    ap.add_argument("--speed", type=float, default=40)
    ap.add_argument("--seconds", type=float, default=0.8)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    rc = cfg["robots"][args.robot]
    link = RobotLink(cfg["robot_udp_port"], cfg["hello_udp_port"], {args.robot: rc["ip"]} if rc.get("ip") else None)
    cam = CameraTracker(cfg)
    try:
        print(f"waiting for robot {args.robot} to say HELLO over WiFi ...")
        end = time.time() + 15
        while args.robot not in link.addr and time.time() < end:
            time.sleep(0.2)
        if args.robot not in link.addr:
            raise SystemExit("no HELLO: is the robot on the same network as this laptop, and is Python allowed "
                             "through the Windows firewall (private networks)?")
        print(f"robot {args.robot} at {link.addr[args.robot]}")
        start = averaged(cam, args.robot)
        push(link, args.robot, args.speed, 0, args.seconds)
        p1 = averaged(cam, args.robot)
        push(link, args.robot, args.speed, 90, args.seconds)
        p2 = averaged(cam, args.robot)
        d0, d90 = math.dist((start.x, start.y), (p1.x, p1.y)), math.dist((p1.x, p1.y), (p2.x, p2.y))
        print(f"moved {d0:.2f} m in direction 0 and {d90:.2f} m in direction 90 "
              f"({d0 / args.seconds:.2f} m/s at speed {args.speed:.0f})")
        if min(d0, d90) < 0.05:
            raise SystemExit("the robot barely moved: raise --speed or --seconds, check the battery and wheels")
        cal = solve_calibration(start, (p1.x, p1.y), (p2.x, p2.y))
        print(f"robot {args.robot}: direction 0 = marker heading {cal.offset:+.0f} deg, "
              f"{'clockwise' if cal.mirror else 'counter-clockwise'} direction angles")
        allc = json.loads(CAL.read_text()) if CAL.exists() else {}
        allc[args.robot] = {k: getattr(cal, k) for k in CAL_FIELDS}
        CAL.write_text(json.dumps(allc, indent=2))
        print(f"saved to {CAL.name}; driving back to the start as a check ...")
        driver = Driver({args.robot: cal}, Gains(**cfg.get("drive", {})))
        end = time.time() + 10
        while time.time() < end:
            p = cam.poses(max_age=0.2).get(args.robot)
            if p is None or p.heading is None:
                link.stop(args.robot)
            else:
                cmd = driver.step(args.robot, p, (start.x, start.y))
                if cmd.fault:
                    print(cmd.fault)
                    break
                if cmd.arrived:
                    break
                link.drive(args.robot, cmd.speed, cmd.direction)
            time.sleep(0.05)
        link.stop(args.robot)
        back = averaged(cam, args.robot)
        print(f"back within {math.dist((back.x, back.y), (start.x, start.y)) * 100:.0f} cm of the start")
    finally:
        for _ in range(3):
            link.stop(args.robot)
        cam.close()
        link.close()


if __name__ == "__main__":
    main()
