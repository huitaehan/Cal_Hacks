"""Robot motor controller on a Raspberry Pi, for a robot whose ESP32 died: firmware/robot_motor in Python.

It speaks the firmware's UDP protocol, so the laptop side (drive.py, run.py) doesn't change:
  D <seq> <speed 0..100> <direction deg> <turn -100..100>   drive (direction 0 = the robot's own "dir 0")
  S                                                         stop
The motors stop by themselves if no command arrives for 300 ms. Every second the Pi broadcasts
"HELLO <id> <ip>" on UDP port 4211 so the laptop finds it.

Wiring: move each motor-driver wire from the dead ESP32's pin to a Pi GPIO (BCM numbers):
  ESP pin 2 -> GPIO17,  pin 3 -> GPIO27    (motor A)
  ESP pin 0 -> GPIO22,  pin 1 -> GPIO23    (motor B)
  ESP pin 8 -> GPIO24,  pin 9 -> GPIO25    (motor C)
  and a Pi GND to the motor driver's GND.
Other GPIOs: --pins 17,27,22,23,24,25 in that order.

On the Pi (gpiozero ships with Raspberry Pi OS):
  python3 pi_robot.py A --spin     wheels off the floor: each motor forward then back, one at a time
  python3 pi_robot.py A            run, until Ctrl+C
"""
from __future__ import annotations

import argparse
import math
import socket
import threading
import time

CMD_PORT = 4210
HELLO_PORT = 4211
WATCHDOG_S = 0.3
DEFAULT_PINS = (17, 27, 22, 23, 24, 25)


def wheel_speeds(speed: float, direction: float, turn: float) -> tuple[int, int, int]:
    """drive() and move() from the firmware: -100..100 for motors A, B and C."""
    speed = min(max(speed, 0.0), 100.0)
    turn = min(max(turn, -100.0), 100.0)
    a = math.radians(direction)
    ax, ay = speed * math.cos(a), speed * math.sin(a)
    f1 = 0.578 * ax - 0.333 * ay + 0.333 * turn
    f2 = 0.567 * ay + 0.333 * turn
    f3 = -0.578 * ax - 0.333 * ay + 0.333 * turn
    top = max(speed, abs(turn))
    m = max(abs(f1), abs(f2), abs(f3))
    if m > 0:
        f1, f2, f3 = f1 * top / m, f2 * top / m, f3 * top / m
    # The team's move() sends F1 to motor A, F3 to motor B and F2 to motor C.
    return int(f1), int(f3), int(f2)


class Robot:
    """Three motors, each a (forward, backward) pair of PWM outputs with a .value from 0 to 1."""

    def __init__(self, outputs, watchdog_s: float = WATCHDOG_S):
        self.pairs = [tuple(outputs[i:i + 2]) for i in range(0, 6, 2)]
        self.watchdog_s = watchdog_s
        self.last_cmd = 0.0
        self.stopped = True
        self.set_wheels(0, 0, 0)

    def set_wheels(self, *wheels):
        for (fwd, back), w in zip(self.pairs, wheels):
            duty = min(abs(int(w * 2.55)), 255) / 255     # analogWrite(ma * 2.55) on the ESP
            fwd.value, back.value = (duty, 0) if w > 0 else (0, duty)
        self.stopped = not any(wheels)

    def handle(self, line: str, now: float):
        parts = line.split()
        if not parts:
            return
        if parts[0] == "S":
            self.set_wheels(0, 0, 0)
            self.last_cmd = now
        elif parts[0] == "D" and len(parts) >= 5:
            try:
                speed, direction, turn = (float(p) for p in parts[2:5])
            except ValueError:
                return
            if not all(map(math.isfinite, (speed, direction, turn))):
                self.set_wheels(0, 0, 0)                     # never turn "nan" into wheel speeds
                return
            self.set_wheels(*wheel_speeds(speed, direction, turn))
            self.last_cmd = now

    def tick(self, now: float):
        if not self.stopped and now - self.last_cmd > self.watchdog_s:
            self.set_wheels(0, 0, 0)


def own_ip():
    """This Pi's address on the network the laptop is on, or None while WiFi is down."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))      # sends nothing; just picks the outgoing interface
        ip = s.getsockname()[0]
        return None if ip.startswith("127.") or ip == "0.0.0.0" else ip
    except OSError:
        return None
    finally:
        s.close()


def serve(robot: Robot, robot_id: str, cmd_port: int = CMD_PORT, hello_port: int = HELLO_PORT,
          stop: threading.Event | None = None, bound=None):
    """Drive commands in, HELLO out, until stop is set. `bound` is set once the command port is open."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("", cmd_port))
    rx.settimeout(0.05)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    if bound:
        bound.set()
    last_hello, last_ip = 0.0, ""
    try:
        while not (stop and stop.is_set()):
            try:
                data, _ = rx.recvfrom(200)
                robot.handle(data.decode(errors="replace"), time.monotonic())
            except socket.timeout:
                pass
            robot.tick(time.monotonic())
            now = time.monotonic()
            if hello_port and now - last_hello > 1.0:
                last_hello = now
                ip = own_ip()
                if ip != last_ip:
                    print(f"HELLO {robot_id} {ip}" if ip else "wifi: not connected", flush=True)
                    last_ip = ip
                if ip:
                    try:
                        tx.sendto(f"HELLO {robot_id} {ip}".encode(), ("255.255.255.255", hello_port))
                    except OSError:
                        pass
    finally:
        robot.set_wheels(0, 0, 0)
        rx.close()
        tx.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot_id", choices=["A", "B"], help="must match robot pipeline/config.json")
    ap.add_argument("--pins", default=",".join(map(str, DEFAULT_PINS)),
                    help="BCM GPIOs for ESP pins 2,3,0,1,8,9 (motor A fwd/back, B, C)")
    ap.add_argument("--spin", action="store_true", help="wiring check: each motor forward then back")
    args = ap.parse_args()

    from gpiozero import PWMOutputDevice
    pins = [int(p) for p in args.pins.split(",")]
    if len(pins) != 6:
        raise SystemExit("--pins needs 6 GPIO numbers")
    robot = Robot([PWMOutputDevice(p, frequency=1000) for p in pins])

    if args.spin:
        for i, name in enumerate("ABC"):
            for label, w in (("forward", 40), ("backward", -40)):
                wheels = [0, 0, 0]
                wheels[i] = w
                print(f"motor {name} {label}", flush=True)
                robot.set_wheels(*wheels)
                time.sleep(1.0)
                robot.set_wheels(0, 0, 0)
                time.sleep(0.5)
        return

    print(f"robot {args.robot_id}: listening on UDP {CMD_PORT}, Ctrl+C to stop", flush=True)
    try:
        serve(robot, args.robot_id)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
