"""Tests for robot tracking and stop gating.  Run: python test_robots.py"""

import time
from pathlib import Path

import numpy as np

from presence_server import App, Link, Settings
from robots import RobotTracker

EXAMPLE = Path(__file__).resolve().parent / "floorplan.example.json"


def _post(tracker, t, x, y, mac="rr"):
    tracker.update([{"id": "A", "mac": mac, "x": x, "y": y}], now=t)


def test_stop_detection():
    tr = RobotTracker()
    t = 100.0
    for i in range(31):                      # 3 s moving at 10 cm/s, 10 Hz
        _post(tr, t, 0.01 * i, 0.0)
        t += 0.1
    assert not tr.is_stopped("rr", t - 0.1)
    for _ in range(32):                      # 3.2 s holding still
        _post(tr, t, 0.30, 0.0)
        t += 0.1
    assert tr.is_stopped("rr", t - 0.1)


def test_stale_positions():
    tr = RobotTracker()
    for i in range(40):
        _post(tr, 10.0 + i * 0.1, 1.0, 1.0)
    last = 10.0 + 39 * 0.1
    assert tr.is_stopped("rr", last)
    assert tr.position("rr", last + 1.5) is None
    assert not tr.is_stopped("rr", last + 1.5)


def test_unknown_mac_warning():
    tr = RobotTracker()
    _post(tr, 5.0, 1.0, 1.0, mac="zz:zz")
    warnings = tr.warnings(heard={"aa:aa", "bb:bb"}, fixed={"bb:bb"}, now=5.0)
    assert any("zz:zz" in w for w in warnings), warnings      # posted but no board heard
    assert any("aa:aa" in w for w in warnings), warnings      # heard but no position
    assert not any("bb:bb" in w for w in warnings), warnings  # fixed in the floor plan


def test_gating():
    tr = RobotTracker()
    clock = {"t": 0.0}
    link = Link("rr", "aa", Settings(10, detector="denoised"),
                still=lambda mac, now: mac != "rr" or tr.is_stopped("rr", now))
    amp = 25 + np.zeros(57, np.float32)
    for i in range(20):                       # robot moving: samples dropped
        clock["t"] = i * 0.1
        _post(tr, clock["t"], 0.05 * i, 0.0)
        link.push(clock["t"], amp, -50)
    assert link.n == 0
    for i in range(20, 60):                   # robot still for 4 s: samples accepted after 3 s
        clock["t"] = i * 0.1
        _post(tr, clock["t"], 1.0, 0.0)
        link.push(clock["t"], amp, -50)
    assert link.n > 0


def test_sentry_static_boards():
    app = App([], Settings(10, detector="denoised"), record=False, reset_on_start=False,
              floorplan_path=EXAMPLE)
    a, b = "d0:cf:13:e1:66:2c", "d0:cf:13:e1:63:ac"   # both listed in the example's "boards"
    now = time.time()
    assert app.is_still(a, now) and app.is_still(b, now)
    link = app.registry.get(a, b)
    link.push(now, 25 + np.zeros(57, np.float32), -50)
    assert link.n == 1


if __name__ == "__main__":
    test_stop_detection()
    test_stale_positions()
    test_unknown_mac_warning()
    test_gating()
    test_sentry_static_boards()
    print("ok")
