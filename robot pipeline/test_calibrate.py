"""Calibration against a simulated robot: it recovers the robot's drive frame (here turned and mirrored),
records each step for the window, drives back to the start, and stops when the window is closed."""
import json
import math
import tempfile
import threading
import time
from pathlib import Path

from calibrate import CalibrationFailed, Progress, calibrate
from drive import Gains, wrap
from tracker import Pose


class SimRobot:
    """Moves at 0.3 m/s at speed 40 while commands keep coming (the firmware's 300 ms stop)."""

    def __init__(self, x, y, heading, offset, mirror):
        self.x, self.y, self.heading, self.offset, self.mirror = x, y, heading, offset, mirror
        self.lock = threading.Lock()
        self.cmd = None             # (speed, direction, time sent)
        self.t = time.time()
        self.moved = 0.0

    def _advance(self, now):
        if self.cmd and now - self.cmd[2] < 0.3:
            speed, direction, _ = self.cmd
            a = math.radians(self.heading + self.offset + (-direction if self.mirror else direction))
            step = 0.3 * speed / 40 * (now - self.t)
            self.x += step * math.cos(a)
            self.y += step * math.sin(a)
            self.moved += step
        self.t = now

    def command(self, speed, direction):
        with self.lock:
            now = time.time()
            self._advance(now)
            self.cmd = (speed, direction, now) if speed > 0 else None

    def pose(self):
        with self.lock:
            now = time.time()
            self._advance(now)
            return Pose(self.x, self.y, self.heading, now)


class FakeCam:
    def __init__(self, sim):
        self.sim = sim

    def poses(self, max_age=0.5):
        return {"A": self.sim.pose()}


class FakeLink:
    def __init__(self, sim):
        self.sim, self.addr = sim, {"A": "sim"}

    def drive(self, robot, speed, direction):
        self.sim.command(speed, direction)

    def stop(self, robot):
        self.sim.command(0, 0)


def test_calibration_recovers_a_turned_mirrored_robot_and_reports_each_step():
    sim = SimRobot(0.75, 0.7, heading=30, offset=90, mirror=True)
    prog = Progress(echo=False)
    with tempfile.TemporaryDirectory() as d:
        cal_path = Path(d) / "calibration.json"
        cal = calibrate(FakeCam(sim), FakeLink(sim), "A", Gains(), prog, speed=40, seconds=0.4, cal_path=cal_path)
        assert abs(wrap(cal.offset - 90)) < 3 and cal.mirror, cal
        assert json.loads(cal_path.read_text())["A"]["mirror"] is True
    assert list(prog.points) == ["start", "after dir 0", "after dir 90", "back"]
    start, back = prog.points["start"], prog.points["back"]
    assert math.dist((start.x, start.y), (back.x, back.y)) < 0.1
    assert prog.done and not prog.failed
    assert any("direction 0" in text for text, _ in prog.lines)


def test_closing_the_window_stops_the_robot():
    sim = SimRobot(0.75, 0.7, heading=0, offset=0, mirror=False)
    prog = Progress(echo=False)
    prog.cancel()
    try:
        calibrate(FakeCam(sim), FakeLink(sim), "A", Gains(), prog, speed=40, seconds=0.4,
                  cal_path=Path(tempfile.gettempdir()) / "unused_calibration.json")
        raise AssertionError("calibration should stop when the window is closed")
    except CalibrationFailed:
        pass
    assert sim.moved == 0.0


class OtherRobotCam:
    """The camera sees only robot B, e.g. the robot wears the other robot's colour top."""

    def __init__(self, sim):
        self.sim = sim

    def poses(self, max_age=0.5):
        return {"B": self.sim.pose()}


def test_a_robot_the_camera_cant_find_names_what_it_does_see():
    sim = SimRobot(0.75, 0.7, heading=0, offset=0, mirror=False)
    prog = Progress(echo=False)
    t0 = time.time()
    try:
        calibrate(OtherRobotCam(sim), FakeLink(sim), "A", Gains(), prog, find_s=1.0,
                  cal_path=Path(tempfile.gettempdir()) / "unused_calibration.json")
        raise AssertionError("calibration should fail without robot A's pattern")
    except CalibrationFailed as e:
        assert "robot B" in str(e) and "pattern" in str(e), str(e)
    assert time.time() - t0 >= 1.0, "it waits for the camera before giving up"
    assert sim.moved == 0.0


class SlowStartCam(FakeCam):
    """No poses for the first 0.8 s, like a camera still starting up."""

    def __init__(self, sim):
        super().__init__(sim)
        self.t0 = time.time()

    def poses(self, max_age=0.5):
        return {} if time.time() - self.t0 < 0.8 else super().poses(max_age)


def test_calibration_waits_for_the_camera_to_start():
    sim = SimRobot(0.75, 0.7, heading=0, offset=0, mirror=False)
    prog = Progress(echo=False)
    cal = calibrate(SlowStartCam(sim), FakeLink(sim), "A", Gains(), prog, seconds=0.4,
                    cal_path=Path(tempfile.gettempdir()) / "unused_calibration.json")
    assert abs(wrap(cal.offset)) < 3 and not cal.mirror


if __name__ == "__main__":
    test_calibration_recovers_a_turned_mirrored_robot_and_reports_each_step()
    test_closing_the_window_stops_the_robot()
    test_a_robot_the_camera_cant_find_names_what_it_does_see()
    test_calibration_waits_for_the_camera_to_start()
    print("ok")
