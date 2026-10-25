"""Drive-to-point maths: camera pose + goal -> (speed, direction in the robot's own frame)."""
import socket

from drive import Driver, Gains, RobotCal, RobotLink, drive_command, solve_calibration
from tracker import Pose

G = Gains()


def test_direction_is_turned_into_the_robots_own_frame():
    # Marker top (and robot dir 0) faces +y; the goal is along +x, i.e. 90 degrees clockwise of dir 0.
    cmd = drive_command(Pose(1.0, 1.0, 90.0, 0), (2.0, 1.0), RobotCal(), G)
    assert abs(cmd.direction - (-90.0)) < 1e-6, cmd
    # Robot wiring with clockwise direction angles: the same move is +90.
    cmd = drive_command(Pose(1.0, 1.0, 90.0, 0), (2.0, 1.0), RobotCal(mirror=True), G)
    assert abs(cmd.direction - 90.0) < 1e-6, cmd
    # Marker taped 90 degrees off: offset says dir 0 faces +y although the marker's top faces +x.
    cmd = drive_command(Pose(1.0, 1.0, 0.0, 0), (1.0, 3.0), RobotCal(offset=90.0), G)
    assert abs(cmd.direction) < 1e-6, cmd


def test_speed_slows_near_the_goal_and_stops_on_arrival():
    far = drive_command(Pose(0, 0, 0, 0), (2.0, 0.0), RobotCal(), G)
    near = drive_command(Pose(0, 0, 0, 0), (0.25, 0.0), RobotCal(), G)
    there = drive_command(Pose(0, 0, 0, 0), (0.05, 0.0), RobotCal(), G)
    assert far.speed == G.speed_max and not far.arrived
    assert G.speed_min < near.speed < G.speed_max, near
    assert there.speed == 0 and there.arrived


def test_calibration_from_two_short_test_drives():
    # True robot dir 0 points along field 120 deg; the marker reads 30 deg -> offset 90.
    start = Pose(2.0, 2.0, 30.0, 0)
    after_dir0 = (2.0 - 0.15, 2.0 + 0.26)          # moved toward 120 deg
    after_dir90_ccw = (after_dir0[0] - 0.26, after_dir0[1] - 0.15)   # dir 90 -> field 210 deg
    cal = solve_calibration(start, after_dir0, after_dir90_ccw)
    assert abs(cal.offset - 90.0) < 1.0 and not cal.mirror, cal
    after_dir90_cw = (after_dir0[0] + 0.26, after_dir0[1] + 0.15)    # dir 90 -> field 30 deg
    cal = solve_calibration(start, after_dir0, after_dir90_cw)
    assert abs(cal.offset - 90.0) < 1.0 and cal.mirror, cal


def test_link_sends_text_commands_to_the_robot_that_said_hello():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2.0)
    link = RobotLink(cmd_port=rx.getsockname()[1], hello_port=0)
    try:
        link.on_hello(b"HELLO A 127.0.0.1", now=0.0)
        link.drive("A", 40.0, -35.04)
        assert rx.recv(100).decode().split()[2:] == ["40", "-35.0", "0"]
        link.stop("A")
        assert rx.recv(100).decode().strip() == "S"
        link.drive("B", 40.0, 0.0)          # never said hello: nothing sent, no error
    finally:
        link.close()
        rx.close()


def test_driver_stays_stopped_after_arriving_despite_camera_jitter():
    d = Driver({"A": RobotCal()}, Gains(arrive_m=0.08), rearm_m=0.15)
    assert d.step("A", Pose(0, 0, 0, 0), (1.0, 0.0)).speed > 0
    assert d.step("A", Pose(0.95, 0, 0, 0), (1.0, 0.0)).arrived
    jitter = d.step("A", Pose(0.88, 0.03, 0, 0), (1.0, 0.0))        # 12 cm off: still holding
    assert jitter.speed == 0 and jitter.arrived, jitter
    pushed = d.step("A", Pose(0.80, 0.0, 0, 0), (1.0, 0.0))          # 20 cm off: drive back
    assert pushed.speed > 0 and not pushed.arrived, pushed
    d.step("A", Pose(0.97, 0, 0, 0), (1.0, 0.0))
    moved_goal = d.step("A", Pose(0.97, 0, 0, 0), (1.1, 0.0))        # new goal 13 cm away: go
    assert moved_goal.speed > 0, moved_goal


def test_uncalibrated_robots_are_not_driven():
    """Review finding: without calibration a robot can drive the wrong way at full speed."""
    cmd = Driver({}, Gains()).step("A", Pose(0, 0, 0, 0), (1.0, 0.0))
    assert cmd.speed == 0 and "calibrat" in cmd.fault, cmd


def test_a_robot_driving_away_from_its_goal_is_stopped_until_it_gets_a_new_goal():
    d = Driver({"A": RobotCal()}, Gains())
    assert d.step("A", Pose(0, 0, 0, 0), (1.0, 0.0)).speed > 0
    assert d.step("A", Pose(-0.1, 0, 0, 0), (1.0, 0.0)).speed > 0       # 10 cm worse: could be noise
    wrong = d.step("A", Pose(-0.3, 0, 0, 0), (1.0, 0.0))                 # 30 cm worse than its best
    assert wrong.speed == 0 and "wrong way" in wrong.fault, wrong
    assert d.step("A", Pose(-0.3, 0, 0, 0), (1.0, 0.0)).speed == 0      # stays stopped
    assert d.step("A", Pose(-0.3, 0, 0, 0), (0.0, 0.0)).speed > 0       # new goal: tries again


def test_a_robot_that_changes_its_hello_address_is_reported():
    link = RobotLink(cmd_port=9, hello_port=0)
    try:
        link.on_hello(b"HELLO A 10.0.0.5", now=0.0)
        link.on_hello(b"HELLO A 10.0.0.5", now=1.0)
        assert link.warnings == []
        link.on_hello(b"HELLO A 10.0.0.7", now=2.0)       # two boards flashed as A?
        assert link.warnings and "10.0.0.7" in link.warnings[-1], link.warnings
    finally:
        link.close()


if __name__ == "__main__":
    test_uncalibrated_robots_are_not_driven()
    test_a_robot_driving_away_from_its_goal_is_stopped_until_it_gets_a_new_goal()
    test_a_robot_that_changes_its_hello_address_is_reported()
    test_driver_stays_stopped_after_arriving_despite_camera_jitter()
    test_direction_is_turned_into_the_robots_own_frame()
    test_speed_slows_near_the_goal_and_stops_on_arrival()
    test_calibration_from_two_short_test_drives()
    test_link_sends_text_commands_to_the_robot_that_said_hello()
    print("ok")
