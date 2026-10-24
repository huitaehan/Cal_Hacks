"""The control step: goals + camera poses -> UDP drive/stop commands for each robot."""
from drive import Driver, Gains, RobotCal
from run import control_step
from tracker import Pose


class FakeLink:
    def __init__(self):
        self.sent = []

    def drive(self, robot, speed, direction):
        self.sent.append((robot, "drive", round(speed), round(direction)))

    def stop(self, robot):
        self.sent.append((robot, "stop"))


def test_drives_toward_goals_and_stops_robots_that_arrived_have_no_goal_or_arent_seen():
    link = FakeLink()
    driver = Driver({"A": RobotCal(), "B": RobotCal(), "C": RobotCal(), "D": RobotCal()}, Gains())
    poses = {"A": Pose(0, 0, 0, 0), "B": Pose(1, 1, 0, 0), "D": Pose(2, 2, 0, 0)}
    goals = {"A": (0, 2.0), "B": (1.02, 1.0), "C": (3, 3), "D": None}
    control_step(goals, poses, driver, link)
    assert ("A", "drive", Gains().speed_max, 90) in link.sent, link.sent       # +y = 90 deg in A's frame
    assert ("B", "stop") in link.sent                                         # already there
    assert ("C", "stop") in link.sent                                         # camera can't see it
    assert ("D", "stop") in link.sent                                         # paused / no goal


def test_a_robot_with_unknown_heading_is_stopped_not_guessed():
    link = FakeLink()
    control_step({"A": (2.0, 2.0)}, {"A": Pose(0, 0, None, 0)}, Driver({}, Gains()), link)
    assert link.sent == [("A", "stop")], link.sent


def test_driver_faults_stop_the_robot_and_are_reported():
    link = FakeLink()
    faults = control_step({"A": (2.0, 2.0)}, {"A": Pose(0, 0, 0, 0)}, Driver({}, Gains()), link)
    assert link.sent == [("A", "stop")] and len(faults) == 1 and "calibrat" in faults[0], (link.sent, faults)


if __name__ == "__main__":
    test_driver_faults_stop_the_robot_and_are_reported()
    test_a_robot_with_unknown_heading_is_stopped_not_guessed()
    test_drives_toward_goals_and_stops_robots_that_arrived_have_no_goal_or_arent_seen()
    print("ok")
