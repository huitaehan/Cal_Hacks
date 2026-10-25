"""Paths around the cardboard walls, and driving along them."""
import math

from drive import Driver, Gains, RobotCal
from planner import Navigator, Planner
from tracker import Pose

AREA = ((0.0, 0.0), (3.0, 2.0))
# A wall across the middle (x = 1.5) with a 0.5 m doorway from y = 0.75 to 1.25.
WALL = [[[1.5, 0.0], [1.5, 0.75]], [[1.5, 1.25], [1.5, 2.0]]]
CLEAR = 0.13


def crosses(p, q, a, b):
    d = lambda o, s, t: (s[0] - o[0]) * (t[1] - o[1]) - (s[1] - o[1]) * (t[0] - o[0])
    return d(p, q, a) * d(p, q, b) < 0 and d(a, b, p) * d(a, b, q) < 0


def seg_point_dist(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def path_ok(start, path, walls):
    pts = [start] + list(path)
    for p, q in zip(pts, pts[1:]):
        for a, b in walls:
            if crosses(p, q, a, b):
                return False
        for k in range(21):                      # keep clear of walls along the way (except at the start)
            s = (p[0] + (q[0] - p[0]) * k / 20, p[1] + (q[1] - p[1]) * k / 20)
            if math.dist(s, start) > CLEAR and min(seg_point_dist(a, b, s) for a, b in walls) < CLEAR - 0.02:
                return False
    return True


def test_open_floor_is_a_straight_line():
    assert Planner([], AREA, clearance=CLEAR).plan((0.5, 0.5), (2.5, 1.5)) == [(2.5, 1.5)]


def test_a_wall_in_the_way_is_passed_through_the_doorway():
    pl = Planner(WALL, AREA, clearance=CLEAR)
    path = pl.plan((0.5, 0.3), (2.5, 0.3))
    assert path and path[-1] == (2.5, 0.3), path
    assert path_ok((0.5, 0.3), path, WALL), path
    assert any(0.75 < y < 1.25 for x, y in path if 1.2 < x < 1.8) or len(path) >= 2, path
    length = sum(math.dist(p, q) for p, q in zip([(0.5, 0.3)] + path, path))
    assert length < 3.2, length                  # straight line is 2.0; via the door about 2.7


def test_a_goal_against_a_wall_moves_to_free_floor():
    pl = Planner(WALL, AREA, clearance=CLEAR)
    path = pl.plan((0.5, 0.3), (1.45, 0.3))      # 5 cm from the wall
    end = path[-1]
    assert min(seg_point_dist(a, b, end) for a, b in WALL) >= CLEAR - 1e-6 and end[0] < 1.5, end


def test_an_enclosed_goal_has_no_path():
    box = [[[2.0, 0.5], [2.6, 0.5]], [[2.6, 0.5], [2.6, 1.5]], [[2.6, 1.5], [2.0, 1.5]], [[2.0, 1.5], [2.0, 0.5]]]
    assert Planner(box, AREA, clearance=CLEAR).plan((0.5, 1.0), (2.3, 1.0)) is None


def test_navigator_steers_for_the_doorway_not_straight_through_the_wall():
    nav = Navigator(Driver({"A": RobotCal()}, Gains()), Planner(WALL, AREA, clearance=CLEAR))
    cmd = nav.step("A", Pose(0.5, 0.3, 0.0, 0.0), (2.5, 0.3), now=0.0)
    assert cmd.speed > 0 and 10 < cmd.direction < 80, cmd      # up toward the door, not straight along +x


def test_going_around_a_wall_is_not_mistaken_for_driving_the_wrong_way():
    """The straight-line distance grows while going round; the wrong-way guard must use the path."""
    nav = Navigator(Driver({"A": RobotCal()}, Gains()), Planner(WALL, AREA, clearance=CLEAR))
    pos, t = (1.2, 0.3), 0.0
    for _ in range(200):
        cmd = nav.step("A", Pose(pos[0], pos[1], 0.0, t), (1.8, 0.3), now=t)
        assert not cmd.fault, (pos, cmd)
        if cmd.arrived:
            break
        a = math.radians(cmd.direction)          # heading 0, no offset: robot frame = floor frame
        pos, t = (pos[0] + 0.03 * math.cos(a), pos[1] + 0.03 * math.sin(a)), t + 0.05
    assert math.dist(pos, (1.8, 0.3)) < 0.1, pos


def test_no_path_stops_the_robot_with_a_reason():
    box = [[[2.0, 0.5], [2.6, 0.5]], [[2.6, 0.5], [2.6, 1.5]], [[2.6, 1.5], [2.0, 1.5]], [[2.0, 1.5], [2.0, 0.5]]]
    nav = Navigator(Driver({"A": RobotCal()}, Gains()), Planner(box, AREA, clearance=CLEAR))
    cmd = nav.step("A", Pose(0.5, 1.0, 0.0, 0.0), (2.3, 1.0), now=0.0)
    assert cmd.speed == 0 and "no path" in cmd.fault, cmd


if __name__ == "__main__":
    test_open_floor_is_a_straight_line()
    test_a_wall_in_the_way_is_passed_through_the_doorway()
    test_a_goal_against_a_wall_moves_to_free_floor()
    test_an_enclosed_goal_has_no_path()
    test_navigator_steers_for_the_doorway_not_straight_through_the_wall()
    test_going_around_a_wall_is_not_mistaken_for_driving_the_wrong_way()
    test_no_path_stops_the_robot_with_a_reason()
    print("ok")
