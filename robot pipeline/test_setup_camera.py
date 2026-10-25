"""Camera setup tests: the taped zone's size goes into config.json, corner clicks are sanity-checked, the
tape mapping is lifted to the robots' tops, a wrongly measured zone is spotted from the robots' dots, and
the far-edge check measures how often each robot's pattern is read."""
import json

import cv2
import numpy as np

from setup_camera import (ReadRate, config_with_zone, corner_problem, depth_from_tape, estimate_depth,
                          robot_top_mapping)
from tracker import CONFIG, apply_h

SIZE = (1920, 1080)
ZONE = (1.5, 1.35)
TOP = 0.10                                   # robot-top height (m)


def camera(f=1360.0, c=(0.75, -0.5, 1.3), look=(0.75, 0.6, 0.0)):
    """A tilted camera in front of the zone: floor metres (x, y, z) -> image pixels."""
    c = np.array(c, float)
    z = np.array(look, float) - c
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    R = np.array([x, np.cross(z, x), z])
    K = np.array([[f, 0, SIZE[0] / 2], [0, f, SIZE[1] / 2], [0, 0, 1]])

    def project(pts):
        p = (K @ (R @ (np.asarray(pts, float) - c).T)).T
        return p[:, :2] / p[:, 2:]
    return project


VIEWS = [dict(), dict(c=(0.75, 0.2, 1.4)), dict(c=(0.75, 0.6, 1.4), look=(0.75, 0.62, 0.0))]


def tape_mapping(project, w=ZONE[0], d=ZONE[1], d_entered=None):
    corners = [(0, 0, 0), (w, 0, 0), (w, d, 0), (0, d, 0)]
    clicks = project(corners)
    de = d if d_entered is None else d_entered
    H = cv2.getPerspectiveTransform(np.float32(clicks), np.float32([(0, 0), (w, 0), (w, de), (0, de)]))
    return clicks, H


def robot_blobs(project, x, y, r=0.065):
    """A robot's top seen by the camera: blue centre and 4 SSL dots at radius r, all at robot-top height."""
    dots = [(r * np.cos(a), r * np.sin(a)) for a in np.radians([57.4, -57.4, 147.4, -147.4])]
    out = [("blue", *project([(x, y, TOP)])[0], (0, 0, 36, 36))]
    out += [("pink", *project([(x + dx, y + dy, TOP)])[0], (0, 0, 24, 24)) for dx, dy in dots]
    return out


CONFIG_TEXT = """{
  "comment": "Floor metres.",
  "tracking": "colour",
  "camera": {"index": 1, "width": 1920, "height": 1080},
  "area": [[0.0, 0.0], [4.5, 4.0]],
  "robots": {
    "A": {"pattern": {"pink": 4}, "marker": 10, "sentry": [4.5, 2.0], "board_if_carried": "d0:cf:13:e1:63:30"},
    "B": {"pattern": {"pink": 2, "green": 2}, "marker": 11, "sentry": [0.0, 2.0], "board_if_carried": "d0:cf:13:e1:2e:30"}
  },
  "robot_radius_m": 0.09
}
"""


def test_zone_size_goes_into_the_config_and_sentries_move_inside():
    text = CONFIG_TEXT
    out = config_with_zone(text, 3.0, 2.0)
    cfg, old = json.loads(out), json.loads(text)
    assert cfg["area"] == [[0.0, 0.0], [3.0, 2.0]]
    assert cfg["robots"]["A"]["sentry"] == [2.75, 1.75]       # was (4.5, 2.0): outside a 3 x 2 m zone
    assert cfg["robots"]["B"]["sentry"] == [0.25, 1.75]
    for k in old:
        if k not in ("area", "robots"):
            assert cfg[k] == old[k], k
    assert cfg["robots"]["A"]["pattern"] == old["robots"]["A"]["pattern"]
    assert len(out.splitlines()) == len(text.splitlines()), "the file keeps its one-line-per-robot layout"


def test_a_sentry_well_inside_the_zone_stays_where_it_is():
    text = json.dumps({"area": [[0, 0], [4.5, 4.0]], "robots": {"A": {"sentry": [1.0, 1.2]}}})
    assert json.loads(config_with_zone(text, 3.0, 2.0))["robots"]["A"]["sentry"] == [1.0, 1.2]


def test_corner_clicks_must_go_round_the_zone_the_right_way():
    # Camera in front of the zone, tilted: the front edge is at the bottom of the image.
    fl, fr, br, bl = (100, 700), (1100, 700), (950, 120), (300, 140)
    assert corner_problem([fl, fr, br, bl]) is None
    assert "mirrored" in corner_problem([fr, fl, bl, br])       # left and right swapped
    assert "order" in corner_problem([fl, br, fr, bl])          # crossed: not going round the edge
    # Camera behind the zone: the same corners appear rotated 180 degrees, which is fine.
    rot = lambda p: (1200 - p[0], 800 - p[1])
    assert corner_problem([rot(fl), rot(fr), rot(br), rot(bl)]) is None


def test_read_rate_is_the_share_of_recent_frames_with_the_robot_and_its_heading():
    rr = ReadRate(window_s=3.0)
    for i in range(10):
        rr.add(i * 0.1, {"B": i % 2 == 0} if i < 8 else {})     # found 8 of 10 frames, heading read in 4
    found, heading = rr.rates("B")
    assert abs(found - 0.8) < 1e-9 and abs(heading - 0.4) < 1e-9
    assert rr.rates("A") == (0.0, 0.0)
    rr.add(10.0, {"B": True})                                    # the old frames have left the window
    assert rr.rates("B") == (1.0, 1.0)


def test_tape_mapping_lifted_to_the_robots_tops_reads_their_true_position():
    pts = [(x, y, TOP) for x in (0.0, 0.75, 1.5) for y in (0.0, 0.7, 1.35)]
    truth = np.array(pts)[:, :2]
    project = camera()
    _, H = tape_mapping(project)
    seen = project(pts)
    assert np.abs(apply_h(H, seen) - truth).max() > 0.03, "the tape mapping alone is off by several cm"
    H_top, cam = robot_top_mapping(H, TOP, SIZE)                   # the C922's nominal focal length
    assert np.abs(apply_h(H_top, seen) - truth).max() < 0.002
    assert np.allclose(cam, (0.75, -0.5, 1.3), atol=0.01), cam
    # This camera's real focal length may be a few % off the nominal one: still within 1 cm.
    for view in VIEWS:
        for f in (1360 * 0.97, 1360 * 1.03):
            H_top, _ = robot_top_mapping(tape_mapping(camera(f=f, **view))[1], TOP, SIZE)
            err = np.abs(apply_h(H_top, camera(f=f, **view)(pts)) - truth).max()
            assert err < 0.01, (view, f, err)


def test_a_tilted_camera_sees_from_the_tape_alone_whether_the_depth_was_measured_right():
    clicks, _ = tape_mapping(camera())
    assert abs(depth_from_tape(clicks, ZONE[0], SIZE) - ZONE[1]) < 0.01
    clicks, _ = tape_mapping(camera(c=(0.75, 0.2, 1.4)))                   # tilted less
    assert abs(depth_from_tape(clicks, ZONE[0], SIZE) - ZONE[1]) < 0.02
    clicks, _ = tape_mapping(camera(**VIEWS[2]))                             # straight down: can't tell
    assert depth_from_tape(clicks, ZONE[0], SIZE) is None


def test_a_wrongly_measured_depth_shows_in_the_robots_squashed_dots():
    project = camera()
    blobs = robot_blobs(project, 0.4, 0.5) + robot_blobs(project, 1.1, 1.0)
    clicks, _ = tape_mapping(project)
    assert abs(estimate_depth(clicks, ZONE[0], blobs) - ZONE[1]) < 0.03
    assert estimate_depth(clicks, ZONE[0], blobs[:3]) is None, "no robot with all 4 dots: no estimate"


if __name__ == "__main__":
    test_zone_size_goes_into_the_config_and_sentries_move_inside()
    test_a_sentry_well_inside_the_zone_stays_where_it_is()
    test_corner_clicks_must_go_round_the_zone_the_right_way()
    test_read_rate_is_the_share_of_recent_frames_with_the_robot_and_its_heading()
    test_tape_mapping_lifted_to_the_robots_tops_reads_their_true_position()
    test_a_tilted_camera_sees_from_the_tape_alone_whether_the_depth_was_measured_right()
    test_a_wrongly_measured_depth_shows_in_the_robots_squashed_dots()
    print("ok")
