"""Tracker tests: floor markers fix the camera-to-floor mapping, robot markers give x, y and heading."""
import math

import cv2
import numpy as np

from tracker import DICT, FloorTracker, Pose, draw, nearest_wall_m, pose_from_corners

FLOOR = {0: (0.0, 0.0), 1: (4.5, 0.0), 2: (4.5, 4.0), 3: (0.0, 4.0)}
ROBOTS = {10: "A", 11: "B"}
# A tilted camera: floor metres -> pixels.
CAM = np.array([[260.0, 40.0, 300.0], [-20.0, -150.0, 900.0], [0.01, 0.06, 1.0]])


def to_px(pts):
    p = np.c_[np.asarray(pts, float), np.ones(len(pts))] @ CAM.T
    return p[:, :2] / p[:, 2:]


def square(cx, cy, heading_deg, side=0.15):
    """Marker corners in ArUco order (top-left, top-right, bottom-right, bottom-left); 'top' faces heading."""
    h = math.radians(heading_deg)
    f, r = np.array([math.cos(h), math.sin(h)]), np.array([math.sin(h), -math.cos(h)])
    c, s = np.array([cx, cy]), side / 2
    return np.array([c + s * f - s * r, c + s * f + s * r, c - s * f + s * r, c - s * f - s * r])


def floor_detections():
    return [to_px(square(x, y, 90)) for x, y in FLOOR.values()], list(FLOOR)


def test_pose_from_corners_recovers_position_and_heading():
    H = np.linalg.inv(CAM)
    for x, y, hd in [(1.5, 2.0, 30.0), (3.9, 0.4, -120.0), (0.2, 3.7, 179.0)]:
        p = pose_from_corners(H, to_px(square(x, y, hd)))
        assert abs(p[0] - x) < 1e-6 and abs(p[1] - y) < 1e-6, (p, x, y)
        assert abs((p[2] - hd + 180) % 360 - 180) < 1e-6, (p, hd)


def test_tracker_needs_the_floor_markers_once_then_keeps_the_mapping():
    tr = FloorTracker(FLOOR, ROBOTS)
    robot = to_px(square(2.0, 1.0, 45))
    assert tr.update([robot], [10], t=0.0) == {}, "no floor mapping yet"
    corners, ids = floor_detections()
    poses = tr.update(corners + [robot], ids + [10], t=1.0)
    assert abs(poses["A"].x - 2.0) < 1e-4 and abs(poses["A"].heading - 45) < 1e-4
    # People or robots may hide floor markers later; the camera hasn't moved.
    poses = tr.update([robot, corners[0]], [10, 0], t=2.0)
    assert abs(poses["A"].y - 1.0) < 1e-4 and poses["A"].t == 2.0


def test_unknown_marker_ids_are_ignored():
    tr = FloorTracker(FLOOR, ROBOTS)
    corners, ids = floor_detections()
    poses = tr.update(corners + [to_px(square(1, 1, 0))], ids + [33], t=0.0)
    assert poses == {}


def test_real_detection_on_a_rendered_tilted_view():
    """Draw the markers on a floor image, view it through a tilted camera, detect: right positions and headings."""
    ppm = 200                                    # floor image pixels per metre
    floor = np.full((int(4.6 * ppm), int(5.1 * ppm)), 255, np.uint8)
    to_img = lambda x, y: (int((x + 0.3) * ppm), int((4.3 - y) * ppm))   # floor image is y-down
    side = 0.3

    def stamp(mid, x, y, heading):
        m = cv2.aruco.generateImageMarker(DICT, mid, int(side * ppm))
        m = cv2.copyMakeBorder(m, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)   # room to rotate
        n = m.shape[0]
        # Rotate the marker so its top points along heading (counter-clockwise, y up).
        R = cv2.getRotationMatrix2D((n / 2, n / 2), heading - 90, 1.0)
        m = cv2.warpAffine(m, R, (n, n), borderValue=255)
        cx, cy = to_img(x, y)
        floor[cy - n // 2:cy - n // 2 + n, cx - n // 2:cx - n // 2 + n] = np.minimum(
            floor[cy - n // 2:cy - n // 2 + n, cx - n // 2:cx - n // 2 + n], m)

    for mid, (x, y) in FLOOR.items():
        stamp(mid, x, y, 90)
    stamp(10, 1.4, 2.6, 30)
    stamp(11, 3.2, 1.1, -100)
    # Tilted camera view of the floor image.
    src = np.float32([to_img(-0.3, -0.3), to_img(4.8, -0.3), to_img(4.8, 4.3), to_img(-0.3, 4.3)])
    dst = np.float32([[180, 700], [1100, 690], [950, 120], [320, 140]])
    view = cv2.warpPerspective(floor, cv2.getPerspectiveTransform(src, dst), (1280, 760), borderValue=255)
    tr = FloorTracker(FLOOR, ROBOTS)
    poses = tr.process(view, t=5.0)
    assert set(poses) == {"A", "B"}, poses
    for rid, (x, y, hd) in {"A": (1.4, 2.6, 30), "B": (3.2, 1.1, -100)}.items():
        p = poses[rid]
        assert math.hypot(p.x - x, p.y - y) < 0.03, (rid, p)
        assert abs((p.heading - hd + 180) % 360 - 180) < 3, (rid, p)


def test_nearest_wall_is_the_distance_to_the_closest_wall_segment():
    walls = [[[0.5, 2.0], [1.8, 2.0]], [[3.0, 0.0], [3.0, 1.0]]]
    assert abs(nearest_wall_m(1.0, 1.91, walls) - 0.09) < 1e-9      # beside the first wall
    assert abs(nearest_wall_m(2.0, 2.0, walls) - 0.2) < 1e-9        # past its end: to the end point
    assert abs(nearest_wall_m(3.1, 0.5, walls) - 0.1) < 1e-9
    assert nearest_wall_m(1.0, 1.0, []) is None


def test_draw_shows_the_walls_and_each_robots_distance_to_them():
    class Ft:                                   # floor metres = pixels / 100, y up
        H = np.array([[0.01, 0, 0], [0, -0.01, 4.0], [0, 0, 1]])
        def annotate(self, frame, det):
            pass
    frame = np.zeros((400, 450, 3), np.uint8)
    walls = [[[0.5, 2.0], [1.8, 2.0]]]
    draw(frame, Ft(), None, {"A": Pose(1.0, 1.5, 90.0, 0.0)}, walls=walls)
    assert (frame[200, 150] == (40, 40, 220)).all(), "the wall is drawn in red at y = 2 m"
    blank = np.zeros_like(frame)
    draw(blank, Ft(), None, {"A": Pose(1.0, 1.5, 90.0, 0.0)})
    assert (frame != 0).any(axis=2).sum() > (blank != 0).any(axis=2).sum() + 300, "wall line plus distance label"


if __name__ == "__main__":
    test_pose_from_corners_recovers_position_and_heading()
    test_tracker_needs_the_floor_markers_once_then_keeps_the_mapping()
    test_unknown_marker_ids_are_ignored()
    test_real_detection_on_a_rendered_tilted_view()
    test_nearest_wall_is_the_distance_to_the_closest_wall_segment()
    test_draw_shows_the_walls_and_each_robots_distance_to_them()
    print("ok")
