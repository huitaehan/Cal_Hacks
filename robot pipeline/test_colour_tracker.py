"""Colour tracking (SSL-Vision style): blue centre blob + pink/green dots -> robot id, x, y, heading."""
import math
import time

import cv2
import numpy as np

from tracker import CameraTracker, Pose

from colour_tracker import (DEFAULT_COLOURS, ColourTracker, assemble, heading_from_dots, range_from_sample,
                            range_from_samples)

PATTERNS = {"A": {"pink": 4}, "B": {"pink": 2, "green": 2}}
# SSL standard pattern, metres, robot frame (x forward): front dots wider apart than the back ones.
SSL = [(0.035, 0.0548), (-0.0548, 0.035), (-0.0548, -0.035), (0.035, -0.0548)]   # FL, BL, BR, FR


def place(x, y, heading_deg, colours):
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    return [(x + c * dx - s * dy, y + s * dx + c * dy, col) for (dx, dy), col in zip(SSL, colours)]


def ang_err(a, b):
    return abs((a - b + 180) % 360 - 180)


def test_heading_is_the_widest_gap_between_dots():
    for hd in (0, 37, 170, -95):
        dots = place(1.0, 2.0, hd, ["pink"] * 4)
        assert ang_err(heading_from_dots((1.0, 2.0), dots), hd) < 0.5, hd


def test_evenly_spaced_dots_use_colours_or_give_up():
    ring = lambda hd, cols: [(math.cos(math.radians(hd + a)) * 0.065, math.sin(math.radians(hd + a)) * 0.065, c)
                             for a, c in zip((45, 135, 225, 315), cols)]
    # Pink pair adjacent, green pair adjacent: front = toward the pink pair.
    assert ang_err(heading_from_dots((0, 0), ring(30, ["pink", "green", "green", "pink"]), "even"), 30) < 0.5
    # All one colour and evenly spaced: no way to tell the front.
    assert heading_from_dots((0, 0), ring(30, ["pink"] * 4), "even") is None
    # Read as SSL, an evenly spaced pattern is a tie: no heading rather than a guess.
    assert heading_from_dots((0, 0), ring(30, ["pink", "green", "green", "pink"]), "ssl") is None


def test_assemble_identifies_robots_by_dot_colours_and_ignores_strays():
    centres = [(1.0, 1.0), (3.0, 2.5), (2.0, 3.5)]                 # the third is a stray blue blob
    dots = place(1.0, 1.0, 20, ["pink"] * 4) + place(3.0, 2.5, -60, ["pink", "green", "green", "pink"])
    dots.append((2.6, 0.4, "pink"))                                # stray pink far from any centre
    robots = assemble(centres, dots, PATTERNS, radius=0.12)
    assert set(robots) == {"A", "B"}, robots
    assert math.dist(robots["A"][:2], (1.0, 1.0)) < 1e-9 and ang_err(robots["A"][2], 20) < 0.5
    assert ang_err(robots["B"][2], -60) < 0.5


def test_a_hidden_dot_still_gives_position_and_identity_but_no_heading():
    dots = place(1.0, 1.0, 20, ["pink", "green", "green", "pink"])[:3]   # one pink hidden by the sensing stick
    robots = assemble([(1.0, 1.0)], dots, PATTERNS, radius=0.12)
    assert robots["B"][:2] == (1.0, 1.0) and robots["B"][2] is None, robots


def test_tracker_keeps_the_last_heading_briefly_when_a_dot_is_hidden():
    tr = ColourTracker(PATTERNS, H=np.eye(3))
    tr.update([(1.0, 1.0)], place(1.0, 1.0, 45, ["pink"] * 4), t=0.0)
    poses = tr.update([(1.0, 1.0)], place(1.0, 1.0, 45, ["pink"] * 4)[:3], t=0.3)
    assert ang_err(poses["A"].heading, 45) < 0.5 and poses["A"].t == 0.3


def test_an_old_heading_is_not_reused():
    """Review finding: the robots drift in rotation, so a heading from seconds ago steers them wrong."""
    tr = ColourTracker(PATTERNS, H=np.eye(3))
    tr.update([(1.0, 1.0)], place(1.0, 1.0, 45, ["pink"] * 4), t=0.0)
    poses = tr.update([(1.0, 1.0)], place(1.0, 1.0, 45, ["pink"] * 4)[:3], t=1.0)
    assert poses["A"].heading is None, poses


def test_heading_survives_pixel_noise_on_the_ssl_layout():
    """Review finding: the widest-gap rule failed 19-36 % of frames with a few mm of noise. Fit the whole
    SSL dot layout instead: almost always an answer, and never a badly wrong one."""
    rng = np.random.default_rng(1)
    for cols in (["pink"] * 4, ["green", "pink", "pink", "green"]):      # B with its pink pair at the BACK
        errs, none = [], 0
        for _ in range(2000):
            hd = rng.uniform(-180, 180)
            dots = [(x + rng.normal(0, 0.004), y + rng.normal(0, 0.004), c) for x, y, c in place(2.0, 2.0, hd, cols)]
            centre = (2.0 + rng.normal(0, 0.006), 2.0 + rng.normal(0, 0.006))
            got = heading_from_dots(centre, dots)
            if got is None:
                none += 1
            else:
                errs.append(ang_err(got, hd))
        errs = np.array(errs)
        assert none <= 40, (cols, none)       # at most 2 % of frames without a heading (the last one is reused)
        assert (errs < 10).mean() >= 0.99 and errs.max() < 30, (cols, np.percentile(errs, 99), errs.max())


def test_a_second_matching_pattern_doesnt_teleport_the_robot():
    tr = ColourTracker(PATTERNS, H=np.eye(3))
    tr.update([(1.0, 1.0)], place(1.0, 1.0, 0, ["pink"] * 4), t=0.0)
    stray = place(3.5, 3.5, 90, ["pink"] * 4)                    # e.g. a printed copy lying around
    poses = tr.update([(3.5, 3.5), (1.02, 1.0)], stray + place(1.02, 1.0, 0, ["pink"] * 4), t=0.1)
    assert abs(poses["A"].x - 1.02) < 1e-9, poses


def test_a_blue_speck_beside_the_robot_doesnt_become_its_centre():
    """Live frame: specks of the room's blue sat right next to the robot. The real centre sits in the
    middle of the 4 dots; a speck beside them doesn't."""
    dots = place(1.0, 1.0, 30, ["pink", "green", "green", "pink"])
    robots = assemble([(1.04, 1.0), (1.0, 1.0)], dots, PATTERNS, radius=0.12)
    assert robots["B"][:2] == (1.0, 1.0), robots


def test_specks_in_clutter_dont_make_a_phantom_robot():
    """Live frame: a blue speck with 4 pink specks around it in a pile of clothes became 'robot A'.
    Real patterns have the SSL geometry; clutter doesn't."""
    junk = [(1.03, 1.02, "pink"), (0.92, 1.05, "pink"), (1.06, 0.93, "pink"), (0.98, 0.90, "pink")]
    assert assemble([(1.0, 1.0)], junk, PATTERNS, radius=0.12) == {}
    junk3 = [(1.01, 1.0, "pink"), (0.99, 1.01, "pink"), (1.0, 0.99, "pink")]       # 3 specks hugging the centre
    assert assemble([(1.0, 1.0)], junk3, PATTERNS, radius=0.12) == {}
    real = place(1.0, 1.0, 30, ["pink"] * 4)
    assert set(assemble([(1.0, 1.0)], real, PATTERNS, radius=0.12)) == {"A"}


class _FakeCap:
    def __init__(self, frame):
        self.frame = frame

    def read(self):
        time.sleep(0.02)
        return True, self.frame.copy()

    def release(self):
        pass


def test_camera_thread_tracks_in_colour_mode():
    """Review finding: the camera thread called a method ColourTracker didn't have, and died silently."""
    view, Hcam, truth = render_two_robots()
    ct = CameraTracker({"camera": {}}, cap=_FakeCap(view), ft=ColourTracker(PATTERNS, H=Hcam))
    try:
        end = time.time() + 3
        while time.time() < end and set(ct.poses()) != {"A", "B"}:
            time.sleep(0.05)
        assert set(ct.poses()) == {"A", "B"}, (ct.poses(), ct.errors)
    finally:
        ct.close()


def test_camera_thread_survives_a_tracking_error():
    class Flaky:
        H, calls = np.eye(3), 0

        def detect(self, frame):
            Flaky.calls += 1
            if Flaky.calls == 1:
                raise ValueError("boom")
            return []

        def poses_from(self, det, t):
            return {"A": Pose(1.0, 1.0, 0.0, t)}

    ct = CameraTracker({"camera": {}}, cap=_FakeCap(np.zeros((10, 10, 3), np.uint8)), ft=Flaky())
    try:
        end = time.time() + 3
        while time.time() < end and "A" not in ct.poses():
            time.sleep(0.05)
        assert "A" in ct.poses() and ct.errors and "boom" in ct.errors[0], ct.errors
    finally:
        ct.close()


def test_range_from_a_clicked_sample():
    lo, hi = range_from_sample((165, 200, 220))
    assert lo[0] == 153 and hi[0] == 177 and lo[1] < 200 < hi[1] and lo[2] < 220
    lo, hi = range_from_sample((3, 200, 220))            # red-pink wraps around hue 0
    assert lo[0] == 171 and hi[0] == 15


def test_several_clicks_widen_the_range_to_cover_all_of_them():
    in_hue = lambda lo, hi, h: lo[0] <= h <= hi[0] if lo[0] <= hi[0] else (h >= lo[0] or h <= hi[0])
    lo, hi = range_from_samples([(160, 200, 220), (170, 150, 120)])
    assert all(in_hue(lo, hi, h) for h in (150, 160, 170, 179)) and not in_hue(lo, hi, 140), (lo, hi)
    assert lo[1] <= 90 and lo[2] <= 40, (lo, hi)
    lo, hi = range_from_samples([(176, 200, 200), (4, 200, 200)])       # either side of red: wraps
    assert lo[0] > hi[0] and lo[0] <= 166 and hi[0] >= 14, (lo, hi)


def render_two_robots():
    """A tilted camera view of robot A (4 pink) and B (2 pink + 2 green); returns (view, H, truth)."""
    ppm = 400                                               # floor image pixels per metre
    W, Hh = int(4.5 * ppm), int(4.0 * ppm)
    floor = np.full((Hh, W, 3), (120, 125, 130), np.uint8)
    to_img = lambda x, y: (int(round(x * ppm)), int(round((4.0 - y) * ppm)))
    bgr = {"blue": (200, 90, 30), "pink": (180, 70, 255), "green": (60, 190, 50)}
    truth = {"A": (1.2, 2.8, 30.0, ["pink"] * 4), "B": (3.3, 1.0, -120.0, ["pink", "green", "green", "pink"])}
    for x, y, hd, cols in truth.values():
        cv2.circle(floor, to_img(x, y), int(0.025 * ppm), bgr["blue"], -1)
        for dx, dy, col in place(x, y, hd, cols):
            cv2.circle(floor, to_img(dx, dy), int(0.02 * ppm), bgr[col], -1)
    src = np.float32([to_img(0, 0), to_img(4.5, 0), to_img(4.5, 4.0), to_img(0, 4.0)])
    dst = np.float32([[150, 1000], [1750, 990], [1450, 200], [450, 220]])
    P = cv2.getPerspectiveTransform(src, dst)
    view = cv2.warpPerspective(floor, P, (1920, 1080), borderValue=(120, 125, 130))
    # Camera pixels -> floor metres, as floor_setup.py would compute from 4 clicked corners.
    Hcam = cv2.getPerspectiveTransform(dst, np.float32([[0, 0], [4.5, 0], [4.5, 4.0], [0, 4.0]]))
    return view, Hcam, truth


def test_rendered_tilted_view_of_two_ssl_robots():
    view, Hcam, truth = render_two_robots()
    tr = ColourTracker(PATTERNS, H=Hcam, colours=DEFAULT_COLOURS)
    poses = tr.process(view, t=3.0)
    assert set(poses) == {"A", "B"}, poses
    for rid, (x, y, hd, _) in truth.items():
        p = poses[rid]
        assert math.hypot(p.x - x, p.y - y) < 0.02, (rid, p)
        assert ang_err(p.heading, hd) < 5, (rid, p)


if __name__ == "__main__":
    test_heading_is_the_widest_gap_between_dots()
    test_evenly_spaced_dots_use_colours_or_give_up()
    test_assemble_identifies_robots_by_dot_colours_and_ignores_strays()
    test_a_hidden_dot_still_gives_position_and_identity_but_no_heading()
    test_tracker_keeps_the_last_heading_briefly_when_a_dot_is_hidden()
    test_an_old_heading_is_not_reused()
    test_heading_survives_pixel_noise_on_the_ssl_layout()
    test_a_second_matching_pattern_doesnt_teleport_the_robot()
    test_a_blue_speck_beside_the_robot_doesnt_become_its_centre()
    test_specks_in_clutter_dont_make_a_phantom_robot()
    test_camera_thread_tracks_in_colour_mode()
    test_camera_thread_survives_a_tracking_error()
    test_range_from_a_clicked_sample()
    test_several_clicks_widen_the_range_to_cover_all_of_them()
    test_rendered_tilted_view_of_two_ssl_robots()
    print("ok")
