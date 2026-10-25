"""Camera setup over the taped zone, all in one window: aim, colours, corners, check. Saved as you go.

    python setup_camera.py [--camera 1]

First it asks the taped zone's size and the robots' height, in cm. Front = the side you and the laptop are
on: (0, 0) is the front-left corner, x runs to your right along the front edge as you face the zone, y away
from you. Then 4 steps (Enter: next step, 1-4: jump to a step, h: hide/show the text, z: re-enter the
sizes, q: quit):
  1 AIM      tilt the camera until the whole tape is in view, with about 15 cm spare all round. A zone saved
             earlier is outlined: if it no longer sits on the tape, the camera has moved, so redo step 3.
  2 COLOURS  press b, p or g (blue centre, pink dot, green dot), then click that colour on the robots, a few
             times each, near and far. Each robot should show 1 blue and 4 dot circles, and nothing else
             should light up. m: show the mask, r: forget this colour's clicks. Enter saves colours.json.
  3 CORNERS  click the 4 taped corners, in order front-left, front-right, back-right, back-left (a magnifier
             follows the mouse). u: undo. The tape's shape in the image (tilted camera) and the robots' dots
             (any robot in view) check the measured depth. Enter saves floor.json and the zone's size in
             config.json.
  4 CHECK    the zone's border on the tape, a 0.5 m grid, the robots' parking spots and each robot's position.
             Stand a robot centred on a taped corner: it should read that corner within about 2 cm. Move each
             robot along the edge farthest from the camera, where the dots look smallest: "read" should stay
             near 100 %.
The colours sit on the robots' tops, above the floor, and the camera sees a top in line with a floor point
farther out. So the tape mapping is lifted to robot-top height, using the camera position it implies.
"""
import argparse
import collections
import json
import math
import re
import time

import cv2
import numpy as np

from colour_tracker import DEFAULT_COLOURS, ColourTracker, colour_mask, range_from_samples
from tracker import COLOURS, CONFIG, FLOOR, apply_h, draw, open_camera
from wifi_area import update_floorplan

WINDOW = "camera setup"
STEPS = ["AIM", "COLOURS", "CORNERS", "CHECK"]
CORNERS = ["front-left", "front-right", "back-right", "back-left"]
COLOUR_KEYS = {ord("b"): "blue", ord("p"): "pink", ord("g"): "green"}
ENTER = (10, 13)
SPARE_M = 0.25          # parking spots are kept this far inside the zone
NOMINAL_F_PX = 1360.0   # C922 focal length in px at 1920 wide, from its 78 deg diagonal view ("focal_px" overrides)
MIN_TILT_DEG = 10       # below this the tape alone can't check the depth (it looks the same at any focal length)
DEPTH_WARN = 0.08       # warn when the robots' dots say the depth is this much (share) off
TAPE_DEPTH_WARN = 0.04  # ... or the tape's shape does (more precise, but only with a tilted camera)
FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE, RED, GREEN, YELLOW = (255, 255, 255), (60, 60, 255), (80, 230, 80), (0, 230, 255)


def config_with_zone(text: str, w: float, d: float, margin: float = SPARE_M) -> str:
    """config.json's text with "area" set to the taped zone, and any parking spot ("sentry") closer than
    `margin` to its edge or outside it moved inside. Keeps the file's layout."""
    cfg = json.loads(text)
    area = [[0.0, 0.0], [round(w, 3), round(d, 3)]]
    out = re.sub(r'"area":\s*\[\[[^\]]*\],\s*\[[^\]]*\]\]', '"area": ' + json.dumps(area), text, count=1)
    cfg["area"] = area
    m = min(margin, w / 2, d / 2)
    for name, r in cfg["robots"].items():
        x, y = r["sentry"]
        spot = [round(min(max(x, m), w - m), 3), round(min(max(y, m), d - m), 3)]
        if spot != [x, y]:
            r["sentry"] = spot
            out = re.sub(r'("' + re.escape(name) + r'":\s*\{[^\n]*?"sentry":\s*)\[[^\]]*\]',
                         lambda mo: mo.group(1) + json.dumps(spot), out, count=1)
    if json.loads(out) != cfg:              # the layout wasn't what the patterns expect: write it plainly
        out = json.dumps(cfg, indent=2) + "\n"
    return out


def corner_problem(px) -> str | None:
    """None if the 4 clicks (front-left, front-right, back-right, back-left) go round the zone the right way;
    else what looks wrong. Seen from any camera above, they turn clockwise in the image (y down)."""
    if len(px) != 4:
        return "need all 4 corners"
    turns = []
    for i in range(4):
        a, b, c = px[i], px[(i + 1) % 4], px[(i + 2) % 4]
        turns.append((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
    if all(t < 0 for t in turns):
        return None
    if all(t > 0 for t in turns):
        return ("the corners look mirrored: are front-left and front-right swapped? Facing the zone from the "
                "front, x runs to your right")
    return "the clicks don't go round the zone in order (front-left, front-right, back-right, back-left)"


def focal_from_tape(H_floor, image_size) -> float | None:
    """The camera's focal length (px) from how it sees the tape rectangle: its sides are square to each other
    and the mapping is a rotation seen through the lens. Needs a tilted view; looking straight down it is
    unmeasurable (None or absurd values)."""
    w, h = image_size
    f0 = NOMINAL_F_PX * w / 1920
    T = np.array([[1 / f0, 0, -w / 2 / f0], [0, 1 / f0, -h / 2 / f0], [0, 0, 1]])
    G = T @ np.linalg.inv(np.asarray(H_floor, float))        # floor -> camera rays, in units of f0
    g1, g2 = G[:, 0], G[:, 1]
    a = np.array([g1[0] * g2[0] + g1[1] * g2[1], g1[0] ** 2 + g1[1] ** 2 - g2[0] ** 2 - g2[1] ** 2])
    b = np.array([g1[2] * g2[2], g1[2] ** 2 - g2[2] ** 2])
    s = -(a @ b) / (a @ a)                                    # (f0 / f) squared, by least squares
    return f0 / math.sqrt(s) if s > 0 else None


def camera_pose(H_floor, image_size, f_px: float | None = None):
    """(K, R, t): the camera's lens and its pose over the floor (floor metres -> camera), from the tape
    mapping and the focal length (default: the C922's nominal one)."""
    w, h = image_size
    f = f_px or NOMINAL_F_PX * w / 1920
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
    M = np.linalg.inv(K) @ np.linalg.inv(np.asarray(H_floor, float))     # floor -> camera, up to scale
    M = M * 2 / (np.linalg.norm(M[:, 0]) + np.linalg.norm(M[:, 1]))
    if M[2, 2] < 0:                                                        # the floor is in front of the camera
        M = -M
    U, _, Vt = np.linalg.svd(np.c_[M[:, 0], M[:, 1], np.cross(M[:, 0], M[:, 1])])
    return K, U @ Vt, M[:, 2]


def robot_top_mapping(H_floor, top_m: float, image_size, f_px: float | None = None):
    """The tape mapping (image -> floor) lifted to the robots' tops, top_m above the floor. A focal length
    3 % off the real one moves positions by under 1 cm. Returns (image -> robot-top mapping, camera
    (x, y, height) in metres)."""
    K, R, t = camera_pose(H_floor, image_size, f_px)
    H = np.linalg.inv(K @ np.c_[R[:, 0], R[:, 1], R[:, 2] * top_m + t])
    return H / H[2, 2], -R.T @ t


def depth_from_tape(clicks, w: float, image_size, f_px: float | None = None) -> float | None:
    """The zone depth that makes the tape's outline in the image fit the camera's focal length, given the
    width; None when the camera looks too straight down to tell. With a tilted camera this is precise
    (a 3 % wrong focal length moves it by well under 1 %), so it checks the measured depth."""
    f0 = f_px or NOMINAL_F_PX * image_size[0] / 1920

    def mismatch(d):
        H = cv2.getPerspectiveTransform(np.float32(clicks), np.float32([(0, 0), (w, 0), (w, d), (0, d)]))
        f = focal_from_tape(H, image_size)
        return math.inf if f is None else abs(f - f0)
    coarse = min(np.arange(0.3, 3.0, 0.005) * w, key=mismatch)
    d = float(min(np.arange(coarse - 0.005 * w, coarse + 0.005 * w, 0.0002 * w), key=mismatch))
    if mismatch(d) > 0.05 * f0:
        return None
    H = cv2.getPerspectiveTransform(np.float32(clicks), np.float32([(0, 0), (w, 0), (w, d), (0, d)]))
    _, R, _ = camera_pose(H, image_size, f0)
    tilt = math.degrees(math.acos(min(1.0, -R[2, 2])))       # optical axis vs straight down
    return d if tilt >= MIN_TILT_DEG else None


def estimate_depth(clicks, w: float, blobs) -> float | None:
    """The zone depth at which the robots' 4 dots sit at equal distances from their blue centres, as they do
    on the robots (so a mis-measured depth shows as squashed patterns); None without a robot showing all 4
    dots. Only the depth-to-width ratio can be checked this way."""
    dots = [(b[1], b[2]) for b in blobs if b[0] != "blue"]
    robots = []
    for col, cx, cy, (_, _, bw, bh) in blobs:
        near = sorted(dots, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)[:4]
        if col == "blue" and len(near) == 4 and all(math.dist(p, (cx, cy)) <= 3 * max(bw, bh) for p in near):
            robots.append(((cx, cy), near))
    if not robots:
        return None
    best = None
    for d in np.arange(0.3, 3.0, 0.005) * w:
        H = cv2.getPerspectiveTransform(np.float32(clicks), np.float32([(0, 0), (w, 0), (w, d), (0, d)]))
        spread = 0.0
        for c, near in robots:
            r = np.hypot(*(apply_h(H, near) - apply_h(H, [c])[0]).T)
            spread += r.std() / r.mean()
        if best is None or spread < best[0]:
            best = (spread, float(d))
    return best[1]


class ReadRate:
    """Share of the last few seconds' frames in which each robot was found, and in which its heading was read
    from its dots (not carried over from an earlier frame)."""

    def __init__(self, window_s: float = 3.0):
        self.window_s = window_s
        self.frames = collections.deque()       # (t, {robot: heading read this frame})

    def add(self, t: float, found: dict):
        self.frames.append((t, dict(found)))
        while self.frames and self.frames[0][0] < t - self.window_s:
            self.frames.popleft()

    def rates(self, robot) -> tuple[float, float]:
        n = len(self.frames)
        if not n:
            return 0.0, 0.0
        return (sum(robot in f for _, f in self.frames) / n,
                sum(f.get(robot) is True for _, f in self.frames) / n)


def ask_cm(prompt: str, default_m: float | None, lo_cm: float, hi_cm: float) -> float:
    while True:
        s = input(f"{prompt}" + (f" [{default_m * 100:.1f}]" if default_m else "") + ": ").strip()
        if not s and default_m:
            return default_m
        try:
            v = float(s)
        except ValueError:
            print("  a number of centimetres, please")
            continue
        if lo_cm <= v <= hi_cm:
            return v / 100
        print(f"  between {lo_cm:.0f} and {hi_cm:.0f} cm, please")


def ask_sizes(w0, d0, top0):
    print("Front = the side you and the laptop are on. (0, 0) is the front-left corner of the tape.")
    print("Measure in centimetres (not inches).")
    w = ask_cm("Taped zone width, along the front edge, in cm", w0, 30, 2000)
    d = ask_cm("Taped zone depth, front to back, in cm", d0, 30, 2000)
    top = ask_cm("Robot height, floor to the coloured dots on top, in cm", top0, 1, 60)
    print(f"Zone {w * 100:.1f} x {d * 100:.1f} cm (if the tape is square, both diagonals are "
          f"{math.hypot(w, d) * 100:.0f} cm); robot tops {top * 100:.1f} cm up.")
    return w, d, top


def banner(frame, lines):
    """Text lines on a darkened strip across the top; each line is a string or (string, colour)."""
    s = frame.shape[1] / 1280                   # sized for a window about 1280 px wide
    step = int(32 * s)
    strip = frame[:step * len(lines) + int(14 * s)]
    strip[:] = (strip * 0.45).astype(np.uint8)
    for i, line in enumerate(lines):
        text, col = (line, WHITE) if isinstance(line, str) else line
        (tw, _), _ = cv2.getTextSize(text, FONT, 1.0, 2)
        scale = min(0.7 * s, (frame.shape[1] - 24 * s) / tw)     # long lines shrink to fit
        cv2.putText(frame, text, (int(12 * s), (i + 1) * step), FONT, scale, col, max(1, round(2 * s)))


def magnifier(frame, src, at, zoom=5, half=24):
    """A zoomed view of `src` around `at`, with a cross on the exact pixel, in the bottom corner away from it."""
    x, y = int(at[0]), int(at[1])
    pad = cv2.copyMakeBorder(src, half, half, half, half, cv2.BORDER_CONSTANT)
    crop = cv2.resize(pad[y:y + 2 * half + 1, x:x + 2 * half + 1], None, fx=zoom, fy=zoom,
                      interpolation=cv2.INTER_NEAREST)
    c = half * zoom + zoom // 2
    cv2.line(crop, (c, 0), (c, crop.shape[0]), RED, 1)
    cv2.line(crop, (0, c), (crop.shape[1], c), RED, 1)
    H, W = frame.shape[:2]
    x0 = 10 if x > W / 2 else W - crop.shape[1] - 10
    frame[H - crop.shape[0] - 10:H - 10, x0:x0 + crop.shape[1]] = crop


def grid(frame, px, w, d):
    for gx in np.arange(0.5, w - 1e-6, 0.5):
        cv2.line(frame, *map(tuple, px([(gx, 0), (gx, d)])), (170, 170, 170), 1)
    for gy in np.arange(0.5, d - 1e-6, 0.5):
        cv2.line(frame, *map(tuple, px([(0, gy), (w, gy)])), (170, 170, 170), 1)
    o, ex, ey = px([(0, 0), (0.4, 0), (0, 0.4)])
    cv2.arrowedLine(frame, tuple(o), tuple(ex), RED, 3, tipLength=0.25)
    cv2.arrowedLine(frame, tuple(o), tuple(ey), RED, 3, tipLength=0.25)
    cv2.putText(frame, "x", tuple(ex + [8, 8]), FONT, 1.0, RED, 2)
    cv2.putText(frame, "y", tuple(ey + [8, -8]), FONT, 1.0, RED, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    if args.camera is not None:
        cfg["camera"]["index"] = args.camera
    saved = json.loads(FLOOR.read_text()) if FLOOR.exists() else None
    (_, _), (w0, d0) = cfg["area"]
    w, d, top = ask_sizes(w0, d0, saved.get("robot_top_m") if saved else None)
    ranges = json.loads(COLOURS.read_text()) if COLOURS.exists() else {k: list(v) for k, v in DEFAULT_COLOURS.items()}
    patterns = {n: r["pattern"] for n, r in cfg["robots"].items()}
    focal = cfg["camera"].get("focal_px")                       # default: the C922's nominal focal length
    finder = ColourTracker(patterns, H=None, colours=ranges)      # finds blobs while tuning (shares `ranges`)
    st = {"step": 0, "colour": "blue", "mask": False, "hsv": None, "flash": ("", 0.0), "text": True,
          "mouse": None, "depth": (None, None, 0.0)}
    samples = {c: [] for c in ranges}
    clicks: list = []
    checker, rates = None, ReadRate()

    def flash(text):
        print(text)
        st["flash"] = (text, time.time() + 4)

    def start_check():
        nonlocal checker, rates, cfg
        cfg = json.loads(CONFIG.read_text())
        fl = json.loads(FLOOR.read_text()) if FLOOR.exists() else None
        checker = None
        if fl is not None:
            checker = ColourTracker(patterns, fl["H"], ranges, cfg.get("pattern_radius_m", 0.065),
                                    cfg.get("pattern_layout", "ssl"))
            checker.H_floor = np.asarray(fl.get("H_floor", fl["H"]))     # the border is drawn on the tape
        rates = ReadRate()

    def go(n):
        if STEPS[st["step"]] == "COLOURS" and n != st["step"]:
            COLOURS.write_text(json.dumps(ranges, indent=2))     # however you leave the step
            flash(f"saved {COLOURS.name}")
        st["step"] = n
        if STEPS[n] == "CHECK":
            start_check()

    def on_mouse(ev, x, y, *_):
        st["mouse"] = (x, y)
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        if STEPS[st["step"]] == "COLOURS" and st["hsv"] is not None:
            patch = st["hsv"][max(0, y - 2):y + 3, max(0, x - 2):x + 3].reshape(-1, 3)
            col = st["colour"]
            samples[col].append(tuple(int(v) for v in np.median(patch, axis=0)))
            ranges[col] = range_from_samples(samples[col])
            print(f"{col}: sample {samples[col][-1]} -> range {ranges[col]}")
        elif STEPS[st["step"]] == "CORNERS" and len(clicks) < 4:
            clicks.append((float(x), float(y)))
            print(f"{CORNERS[len(clicks) - 1]} tape corner at pixel ({x}, {y})")

    cap = open_camera(cfg["camera"])
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720)
    cv2.setMouseCallback(WINDOW, on_mouse)
    n_frames, t0, fps = 0, time.time(), 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            now = time.time()
            n_frames += 1
            if now - t0 >= 1.0:
                fps, n_frames, t0 = n_frames / (now - t0), 0, now
            step = STEPS[st["step"]]
            head = f"STEP {st['step'] + 1}/4 {step}"
            floor = [(0.0, 0.0), (w, 0.0), (w, d), (0.0, d)]
            lines = []

            if step == "AIM":
                fh, fw = frame.shape[:2]
                cv2.drawMarker(frame, (fw // 2, fh // 2), YELLOW, cv2.MARKER_CROSS, 60, 2)
                if saved:
                    Hi = np.linalg.inv(saved.get("H_floor", saved["H"]))
                    cv2.polylines(frame, [apply_h(Hi, saved["floor"]).astype(int)], True, YELLOW, 3)
                lines = [(f"{head}: tilt the camera until all the tape is in view, with ~15 cm spare all round", YELLOW),
                         f"zone {w * 100:.1f} x {d * 100:.1f} cm: both tape diagonals should be "
                         f"{math.hypot(w, d) * 100:.0f} cm.   camera {fw}x{fh}, {fps:.0f} fps",
                         ("yellow outline = the zone saved earlier: if it's off the tape, redo step 3"
                          if saved else "no zone saved yet"),
                         "Enter: next   1-4: jump to a step   h: hide this text   z: re-enter sizes   q: quit"]

            elif step == "COLOURS":
                st["hsv"] = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
                blobs = finder.detect(frame)
                if st["mask"]:
                    frame = cv2.cvtColor(colour_mask(st["hsv"], ranges[st["colour"]]), cv2.COLOR_GRAY2BGR)
                else:
                    finder.annotate(frame, blobs)
                counts = ", ".join(f"{c} {sum(1 for b in blobs if b[0] == c)}" for c in ranges)
                lines = [(f"{head}: clicking {st['colour'].upper()} ({len(samples[st['colour']])} clicks so far)", YELLOW),
                         "b blue centre   p pink dot   g green dot: then click that colour on the robots, near and far",
                         f"circles found: {counts}   (each robot: 1 blue + 4 dots; nothing else should light up)",
                         "m mask on/off   r forget this colour's clicks   Enter: save colours, next   q: quit"]

            elif step == "CORNERS":
                raw = frame.copy()
                for (cx, cy), name in zip(clicks, CORNERS):
                    cv2.drawMarker(frame, (int(cx), int(cy)), RED, cv2.MARKER_CROSS, 30, 2)
                    cv2.putText(frame, name, (int(cx) + 14, int(cy) - 14), FONT, 1.0, RED, 2)
                if len(clicks) == 4:
                    problem = corner_problem(clicks)
                    H = cv2.getPerspectiveTransform(np.float32(clicks), np.float32(floor))
                    px = lambda pts: apply_h(np.linalg.inv(H), pts).astype(int)
                    cv2.polylines(frame, [px(floor)], True, GREEN, 3)
                    if problem is None:
                        grid(frame, px, w, d)
                        if now - st["depth"][2] > 1.0:          # two checks on the measured depth
                            st["depth"] = (depth_from_tape(clicks, w, frame.shape[1::-1], focal),
                                           estimate_depth(clicks, w, finder.detect(raw)), now)
                    by_tape, by_dots, _ = st["depth"]
                    if problem:
                        lines = [(f"{head}: these corners look wrong", RED), (problem, RED), "u undo   q: quit"]
                    else:
                        lines = [(f"{head}: the green outline and 0.5 m grid should sit on the tape", YELLOW)]
                        if by_tape is not None and abs(by_tape - d) > TAPE_DEPTH_WARN * d:
                            warn = (f"the tape's shape in the image says the depth is about {by_tape * 100:.0f} cm, "
                                    f"not {d * 100:.1f} (for {w * 100:.1f} cm wide)")
                        elif by_dots is not None and abs(by_dots - d) > DEPTH_WARN * d:
                            warn = (f"the robots' dots look squashed: for {w * 100:.1f} cm wide, the depth looks "
                                    f"like about {by_dots * 100:.0f} cm, not {d * 100:.1f}")
                        else:
                            warn = None
                        if warn:
                            lines += [(warn, RED),
                                      ("re-measure both sides in cm (not inches), then press z to re-enter them", RED),
                                      "u undo   z re-enter sizes   q: quit   (Enter saves anyway)"]
                        else:
                            said = ", ".join(x for x in [
                                f"the tape's shape says {by_tape * 100:.0f} cm" if by_tape is not None else "",
                                f"the robots' dots say {by_dots * 100:.0f} cm" if by_dots is not None else ""] if x)
                            lines += [(f"the measured depth checks out ({said})", GREEN) if said else
                                      "(put a robot in view so its dots can check the measured depth)",
                                      "u undo   z re-enter sizes   Enter: save the mapping, next   q: quit"]
                else:
                    x, y = floor[len(clicks)]
                    lines = [(f"{head}: click the {CORNERS[len(clicks)]} tape corner ({x * 100:.0f}, {y * 100:.0f} cm)",
                              YELLOW),
                             "front = your side; corners in order front-left, front-right, back-right, back-left",
                             "click the exact corner point of the tape; the magnifier follows the mouse   u undo   q: quit"]
                if st["mouse"] is not None:
                    magnifier(frame, raw, st["mouse"])

            else:   # CHECK
                if checker is None:
                    lines = [(f"{head}: no mapping saved yet: do step 3 first", RED), "3: corners   q: quit"]
                else:
                    blobs = checker.detect(frame)
                    poses = checker.poses_from(blobs, now)
                    rates.add(now, {r: checker.last_heading.get(r, (None, None))[1] == now for r in poses})
                    (_, _), (cw, cd) = cfg["area"]

                    def extra(f, px):
                        for r, spot in ((r, v["sentry"]) for r, v in cfg["robots"].items()):
                            c = px([spot])[0]
                            cv2.circle(f, tuple(c), 14, (200, 200, 200), 2)
                            cv2.putText(f, f"{r}'s spot", tuple(c + [16, 24]), FONT, 0.7, (200, 200, 200), 2)
                    grid(frame, lambda pts: apply_h(np.linalg.inv(checker.H_floor), pts).astype(int), cw, cd)
                    draw(frame, checker, blobs, poses, cfg["area"], extra=extra)
                    lines = [(f"{head}: stand a robot centred on a tape corner: it should read that corner within ~2 cm",
                              YELLOW),
                             "then move each robot along the edge farthest from the camera: 'read' should stay near 100 %"]
                    for r in cfg["robots"]:
                        found, hd = rates.rates(r)
                        p = poses.get(r)
                        where = "not seen" if p is None else f"at ({p.x * 100:.0f}, {p.y * 100:.0f}) cm"
                        if p is not None and not (-0.05 <= p.x <= cw + 0.05 and -0.05 <= p.y <= cd + 0.05):
                            where += ", OUTSIDE the zone"
                        good = hd >= 0.9
                        lines.append((f"robot {r} {where}: read in {found * 100:.0f} % of frames, heading in "
                                      f"{hd * 100:.0f} %" + ("" if good else "  <- should be near 100 %"),
                                      GREEN if good else RED))
                    lines.append("everything is saved   1-3: redo a step   h: hide this text   q: quit")

            text, until = st["flash"]
            if now < until:
                lines.append((text, GREEN))
            banner(frame, lines if st["text"] else ["h: show the text"])
            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if ord("1") <= key <= ord("4"):
                go(key - ord("1"))
            elif key == ord("h"):
                st["text"] = not st["text"]
            elif key == ord("z"):
                print("\n(the camera window pauses while you type here)")
                w, d, top = ask_sizes(w, d, top)
                st["depth"] = (None, None, 0.0)
            elif step == "COLOURS" and key in COLOUR_KEYS:
                st["colour"] = COLOUR_KEYS[key]
            elif step == "COLOURS" and key == ord("m"):
                st["mask"] = not st["mask"]
            elif step == "COLOURS" and key == ord("r"):
                samples[st["colour"]] = []
                ranges[st["colour"]] = list(DEFAULT_COLOURS[st["colour"]])
            elif step == "CORNERS" and key == ord("u") and clicks:
                clicks.pop()
                st["depth"] = (None, None, 0.0)
            elif key in ENTER:
                if step == "CORNERS":
                    if len(clicks) < 4 or corner_problem(clicks):
                        continue
                    H_floor = cv2.getPerspectiveTransform(np.float32(clicks), np.float32(floor))
                    H_top, cam = robot_top_mapping(H_floor, top, frame.shape[1::-1], focal)
                    if cam[2] <= top + 0.2:
                        flash(f"the camera seems to be only {cam[2] * 100:.0f} cm up: check the sizes and clicks")
                        continue
                    saved = {"H": H_top.tolist(), "H_floor": H_floor.tolist(), "robot_top_m": top,
                             "camera_m": [round(float(v), 3) for v in cam],
                             "clicked_px": [list(c) for c in clicks], "floor": [list(p) for p in floor]}
                    FLOOR.write_text(json.dumps(saved, indent=2))
                    old = json.loads(CONFIG.read_text())
                    CONFIG.write_text(config_with_zone(CONFIG.read_text(), w, d))
                    new = json.loads(CONFIG.read_text())
                    for r in new["robots"]:
                        if new["robots"][r]["sentry"] != old["robots"][r]["sentry"]:
                            print(f"robot {r}'s parking spot moved inside the zone: {new['robots'][r]['sentry']} m")
                    print(f"camera about {cam[2] * 100:.0f} cm up, above floor point "
                          f"({cam[0] * 100:.0f}, {cam[1] * 100:.0f}) cm")
                    try:                        # the WiFi boards follow the zone's size too
                        spots = update_floorplan(w, d)
                        print("WiFi floor plan updated; put the boards here (cm from the front-left tape corner):\n  "
                              + "\n  ".join(spots))
                    except (OSError, ValueError, KeyError) as e:
                        print(f"WiFi floor plan NOT updated ({e}): run  python wifi_area.py {w * 100:.0f} {d * 100:.0f}")
                    flash(f"saved {FLOOR.name}, the {w * 100:.1f} x {d * 100:.1f} cm zone in {CONFIG.name}, "
                          f"and the WiFi board spots (listed in the console)")
                if st["step"] < 3:
                    go(st["step"] + 1)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
