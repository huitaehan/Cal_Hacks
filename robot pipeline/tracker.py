"""Overhead camera -> robot positions on the floor.

Two ways, chosen by "tracking" in config.json:
- "colour" (default): the robots' SSL-Vision colour patterns (colour_tracker.py), with the floor mapping
  from 4 clicked corners (floor_setup.py).
- "aruco": printed markers; four on the floor at known spots fix the camera-to-floor mapping, one on top
  of each robot gives its position and heading.
Only these markers are looked for; the camera never looks for people.

    python tracker.py                 # live window: markers, floor outline, robot poses
    python tracker.py --snapshot f.png  # save one annotated frame and print what was seen
    python tracker.py --camera 1        # another camera than config.json's (a built-in webcam is often 0)

With walls in the floor plan, they're drawn in red and each robot shows its distance to the nearest one:
push a robot flat against a cardboard wall and it should read the robot's radius (9 cm).
"""
import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
CONFIG = Path(__file__).parent / "config.json"
FLOOR = Path(__file__).parent / "floor.json"       # colour mode: clicked corners -> homography
COLOURS = Path(__file__).parent / "colours.json"   # colour mode: tuned colour ranges


@dataclass
class Pose:
    x: float
    y: float
    heading: float      # degrees counter-clockwise from +x, direction the marker's top edge faces
    t: float


def apply_h(H, pts):
    p = np.c_[np.asarray(pts, float), np.ones(len(pts))] @ np.asarray(H).T
    return p[:, :2] / p[:, 2:]


def centre_px(c):
    """Marker centre in the image: where its diagonals cross (exact under perspective)."""
    p0, p1, p2, p3 = (np.asarray(v, float) for v in c)
    d1, d2 = p2 - p0, p3 - p1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return np.asarray(c, float).mean(0)
    s = ((p1[0] - p0[0]) * d2[1] - (p1[1] - p0[1]) * d2[0]) / den
    return p0 + s * d1


def pose_from_corners(H, corners):
    """(x, y, heading) on the floor from a marker's 4 image corners (ArUco order: TL, TR, BR, BL)."""
    f = apply_h(H, corners)
    c = f.mean(0)
    fwd = (f[0] + f[1]) / 2 - (f[2] + f[3]) / 2
    return float(c[0]), float(c[1]), math.degrees(math.atan2(fwd[1], fwd[0]))


class FloorTracker:
    def __init__(self, floor_markers: dict, robot_markers: dict):
        self.floor = {int(k): tuple(v) for k, v in floor_markers.items()}
        self.robots = {int(k): v for k, v in robot_markers.items()}
        self.H = None           # image pixels -> floor metres; kept once found (the camera is fixed)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(DICT, params)

    def detect(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return [], []
        return [c.reshape(4, 2) for c in corners], [int(i) for i in ids.ravel()]

    def update(self, corners, ids, t: float) -> dict:
        seen = dict(zip(ids, corners))
        on_floor = [i for i in self.floor if i in seen]
        if len(on_floor) >= 4:
            src = np.array([centre_px(seen[i]) for i in on_floor])
            dst = np.array([self.floor[i] for i in on_floor])
            H, _ = cv2.findHomography(src, dst, cv2.RANSAC if len(on_floor) > 4 else 0)
            if H is not None:
                self.H = H
        if self.H is None:
            return {}
        return {name: Pose(*pose_from_corners(self.H, seen[mid]), t)
                for mid, name in self.robots.items() if mid in seen}

    def poses_from(self, det, t: float) -> dict:
        return self.update(*det, t)

    def process(self, frame, t: float) -> dict:
        return self.poses_from(self.detect(frame), t)

    @property
    def status(self):
        return None if self.H is not None else "floor markers 0-3 not all seen yet"

    def annotate(self, frame, det):
        corners, ids = det
        if ids:
            cv2.aruco.drawDetectedMarkers(frame, [c.reshape(1, 4, 2).astype(np.float32) for c in corners],
                                          np.array(ids).reshape(-1, 1))


def make_tracker(cfg: dict):
    if cfg.get("tracking", "colour") == "aruco":
        return FloorTracker(cfg["floor_markers"], {r["marker"]: name for name, r in cfg["robots"].items()})
    from colour_tracker import DEFAULT_COLOURS, ColourTracker
    H = json.loads(FLOOR.read_text())["H"] if FLOOR.exists() else None
    colours = json.loads(COLOURS.read_text()) if COLOURS.exists() else DEFAULT_COLOURS
    return ColourTracker({name: r["pattern"] for name, r in cfg["robots"].items()}, H, colours,
                         cfg.get("pattern_radius_m", 0.065), cfg.get("pattern_layout", "ssl"))


def open_camera(cam: dict):
    cap = cv2.VideoCapture(cam["index"], cv2.CAP_DSHOW)
    # Size first, then MJPG: the other order leaves the camera on uncompressed YUY2 (5 fps at 1080p).
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # always the newest frame, not a queued one
    if not cap.isOpened():
        raise RuntimeError(f"camera {cam['index']} didn't open")
    return cap


class CameraTracker:
    """Reads the camera in a thread; `poses()` gives the latest pose per robot."""

    def __init__(self, cfg: dict, cap=None, ft=None):
        self.ft = ft or make_tracker(cfg)
        self.cap = cap or open_camera(cfg["camera"])
        self.lock = threading.Lock()
        self._poses: dict = {}
        self.frame = None
        self.det = None
        self.fps = 0.0
        self.errors: list[str] = []     # tracking errors (the thread keeps going)
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        n, t0 = 0, time.time()
        while not self._stop:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            now = time.time()
            try:
                det = self.ft.detect(frame)
                poses = self.ft.poses_from(det, now)
            except Exception as e:      # one bad frame must not stop tracking for good
                msg = f"tracking error: {type(e).__name__}: {e}"
                if msg not in self.errors:
                    print(msg, flush=True)
                self.errors = (self.errors + [msg])[-20:]
                det, poses = None, {}
            with self.lock:
                self._poses.update(poses)
                self.frame, self.det = frame, det
            n += 1
            if now - t0 >= 1.0:
                self.fps, n, t0 = n / (now - t0), 0, now

    def poses(self, max_age: float = 0.5) -> dict:
        """Latest pose per robot seen within max_age seconds."""
        now = time.time()
        with self.lock:
            return {k: p for k, p in self._poses.items() if now - p.t <= max_age}

    def snapshot(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy()), self.det

    def close(self):
        self._stop = True
        self.cap.release()


def nearest_wall_m(x: float, y: float, walls) -> float | None:
    """Distance from (x, y) to the closest wall segment, or None without walls."""
    best = None
    for (ax, ay), (bx, by) in walls:
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else min(max(((x - ax) * dx + (y - ay) * dy) / l2, 0.0), 1.0)
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        best = d if best is None else min(best, d)
    return best


def draw(frame, ft, det, poses: dict, area=None, extra=None, walls=None):
    """Annotate a frame: detections, floor outline, walls, robot arrows; `extra` draws more in floor metres.
    With walls, each robot also shows its centre's distance to the nearest wall: pushed flat against a
    cardboard wall it should read the robot's radius, else the wall isn't where the floor plan says."""
    ft.annotate(frame, det)
    if ft.H is None:
        cv2.putText(frame, ft.status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return frame
    Hi = np.linalg.inv(ft.H)
    px = lambda pts: apply_h(Hi, pts).astype(int)
    if area:
        (x0, y0), (x1, y1) = area
        cv2.polylines(frame, [px([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])], True, (0, 200, 0), 2)
    for a, b in walls or []:
        cv2.line(frame, *map(tuple, px([a, b])), (40, 40, 220), 4)
    for name, p in poses.items():
        a = px([(p.x, p.y)])[0]
        if p.heading is None:
            cv2.circle(frame, tuple(a), 14, (255, 0, 0), 3)
        else:
            h = math.radians(p.heading)
            b = px([(p.x + 0.3 * math.cos(h), p.y + 0.3 * math.sin(h))])[0]
            cv2.arrowedLine(frame, tuple(a), tuple(b), (255, 0, 0), 3, tipLength=0.3)
        hd = "heading ?" if p.heading is None else f"{p.heading:.0f}deg"
        d = nearest_wall_m(p.x, p.y, walls or [])
        hd += "" if d is None else f"  wall {d * 100:.0f} cm"
        cv2.putText(frame, f"{name} ({p.x:.2f}, {p.y:.2f}) {hd}", tuple(a + [10, -10]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    if extra:
        extra(frame, px)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", help="save one annotated frame here and exit")
    ap.add_argument("--camera", type=int, help="camera index (default: config.json)")
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    if args.camera is not None:
        cfg["camera"]["index"] = args.camera
    from run import load_walls
    walls = load_walls(cfg)
    ct = CameraTracker(cfg)
    try:
        time.sleep(2.0)
        if args.snapshot:
            frame, det = ct.snapshot()
            if frame is None:
                raise SystemExit("no frame from the camera")
            cv2.imwrite(args.snapshot, draw(frame, ct.ft, det, ct.poses(), cfg["area"], walls=walls))
            print(f"frame {frame.shape[1]}x{frame.shape[0]}, {ct.fps:.0f} fps, "
                  f"floor mapping: {'yes' if ct.ft.H is not None else 'no (' + ct.ft.status + ')'}")
            if cfg.get("tracking", "colour") != "aruco":
                from collections import Counter
                print("  colour blobs:", dict(Counter(b[0] for b in det or [])))
                for col, cx, cy, (_, _, w, h) in det or []:
                    print(f"    {col:6s} at pixel ({cx:7.1f}, {cy:7.1f}), {w}x{h} px")
            for name, p in ct.poses().items():
                hd = "?" if p.heading is None else f"{p.heading:.0f}"
                d = nearest_wall_m(p.x, p.y, walls)
                print(f"  robot {name}: x {p.x:.2f} m, y {p.y:.2f} m, heading {hd} deg"
                      + ("" if d is None else f", nearest wall {d * 100:.0f} cm from its centre"))
            return
        while True:
            frame, det = ct.snapshot()
            if frame is not None:
                draw(frame, ct.ft, det, ct.poses(), cfg["area"], walls=walls)
                tip = (f"   walls: {len(walls)} (red); a robot flat against one should read "
                       f"{cfg.get('robot_radius_m', 0.09) * 100:.0f} cm") if walls else ""
                cv2.putText(frame, f"{ct.fps:.0f} fps   q: quit{tip}", (20, frame.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow("tracker", frame)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        ct.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
