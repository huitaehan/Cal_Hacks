"""Colour tracking of the robots' SSL-Vision patterns: a blue centre blob plus four pink/green dots.

The dot colours say which robot it is (e.g. 4 pink = A, 2 pink + 2 green = B). The heading comes from
the dot layout ("pattern_layout" in config.json): on the SSL pattern ("ssl") the two front dots are wider
apart than the two back ones, so the SSL layout is fitted to the 4 dots (all 4 ways round) and a clearly
best fit gives the front. Evenly spaced dots ("even") can't show the front by position, so the adjacent
pink pair is taken as the front; an all-pink robot then has no heading.
Blobs are found in the image and mapped to floor metres with the homography from setup_camera.py, so
distances and angles are real ones.
Only these coloured markers are looked for; the camera never looks for people.
"""
import math
from collections import Counter

import cv2
import numpy as np

from tracker import Pose, apply_h

# OpenCV HSV (hue 0-179). A lower hue above the upper one wraps around red. Tune with setup_camera.py (step 2).
DEFAULT_COLOURS = {"blue": [[95, 100, 60], [130, 255, 255]],
                   "pink": [[140, 70, 80], [179, 255, 255]],
                   "green": [[40, 80, 50], [85, 255, 255]]}
CENTRE = "blue"
FRONT_COLOUR = "pink"       # layout "even": the adjacent pink pair is the front
MAX_SIZE_M = {"centre": 0.10, "dot": 0.07}   # larger blobs (clothes, posters) are ignored
MIN_AREA_PX = 8
# SSL standard dot positions in metres, robot frame (x = front), counter-clockwise: FR, FL, BL, BR.
SSL_DOTS = np.array([(0.035, -0.0548), (0.035, 0.0548), (-0.0548, 0.035), (-0.0548, -0.035)])
FIT_MARGIN_M = 0.003        # the best layout fit must beat the next by this much (4 mm noise: ~1 % ties)
FIT_MAX_RES_M = 0.012       # dots further than this (rms) from the pattern's layout aren't a robot (clutter)
SEARCH_FACTOR = 1.8         # dots are looked for within this many pattern radii of a blue centre
HEADING_KEEP_S = 0.5        # reuse a heading this long while a dot is hidden; the robots drift in rotation
JUMP_M = 0.3                # a robot can't move this far between frames: prefer the candidate nearest it


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def range_from_sample(hsv, dh=12, ds=80, dv=90):
    """An HSV range around a clicked colour sample."""
    h, s, v = (int(c) for c in hsv)
    return [(h - dh) % 180, max(50, s - ds), max(40, v - dv)], [(h + dh) % 180, 255, 255]


def range_from_samples(samples, dh=10, ds=60, dv=80):
    """An HSV range covering several clicked samples (hue is circular: red sits at both ends)."""
    ang = [math.radians(2 * h) for h, _, _ in samples]
    mean = math.degrees(math.atan2(sum(map(math.sin, ang)), sum(map(math.cos, ang)))) / 2 % 180
    spread = max(abs((h - mean + 90) % 180 - 90) for h, _, _ in samples)
    lo_h, hi_h = round(mean - spread - dh) % 180, round(mean + spread + dh) % 180
    return ([lo_h, max(50, min(s for _, s, _ in samples) - ds), max(40, min(v for _, _, v in samples) - dv)],
            [hi_h, 255, 255])


def colour_mask(hsv, rng):
    (h0, s0, v0), (h1, s1, v1) = rng
    if h0 <= h1:
        return cv2.inRange(hsv, np.array([h0, s0, v0]), np.array([h1, s1, v1]))
    return cv2.inRange(hsv, np.array([h0, s0, v0]), np.array([179, s1, v1])) | \
        cv2.inRange(hsv, np.array([0, s0, v0]), np.array([h1, s1, v1]))


def _fit(obs, tmpl):
    """Best rotation (degrees) taking the template onto the observed points, and the rms residual (m)."""
    a, b = tmpl - tmpl.mean(0), obs - obs.mean(0)
    th = math.atan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    c, s = math.cos(th), math.sin(th)
    res = math.sqrt(((b - a @ np.array([[c, s], [-s, c]])) ** 2).sum(1).mean())
    return res, math.degrees(th)


def heading_from_dots(centre, dots, layout: str = "ssl"):
    """Front direction (degrees) from the 4 dots, or None if they can't tell.
    layout "ssl": fit the SSL layout (front dots wider apart); None unless one fit clearly wins.
    layout "even": evenly spaced dots; the adjacent pink pair is the front (all-pink robots get None).
    The blue centre isn't used: the dots alone are steadier."""
    if len(dots) != 4:
        return None
    pts = np.array([d[:2] for d in dots], float)
    c = pts.mean(0)
    ang = np.degrees(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])) % 360
    order = np.argsort(ang)
    obs, cols, ang = pts[order], [dots[i][2] for i in order], ang[order]
    if layout == "ssl":
        fits = sorted(_fit(np.roll(obs, -k, axis=0), SSL_DOTS) for k in range(4))
        return wrap(fits[0][1]) if fits[1][0] - fits[0][0] >= FIT_MARGIN_M else None
    idx = [k for k in range(4) if cols[k] == FRONT_COLOUR]
    if len(idx) == 2 and (idx[1] - idx[0]) % 4 in (1, 3):
        sx = sum(math.cos(math.radians(ang[k])) for k in idx)
        sy = sum(math.sin(math.radians(ang[k])) for k in idx)
        return math.degrees(math.atan2(sy, sx))
    return None


def _template(layout: str, r: float):
    if layout == "ssl":
        return SSL_DOTS * (r / 0.065)
    a = np.radians([45, 135, 225, 315])
    return np.c_[np.cos(a), np.sin(a)] * r


def pattern_fits(centre, near, template) -> bool:
    """Do these 3-4 dots around this centre have the robots' layout? Clutter specks don't."""
    pts = np.array([d[:2] for d in near], float) - np.asarray(centre, float)
    pts = pts[np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))]
    if len(pts) == 4:
        best = min(_fit(np.roll(pts, -k, axis=0), template)[0] for k in range(4))
    else:       # 3 dots: the centre is the 4th point, so the shape is pinned down
        best = min(_fit(np.vstack([[0.0, 0.0], np.roll(pts, -k, axis=0)]),
                        np.vstack([[0.0, 0.0], np.delete(template, drop, axis=0)]))[0]
                   for drop in range(4) for k in range(3))
    return best <= FIT_MAX_RES_M


def assemble(centres, dots, patterns: dict, radius: float, prev: dict | None = None, layout: str = "ssl") -> dict:
    """Match each centre blob with its nearest dots; robot id from the dot colours. `radius` is the search
    radius (SEARCH_FACTOR pattern radii); the dots must also fit the layout (pattern_fits).
    Returns {robot: (x, y, heading or None)}; 3 visible dots give position and id but no heading.
    If two candidates match one robot, one within JUMP_M of its previous position (`prev`) wins, then
    the one with more dots, then the one whose centre sits closest to the middle of its dots (a speck of
    blue beside the robot doesn't)."""
    out, rank = {}, {}
    template = _template(layout, radius / SEARCH_FACTOR)
    for c in centres:
        near = sorted((d for d in dots if math.dist(d[:2], c) <= radius), key=lambda d: math.dist(d[:2], c))[:4]
        if len(near) < 3 or not pattern_fits(c, near, template):
            continue
        counts = Counter(d[2] for d in near)
        fits = [r for r, p in patterns.items() if all(n <= p.get(col, 0) for col, n in counts.items())]
        exact = [r for r in fits if dict(counts) == patterns[r]]
        rid = exact[0] if len(exact) == 1 else fits[0] if len(fits) == 1 else None
        if rid is None:
            continue
        near_prev = prev is not None and rid in prev and math.dist(prev[rid], c) <= JUMP_M
        mid = (sum(d[0] for d in near) / len(near), sum(d[1] for d in near) / len(near))
        r = (near_prev, len(near), -math.dist(mid, c))
        if rid in rank and rank[rid] >= r:
            continue
        out[rid] = (c[0], c[1], heading_from_dots(c, near, layout) if len(near) == 4 else None)
        rank[rid] = r
    return out


class ColourTracker:
    def __init__(self, patterns: dict, H=None, colours: dict | None = None, pattern_radius_m: float = 0.065,
                 layout: str = "ssl"):
        self.patterns = patterns
        self.H = None if H is None else np.asarray(H, float)   # image pixels -> floor metres
        self.colours = colours or DEFAULT_COLOURS
        self.radius = SEARCH_FACTOR * pattern_radius_m
        self.layout = layout              # "ssl" or "even" (see heading_from_dots)
        self.dot_colours = sorted({c for p in patterns.values() for c in p})
        self.last_heading: dict = {}    # robot -> (heading, t)
        self.last_pos: dict = {}        # robot -> (x, y)

    @property
    def status(self):
        return None if self.H is not None else "no floor mapping yet: run  python setup_camera.py"

    def detect(self, frame):
        """Pixel blobs: [(colour, cx, cy, (x, y, w, h))]."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)
        blobs = []
        for col in [CENTRE] + self.dot_colours:
            mask = cv2.morphologyEx(colour_mask(hsv, self.colours[col]), cv2.MORPH_OPEN, kernel)
            n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] >= MIN_AREA_PX:
                    blobs.append((col, float(cents[i][0]), float(cents[i][1]), tuple(int(v) for v in stats[i, :4])))
        return blobs

    def to_floor(self, blobs):
        centres, dots = [], []
        for col, cx, cy, (x, y, w, h) in blobs:
            box = apply_h(self.H, [(x, y), (x + w, y), (x + w, y + h), (x, y + h)])
            size = max(np.ptp(box[:, 0]), np.ptp(box[:, 1]))
            fx, fy = apply_h(self.H, [(cx, cy)])[0]
            if col == CENTRE and size <= MAX_SIZE_M["centre"]:
                centres.append((float(fx), float(fy)))
            elif col != CENTRE and size <= MAX_SIZE_M["dot"]:
                dots.append((float(fx), float(fy), col))
        return centres, dots

    def update(self, centres, dots, t: float) -> dict:
        poses = {}
        for rid, (x, y, hd) in assemble(centres, dots, self.patterns, self.radius, self.last_pos,
                                             self.layout).items():
            if hd is None:
                old = self.last_heading.get(rid)
                hd = old[0] if old and t - old[1] <= HEADING_KEEP_S else None
            else:
                self.last_heading[rid] = (hd, t)
            self.last_pos[rid] = (x, y)
            poses[rid] = Pose(x, y, hd, t)
        return poses

    def poses_from(self, blobs, t: float) -> dict:
        return {} if self.H is None else self.update(*self.to_floor(blobs), t)

    def process(self, frame, t: float) -> dict:
        return self.poses_from(self.detect(frame), t)

    def annotate(self, frame, blobs):
        bgr = {"blue": (255, 120, 0), "pink": (200, 80, 255), "green": (0, 220, 0)}
        for col, cx, cy, _ in blobs or []:
            cv2.circle(frame, (int(cx), int(cy)), 9, bgr.get(col, (255, 255, 255)), 2)
