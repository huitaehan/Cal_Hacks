"""
WiFi presence dashboard host for ESP32-C5 presence nodes.

Reads every node's USB console, runs a CSI amplitude motion detector per link,
relays RuView's on-device edge results, and serves a live dashboard.

    python presence_server.py                 # auto-detect Espressif boards
    python presence_server.py --ports COM10 COM11 --http-port 8000

Open http://localhost:8000
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import re
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

import ruview_bridge
from espectre_link import EspectreLinkDetector, espectre_config
from floorplan import FloorPlanWatcher
from robots import RobotTracker
from rooms import RoomEvidence
from tracking import PositionKalman
from wander import WanderLinkDetector

ESPRESSIF_VID = 0x303A
STATIC_DIR = Path(__file__).resolve().parent / "static"
RECORD_DIR = Path(__file__).resolve().parent / "recordings"
RUVIEW_UI_DIR = Path(__file__).resolve().parent.parent / "RuView" / "ui"
LAYOUT_PATH = Path(__file__).resolve().parent / "layout.json"
FLOORPLAN_PATH = Path(__file__).resolve().parent / "floorplan.json"
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8",
    ".json": "application/json", ".svg": "image/svg+xml", ".png": "image/png",
    ".jpg": "image/jpeg", ".ico": "image/x-icon", ".wasm": "application/wasm",
    ".glb": "model/gltf-binary", ".woff2": "font/woff2",
}
# The Observatory probes GET /health with a 1.5 s abort while its 3D scene is
# still being built on the main thread (~2-3 s), so the probe is aborted even
# though the server answers at once, and it falls back to demo data. Let the
# same-origin probe finish instead. RuView's own files stay unmodified.
OBSERVATORY_PROBE_SHIM = b"""
<script>
(() => {
  const f = window.fetch;
  window.fetch = function (url, opts) {
    if (url === location.origin + '/health' && opts && opts.signal) {
      const { signal, ...rest } = opts;
      return f.call(this, url, rest);
    }
    return f.apply(this, arguments);
  };
})();
</script>"""

# RuView edge calibration length (EDGE_CALIB_FRAMES in edge_processing.h).
RUVIEW_CALIB_FRAMES = 1200
CALIB_DONE_RE = re.compile(r"Adaptive calibration complete: mean=([\d.]+) sigma=([\d.]+) threshold=([\d.]+)")


class Settings:
    def __init__(self, calib_seconds: float, detector: str = "wander"):
        self.lock = threading.Lock()
        self.detector = detector        # per-link backend: "wander", "espectre" (no calibration) or "denoised"
        self.sensitivity = 1.0          # >1 lowers thresholds
        self.include_static = False     # also flag a still person via baseline deviation
        # "host", "ruview" or "either". RuView's raw-phase flag fires at random on
        # ESP-NOW CSI (see README), so the alarm follows the host detector by default.
        self.banner_source = "host"
        self.calib_seconds = calib_seconds

    def as_dict(self) -> dict:
        with self.lock:
            return {
                "sensitivity": self.sensitivity,
                "include_static": self.include_static,
                "banner_source": self.banner_source,
                "calib_seconds": self.calib_seconds,
            }


class LinkDetector:
    """Amplitude-variance motion detector for one CSI link.

    Each frame is normalised by its mean amplitude (cancelling AGC gain steps),
    then the motion score is the mean coefficient of variation per subcarrier
    over a 1 s window. A person moving near the link scatters the multipath and
    drives this up. Thresholds come from an ambient calibration window.
    """

    WINDOW_S = 1.0
    WARMUP_S = 5.0          # after a reset: AGC settles, the operator steps away
    HISTORY = 600
    ENTER_UPDATES = 2       # consecutive updates above threshold to assert
    HOLD_S = 2.5            # time below the low threshold before clearing
    HYST = 0.75

    def __init__(self, settings: Settings):
        self.settings = settings
        self.lock = threading.Lock()
        self.n_sc = None
        self.buf = None
        self.times = np.zeros(self.HISTORY)
        self.count = 0
        self.reset_calibration()

    def reset_calibration(self):
        with getattr(self, "lock", threading.Lock()):
            self.warmup_start = None
            self.calib_start = None
            self.calib_motion: list[float] = []
            self.calib_means: list[np.ndarray] = []
            self.calibrated = False
            self.thr_motion = None
            self.thr_static = None
            self.baseline = None
            self.valid = None
            self.motion = 0.0
            self.motion_ema = 0.0
            self.cv = None
            self.static_dev = 0.0
            self.detected = False
            self.static_detected = False
            self.above = 0
            self.last_above_t = 0.0
            self.last_update_t = 0.0

    def push(self, t: float, amp: np.ndarray):
        with self.lock:
            if self.n_sc != amp.size:
                if self.n_sc is not None and self.count > 0:
                    return  # keep the dominant frame layout
                self.n_sc = amp.size
                self.buf = np.zeros((self.HISTORY, self.n_sc), dtype=np.float32)
            i = self.count % self.HISTORY
            self.buf[i] = amp
            self.times[i] = t
            self.count += 1

    MIN_FRAMES = 12
    MAX_WINDOW_S = 3.0  # slow (weak or relayed) links stretch the window up to this

    def _window(self, now: float):
        n = min(self.count, self.HISTORY)
        if n < self.MIN_FRAMES:
            return None
        idx = (np.arange(self.count - n, self.count)) % self.HISTORY
        t = self.times[idx]
        sel = idx[t >= now - self.WINDOW_S]
        if sel.size < self.MIN_FRAMES:
            sel = idx[-self.MIN_FRAMES:]
            if now - self.times[sel[0]] > self.MAX_WINDOW_S:
                return None
        return self.buf[sel]

    BAND = 8
    SMOOTH = 4  # packets; fixed so the score's noise floor does not move with the packet rate

    @classmethod
    def _denoise(cls, Xn: np.ndarray) -> np.ndarray:
        n_band = Xn.shape[1] // cls.BAND
        Y = Xn[:, :n_band * cls.BAND].reshape(Xn.shape[0], n_band, cls.BAND).mean(axis=2)
        k = min(cls.SMOOTH, Y.shape[0] - 1)
        c = np.cumsum(np.vstack([np.zeros((1, Y.shape[1])), Y]), axis=0)
        return (c[k:] - c[:-k]) / k

    def update(self, now: float):
        with self.lock:
            X = self._window(now)
            if X is None:
                return
            col_mean = X.mean(axis=0)
            # Choose usable subcarriers during warm-up only, then freeze the set:
            # calibration and detection must compare the same subcarriers.
            if self.valid is None or self.calib_start is None:
                med = np.median(col_mean)
                self.valid = col_mean > 0.2 * max(med, 1e-6)
            v = self.valid
            if v.sum() < 8:
                return
            Xv = X[:, v]
            frame_mean = Xv.mean(axis=1, keepdims=True)
            Xn = Xv / np.maximum(frame_mean, 1e-6)
            mean_n = Xn.mean(axis=0)
            # Denoise before measuring variation: per-packet noise is independent
            # across subcarriers and packets, while body motion is slow and shared
            # by neighbouring subcarriers. Average bands of 8 subcarriers, then a
            # ~0.1 s moving average. On a -60..-73 dBm link this lifted walking from
            # ~1.4x to ~2.5x the quiet level.
            Y = self._denoise(Xn)
            cv = Y.std(axis=0) / np.maximum(Y.mean(axis=0), 1e-6)
            self.cv = cv
            self.motion = float(cv.mean())
            self.motion_ema = 0.35 * self.motion + 0.65 * self.motion_ema if self.last_update_t else self.motion
            self.last_update_t = now

            if not self.calibrated:
                if self.warmup_start is None:
                    self.warmup_start = now
                if now - self.warmup_start < self.WARMUP_S:
                    return
                if self.calib_start is None:
                    self.calib_start = now
                self.calib_motion.append(self.motion_ema)
                self.calib_means.append(mean_n)
                if now - self.calib_start >= self.settings.calib_seconds and len(self.calib_motion) >= 20:
                    # Median/MAD so a brief movement during calibration cannot inflate the threshold.
                    m = np.array(self.calib_motion)
                    med = float(np.median(m))
                    mad = 1.4826 * float(np.median(np.abs(m - med)))
                    self.thr_motion = max(med + 6.0 * mad, 2.0 * med, 0.01)
                    self.baseline = np.median(np.array(self.calib_means), axis=0)
                    devs = np.array([np.mean(np.abs(cm - self.baseline) / self.baseline) for cm in self.calib_means])
                    dmed = float(np.median(devs))
                    dmad = 1.4826 * float(np.median(np.abs(devs - dmed)))
                    self.thr_static = max(dmed + 6.0 * dmad, 2.0 * dmed, 0.02)
                    self.calibrated = True
                return

            self.static_dev = float(np.mean(np.abs(mean_n - self.baseline) / self.baseline))
            sens = max(self.settings.sensitivity, 0.05)
            thr = self.thr_motion / sens
            if self.motion_ema > thr:
                self.above += 1
                self.last_above_t = now
            elif self.motion_ema > thr * self.HYST:
                self.last_above_t = now
                self.above = 0
            else:
                self.above = 0
            if not self.detected and self.above >= self.ENTER_UPDATES:
                self.detected = True
            elif self.detected and now - self.last_above_t > self.HOLD_S:
                self.detected = False
            self.static_detected = self.static_dev > self.thr_static / sens

    def snapshot(self, now: float) -> dict:
        with self.lock:
            sens = max(self.settings.sensitivity, 0.05)
            thr = self.thr_motion / sens if self.thr_motion else None
            thr_s = self.thr_static / sens if self.thr_static else None
            calib_left = warmup_left = None
            if not self.calibrated:
                if self.calib_start is None:
                    warmup_left = self.WARMUP_S if self.warmup_start is None else max(
                        0.0, self.WARMUP_S - (now - self.warmup_start))
                    calib_left = warmup_left + self.settings.calib_seconds
                else:
                    calib_left = max(0.0, self.settings.calib_seconds - (now - self.calib_start))
            return {
                "calibrated": self.calibrated,
                "calib_left": calib_left,
                "warmup_left": warmup_left,
                "motion": self.motion_ema,
                "threshold": thr,
                "ratio": (self.motion_ema / thr) if thr else None,
                "static_dev": self.static_dev,
                "static_threshold": thr_s,
                "static_ratio": (self.static_dev / thr_s) if thr_s else None,
                "detected": self.detected,
                "static_detected": self.static_detected,
                "n_subcarriers": int(self.valid.sum()) if self.valid is not None else 0,
                "subcarrier_cv": [round(float(x), 4) for x in self.cv] if self.cv is not None else None,
            }

    def display_row(self) -> list[float] | None:
        """Mean normalised amplitude over the last ~100 ms, subcarriers in frequency order."""
        with self.lock:
            if self.count == 0:
                return None
            k = min(5, self.count)
            idx = (np.arange(self.count - k, self.count)) % self.HISTORY
            row = self.buf[idx].mean(axis=0)
            if self.valid is not None:
                m = row[self.valid].mean() if self.valid.any() else 1.0
                row = np.where(self.valid, row / max(m, 1e-6), 0.0)
            else:
                row = row / max(row.mean(), 1e-6)
            if row.size == 64:
                row = np.roll(row, 32)  # HT20 layout: 0..31, -32..-1 -> -32..31
            return [round(float(x), 3) for x in row]


EMPTY_DETECTOR = {
    "calibrated": False, "calib_left": None, "warmup_left": None, "motion": 0.0, "threshold": None,
    "ratio": None, "static_dev": 0.0, "static_threshold": None, "static_ratio": None, "detected": False,
    "static_detected": False, "n_subcarriers": 0, "subcarrier_cv": None, "score": None,
}


MIN_AGREE_PAIRS = 2


def agree_pairs_needed(detector: str) -> int:
    """ESPectre and the denoised score need two agreeing pairs; wander pairs are already judged
    against their own calm level, and one pair caught 94 % of walking inside vs 6 % outside."""
    return 1 if detector == "wander" else MIN_AGREE_PAIRS


def pairs_agree(detected_pairs: int, live_pairs: int, need: int = MIN_AGREE_PAIRS) -> bool:
    """Motion counts when at least `need` board pairs see it (fewer if fewer pairs exist).

    Live 4-board test (boards at chest height), ESPectre: any single link fired 5 % of the
    time with nobody near and 70 % with someone walking 0.5-1 m outside; two agreeing pairs
    fired 0 % and 45 %, and still caught 87 % of the time someone moved inside.
    """
    return detected_pairs >= max(1, min(need, live_pairs))


def fuse_pairs(snaps: list[dict], detector: str, include_static: bool = False) -> dict[frozenset, dict]:
    """One entry per board pair from its live, ready directed links: {"ratio", "detected"}.

    wander: the two directions measure the same channel, so their scores are averaged and
    the pair counts when the average clears the threshold. ESPectre / denoised: the
    stronger direction's ratio, and motion if either direction's debounced detector says so.
    """
    acc: dict[frozenset, list] = {}
    for s in snaps:
        d = s["detector"]
        if s.get("rate", 1) <= 0:
            continue
        key = frozenset((s["tx"], s["rx"]))
        if detector == "wander":
            if d.get("score") is not None:
                acc.setdefault(key, []).append(d["score"])
            continue
        hit = bool(d["detected"] or (include_static and d.get("static_detected")))
        if d["ratio"] is None and not hit:
            continue
        r = d["ratio"] if d["ratio"] is not None else 0.0
        acc[key] = [max(acc[key][0], r), acc[key][1] or hit] if key in acc else [r, hit]
    if detector == "wander":
        th = WanderLinkDetector.THRESHOLD
        return {k: {"ratio": max(0.0, float(np.mean(v))) / th, "detected": bool(np.mean(v) > th)}
                for k, v in acc.items()}
    return {k: {"ratio": float(v[0]), "detected": v[1]} for k, v in acc.items()}


class Link:
    """One directed radio link tx -> rx: the rx node's CSI of tx's beacons.

    Fed either by the rx node itself over USB (CSIA lines) or relayed through
    another node's beacon (RLY lines), so links between USB-less nodes count too.
    """

    OWN_HOLD_S = 2.0  # while own CSI arrives, ignore relayed copies of the same link
    RETARGET_AFTER_S = 3.0
    RETARGET_EVERY_S = 0.5
    RETARGET_CHECKS = 10   # 5 s of agreeing checks: live links at a boundary rebuilt 5-21 times in 2.5 min

    def __init__(self, tx: str, rx: str, settings: Settings, still=None):
        self.tx, self.rx = tx, rx
        self.settings = settings
        self.still = still            # (mac, now) -> bool; None = every board is stationary
        self._gate_open = True
        self.last_rx_t = 0.0
        self.espectre = settings.detector == "espectre"
        self.wander = settings.detector == "wander"
        self.detector = (EspectreLinkDetector(target_pps=20) if self.espectre else
                         WanderLinkDetector() if self.wander else LinkDetector(settings))
        self.lock = threading.Lock()
        self.times = collections.deque(maxlen=400)
        self.rssi = collections.deque(maxlen=100)
        self.last_relay_seq = None
        self.own_t = 0.0
        self.n = 0
        self._first_t = None
        self._next_retarget = 0.0
        self._proposal = None
        self._agree = 0

    def _maybe_retarget(self, now: float):
        """Keep ESPectre's slot rate matched to this link's real packet rate (RETARGET_CHECKS agreeing checks)."""
        if self._first_t is None:
            self._first_t = now
        if now - self._first_t < self.RETARGET_AFTER_S or now < self._next_retarget:
            return
        self._next_retarget = now + self.RETARGET_EVERY_S
        with self.lock:
            rate = sum(1 for t in self.times if t >= now - 3.0) / 3.0
        target = espectre_config(rate)              # (slot rate, window ms)
        if target == (self.detector.target_pps, self.detector.window_ms):
            self._proposal, self._agree = None, 0
            return
        self._agree = self._agree + 1 if self._proposal == target else 1
        self._proposal = target
        if self._agree >= self.RETARGET_CHECKS:
            self.detector = EspectreLinkDetector(target_pps=target[0], window_ms=target[1])
            self._proposal, self._agree = None, 0

    def push(self, now: float, amp: np.ndarray, rssi: int, relay_seq: int | None = None,
             device_ts_us: int | None = None):
        self.last_rx_t = now                       # heard, even if the gate drops the sample
        # Only trust a link while both of its ends are standing still.
        if self.still is not None and not (self.still(self.tx, now) and self.still(self.rx, now)):
            self._gate_open = False
            return
        if not self._gate_open:
            self._gate_open = True
            if self.espectre or self.wander:
                self.detector.reset_calibration()  # new stop: don't mix in the previous spot
            with self.lock:
                self.times.clear()                 # the drive left no samples: measure the rate afresh
            self._first_t = None
            self._proposal, self._agree = None, 0
        with self.lock:
            if relay_seq is not None:
                if now - self.own_t < self.OWN_HOLD_S:
                    return
                if self.last_relay_seq is not None:
                    d = (relay_seq - self.last_relay_seq) & 0xFFFF
                    if d == 0 or d > 0x8000:
                        return  # duplicate via a second gateway, or stale
                self.last_relay_seq = relay_seq
            else:
                self.own_t = now
            self.times.append(now)
            self.rssi.append(rssi)
            self.n += 1
            n = self.n
        if self.espectre:
            self._maybe_retarget(now)
            self.detector.push(now, amp, device_ts_us=device_ts_us)
        elif self.wander:
            self.detector.push(now, amp, device_ts_us=device_ts_us)
        else:
            self.detector.push(now, amp)
        if self.espectre or self.wander or n % 5 == 0:   # both limit themselves to one evaluation per 250 ms
            self.detector.update(now)

    def rate(self, now: float) -> float:
        with self.lock:
            return sum(1 for t in self.times if t >= now - 2.0) / 2.0

    def snapshot(self, now: float) -> dict:
        det = self.detector.snapshot(now)
        rate = self.rate(now)
        if rate == 0:
            # No fresh data (node off or out of range): don't report a frozen score.
            det.update(ratio=None, static_ratio=None, detected=False, static_detected=False)
        with self.lock:
            rssi = list(self.rssi)[-25:]
            relayed = now - self.own_t >= self.OWN_HOLD_S
        return {"tx": self.tx, "rx": self.rx, "rate": rate, "relayed": relayed,
                "rssi": float(np.mean(rssi)) if rssi else None, "detector": det}


class LinkRegistry:
    def __init__(self, settings: Settings, still=None):
        self.settings = settings
        self.still = still
        self.lock = threading.Lock()
        self.links: dict[tuple[str, str], Link] = {}

    def get(self, tx: str, rx: str) -> Link:
        with self.lock:
            link = self.links.get((tx, rx))
            if link is None:
                link = self.links[(tx, rx)] = Link(tx, rx, self.settings, still=self.still)
            return link

    def find(self, tx: str | None, rx: str | None) -> Link | None:
        with self.lock:
            return self.links.get((tx, rx))

    def all(self) -> list[Link]:
        with self.lock:
            return list(self.links.values())

    def reset(self, rx: str | None = None):
        for link in self.all():
            if rx is None or link.rx == rx:
                link.detector.reset_calibration()


class Localizer:
    """Radio tomographic imaging over all calibrated links.

    A person disturbs the links whose propagation ellipse they stand in. Each
    moving link adds weight along its ellipse; each quiet link subtracts a
    little, so the peak sits where the active ellipses cross. Board positions
    (metres) come from layout.json, which is created with default square
    corners the first time boards are seen and can be edited while running.
    """

    GRID_M = 0.1
    MARGIN_M = 0.75
    ELLIPSE_M = 0.35   # excess path length scale of a link's sensing ellipse
    ACTIVE = 0.8       # link ratio above which a link counts as disturbed

    def __init__(self, registry: LinkRegistry, layout_path: Path, area_m: float, positions_fn=None,
                 background: bool = True):
        self.registry = registry
        self.layout_path = layout_path
        self.area_m = area_m
        self.positions_fn = positions_fn  # (now) -> {mac: [x, y]} from floor plan + robots, or None
        self.lock = threading.Lock()
        self.positions: dict[str, list[float]] = {}
        self._layout_mtime = None
        self.result: dict = {"positions": {}, "pairs": [], "person": None, "grid": None, "obs_field": None}
        self._track = PositionKalman()
        if background:                    # off for tests and replays, which call step() themselves
            threading.Thread(target=self._run, name="localizer", daemon=True).start()

    def _defaults(self, i: int) -> list[float]:
        s = self.area_m
        corners = [[0.0, 0.0], [s, 0.0], [s, s], [0.0, s], [s / 2, 0.0], [s, s / 2], [s / 2, s], [0.0, s / 2]]
        return corners[i] if i < len(corners) else [s / 2, s / 2]

    def _sync_layout(self, macs: set[str]):
        try:
            mtime = self.layout_path.stat().st_mtime
        except FileNotFoundError:
            mtime = None
        if mtime is not None and mtime != self._layout_mtime:
            try:
                data = json.loads(self.layout_path.read_text())
                self.positions = {k.lower(): [float(v[0]), float(v[1])] for k, v in data.get("boards", {}).items()}
                self._layout_mtime = mtime
            except (ValueError, KeyError, TypeError, IndexError):
                pass  # half-written or invalid edit: keep the previous layout
        new = sorted(m for m in macs if m not in self.positions)
        if new:
            for m in new:
                self.positions[m] = self._defaults(len(self.positions))
            body = {"comment": "Board positions in metres (x, y). Edit to match where the boards are.",
                    "boards": self.positions}
            self.layout_path.write_text(json.dumps(body, indent=2))
            self._layout_mtime = self.layout_path.stat().st_mtime

    def _ellipses(self, pairs, X, Y):
        active = np.zeros_like(X)
        quiet = np.zeros_like(X)
        for p in pairs:
            (ax, ay), (bx, by), r = p["a_pos"], p["b_pos"], p["ratio"]
            d = np.hypot(X - ax, Y - ay) + np.hypot(X - bx, Y - by) - np.hypot(bx - ax, by - ay)
            shape = np.exp(-(d / self.ELLIPSE_M) ** 2)
            if r >= self.ACTIVE:
                active += (r - self.ACTIVE) * shape
            else:
                quiet += min(1.0, self.ACTIVE - r) * shape
        return active - 0.5 * quiet * (active.max() if active.size else 0.0)

    def step(self, now: float):
        snaps = [l.snapshot(now) for l in self.registry.all()]
        live = [s for s in snaps if s["rate"] > 0]
        live_pos = self.positions_fn(now) if self.positions_fn else None
        if live_pos is not None:
            pos = live_pos                   # floor plan + live robot positions
        else:
            self._sync_layout({s["tx"] for s in live} | {s["rx"] for s in live})
            pos = dict(self.positions)

        detector = getattr(getattr(self.registry, "settings", None), "detector", "espectre")
        fused = fuse_pairs([s for s in live if s["tx"] in pos and s["rx"] in pos], detector)
        pairs = []
        for key, f in fused.items():
            a, b = sorted(key)
            pairs.append({"a": a, "b": b, "a_pos": pos[a], "b_pos": pos[b], "ratio": f["ratio"],
                          "detected": f["detected"]})

        result = {"positions": pos, "pairs": pairs, "person": None, "grid": None, "obs_field": None}
        used = [pos[m] for m in {p["a"] for p in pairs} | {p["b"] for p in pairs}]
        if pairs and used:
            xs, ys = [p[0] for p in used], [p[1] for p in used]
            x0, x1 = min(xs) - self.MARGIN_M, max(xs) + self.MARGIN_M
            y0, y1 = min(ys) - self.MARGIN_M, max(ys) + self.MARGIN_M
            gx = np.arange(x0, x1 + 1e-9, self.GRID_M)
            gy = np.arange(y0, y1 + 1e-9, self.GRID_M)
            X, Y = np.meshgrid(gx, gy)
            img = self._ellipses(pairs, X, Y)
            peak = float(img.max())
            agree = pairs_agree(sum(p["detected"] for p in pairs), len(pairs), agree_pairs_needed(detector))
            if agree and peak > 0:
                w = np.where(img >= 0.6 * peak, img, 0.0)
                est = np.array([(w * X).sum() / w.sum(), (w * Y).sum() / w.sum()])
                result["person"] = [round(v, 2) for v in self._track.update(est, now)]
            norm = np.clip(img / max(peak, 1e-6), 0.0, 1.0) if peak > 0 else np.zeros_like(img)
            result["grid"] = {"x0": round(x0, 3), "y0": round(y0, 3), "step": self.GRID_M,
                              "nx": int(gx.size), "ny": int(gy.size),
                              "values": [round(float(v), 3) for v in norm.ravel()]}
            # Same image on the RuView Observatory floor: 20 x 20 cells of 0.5 m, centred on the boards.
            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            ox = cx + (np.arange(20) - 10) * 0.5
            oy = cy + (np.arange(20) - 10) * 0.5
            OX, OY = np.meshgrid(ox, oy)
            o = self._ellipses(pairs, OX, OY)
            result["obs_field"] = [round(float(v), 3) for v in np.clip(o / max(peak, 1e-6), 0, 1).ravel()] \
                if peak > 0 else [0.0] * 400
            result["center"] = [round(cx, 3), round(cy, 3)]
        with self.lock:
            self.result = result

    def _run(self):
        while True:
            try:
                self.step(time.time())
            except Exception as e:  # keep localizing even if one step fails
                print(f"[localizer] {e!r}", flush=True)
            time.sleep(0.2)

    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.result)


class Node:
    """One ESP32-C5 node on a serial port."""

    def __init__(self, port: str, settings: Settings, record: bool, registry: LinkRegistry):
        self.port = port
        self.settings = settings
        self.record = record
        self.registry = registry
        self.lock = threading.Lock()
        self.mac = None
        self.peer = None
        self.peers: list[dict] = []
        self.channel = None
        self.stat = {}
        self.connected = False
        self.edge = None
        self.edge_rx_t = 0.0
        self._last_dev_us = None   # gateway clock of the latest CSIA line, for relayed lines
        self.edge_calib = None
        self.boot_wall = None
        self.logs = collections.deque(maxlen=200)
        self.log_seq = 0
        self.reset_pending = False
        self._rec = None
        self._stop = False
        self.thread = threading.Thread(target=self._run, name=f"node-{port}", daemon=True)

    def start(self):
        self.thread.start()

    def request_reset(self):
        self.reset_pending = True

    def _log(self, text: str):
        with self.lock:
            self.log_seq += 1
            self.logs.append((self.log_seq, time.time(), text))

    def _open(self) -> serial.Serial:
        # Native USB Serial/JTAG: DTR/RTS drive reset and boot strapping.
        # Keep both deasserted so opening the port does not reset the chip.
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = 115200
        ser.timeout = 0.5
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    def _pulse_reset(self, ser: serial.Serial):
        # Windows usbser.sys only sends the line state when DTR is written,
        # so re-write DTR after each RTS change (same trick as esptool).
        ser.rts = True
        ser.dtr = False
        time.sleep(0.12)
        ser.rts = False
        ser.dtr = False
        if self.mac:
            self.registry.reset(rx=self.mac)
        with self.lock:
            self.edge = None
            self.edge_calib = None
            self.boot_wall = time.time()
        self._log("[host] node reset - recalibrating")

    def _run(self):
        opens = 0
        while not self._stop:
            try:
                ser = self._open()
            except (serial.SerialException, OSError) as e:
                self.connected = False
                self._log(f"[host] cannot open {self.port}: {e}")
                time.sleep(2.0)
                continue
            self.connected = True
            opens += 1
            self._log(f"[host] opened {self.port}")
            if opens > 1:
                # Reconnected (e.g. replugged): a board that rebooted with no host
                # reading was seen to stay silent on its USB console until reset.
                self.reset_pending = True
            buf = b""
            try:
                while not self._stop:
                    if self.reset_pending:
                        self.reset_pending = False
                        self._pulse_reset(ser)
                        buf = b""
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            self._handle(line.decode("ascii", "replace").strip())
                        except Exception as e:  # never let one bad line kill the reader
                            self._log(f"[host] error processing line: {e!r}")
            except (serial.SerialException, OSError) as e:
                self._log(f"[host] {self.port} disconnected: {e}")
            finally:
                self.connected = False
                try:
                    ser.close()
                except Exception:
                    pass
            time.sleep(1.0)

    def _record(self, line: str):
        if not self.record:
            return
        if self._rec is None:
            RECORD_DIR.mkdir(exist_ok=True)
            name = f"{datetime.now():%Y%m%d_%H%M%S}_{self.port}.log"
            self._rec = open(RECORD_DIR / name, "a", buffering=1)
        self._rec.write(f"{time.time():.4f},{line}\n")

    def _handle(self, line: str):
        if not line:
            return
        now = time.time()
        if line.startswith("CSIA,"):
            # Own measurement of the link tx -> this node (multi-peer firmware).
            self._record(line)
            p = line.split(",", 7)
            if len(p) != 8 or not self.mac:
                return
            try:
                amp = np.frombuffer(bytes.fromhex(p[7]), dtype=np.uint8).astype(np.float32)
                rssi = int(p[4])
                dev_ts = int(p[3])
            except ValueError:
                return
            self._last_dev_us = dev_ts
            if amp.size >= 8:
                self.registry.get(p[1], self.mac).push(now, amp, rssi, device_ts_us=dev_ts)
        elif line.startswith("RLY,"):
            # Another node's measurement of one of its links, relayed in its beacon.
            self._record(line)
            p = line.split(",", 6)
            if len(p) != 7:
                return
            try:
                amp = np.frombuffer(bytes.fromhex(p[6]), dtype=np.uint8).astype(np.float32)
                seq, rssi = int(p[3]), int(p[4])
            except ValueError:
                return
            if amp.size >= 8:
                # The gateway prints relayed and own lines through one queue in event order, so the
                # previous CSIA line's clock times this one; host arrival jitters 40 ms (p99 250 ms).
                self.registry.get(p[2], p[1]).push(now, amp, rssi, relay_seq=seq,
                                                   device_ts_us=self._last_dev_us)
        elif line.startswith("CSI,"):
            # Single-peer firmware: raw I/Q of the one peer named in STAT.
            self._record(line)
            parts = line.split(",", 7)
            if len(parts) != 8 or not self.mac or not self.peer:
                return
            try:
                rssi = int(parts[3])
                first_invalid = parts[5] == "1"
                raw = np.frombuffer(bytes.fromhex(parts[7]), dtype=np.int8).astype(np.float32)
            except ValueError:
                return
            if raw.size < 8 or raw.size % 2:
                return
            iq = raw.reshape(-1, 2)
            amp = np.sqrt(iq[:, 0] ** 2 + iq[:, 1] ** 2)
            if first_invalid:
                amp[:2] = 0.0
            self.registry.get(self.peer, self.mac).push(now, amp, rssi, device_ts_us=int(parts[2]))
        elif line.startswith("PEERS,"):
            peers = []
            for item in line.split(",")[2:]:
                f = item.split(":")
                if len(f) == 8:
                    try:
                        peers.append({"mac": ":".join(f[:6]), "csi": int(f[6]), "rssi": int(f[7])})
                    except ValueError:
                        pass
            with self.lock:
                self.peers = peers
        elif line.startswith("EDGE,"):
            self._record(line)
            p = line.split(",")
            if len(p) < 10:
                return
            try:
                flags = int(p[1])
                ts_ms = int(p[8])
                edge = {
                    "presence": bool(flags & 0x01),
                    "fall": bool(flags & 0x02),
                    "motion_flag": bool(flags & 0x04),
                    "motion_energy": float(p[2]),
                    "presence_score": float(p[3]),
                    "persons": int(p[4]),
                    "breathing_bpm": float(p[5]),
                    "heart_bpm": float(p[6]),
                    "rssi": int(p[7]),
                    "uptime_s": ts_ms / 1000.0,
                    "dsp_hz": float(p[9]),
                }
            except ValueError:
                return
            with self.lock:
                self.edge = edge
                self.edge_rx_t = now
        elif line.startswith("HELLO,"):
            p = line.split(",")
            if len(p) >= 3:
                with self.lock:
                    self.mac = p[1]
                    self.channel = int(p[2]) if p[2].isdigit() else None
        elif line.startswith("STAT,"):
            p = line.split(",")
            if len(p) >= 9:
                with self.lock:
                    self.mac = p[1]
                    self.peer = None if p[2] == "none" else p[2]
                    self.stat = {
                        "tx_ok": int(p[4]), "tx_fail": int(p[5]),
                        "beacons_rx": int(p[6]), "csi_rx": int(p[7]), "csi_dropped": int(p[8]),
                    }
        elif line.startswith("FEAT,"):
            self._record(line)
        else:
            m = CALIB_DONE_RE.search(line)
            if m:
                with self.lock:
                    self.edge_calib = {"mean": float(m.group(1)), "sigma": float(m.group(2)),
                                       "threshold": float(m.group(3))}
            self._log(line)

    def snapshot(self, now: float, log_after: int) -> dict:
        # Per-node dashboard view: this node's link from its first peer.
        link = self.registry.find(self.peer, self.mac)
        ls = link.snapshot(now) if link else None
        det = ls["detector"] if ls else dict(EMPTY_DETECTOR)
        rate = ls["rate"] if ls else 0.0
        with self.lock:
            edge = dict(self.edge) if self.edge else None
            edge_age = now - self.edge_rx_t if self.edge else None
            if edge is not None:
                # RuView's 1200-frame calibration runs at the measured DSP rate (~14 Hz on C5).
                calib_s = RUVIEW_CALIB_FRAMES / max(edge["dsp_hz"], 8.0) + 5.0
                edge["calib_s"] = calib_s
                edge["calibrated"] = self.edge_calib is not None or edge["uptime_s"] > calib_s
                edge["calibration"] = self.edge_calib
                edge["stale"] = edge_age is not None and edge_age > 3.0
            logs = [{"seq": s, "t": t, "text": x} for (s, t, x) in self.logs if s > log_after]
            return {
                "port": self.port,
                "connected": self.connected,
                "mac": self.mac,
                "peer": self.peer,
                "peers": list(self.peers),
                "channel": self.channel,
                "stat": dict(self.stat),
                "csi_rate": rate,
                "rssi": ls["rssi"] if ls else None,
                "detector": det,
                "edge": edge,
                "row": link.detector.display_row() if link and rate > 0 else None,
                "logs": logs,
            }


class App:
    def __init__(self, ports: list[str], settings: Settings, record: bool, reset_on_start: bool,
                 link_length_m: float = 3.0, area_m: float = 3.0, floorplan_path: Path | None = None,
                 rooms_loop: bool = True):
        self.settings = settings
        self.link_length_m = link_length_m
        self.floorplan = FloorPlanWatcher(floorplan_path or FLOORPLAN_PATH)
        self.tracker = RobotTracker()
        self._evidence: RoomEvidence | None = None
        self._evidence_lock = threading.Lock()
        if rooms_loop:
            threading.Thread(target=self._rooms_loop, name="rooms", daemon=True).start()
        self.registry = LinkRegistry(settings, still=self.is_still)
        self.localizer = Localizer(self.registry, LAYOUT_PATH, area_m, positions_fn=self.board_positions)
        self.nodes = [Node(p, settings, record, self.registry) for p in ports]
        for n in self.nodes:
            n.start()
            if reset_on_start:
                n.request_reset()

    def post_robots(self, robots: list[dict]):
        """Validate every entry before storing any (ValueError -> HTTP 400 with the reason)."""
        for i, r in enumerate(robots):
            if not isinstance(r, dict):
                raise ValueError(f"robots[{i}] must be an object")
            if not isinstance(r.get("mac"), str) or not r["mac"].strip():
                raise ValueError(f"robots[{i}].mac is missing: send the sensing board's MAC")
            for k in ("x", "y"):
                v = r.get(k)
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                    raise ValueError(f"robots[{i}].{k} must be a finite number of metres, got {v!r}")
        self.tracker.update(robots, now=time.time())

    def is_still(self, mac: str, now: float) -> bool:
        """Whether link data touching this board can be trusted right now."""
        if self.tracker.position(mac, now) is not None:
            return self.tracker.is_stopped(mac, now)       # live robot: must be stopped
        if self.tracker.tracked(mac):
            plan, _ = self.floorplan.current()
            return bool(plan and mac in plan.boards)        # stale robot: only if parked in the plan
        return True                                         # fixed or unknown boards don't move

    def board_positions(self, now: float) -> dict | None:
        """Floor-plan boards, overridden by fresh robot positions; None without a floor plan."""
        plan, _ = self.floorplan.current()
        if plan is None:
            return None
        pos = {mac: list(p) for mac, p in plan.boards.items()}
        for r in self.tracker.snapshot(now):
            if r["fresh"]:
                pos[r["mac"]] = [r["x"], r["y"]]
        return pos

    def stops(self) -> list[dict]:
        plan, _ = self.floorplan.current()
        return plan.to_dict()["stops"] if plan else []

    ROOMS_DT = 0.25

    def _evidence_for(self, plan) -> RoomEvidence:
        with self._evidence_lock:
            if self._evidence is None or self._evidence.plan is not plan:
                self._evidence = RoomEvidence(plan)   # new or edited floor plan: start over
            return self._evidence

    def feed_rooms(self, now: float, dt: float):
        """Add one window of every stationary, positioned board pair.

        Each pair's two directions are combined as fuse_pairs() does.
        """
        plan, _ = self.floorplan.current()
        if plan is None:
            return
        ev = self._evidence_for(plan)
        pos = self.board_positions(now) or {}
        snaps = []
        for link in self.registry.all():
            s = link.snapshot(now)
            tx, rx = s["tx"], s["rx"]
            if s["rate"] <= 0 or s["detector"]["ratio"] is None or tx not in pos or rx not in pos:
                continue
            if not (self.is_still(tx, now) and self.is_still(rx, now)):
                continue
            snaps.append(s)
        detector = self.settings.detector
        fused = fuse_pairs(snaps, detector)
        agree = pairs_agree(sum(f["detected"] for f in fused.values()), len(fused), agree_pairs_needed(detector))
        for key, f in fused.items():
            a, b = sorted(key)
            ev.add(tuple(pos[a]), tuple(pos[b]), f["ratio"], now, dt, detected=f["detected"] and agree)

    def _rooms_loop(self):
        while True:
            try:
                self.feed_rooms(time.time(), self.ROOMS_DT)
            except Exception as e:  # keep scanning even if one step fails
                print(f"[rooms] {e!r}", flush=True)
            time.sleep(self.ROOMS_DT)

    def _stop_done(self, plan, robot: dict) -> bool:
        if not robot["stopped"] or robot["x"] is None:
            return False
        listen = 15.0
        best = 0.5
        for s in plan.stops if plan else []:
            for p in (s.a, s.b):
                d = math.hypot(p[0] - robot["x"], p[1] - robot["y"])
                if d < best:
                    best, listen = d, s.listen_s
        settle = RobotTracker.STILL_WINDOW_S + RobotTracker.SETTLE_S
        return robot["stop_s"] - settle >= listen

    def scan(self) -> dict:
        now = time.time()
        plan, err = self.floorplan.current()
        rooms = self._evidence_for(plan).status(now) if plan else []
        warnings = [err] if err else []
        if plan is not None:
            heard = {m for l in self.registry.all() if now - getattr(l, "last_rx_t", 0.0) <= 2.0
                     for m in (l.tx, l.rx)}
            warnings += self.tracker.warnings(heard=heard, fixed=set(plan.boards), now=now)
        robots = self.tracker.snapshot(now)
        for r in robots:
            r["stop_done"] = self._stop_done(plan, r)
        detections = [f"Person likely in {r['name']} ({round(r['confidence'] * 100)}%)"
                      for r in rooms if r["status"] == "occupied"]
        return {"rooms": rooms, "robots": robots, "detections": detections, "warnings": warnings}

    def recalibrate(self, reset_boards: bool):
        self.registry.reset()
        with self._evidence_lock:
            self._evidence = None
        if reset_boards:
            for n in self.nodes:
                n.request_reset()

    def state(self, log_after: dict[str, int] | None = None) -> dict:
        now = time.time()
        log_after = log_after or {}
        nodes = [n.snapshot(now, log_after.get(n.port, 0)) for n in self.nodes]
        links = [l for l in (link.snapshot(now) for link in self.registry.all()) if l["rate"] > 0]
        s = self.settings.as_dict()

        detector = self.settings.detector
        fused = fuse_pairs(links, detector, include_static=s["include_static"])
        host_hit = pairs_agree(sum(f["detected"] for f in fused.values()), len(fused),
                               agree_pairs_needed(detector))
        ruview_hit = any(n["edge"] and n["edge"]["calibrated"] and not n["edge"]["stale"] and n["edge"]["presence"]
                         for n in nodes)
        src = s["banner_source"]
        detected = (host_hit if src == "host" else ruview_hit if src == "ruview" else (host_hit or ruview_hit))

        ratios = [f["ratio"] for f in fused.values()]
        confidence = 0.0
        if ratios:
            confidence = float(np.clip((max(ratios) - 0.6) / 1.4, 0.0, 1.0))
        if detected and confidence < 0.5:
            confidence = 0.5 if src != "host" else max(confidence, 0.5)

        pending = [l["detector"] for l in links if not l["detector"]["calibrated"]]
        # ESPectre links report calib_left None while not ready (no calibration countdown).
        calibrating = [d["calib_left"] for d in pending if d["calib_left"] is not None]
        warmup = [d["warmup_left"] for d in pending if d["warmup_left"] is not None]
        linked = any(l["rate"] > 5 for l in links)
        loc = self.localizer.snapshot()
        return {
            "t": now,
            "settings": s,
            "nodes": nodes,
            "links": [{k: v for k, v in l.items() if k != "detector"} | {
                "ratio": l["detector"]["ratio"], "detected": l["detector"]["detected"],
                "calibrated": l["detector"]["calibrated"]} for l in links],
            "localization": loc,
            "floorplan": plan.to_dict() if (plan := self.floorplan.current()[0]) else None,
            "scan": self.scan(),
            "summary": {
                "detected": bool(detected),
                "host_detected": bool(host_hit),
                "ruview_detected": bool(ruview_hit),
                "confidence": confidence,
                "calibrating": max(calibrating) if calibrating else None,
                "warmup": max(warmup) if warmup else None,
                "linked": linked,
            },
        }


def make_handler(app: App):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            print(f"[http] {self.client_address[0]} {fmt % args}", flush=True)

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _ruview_static(self, rel: str):
            self._serve_file(RUVIEW_UI_DIR, rel)

        def _serve_file(self, root_dir: Path, rel: str):
            root = root_dir.resolve()
            target = (root / rel).resolve()
            if not target.is_relative_to(root) or not target.is_file():
                self._send(404, b"not found", "text/plain")
                return
            ctype = CONTENT_TYPES.get(target.suffix.lower(), "application/octet-stream")
            body = target.read_bytes()
            if root_dir == RUVIEW_UI_DIR and rel == "observatory.html":
                body = body.replace(b"<head>", b"<head>" + OBSERVATORY_PROBE_SHIM, 1)
            self._send(200, body, ctype)

        def _ws_sensing(self):
            # RuView sensing-server stream (RFC 6455), consumed by the 3D Observatory.
            key = self.headers.get("Sec-WebSocket-Key")
            if not key or "websocket" not in (self.headers.get("Upgrade") or "").lower():
                self._send(400, b"websocket upgrade required", "text/plain")
                return
            self.send_response(101, "Switching Protocols")
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", ruview_bridge.ws_accept_key(key))
            self.end_headers()
            self.close_connection = True
            no_logs = {n.port: float("inf") for n in app.nodes}
            try:
                while True:
                    frame = ruview_bridge.build_frame(app.state(no_logs), app.link_length_m)
                    self.wfile.write(ruview_bridge.ws_text_frame(json.dumps(frame)))
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                return

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/health":
                self._send(200, b'{"status":"ok","source":"esp32c5-presence"}', "application/json")
                return
            if path == "/3d":
                self._send(200, (STATIC_DIR / "scene3d.html").read_bytes(), "text/html; charset=utf-8")
                return
            if path == "/mission":
                self._send(200, (STATIC_DIR / "mission.html").read_bytes(), "text/html; charset=utf-8")
                return
            if path == "/ruview-3d":
                self.send_response(302)
                self.send_header("Location", "/ruview/observatory.html")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            if path == "/ws/sensing":
                self._ws_sensing()
                return
            if path.startswith("/ruview/"):
                self._ruview_static(path[len("/ruview/"):])
                return
            if path.startswith("/static/"):
                self._serve_file(STATIC_DIR, path[len("/static/"):])   # incl. offline three.js
                return
            if path == "/api/stops":
                self._send(200, json.dumps({"stops": app.stops()}).encode(), "application/json")
                return
            if path == "/api/scan":
                self._send(200, json.dumps(app.scan()).encode(), "application/json")
                return
            if self.path in ("/", "/index.html"):
                self._send(200, (STATIC_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, json.dumps(app.state()).encode(), "application/json")
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                log_after: dict[str, int] = {}
                try:
                    while True:
                        st = app.state(log_after)
                        for n in st["nodes"]:
                            if n["logs"]:
                                log_after[n["port"]] = n["logs"][-1]["seq"]
                        self.wfile.write(b"data: " + json.dumps(st).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    return
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                body = None
            if self.path == "/api/robots":
                try:
                    if not isinstance(body, dict) or not isinstance(body.get("robots"), list):
                        raise ValueError('expected a JSON object {"robots": [...]}')
                    app.post_robots(body["robots"])
                except ValueError as e:
                    self._send(400, json.dumps({"error": f"bad robots payload: {e}"}).encode(), "application/json")
                    return
                self._send(200, b'{"ok":true}', "application/json")
                return
            body = body if isinstance(body, dict) else {}
            if self.path == "/api/recalibrate":
                app.recalibrate(bool(body.get("reset_boards", True)))
                self._send(200, b'{"ok":true}', "application/json")
            elif self.path == "/api/settings":
                s = app.settings
                with s.lock:
                    if "sensitivity" in body:
                        s.sensitivity = float(np.clip(float(body["sensitivity"]), 0.25, 4.0))
                    if "include_static" in body:
                        s.include_static = bool(body["include_static"])
                    if body.get("banner_source") in ("host", "ruview", "either"):
                        s.banner_source = body["banner_source"]
                self._send(200, json.dumps(s.as_dict()).encode(), "application/json")
            else:
                self._send(404, b"not found", "text/plain")

    return Handler


def lan_ip() -> str:
    """This machine's address on the local network (no packet is sent)."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def find_ports() -> list[str]:
    return sorted(p.device for p in serial.tools.list_ports.comports() if p.vid == ESPRESSIF_VID)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ports", nargs="*", help="serial ports (default: every Espressif USB device)")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1",
                    help="address to listen on; 0.0.0.0 lets other computers (robot team) reach the API")
    ap.add_argument("--calib-seconds", type=float, default=10.0,
                    help="host detector ambient calibration time (keep the link clear)")
    ap.add_argument("--no-reset", action="store_true", help="do not reset the boards on start")
    ap.add_argument("--record", action="store_true", help=f"save raw CSI/EDGE lines under {RECORD_DIR}")
    ap.add_argument("--link-length", type=float, default=3.0,
                    help="distance between the two boards in metres (3D view layout)")
    ap.add_argument("--detector", choices=("wander", "espectre", "denoised"), default="wander",
                    help="per-link motion detector: CSI shape change against the link's own calm level "
                         "(wander), ESPectre's model (no calibration), or the denoised score")
    ap.add_argument("--area-size", type=float, default=3.0,
                    help=f"side of the default square board layout in metres (edit {LAYOUT_PATH.name} to change)")
    args = ap.parse_args()

    ports = args.ports or find_ports()
    if not ports:
        raise SystemExit("No Espressif boards found. Plug them in or pass --ports.")
    print(f"Nodes: {', '.join(ports)}")

    app = App(ports, Settings(args.calib_seconds, detector=args.detector), args.record,
              reset_on_start=not args.no_reset,
              link_length_m=args.link_length, area_m=args.area_size)
    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(app))
    server.daemon_threads = True
    print(f"Dashboard: http://localhost:{args.http_port}")
    if args.host != "127.0.0.1":
        print(f"Robot team API: http://{lan_ip()}:{args.http_port}/api/robots  (stops: /api/stops, scan: /api/scan)")
    print(f"3D view:   http://localhost:{args.http_port}/3d")
    print(f"RuView 3D: http://localhost:{args.http_port}/ruview-3d")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    os.chdir(Path(__file__).resolve().parent)
    main()
