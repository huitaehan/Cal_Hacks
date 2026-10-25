"""
Per-link motion detector: CSI shape "wander", judged against the link's own quiet level.

Each packet's amplitudes are compared with the average shape of the last 2 s
(1 - Pearson correlation, the measure behind Espressif esp-radar's "wander").
Correlation ignores how loud the packet is, so AGC gain steps don't count as
motion. Links differ a lot in how noisy they are at rest, so the score is
log(wander / the link's 5th percentile over the last 5 minutes): 0 means as calm
as this link gets, and 1.4 (4x its calm level) counts as motion.

On the labelled live test 2 (replayed), one board pair above 1.4 for 0.5 s
caught 99 % of the time someone walked inside, against 26 % for someone walking
0.5-1 m outside and 1 % with nobody near (1.8, 6x: 94 %, 6 %, 0 %; lowered on
2026-10-25 because small motions such as waving while crouched were being missed). ESPectre with two agreeing pairs: 86 %,
48 % and 0 %. ESPectre's model was trained at 100 packets/s and our links run at
20-44; packet rate barely matters here (thinning 44 -> 22 pps: AUC 0.90 -> 0.88).
"""

from __future__ import annotations

import collections
import math
import threading

import numpy as np

__all__ = ["WanderLinkDetector"]


class WanderLinkDetector:
    """Same push/update/snapshot surface as presence_server.LinkDetector."""

    WINDOW_S = 2.0          # shape window
    EVAL_S = 0.25           # one evaluation per 250 ms
    MIN_SAMPLES = 5         # per window
    TARGET_RATE = 14.0      # packets/s kept: a faster link keeps every k-th packet, so the gateway's own links
                            # (40-46/s) are judged like relayed ones (about 16/s). On every packet they read numb:
                            # Oct 25 front edge fired 3 % of the time vs 17-24 %; every 3rd packet, 10 %.
    BASELINE_S = 300.0      # history the calm level is taken from
    BASELINE_PCT = 5
    BASELINE_MIN = 20       # evaluations (5 s) before scores are reported
    REFRESH_EVALS = 8       # recompute the calm level every 2 s
    THRESHOLD = 1.4         # log(wander / calm level): 4x the link's calm level
    HOLD_EVALS = 2          # the score must hold for 2 evaluations (0.5 s)
    VALID_AMP = 2.0         # subcarriers averaging below this are nulls

    def __init__(self):
        self.lock = threading.Lock()
        self.reset_calibration()

    def reset_calibration(self):
        with self.lock:
            self._t = collections.deque()
            self._a = collections.deque()
            self._arrivals = collections.deque()   # every packet's arrival time, for the link's rate
            self._seen = 0
            self._keep_every = 1
            self._hist = collections.deque(maxlen=int(self.BASELINE_S / self.EVAL_S))
            self._recent = collections.deque(maxlen=self.HOLD_EVALS)
            self._baseline = None
            self._since_refresh = 0
            self.last_eval = float("-inf")
            self.wander = None
            self.score = None

    def push(self, t_s: float, amp, device_ts_us: int | None = None):
        a = np.asarray(amp, dtype=np.float32)
        if a.size < 8:
            return
        t = device_ts_us / 1e6 if device_ts_us is not None else t_s
        with self.lock:
            self._arrivals.append(t_s)
            while self._arrivals[0] < t_s - self.WINDOW_S:
                self._arrivals.popleft()
            ideal = len(self._arrivals) / self.WINDOW_S / self.TARGET_RATE    # 38-48/s: 3; relayed: 1
            if abs(ideal - self._keep_every) > 0.75:     # sticky: a pick that keeps shifting adds noise
                self._keep_every = max(1, int(ideal + 0.5))
            self._seen += 1
            if self._seen % self._keep_every:
                return
            if self._t and (t < self._t[-1] - 1.0 or (self._a and a.size != self._a[-1].size)):
                self._t.clear()          # board clock restarted, or a different CSI layout
                self._a.clear()
            self._t.append(t)
            self._a.append(a)
            while self._t and self._t[0] < t - self.WINDOW_S:
                self._t.popleft()
                self._a.popleft()

    def _window_wander(self) -> float | None:
        if len(self._a) < self.MIN_SAMPLES:
            return None
        A = np.array(self._a)
        A = A[:, A.mean(axis=0) > self.VALID_AMP]
        if A.shape[1] < 8:
            return None
        A = A / np.maximum(A.mean(axis=1, keepdims=True), 1e-6)
        m = A.mean(axis=0)
        Ac, mc = A - A.mean(axis=1, keepdims=True), m - m.mean()
        corr = (Ac @ mc) / np.sqrt((Ac ** 2).sum(axis=1) * (mc ** 2).sum() + 1e-12)
        return float(np.mean(1.0 - corr))

    def update(self, now_s: float):
        with self.lock:
            if now_s - self.last_eval < self.EVAL_S:
                return
            self.last_eval = now_s
            w = self._window_wander()
            self.wander = w
            if w is None:
                self._recent.clear()
                self.score = None
                return
            self._hist.append(w)
            self._since_refresh += 1
            if len(self._hist) >= self.BASELINE_MIN and (self._baseline is None or
                                                         self._since_refresh >= self.REFRESH_EVALS):
                self._baseline = float(np.percentile(self._hist, self.BASELINE_PCT))
                self._since_refresh = 0
            if not self._baseline or self._baseline <= 0:
                self.score = None
                return
            self._recent.append(math.log(max(w, 1e-9) / self._baseline))
            self.score = min(self._recent)          # held for HOLD_EVALS evaluations

    def snapshot(self, now_s: float) -> dict:
        with self.lock:
            ready = self.score is not None
            left = max(0.0, (self.BASELINE_MIN - len(self._hist)) * self.EVAL_S)
            return {
                "calibrated": ready, "calib_left": None if ready else left, "warmup_left": None,
                "motion": self.wander, "threshold": self.THRESHOLD,
                "ratio": max(0.0, self.score) / self.THRESHOLD if ready else None,
                "score": self.score,
                "static_dev": 0.0, "static_threshold": None, "static_ratio": None,
                "detected": bool(ready and self.score > self.THRESHOLD), "static_detected": False,
                "n_subcarriers": int(self._a[-1].size) if self._a else 0, "subcarrier_cv": None,
            }

    def display_row(self) -> list[float] | None:
        with self.lock:
            if not self._a:
                return None
            row = np.mean(list(self._a)[-5:], axis=0)
        m = row[row > 0].mean() if (row > 0).any() else 1.0
        return [round(float(x / m), 3) for x in row]
