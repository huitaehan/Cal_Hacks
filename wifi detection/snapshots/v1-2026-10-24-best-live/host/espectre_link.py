"""
Per-link motion detector backed by ESPectre's calibration-free "High Accuracy" model.

ESPectre (github.com/francescopace/espectre, GPLv3) lives unmodified under
third_party/espectre and is imported by path. Its own code converts our C5
57-subcarrier HT20 CSI to its 64-bin layout, assigns packets to fixed time slots
(TemporalReplayController) and runs the 529-parameter MLP. No quiet-room
calibration: the decision threshold is the trained 0.5.

Our multi-peer firmware sends amplitudes only; those are packed as (imag=0,
real=amplitude) pairs, which ESPectre's features read identically because none
of them uses phase.
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import numpy as np

ESPECTRE_ROOT = Path(__file__).resolve().parent.parent / "third_party" / "espectre"
if str(ESPECTRE_ROOT) not in sys.path:
    sys.path.insert(0, str(ESPECTRE_ROOT))
import tools.lib.bootstrap  # noqa: E402,F401  (puts ESPectre's source dirs on sys.path)
from micro_espectre.device_utils import normalize_ht20_csi_payload, prepare_ht20_detector_input  # noqa: E402
from tools.lib.high_accuracy_detector import HighAccuracyDetector  # noqa: E402
from tools.lib.temporal_csi_sampler import minimum_valid_slots, temporal_window_slots  # noqa: E402
from tools.lib.temporal_replay import TemporalReplayController, apply_temporal_admission  # noqa: E402

__all__ = ["EspectreLinkDetector", "amplitudes_to_csi_bytes", "espectre_config", "espectre_target_pps",
           "normalize_ht20_csi_payload"]

SLOT_RATES = (100, 50, 25, 20, 10)


SLOT_FILL = 0.6


def espectre_target_pps(packet_rate: float) -> int:
    """Largest slot rate the link can keep >= 70 % full: <= 0.6 x its real packet rate.

    Replays showed 16-25 pps links never complete ESPectre's debounce at 25
    slots/s (readiness flickers) but catch walks cleanly at 20. In the live
    4-board mesh packets arrive irregularly, and slots near 0.9 x the rate left
    ~30 % of them empty; at 0.6 x links stayed 95-98 % ready.
    """
    for pps in SLOT_RATES:
        if pps <= SLOT_FILL * packet_rate:
            return pps
    return SLOT_RATES[-1]


SLOW_LINK_PPS = 15.0


def espectre_config(packet_rate: float) -> tuple[int, int]:
    """(slot rate, window ms) for a link. Relayed links run at ~9-12 pps live and were ready
    only ~55 % of the time at 10 slots / 1 s; 5 slots / 2 s kept them ~96 % ready. Thinned
    replays: at ~6 pps it had no false motion, at ~8 pps 1.8 s of it in the quiet windows,
    and it can miss weak motion."""
    if packet_rate < SLOW_LINK_PPS:
        return 5, 2000
    return espectre_target_pps(packet_rate), 1000


def amplitudes_to_csi_bytes(amp) -> bytes:
    """Pack amplitudes as ESP32 CSI (imag, real) int8 pairs with imag = 0."""
    a = np.clip(np.rint(np.asarray(amp, dtype=np.float32)), 0, 127).astype(np.uint8)
    out = bytearray(2 * a.size)
    out[1::2] = a.tobytes()
    return bytes(out)


class EspectreLinkDetector:
    """Same push/update/snapshot surface as presence_server.LinkDetector."""

    THRESHOLD = 0.5
    ROW_FRAMES = 5
    ENTER_EVALS = 4   # ESPectre's firmware rule: 4 consecutive MOTION evaluations to enter motion
    EXIT_EVALS = 3    # and 3 consecutive IDLE evaluations to leave it

    def __init__(self, target_pps: int = 100, window_ms: int = 1000, eval_ms: int = 250):
        self.target_pps = int(target_pps)
        self.window_ms = int(window_ms)
        self.eval_s = eval_ms / 1000.0
        self.lock = threading.Lock()
        self._rows: list[np.ndarray] = []
        self.reset_calibration()

    def reset_calibration(self):
        with self.lock:
            slots = temporal_window_slots(self.target_pps, self.window_ms)
            self.controller = TemporalReplayController(self.target_pps, self.window_ms)
            self.det = HighAccuracyDetector(window_size=slots)
            self.det.set_minimum_valid_samples(minimum_valid_slots(slots))
            self.prob = 0.0
            self.ready = False
            self.last_eval = float("-inf")
            self.detected = False
            self._above = 0
            self._below = 0

    def push(self, t_s: float, csi, device_ts_us: int | None = None):
        if isinstance(csi, (bytes, bytearray, memoryview)):
            raw = bytes(csi)
            if len(raw) < 8 or len(raw) % 2:
                return  # corrupted serial line
            iq = np.frombuffer(raw, dtype=np.int8).astype(np.float32).reshape(-1, 2)
            amp = np.sqrt((iq ** 2).sum(axis=1))
        else:
            amp = np.asarray(csi, dtype=np.float32)
            raw = amplitudes_to_csi_bytes(amp)
        data, _raw_len, layout = normalize_ht20_csi_payload(raw)
        if data is None:
            return
        prepared = prepare_ht20_detector_input(data, source_layout=layout)
        if prepared is None:
            return
        ts = int(device_ts_us if device_ts_us is not None else t_s * 1e6) % (1 << 32)
        with self.lock:
            self._rows.append(amp)
            if len(self._rows) > self.ROW_FRAMES:
                self._rows.pop(0)
            admission = self.controller.admit({"wifi_rx_ts_us": ts, "csi": bytes(prepared)})
            if admission is not None:
                apply_temporal_admission(self.det, admission)
                self.det.process_packet(admission.packet["csi"], timestamp_us=admission.timestamp_us)

    def update(self, now_s: float):
        with self.lock:
            if now_s - self.last_eval < self.eval_s:
                return
            self.last_eval = now_s
            result = self.det.update_state()
            self.ready = bool(self.det.is_ready())
            self.prob = float(result["probability"]) if self.ready else 0.0
            if not self.ready:
                self._above = self._below = 0
                self.detected = False
                return
            if self.prob > self.THRESHOLD:
                self._above += 1
                self._below = 0
            else:
                self._below += 1
                self._above = 0
            if not self.detected and self._above >= self.ENTER_EVALS:
                self.detected = True
            elif self.detected and self._below >= self.EXIT_EVALS:
                self.detected = False

    def snapshot(self, now_s: float) -> dict:
        with self.lock:
            ratio = self.prob / self.THRESHOLD if self.ready else None
            return {
                "calibrated": self.ready, "calib_left": None, "warmup_left": None,
                "motion": self.prob, "threshold": self.THRESHOLD, "ratio": ratio,
                "static_dev": 0.0, "static_threshold": None, "static_ratio": None,
                "detected": self.detected, "static_detected": False,
                "n_subcarriers": int(self._rows[-1].size) if self._rows else 0, "subcarrier_cv": None,
            }

    def display_row(self) -> list[float] | None:
        with self.lock:
            if not self._rows or len({r.size for r in self._rows}) != 1:
                return None
            row = np.mean(self._rows, axis=0)
        m = row[row > 0].mean() if (row > 0).any() else 1.0
        return [round(float(x / m), 3) for x in row]
