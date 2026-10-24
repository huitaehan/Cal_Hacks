"""Tests for the ESPectre per-link detector adapter.  Run: python test_espectre_link.py"""

from pathlib import Path

import numpy as np

from espectre_link import EspectreLinkDetector, amplitudes_to_csi_bytes, normalize_ht20_csi_payload

RECORDING = Path(__file__).resolve().parent / "recordings" / "20261024_010613_COM13.log"


def test_layout():
    raw = amplitudes_to_csi_bytes(np.arange(57, dtype=np.float32))
    assert len(raw) == 114
    data, raw_len, layout = normalize_ht20_csi_payload(raw)
    assert len(data) == 128 and raw_len == 114
    assert data[2 * 32 + 1] == 28, "our index 28 (DC) must land on ESPectre bin 32"
    assert all(b == 0 for b in data[0:8]) and all(b == 0 for b in data[122:128])


def test_slow_link_not_ready():
    det = EspectreLinkDetector(target_pps=50)
    rng = np.random.default_rng(1)
    for i in range(25):  # 5 s at 5 pps
        t = i / 5.0
        det.push(t, 25 + rng.normal(0, 1, 57), device_ts_us=int(t * 1e6))
        det.update(t)
    assert det.snapshot(5.0)["ratio"] is None


class _ScriptedModel:
    """Stands in for ESPectre's MLP so the decision rule can be tested on chosen probabilities."""

    def __init__(self, probs):
        self.probs = list(probs)

    def update_state(self):
        return {"probability": self.probs.pop(0)}

    def is_ready(self):
        return True


def test_debounce_like_espectre():
    det = EspectreLinkDetector(target_pps=50)
    det.det = _ScriptedModel([0.9, 0.1, 0.9, 0.9, 0.9, 0.9, 0.1, 0.1, 0.1])
    states = []
    for i in range(9):
        det.update(i * 0.3)
        states.append(det.snapshot(i * 0.3)["detected"])
    # one spike ignored; on after 4 consecutive; off after 3 consecutive
    assert states == [False, False, False, False, False, True, True, True, False], states


def _recorded_ratios(t0, t1):
    """Replay the raw-I/Q CSI lines between t0 and t1 seconds of the recording (a ~17-22 pps link)."""
    det = EspectreLinkDetector(target_pps=20)
    first, ratios = None, []
    for line in RECORDING.open():
        t_s, rest = line.split(",", 1)
        if not rest.startswith("CSI,"):
            continue
        p = rest.rstrip().split(",", 7)
        t = float(t_s)
        first = first if first is not None else t
        if t - first < t0 - 3:
            continue
        if t - first > t1:
            break
        try:
            raw = bytes.fromhex(p[7])
        except ValueError:
            continue
        det.push(t, raw, device_ts_us=int(p[2]))
        det.update(t)
        if t - first >= t0:
            r = det.snapshot(t)["ratio"]
            if r is not None:
                ratios.append(r)
    return np.array(ratios)


def test_real_walk_scores_higher():
    if not RECORDING.exists():
        print("  (skipped: recording missing)")
        return
    quiet = _recorded_ratios(20, 40)
    walk = _recorded_ratios(75, 95)
    assert quiet.size > 20 and walk.size > 20, (quiet.size, walk.size)
    assert walk.mean() > quiet.mean(), (walk.mean(), quiet.mean())


if __name__ == "__main__":
    test_layout()
    test_slow_link_not_ready()
    test_debounce_like_espectre()
    test_real_walk_scores_higher()
    print("ok")
