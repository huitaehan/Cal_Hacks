"""Regression tests for LinkDetector.  Run: python test_detector.py"""

import numpy as np

from presence_server import LinkDetector, Settings


def test_calibration_survives_flickering_subcarrier():
    """A subcarrier hovering near the validity cut-off used to make calibration
    crash (ragged calib_means -> ValueError), killing the node's reader thread."""
    rng = np.random.default_rng(0)
    det = LinkDetector(Settings(calib_seconds=10.0))
    base = np.full(57, 20.0, dtype=np.float32)
    base[28] = 0.0  # DC null
    t = 0.0
    for i in range(50 * 25):  # 25 s at 50 Hz: warm-up + calibration + some live time
        amp = base + rng.normal(0, 0.3, 57).astype(np.float32)
        # Subcarrier 5 alternates every 2 s (longer than the 1 s window) between
        # clearly valid and below 0.2 x median.
        amp[5] = 25.0 if (i // 100) % 2 else 2.0
        det.push(t, amp)
        if i % 5 == 0:
            det.update(t)
        t += 0.02
    assert det.calibrated, "detector never finished calibrating"
    assert det.thr_motion is not None


if __name__ == "__main__":
    test_calibration_survives_flickering_subcarrier()
    print("ok")
