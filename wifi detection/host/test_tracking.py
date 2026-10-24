"""Tests for the person-marker Kalman filter.  Run: python test_tracking.py"""

import numpy as np

from tracking import PositionKalman

DT = 0.2   # the localizer's step


def _run(kf, truth, noise_m, seed=0):
    rng = np.random.default_rng(seed)
    out, raw = [], []
    for i, p in enumerate(truth):
        z = np.asarray(p) + rng.normal(0, noise_m, 2)
        raw.append(z)
        out.append(kf.update(z, i * DT))
    return np.array(out), np.array(raw)


def test_first_measurement_is_returned_as_is():
    kf = PositionKalman()
    assert np.allclose(kf.update((0.8, 0.6), 100.0), (0.8, 0.6))


def test_smooths_a_still_estimate():
    """Live test: the marker jumped a median 0.17 m between updates while the person moved inside."""
    truth = [(0.8, 0.6)] * 100
    out, raw = _run(PositionKalman(), truth, 0.17)
    jumps = lambda a: np.median(np.hypot(*np.diff(a[20:], axis=0).T))
    assert jumps(out) < jumps(raw) / 3, (jumps(out), jumps(raw))
    assert np.hypot(*(out[20:].mean(axis=0) - truth[0])) < 0.05


def test_follows_a_walk_with_less_error_than_raw():
    truth = [(0.2 + 0.5 * i * DT, 0.6) for i in range(60)]      # 0.5 m/s across the area
    out, raw = _run(PositionKalman(), truth, 0.17)
    err = lambda a: np.sqrt(np.mean(np.sum((a[10:] - np.array(truth)[10:]) ** 2, axis=1)))
    assert err(out) < err(raw), (err(out), err(raw))
    assert abs(out[-1][0] - truth[-1][0]) < 0.25                 # no long trail behind the walker


def test_restarts_after_a_gap():
    """A new detection seconds later is a fresh track, not a glide across the area."""
    kf = PositionKalman()
    for i in range(20):
        kf.update((0.3, 0.3), i * DT)
    assert np.allclose(kf.update((1.5, 1.0), 20 * DT + 3.0), (1.5, 1.0))


if __name__ == "__main__":
    test_first_measurement_is_returned_as_is()
    test_smooths_a_still_estimate()
    test_follows_a_walk_with_less_error_than_raw()
    test_restarts_after_a_gap()
    print("ok")
