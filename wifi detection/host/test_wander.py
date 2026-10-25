"""Tests for the wander per-link detector.  Run: python test_wander.py"""

from pathlib import Path

import numpy as np

from wander import WanderLinkDetector

RECORDING = Path(__file__).resolve().parent / "recordings" / "20261024_063341_COM11.log"
TEST2_T0 = 1792838303.13          # live test 2: nobody 0-60 s, walking inside 60-120 s
rng = np.random.default_rng(0)
BASE = (20 + 8 * np.sin(np.arange(57) / 5.0)).astype(np.float32)
BASE[[0, 1, 28]] = 0.0             # invalid first word and the DC null, as the boards send them


def quiet(t):
    return BASE * (1 + rng.normal(0, 0.03, 57)).astype(np.float32)


def moving(t):
    shape = 1 + 0.25 * np.sin(2 * np.pi * 1.2 * t + np.arange(57) / 3.0)
    return BASE * shape.astype(np.float32) * (1 + rng.normal(0, 0.03, 57)).astype(np.float32)


def feed(det, t0, t1, gen, pps=20, gain=1.0):
    """Push gen(t) from t0 to t1 s; return the detected flag after each evaluation."""
    flags = []
    for t in np.arange(t0, t1, 1.0 / pps):
        det.push(t, gen(t) * gain, device_ts_us=int(t * 1e6))
        det.update(t)
        flags.append((t, det.snapshot(t)["detected"]))
    return flags


def test_not_ready_until_it_knows_the_link():
    det = WanderLinkDetector()
    feed(det, 0, 3, quiet)
    snap = det.snapshot(3.0)
    assert snap["ratio"] is None and snap["calibrated"] is False and snap["detected"] is False


def test_quiet_link_never_fires():
    det = WanderLinkDetector()
    flags = feed(det, 0, 120, quiet)
    assert not any(d for _, d in flags)
    assert det.snapshot(120.0)["calibrated"] is True


def test_motion_after_quiet_fires_within_3_s():
    det = WanderLinkDetector()
    feed(det, 0, 60, quiet)
    flags = feed(det, 60, 70, moving)
    first = next(t for t, d in flags if d)
    assert first - 60 <= 3.0, first


def test_agc_gain_step_is_not_motion():
    """AGC rescales every subcarrier at once; the shape-based measure must ignore it."""
    det = WanderLinkDetector()
    feed(det, 0, 60, quiet)
    flags = feed(det, 60, 90, quiet, gain=1.6)
    assert not any(d for _, d in flags)


def test_gateway_clock_restart_starts_a_new_window():
    det = WanderLinkDetector()
    feed(det, 0, 60, quiet)
    flags = feed(det, 0.0, 10, quiet)     # board rebooted: its clock starts again at 0
    assert not any(d for _, d in flags)


def _recorded_link(tx):
    """(host time, device time s, amplitudes) of one direct link in the test-2 recording."""
    out = []
    for line in RECORDING.open():
        p = line.rstrip().split(",", 8)
        if len(p) == 9 and p[1] == "CSIA" and p[2] == tx:
            try:
                amp = np.frombuffer(bytes.fromhex(p[8]), np.uint8).astype(np.float32)
                out.append((float(p[0]), int(p[4]) / 1e6, amp))
            except ValueError:
                continue                     # corrupted serial line (the server skips these too)
    return out


def test_recorded_walk_breadcrumb_to_gateway():
    """Live test 2, breadcrumb -> gateway link: silent with nobody near, fires while someone walks inside."""
    det = WanderLinkDetector()
    nobody, inside = [], []
    for host_t, dev_t, amp in _recorded_link("d0:cf:13:e1:66:2c"):
        det.push(host_t, amp, device_ts_us=int(dev_t * 1e6))
        det.update(host_t)
        rel = host_t - TEST2_T0
        if 5 <= rel < 55:
            nobody.append(det.snapshot(host_t)["detected"])
        elif 65 <= rel < 115:
            inside.append(det.snapshot(host_t)["detected"])
    assert nobody and not any(nobody), np.mean(nobody)
    assert np.mean(inside) >= 0.35, np.mean(inside)


def test_a_fast_link_keeps_every_third_packet_like_a_relayed_one():
    """The gateway measures its own links at 40-46 packets/s; relayed links arrive at about 16/s. Judged on
    every packet, the gateway's links read numb: on Oct 25 the front edge fired 3 % of the time against
    17-24 % for relayed links, and 10 % keeping every 3rd packet (no extra false alarms on labelled test 2)."""
    rng = np.random.default_rng(1)
    base = 20 + 10 * np.sin(np.arange(52) / 4)
    fast, slow = WanderLinkDetector(), WanderLinkDetector()
    for i in range(45 * 6):                                       # 6 s at 45 packets/s
        fast.push(i / 45, base + rng.normal(0, 2.0, 52))
    for i in range(16 * 6):                                       # 6 s at 16 packets/s
        slow.push(i / 16, base + rng.normal(0, 2.0, 52))
    assert 28 <= len(fast._a) <= 34, len(fast._a)                 # a 2 s window at about 15/s
    gaps = np.diff(np.array(fast._t)) * 45
    assert np.allclose(gaps, 3), gaps                             # a fixed every-3rd, not a shifting pick
    assert len(slow._a) >= 31                                     # all of a relayed link's packets kept


if __name__ == "__main__":
    test_not_ready_until_it_knows_the_link()
    test_quiet_link_never_fires()
    test_motion_after_quiet_fires_within_3_s()
    test_agc_gain_step_is_not_motion()
    test_gateway_clock_restart_starts_a_new_window()
    test_recorded_walk_breadcrumb_to_gateway()
    test_a_fast_link_keeps_every_third_packet_like_a_relayed_one()
    print("ok")
