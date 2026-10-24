"""Tests for per-link detector backends.  Run: python test_links.py"""

import time

import numpy as np

from espectre_link import EspectreLinkDetector, espectre_config, espectre_target_pps
from presence_server import Link, LinkDetector, Settings
from test_localizer import FakeLink, FakeRegistry, locate  # noqa: F401  (shared fakes)


def test_target_choice():
    """Live 4-board mesh: real arrivals are irregular, so a slot rate near the packet rate left ~30 % of
    slots empty and readiness flickered. Slots <= 0.6 x the packet rate stayed 95-98 % ready."""
    assert espectre_target_pps(24) == 10
    assert espectre_target_pps(36) == 20
    assert espectre_target_pps(48) == 25
    assert espectre_target_pps(100) == 50


def test_slow_links_get_a_longer_window():
    """Relayed links run at ~9-12 pps live; at 10 slots/s they were ready only ~55 % of the time."""
    assert espectre_config(36) == (20, 1000)
    assert espectre_config(24) == (10, 1000)
    assert espectre_config(12) == (5, 2000)
    assert espectre_config(6) == (5, 2000)
    link = Link("aa", "bb", Settings(10, detector="espectre"))
    _feed(link, 6, 10.0)
    assert (link.detector.target_pps, link.detector.window_ms) == (5, 2000)


def test_rate_change_must_hold_5_s_before_rebuild():
    """Live links hovering at a boundary rebuilt (and reset) their detector 5-21 times in 2.5 min."""
    link = Link("aa", "bb", Settings(10, detector="espectre"))
    t = _feed(link, 24, 10.0)
    assert link.detector.target_pps == 10
    t = _feed(link, 48, 3.0, t0=t)
    assert link.detector.target_pps == 10, "rebuilt before the new rate held for 5 s"
    _feed(link, 48, 7.0, t0=t)
    assert link.detector.target_pps == 25


def _feed(link, pps, seconds, t0=0.0):
    rng = np.random.default_rng(0)
    n = int(pps * seconds)
    for i in range(n):
        t = t0 + i / pps
        link.push(t, 25 + rng.normal(0, 1, 57).astype(np.float32), -55, device_ts_us=int(t * 1e6))
    return t0 + n / pps


def test_backend_selected():
    assert isinstance(Link("aa", "bb", Settings(10, detector="espectre")).detector, EspectreLinkDetector)
    assert isinstance(Link("aa", "bb", Settings(10, detector="denoised")).detector, LinkDetector)


def test_target_follows_rate():
    link = Link("aa", "bb", Settings(10, detector="espectre"))
    _feed(link, 24, 10.0)
    assert link.detector.target_pps == 10, link.detector.target_pps
    link2 = Link("aa", "cc", Settings(10, detector="espectre"))
    _feed(link2, 48, 10.0)
    assert link2.detector.target_pps == 25, link2.detector.target_pps


def test_retarget_survives_a_stop():
    """Review Important 4: after a gated drive the 3 s rate read ~0, so ESPectre was rebuilt at 10 then back at 20."""
    gate = {"open": True}
    link = Link("aa", "bb", Settings(10, detector="espectre"), still=lambda mac, now: gate["open"])
    t = _feed(link, 24, 10.0)                     # settle at 10 slots/s
    assert link.detector.target_pps == 10
    gate["open"] = False
    t = _feed(link, 24, 5.0, t0=t)                # driving: every push gated
    gate["open"] = True
    seen = set()
    rng = np.random.default_rng(1)
    for i in range(int(24 * 5)):                  # new stop: 5 s at 24 pps
        tt = t + i / 24
        link.push(tt, 25 + rng.normal(0, 1, 57).astype(np.float32), -55, device_ts_us=int(tt * 1e6))
        seen.add(link.detector.target_pps)
    assert seen == {10}, seen


def test_slow_link_is_evaluated_every_quarter_second():
    """Links were evaluated every 5th packet: 0.5 s at relayed rates (~10 pps), so ESPectre's
    4-evaluation debounce took 2 s to report motion instead of ~1 s."""
    link = Link("aa", "bb", Settings(10, detector="espectre"))
    _feed(link, 10, 12.0)                         # settle on the slow-link config
    det, calls = link.detector.det, []
    orig = det.update_state
    det.update_state = lambda: (calls.append(1), orig())[1]
    _feed(link, 10, 4.0, t0=12.0)
    assert link.detector.det is det, "detector was rebuilt during the measurement"
    assert len(calls) >= 12, len(calls)


def test_ratio_none_adds_nothing():
    from presence_server import Localizer
    import json, tempfile
    from pathlib import Path
    with tempfile.TemporaryDirectory() as d:
        layout = Path(d) / "layout.json"
        layout.write_text(json.dumps({"boards": {"aa": [0, 0], "bb": [3, 0]}}))
        loc = Localizer(FakeRegistry([FakeLink("aa", "bb", None)]), layout, 3.0)
        time.sleep(0.05)
        loc.step(time.time())
        r = loc.snapshot()
        assert r["pairs"] == [] and r["person"] is None


if __name__ == "__main__":
    test_target_choice()
    test_slow_links_get_a_longer_window()
    test_backend_selected()
    test_target_follows_rate()
    test_retarget_survives_a_stop()
    test_rate_change_must_hold_5_s_before_rebuild()
    test_slow_link_is_evaluated_every_quarter_second()
    test_ratio_none_adds_nothing()
    print("ok")
