"""Tests for the multi-link Localizer.  Run: python test_localizer.py"""

import itertools
import json
import tempfile
import time
from pathlib import Path

import numpy as np

from presence_server import Localizer
from wander import WanderLinkDetector

TH = WanderLinkDetector.THRESHOLD

BOARDS = {"aa": (0.0, 0.0), "bb": (3.0, 0.0), "cc": (3.0, 3.0), "dd": (0.0, 3.0)}


class FakeLink:
    def __init__(self, tx, rx, ratio):
        self.tx, self.rx, self.ratio = tx, rx, ratio

    def snapshot(self, now):
        return {"tx": self.tx, "rx": self.rx, "rate": 40.0,
                "detector": {"ratio": self.ratio, "detected": self.ratio is not None and self.ratio > 1.0}}


class FakeRegistry:
    def __init__(self, links):
        self.links = links

    def all(self):
        return self.links


def links_for_person(p):
    """Ratio per link falls off with the person's excess path to that link's ellipse."""
    out = []
    for a, b in itertools.permutations(BOARDS, 2):
        (ax, ay), (bx, by) = BOARDS[a], BOARDS[b]
        excess = np.hypot(p[0] - ax, p[1] - ay) + np.hypot(p[0] - bx, p[1] - by) - np.hypot(bx - ax, by - ay)
        out.append(FakeLink(a, b, 0.5 + 3.0 * float(np.exp(-(excess / 0.4) ** 2))))
    return out


def locate(person):
    with tempfile.TemporaryDirectory() as d:
        layout = Path(d) / "layout.json"
        layout.write_text(json.dumps({"boards": {k: list(v) for k, v in BOARDS.items()}}))
        loc = Localizer(FakeRegistry(links_for_person(person) if person else
                                     [FakeLink(a, b, 0.5) for a, b in itertools.permutations(BOARDS, 2)]),
                        layout, 3.0)
        time.sleep(0.05)  # let the background thread finish its first step on the temp layout
        loc.step(time.time())
        return loc.snapshot()


def test_person_near_edge():
    est = locate((1.5, 0.3))["person"]
    assert est is not None and np.hypot(est[0] - 1.5, est[1] - 0.3) < 0.6, est


def test_person_in_centre():
    est = locate((1.5, 1.5))["person"]
    assert est is not None and np.hypot(est[0] - 1.5, est[1] - 1.5) < 0.6, est


def _locate_links(links):
    with tempfile.TemporaryDirectory() as d:
        layout = Path(d) / "layout.json"
        layout.write_text(json.dumps({"boards": {k: list(v) for k, v in BOARDS.items()}}))
        loc = Localizer(FakeRegistry(links), layout, 3.0)
        time.sleep(0.05)
        loc.step(time.time())
        return loc.snapshot()


def test_one_disturbed_pair_is_not_a_person():
    """Live test: one pair alone fired 5 % of the time with nobody near; two agreeing pairs never did."""
    def links(hot):
        return [FakeLink(a, b, 2.0 if frozenset((a, b)) in hot else 0.3)
                for a, b in itertools.permutations(BOARDS, 2)]
    assert _locate_links(links({frozenset(("aa", "bb"))}))["person"] is None
    assert _locate_links(links({frozenset(("aa", "bb")), frozenset(("aa", "cc"))}))["person"] is not None


def test_brief_dropout_doesnt_make_the_marker_jump():
    """The old average restarted on every step without agreement, so the marker snapped to the next raw estimate."""
    quiet = [FakeLink(a, b, 0.3) for a, b in itertools.permutations(BOARDS, 2)]
    with tempfile.TemporaryDirectory() as d:
        layout = Path(d) / "layout.json"
        layout.write_text(json.dumps({"boards": {k: list(v) for k, v in BOARDS.items()}}))
        loc = Localizer(FakeRegistry(quiet), layout, 3.0, background=False)
        t = 1000.0
        loc.registry = FakeRegistry(links_for_person((1.5, 0.3)))
        for _ in range(10):
            loc.step(t); t += 0.2
        loc.registry = FakeRegistry(quiet)
        loc.step(t); t += 0.2                         # one step without motion
        loc.registry = FakeRegistry(links_for_person((1.5, 2.7)))
        loc.step(t)
        shown = np.array(loc.snapshot()["person"])
    raw = np.array(_locate_links(links_for_person((1.5, 2.7)))["person"])
    assert np.hypot(*(shown - raw)) > 0.1, (shown, raw)


class _WanderLink:
    def __init__(self, tx, rx, score):
        self.tx, self.rx, self.score = tx, rx, score

    def snapshot(self, now):
        return {"tx": self.tx, "rx": self.rx, "rate": 40.0,
                "detector": {"ratio": max(0.0, self.score) / TH, "score": self.score, "detected": self.score > TH}}


class _WanderRegistry(FakeRegistry):
    class settings:
        detector = "wander"


def test_wander_one_strong_pair_places_the_person():
    hot = frozenset(("aa", "bb"))
    links = [_WanderLink(a, b, 2.5 if frozenset((a, b)) == hot else 0.2)
             for a, b in itertools.permutations(BOARDS, 2)]
    with tempfile.TemporaryDirectory() as d:
        layout = Path(d) / "layout.json"
        layout.write_text(json.dumps({"boards": {k: list(v) for k, v in BOARDS.items()}}))
        loc = Localizer(_WanderRegistry(links), layout, 3.0, background=False)
        loc.step(time.time())
        r = loc.snapshot()
    assert r["person"] is not None
    pair = next(p for p in r["pairs"] if {p["a"], p["b"]} == {"aa", "bb"})
    assert pair["detected"] and abs(pair["ratio"] - 2.5 / TH) < 1e-6, pair


def test_nobody():
    r = locate(None)
    assert r["person"] is None
    assert len(r["obs_field"]) == 400 and len(r["pairs"]) == 6


if __name__ == "__main__":
    test_person_near_edge()
    test_person_in_centre()
    test_nobody()
    test_one_disturbed_pair_is_not_a_person()
    test_brief_dropout_doesnt_make_the_marker_jump()
    test_wander_one_strong_pair_places_the_person()
    print("ok")
