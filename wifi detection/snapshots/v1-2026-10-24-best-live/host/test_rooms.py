"""Tests for room evidence and /api/scan.  Run: python test_rooms.py"""

import json
import tempfile
import time
from pathlib import Path

from floorplan import FloorPlan
from presence_server import App, Settings
from rooms import RoomEvidence
from test_localizer import FakeLink, FakeRegistry

EXAMPLE = Path(__file__).resolve().parent / "floorplan.example.json"


def _run(ev, a, b, ratio, seconds, t0=1000.0, dt=0.25):
    t = t0
    for _ in range(int(seconds / dt)):
        ev.add(a, b, ratio, t, dt, detected=ratio > 1.0)
        t += dt
    return t


RECORDINGS = Path(__file__).resolve().parent / "recordings"


def _replay(name, t0, t1):
    """(ratio, detected) every 0.25 s from ESPectre on a recorded raw-CSI link, as feed_rooms sees it."""
    from espectre_link import EspectreLinkDetector
    det = EspectreLinkDetector(target_pps=20)
    first, out, next_s, n = None, [], None, 0
    for line in (RECORDINGS / name).open():
        t_s, rest = line.split(",", 1)
        if not rest.startswith("CSI,"):
            continue
        p = rest.rstrip().split(",", 7)
        if len(p) != 8:
            continue
        t = float(t_s)
        first = first if first is not None else t
        if t - first < t0 - 5:
            continue
        if t - first > t1:
            break
        try:
            raw = bytes.fromhex(p[7])
        except ValueError:
            continue
        det.push(t, raw, device_ts_us=int(p[2]))
        n += 1
        if n % 5 == 0:
            det.update(t)
        next_s = next_s if next_s is not None else first + t0
        while t >= next_s:
            s = det.snapshot(next_s)
            out.append((s["ratio"], s["detected"]))
            next_s += 0.25
    return out


def test_real_walk_never_reads_empty():
    """Review finding: a real walk on one half of Room B plus quiet on the other half read 'empty' at 100 %."""
    name = "20261024_010614_COM15.log"
    if not (RECORDINGS / name).exists():
        print("  (skipped: recording missing)")
        return
    quiet = _replay(name, 0, 30)
    for window in [(120, 150), (75, 90), (120, 135)]:
        walk = _replay(name, *window)
        ev = RoomEvidence(FloorPlan.load(EXAMPLE))
        t = 1000.0
        for (wr, wd), (qr, qd) in zip(walk, quiet):
            if wr is not None:
                ev.add((2.2, 0.5), (3.8, 0.5), wr, t, 0.25, detected=wd)
            if qr is not None:
                ev.add((2.2, 1.5), (3.8, 1.5), qr, t, 0.25, detected=qd)
            t += 0.25
        st = _status(ev, t)
        assert st["Room B"] != "empty", (window, st)
        if window == (75, 90):
            assert st["Room B"] == "occupied", (window, st)


def test_one_short_stop_never_reads_empty():
    """A single 15 s stop of quiet links covering the whole room is not enough to call it clear."""
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = 1000.0
    for _ in range(60):                        # 15 s
        ev.add((2.2, 0.5), (3.8, 0.5), 0.0, t, 0.25, detected=False)
        ev.add((2.2, 1.5), (3.8, 1.5), 0.0, t, 0.25, detected=False)
        t += 0.25
    assert _status(ev, t)["Room B"] == "unknown"


def test_isolated_spikes_dont_mark_occupied():
    """Raw ESPectre spikes (ratio 2.0) that its debounce rejects must not build occupancy."""
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = 1000.0
    for i in range(240):                       # 60 s: a lone spike every 2 s, otherwise quiet
        spike = i % 8 == 0
        ev.add((0.2, 1.0), (1.8, 1.0), 2.0 if spike else 0.0, t, 0.25, detected=False)
        t += 0.25
    assert _status(ev, t)["Room A"] != "occupied"


def _status(ev, now):
    return {r["name"]: r["status"] for r in ev.status(now)}


def test_link_through_room_a():
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = _run(ev, (0.2, 1.0), (1.8, 1.0), 2.0, 20)
    st = _status(ev, t)
    assert st["Room A"] == "occupied", st
    assert st["Room B"] == "unknown", st          # Review Focus 4: never crossed


def test_quiet_links_mark_empty():
    """Empty needs most of the room covered by quiet links for 30 s: two parallel quiet links through Room B."""
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = 1000.0
    for _ in range(160):                       # 40 s
        ev.add((2.2, 0.5), (3.8, 0.5), 0.4, t, 0.25, detected=False)
        ev.add((2.2, 1.5), (3.8, 1.5), 0.4, t, 0.25, detected=False)
        t += 0.25
    assert _status(ev, t)["Room B"] == "empty"


def test_one_quiet_link_is_not_enough_for_empty():
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = _run(ev, (2.2, 0.5), (3.8, 0.5), 0.4, 20)
    assert _status(ev, t)["Room B"] == "unknown"


def test_quiet_links_elsewhere_dont_cancel_a_disturbed_one():
    """A person on one link's path stays found even when quiet links cross other parts of the room."""
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = 1000.0
    for _ in range(80):
        ev.add((1.0, 0.3), (1.0, 1.7), 2.0, t, 0.25, detected=True)   # through the person
        for a, b in [((0.2, 0.2), (1.8, 0.2)), ((0.2, 1.8), (1.8, 1.8)), ((0.3, 0.3), (0.3, 1.7)),
                     ((1.7, 0.3), (1.7, 1.7)), ((-0.5, -0.5), (4.3, 2.3))]:
            ev.add(a, b, 0.4, t, 0.25, detected=False)      # quiet elsewhere in Room A
        t += 0.25
    assert _status(ev, t)["Room A"] == "occupied", _status(ev, t)


SMALL_HOUSE = FloorPlan.from_dict({"rooms": [
    {"name": "Room A", "polygon": [[0, 0], [0.85, 0], [0.85, 1.2], [0, 1.2]]},
    {"name": "Room B", "polygon": [[0.85, 0], [1.7, 0], [1.7, 1.2], [0.85, 1.2]]}]})


def test_link_mostly_in_one_room_only_counts_there():
    """1.7 x 1.2 m house: a link across Room A also clips 12 % of Room B, which alone marked B occupied."""
    ev = RoomEvidence(SMALL_HOUSE)
    t = _run(ev, (0.42, -0.25), (0.42, 1.45), 2.0, 20)
    st = _status(ev, t)
    assert st == {"Room A": "occupied", "Room B": "unknown"}, st


def test_link_split_across_rooms_counts_for_neither():
    """A diagonal through both rooms can't tell which one the person is in, so it adds no room evidence."""
    ev = RoomEvidence(SMALL_HOUSE)
    t = _run(ev, (0.42, -0.25), (1.28, 1.45), 2.0, 20)
    st = _status(ev, t)
    assert st == {"Room A": "unknown", "Room B": "unknown"}, st


def test_sentry_motion_memory_clears_soon_after_leaving():
    """Mission plans remember motion for minutes; a sentry plan's short motion memory clears a room the person left."""
    link = ((0.42, -0.25), (0.42, 1.45))
    for half_life, still_occupied in [(None, True), (8, False)]:
        d = SMALL_HOUSE.to_dict()
        if half_life:
            d["motion_half_life_s"] = half_life
        ev = RoomEvidence(FloorPlan.from_dict(d))
        t = _run(ev, *link, 2.0, 20)
        assert _status(ev, t)["Room A"] == "occupied"
        t = _run(ev, *link, 0.0, 30, t0=t)
        assert (_status(ev, t)["Room A"] == "occupied") == still_occupied, (half_life, _status(ev, t))
    try:
        FloorPlan.from_dict({**SMALL_HOUSE.to_dict(), "motion_half_life_s": 0})
        raise AssertionError("a zero half-life must be rejected")
    except ValueError:
        pass


def test_decay():
    ev = RoomEvidence(FloorPlan.load(EXAMPLE))
    t = _run(ev, (0.2, 1.0), (1.8, 1.0), 2.0, 20)
    assert _status(ev, t + 600)["Room A"] != "occupied"


def test_scan_from_links():
    # The app's localizer thread may still be reading the file when the directory is removed (Windows).
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
        plan = json.loads(EXAMPLE.read_text())
        plan["boards"] = {"aa": [0.2, 1.0], "bb": [1.8, 1.0]}
        path = Path(d) / "floorplan.json"
        path.write_text(json.dumps(plan))
        app = App([], Settings(10), record=False, reset_on_start=False, floorplan_path=path,
                  rooms_loop=False)
        app.registry = FakeRegistry([FakeLink("aa", "bb", 2.0)])
        now = time.time()
        for i in range(80):
            app.feed_rooms(now + i * 0.25, 0.25)
        scan = app.scan()
        assert set(scan) >= {"rooms", "robots", "detections", "warnings"}
        status = {r["name"]: r["status"] for r in scan["rooms"]}
        assert status["Room A"] == "occupied", status
        assert any(d.startswith("Person likely in Room A") for d in scan["detections"]), scan["detections"]


def test_single_pair_motion_doesnt_mark_a_room():
    """Live test: one pair alone fired 5 % of the time with nobody near; rooms need two agreeing pairs."""
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
        plan = json.loads(EXAMPLE.read_text())
        plan["boards"] = {"aa": [0.2, 1.0], "bb": [1.8, 1.0], "cc": [1.0, 0.2], "dd": [1.0, 1.8]}
        path = Path(d) / "floorplan.json"
        path.write_text(json.dumps(plan))
        for hot, expected in [({("aa", "bb")}, False), ({("aa", "bb"), ("cc", "dd")}, True)]:
            app = App([], Settings(10), record=False, reset_on_start=False, floorplan_path=path,
                      rooms_loop=False)
            app.registry = FakeRegistry([FakeLink(a, b, 2.0 if (a, b) in hot else 0.3)
                                         for a, b in [("aa", "bb"), ("cc", "dd"), ("aa", "cc")]])
            now = time.time()
            for i in range(80):
                app.feed_rooms(now + i * 0.25, 0.25)
            status = {r["name"]: r["status"] for r in app.scan()["rooms"]}
            assert (status["Room A"] == "occupied") == expected, (hot, status)


if __name__ == "__main__":
    test_link_through_room_a()
    test_quiet_links_mark_empty()
    test_one_quiet_link_is_not_enough_for_empty()
    test_quiet_links_elsewhere_dont_cancel_a_disturbed_one()
    test_decay()
    test_scan_from_links()
    test_real_walk_never_reads_empty()
    test_one_short_stop_never_reads_empty()
    test_isolated_spikes_dont_mark_occupied()
    test_link_mostly_in_one_room_only_counts_there()
    test_link_split_across_rooms_counts_for_neither()
    test_sentry_motion_memory_clears_soon_after_leaving()
    test_single_pair_motion_doesnt_mark_a_room()
    print("ok")
