"""App-level regression tests from the final review.  Run: python test_app.py"""

import time
from pathlib import Path

import numpy as np

from floorplan import FloorPlan
from presence_server import App, Settings
from wander import WanderLinkDetector

TH = WanderLinkDetector.THRESHOLD

EXAMPLE = Path(__file__).resolve().parent / "floorplan.example.json"
ROBOT_A = "d0:cf:13:e1:63:30"
ANCHOR = "d0:cf:13:e1:66:2c"


def _app():
    return App([], Settings(10, detector="espectre"), record=False, reset_on_start=False,
               floorplan_path=EXAMPLE, rooms_loop=False)


def test_state_with_unready_espectre_links():
    """Review Critical 1: two live-but-not-ready ESPectre links made state() raise (max of [None, None])."""
    app = _app()
    now = time.time()
    for tx, rx in [("aa", "bb"), ("bb", "aa")]:
        link = app.registry.get(tx, rx)
        for i in range(3):
            link.push(now + i * 0.05, 25 + np.zeros(57, np.float32), -50, device_ts_us=int((now + i * 0.05) * 1e6))
    st = app.state()
    assert st["summary"]["calibrating"] is None
    assert len(st["links"]) == 2


def test_stop_done_after_listening():
    """Review Important 3: stop_s was capped by the 5 s position history, so stop_done never became true."""
    app = _app()
    plan = FloorPlan.load(EXAMPLE)
    t0 = 5000.0
    for i in range(200):                                  # parked at stop 1's "a" for 20 s
        app.tracker.update([{"id": "A", "mac": ROBOT_A, "x": 1.0, "y": 0.3}], now=t0 + i * 0.1)
    now = t0 + 199 * 0.1
    robot = next(r for r in app.tracker.snapshot(now) if r["mac"] == ROBOT_A)
    assert robot["stop_s"] >= 19.0, robot
    assert app._stop_done(plan, robot)                   # 15 s listening + 3 s settle < 19.9 s


def test_no_unheard_warning_while_driving():
    """Review Important 5: a driving robot's gated board dropped out of 'heard' and raised a false warning."""
    app = _app()
    link = app.registry.get(ROBOT_A, ANCHOR)
    for i in range(30):
        now = time.time()
        app.post_robots([{"id": "A", "mac": ROBOT_A, "x": 0.5 + 0.02 * i, "y": 0.3}])   # moving
        link.push(now, 25 + np.zeros(57, np.float32), -50, device_ts_us=int(now * 1e6))
        time.sleep(0.01)
    warnings = app.scan()["warnings"]
    assert not any(ROBOT_A in w and "no board" in w for w in warnings), warnings


class _ReadyLink:
    def __init__(self, tx, rx, detected):
        self.tx, self.rx, self.detected = tx, rx, detected

    def snapshot(self, now):
        from presence_server import EMPTY_DETECTOR
        det = dict(EMPTY_DETECTOR, calibrated=True, ratio=2.0 if self.detected else 0.1, detected=self.detected)
        return {"tx": self.tx, "rx": self.rx, "rate": 25.0, "relayed": False, "rssi": -50.0, "detector": det}


class _Registry:
    def __init__(self, links):
        self.links = links

    def all(self):
        return self.links


class _WanderLink:
    def __init__(self, tx, rx, score):
        self.tx, self.rx, self.score = tx, rx, score

    def snapshot(self, now):
        from presence_server import EMPTY_DETECTOR
        det = dict(EMPTY_DETECTOR, calibrated=True, ratio=max(0.0, self.score) / TH, score=self.score,
                   detected=self.score > TH)
        return {"tx": self.tx, "rx": self.rx, "rate": 25.0, "relayed": False, "rssi": -50.0, "detector": det}


def test_wander_pair_averages_its_two_directions():
    """One noisy direction alone doesn't make a pair; the pair's mean score must clear the threshold."""
    app = App([], Settings(10, detector="wander"), record=False, reset_on_start=False,
              floorplan_path=EXAMPLE, rooms_loop=False)
    app.registry = _Registry([_WanderLink("aa", "bb", 2.4), _WanderLink("bb", "aa", 0.2)])
    assert app.state()["summary"]["detected"] is False       # mean 1.3, under the 1.4 threshold
    app.registry = _Registry([_WanderLink("aa", "bb", 2.4), _WanderLink("bb", "aa", 1.6),
                              _WanderLink("cc", "dd", 0.3)])
    assert app.state()["summary"]["detected"] is True        # mean 2.0, and one pair is enough


def test_relayed_lines_use_the_gateway_clock():
    """Relayed lines carry no time; the gateway prints in event order, so they take the clock of the
    CSIA line before them (host arrival times jitter by 40 ms, up to 250 ms)."""
    from presence_server import Node
    pushes = []

    class _Spy:
        def get(self, tx, rx):
            class _L:
                def push(self, now, amp, rssi, relay_seq=None, device_ts_us=None):
                    pushes.append((tx, rx, relay_seq, device_ts_us))
            return _L()

    node = Node("COMX", Settings(10, detector="wander"), record=False, registry=_Spy())
    node.mac = "d0:cf:13:e1:63:ac"
    node._handle("CSIA,d0:cf:13:e1:63:30,5,123456789,-50,-90,57," + "10" * 57)
    node._handle("RLY,d0:cf:13:e1:2e:30,d0:cf:13:e1:63:30,7,-55,57," + "10" * 57)
    assert pushes[0][3] == 123456789
    assert pushes[1][:3] == ("d0:cf:13:e1:63:30", "d0:cf:13:e1:2e:30", 7)
    assert pushes[1][3] == 123456789, pushes[1]


def test_summary_needs_two_agreeing_pairs():
    """Live test: any single link fired 5 % of the time with nobody near; two agreeing pairs never did."""
    app = _app()
    both = [_ReadyLink("aa", "bb", True), _ReadyLink("bb", "aa", True), _ReadyLink("cc", "dd", False)]
    app.registry = _Registry(both)
    assert app.state()["summary"]["detected"] is False      # one pair, seen from both ends
    app.registry = _Registry(both + [_ReadyLink("aa", "cc", True)])
    assert app.state()["summary"]["detected"] is True
    app.registry = _Registry([_ReadyLink("aa", "bb", True)])
    assert app.state()["summary"]["detected"] is True       # only one pair exists (2-board setup)


if __name__ == "__main__":
    test_state_with_unready_espectre_links()
    test_stop_done_after_listening()
    test_no_unheard_warning_while_driving()
    test_summary_needs_two_agreeing_pairs()
    test_wander_pair_averages_its_two_directions()
    test_relayed_lines_use_the_gateway_clock()
    print("ok")
