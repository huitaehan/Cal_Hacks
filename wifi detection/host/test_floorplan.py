"""Tests for the floor plan and the robot-team API.  Run: python test_floorplan.py"""

import json
import os
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

from floorplan import FloorPlan, FloorPlanWatcher

HERE = Path(__file__).resolve().parent
EXAMPLE = HERE / "floorplan.example.json"


def test_load_example():
    plan = FloorPlan.load(EXAMPLE)
    assert len(plan.rooms) == 2
    assert plan.rooms[0].contains(1, 1)
    assert not plan.rooms[0].contains(3, 1)
    assert plan.stops[0].listen_s == 15.0


def test_invalid_edit_keeps_last_plan():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "floorplan.json"
        path.write_text(EXAMPLE.read_text())
        watcher = FloorPlanWatcher(path)
        plan, err = watcher.current()
        assert plan is not None and err is None
        path.write_text(json.dumps({"rooms": "oops"}))
        later = time.time() + 5
        os.utime(path, (later, later))
        plan2, err2 = watcher.current()
        assert plan2 is plan, "an invalid edit must keep the last valid plan"
        assert err2 is not None and "rooms" in err2


def test_concurrent_reload_never_returns_empty():
    """A reader arriving while another thread is mid-load must get the plan, not None."""
    import floorplan as fp
    real_load = fp.FloorPlan.load.__func__

    def slow_load(cls, path):
        time.sleep(0.3)
        return real_load(cls, path)

    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "floorplan.json"
        path.write_text(EXAMPLE.read_text())
        watcher = FloorPlanWatcher(path)
        fp.FloorPlan.load = classmethod(slow_load)
        try:
            t = threading.Thread(target=watcher.current)
            t.start()
            time.sleep(0.05)
            plan, _ = watcher.current()
            t.join()
        finally:
            fp.FloorPlan.load = classmethod(real_load)
        assert plan is not None


def _serve(app):
    import presence_server
    srv = ThreadingHTTPServer(("127.0.0.1", 0), presence_server.make_handler(app))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}"


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read())


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def test_api_contract():
    import presence_server
    app = presence_server.App([], presence_server.Settings(10.0), record=False, reset_on_start=False,
                              floorplan_path=EXAMPLE)
    srv, base = _serve(app)
    try:
        _post(base + "/api/robots", {"robots": [{"id": "A", "mac": "d0:cf:13:e1:63:30", "x": 1.2, "y": 3.4}]})
        stops = _get(base + "/api/stops")["stops"]
        expected = json.loads(EXAMPLE.read_text())["stops"]
        assert [(s["name"], s["a"], s["b"], s["listen_s"]) for s in stops] == \
               [(s["name"], s["a"], s["b"], float(s["listen_s"])) for s in expected]
        scan = _get(base + "/api/scan")
        assert set(scan) >= {"rooms", "robots", "detections", "warnings"}
        assert [r["status"] for r in scan["rooms"]] == ["unknown", "unknown"]
    finally:
        srv.shutdown()


def test_bad_robot_payloads_rejected():
    """Review Important 6/7: missing mac, invalid JSON, non-object bodies and NaN were accepted or crashed."""
    import presence_server
    app = presence_server.App([], presence_server.Settings(10.0), record=False, reset_on_start=False,
                              floorplan_path=EXAMPLE, rooms_loop=False)
    srv, base = _serve(app)

    def post_raw(body: bytes) -> int:
        req = urllib.request.Request(base + "/api/robots", data=body, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    try:
        assert post_raw(b'{"robots": [{"id": "A", "x": 1, "y": 2}]}') == 400          # no mac
        assert post_raw(b"not json") == 400
        assert post_raw(b"[1, 2]") == 400
        assert post_raw(b'{"robots": [{"id": "A", "mac": "aa", "x": NaN, "y": 2}]}') == 400
        assert post_raw(b'{"robots": [{"id": "A", "mac": "aa", "x": 1, "y": Infinity}]}') == 400
        assert post_raw(b'{"robots": [{"id": "A", "mac": "aa", "x": 1, "y": 2}]}') == 200
    finally:
        srv.shutdown()


def test_malformed_plan_edits_reported():
    """Review Important 8: structural mistakes raised TypeError into a caller and were never reported."""
    for bad in ({"stops": [[1, 2]]}, {"stops": [{"a": [0, 0], "b": [1, 1], "listen_s": None}]},
                {"walls": [5]}, {"rooms": [{"name": "X", "polygon": 7}]}):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "floorplan.json"
            path.write_text(EXAMPLE.read_text())
            watcher = FloorPlanWatcher(path)
            good, _ = watcher.current()
            data = json.loads(EXAMPLE.read_text())
            data.update(bad)
            path.write_text(json.dumps(data))
            later = time.time() + 5
            os.utime(path, (later, later))
            plan, err = watcher.current()
            assert plan is good, bad
            assert err is not None, bad
            plan2, err2 = watcher.current()            # still reported on the next call
            assert err2 is not None, bad


def test_static_files_served_offline():
    import presence_server
    app = presence_server.App([], presence_server.Settings(10.0), record=False, reset_on_start=False,
                              floorplan_path=EXAMPLE, rooms_loop=False)
    srv, base = _serve(app)
    try:
        with urllib.request.urlopen(base + "/static/vendor/three/three.module.js", timeout=5) as r:
            assert r.headers["Content-Type"].startswith("text/javascript")
            assert b"REVISION = '160'" in r.read(400)
        try:
            urllib.request.urlopen(base + "/static/../presence_server.py", timeout=5)
            raise AssertionError("path traversal must be refused")
        except urllib.error.HTTPError as e:
            assert e.code == 404
    finally:
        srv.shutdown()


if __name__ == "__main__":
    test_load_example()
    test_invalid_edit_keeps_last_plan()
    test_concurrent_reload_never_returns_empty()
    test_api_contract()
    test_static_files_served_offline()
    test_bad_robot_payloads_rejected()
    test_malformed_plan_edits_reported()
    print("ok")
