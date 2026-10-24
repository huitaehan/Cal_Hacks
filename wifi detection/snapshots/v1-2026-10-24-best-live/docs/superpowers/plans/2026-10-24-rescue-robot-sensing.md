# Rescue-Robot WiFi Sensing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mark each room of the demo building occupied / empty / unknown from WiFi links between two robot-mounted ESP32-C5 boards, a fixed anchor and a USB gateway, and show it on a mission map.

**Architecture:** The existing 4-board ESP-NOW mesh and `host/presence_server.py` stay. New host modules add the following: a floor plan with rooms and planned stops; a robot-position input from the team's camera tracking, which gates link data to moments when robots are stopped; a per-link detector backend that can be ESPectre's calibration-free model; and a room-evidence accumulator that feeds `/api/scan` and a `/mission` page.

**Tech Stack:** Python 3.13 (venv `wifi detection/.venv`), numpy, pyserial, stdlib `http.server`; ESPectre Python reference code (GPLv3) from `third_party/espectre`; three.js 0.160 / Canvas 2D for pages; ESP-IDF 5.5.6 firmware (only if Task 8 runs).

**Spec:** `wifi detection/docs/rescue-robot-sensing-design.md`

**Repo note:** `wifi detection` is not a git repository, so there are no commit steps. Each task's checkpoint is its passing test run. Tests are plain scripts in the existing style (`python test_x.py` prints `ok`), run from `wifi detection/host` with `..\.venv\Scripts\python.exe`.

## Global Constraints

- All positions are **floor-plan metres (x, y)** with the floor plan's origin; the camera team converts pixels.
- Robot positions arrive as `POST /api/robots` with `{"robots": [{"id": "A", "mac": "<sensing board MAC>", "x": 1.2, "y": 3.4}]}` at 5–10 Hz.
- A robot counts as **stopped** when it moved less than **5 cm over 2 s**. Link data is used only while both ends are stopped, after **1 s** settle time.
- Stops last about **15 s**; `GET /api/scan` gives `stop_done` per robot; polling it is optional for navigation.
- Rooms no link has crossed must show **unknown**, never empty.
- The camera may only track robots; people detection is WiFi only.
- Robot motor firmware and its UDP control path are untouched; sensing boards are separate.
- Sensing mesh stays on **channel 6**, separate from the router's channel.
- Mission map at `http://localhost:8000/mission`; existing `/`, `/3d`, `/ruview-3d` keep working.
- ESPectre code is GPLv3; keep it under `third_party/espectre`, unmodified, imported by path.
- Pages must render correctly at **devicePixelRatio 1.25** (the user's laptop) and 1.0.

## Review Focus

1. **Tracking stops sending** (camera script crashed): a robot's last position must go stale after **1 s** and count as moving, so frozen positions don't add evidence. Test in Task 4.
2. **A board MAC posted for no robot, or a robot posted without a known MAC:** its links get no position. `/api/scan` must list it under `warnings` instead of silently ignoring it. Test in Task 4.
3. **A detector not ready on a slow link** (below ESPectre's 70% slot coverage): it must give `ratio None`, which adds no evidence. It must never be read as "quiet = empty". Test in Task 3.
4. **A room that no link has crossed:** it stays `unknown` even after other rooms get strong evidence. Test in Task 5.
5. **Invalid `floorplan.json` edit while running:** keep the last valid plan and report the error in `/api/scan.warnings`. Test in Task 1.

---

### Task 1: Floor plan, API contract and stub endpoints (unblocks the robot team)

**Files:**
- Create: `host/floorplan.py`, `host/floorplan.example.json`, `host/test_floorplan.py`, `docs/robot-team-api.md`
- Modify: `host/presence_server.py` (routes `POST /api/robots`, `GET /api/stops`, `GET /api/scan`; `App` gets `floorplan` and `robot_positions`)

**Interfaces:**
- Produces:
  - `FloorPlan.load(path: Path) -> FloorPlan` raises `ValueError` on invalid content
  - `FloorPlan.rooms: list[Room]`, `Room(name: str, polygon: list[tuple[float, float]])`, `Room.contains(x, y) -> bool`
  - `FloorPlan.boards: dict[str, tuple[float, float]]` (fixed anchor/gateway MAC → position)
  - `FloorPlan.stops: list[Stop]`, `Stop(name: str, a: tuple[float, float], b: tuple[float, float], listen_s: float = 15.0)`
  - `FloorPlanWatcher(path).current() -> tuple[FloorPlan | None, str | None]` (reloads on mtime; keeps last valid plan; returns error text)
  - JSON file shape: `{"rooms": [{"name", "polygon"}], "boards": {"<mac>": [x, y]}, "walls": [[[x1,y1],[x2,y2]], ...], "stops": [{"name", "a": [x,y], "b": [x,y], "listen_s"}]}`

- [ ] **Step 1: Write failing tests** in `host/test_floorplan.py`:
  - `test_load_example`: loads `floorplan.example.json` (2 rooms "Room A" `[[0,0],[2,0],[2,2],[0,2]]` and "Room B" `[[2,0],[4,0],[4,2],[2,2]]`, 2 stops); asserts `len(rooms) == 2`, `rooms[0].contains(1, 1)` and `not rooms[0].contains(3, 1)`, `stops[0].listen_s == 15.0`.
  - `test_invalid_edit_keeps_last_plan`: watcher on a temp file; write valid JSON, call `current()`, overwrite with `{"rooms": "oops"}` and bump mtime; `current()` returns the previous plan plus an error string containing `rooms`.
  - `test_api_contract`: `make_handler` with a fake app, `POST /api/robots` with the Global Constraints payload, then `GET /api/stops` equals the example stops and `GET /api/scan` gives every room `status == "unknown"`.
- [ ] **Step 2: Run** `..\.venv\Scripts\python.exe test_floorplan.py`. Expected: ImportError / AssertionError.
- [ ] **Step 3: Implement** `floorplan.py` (point-in-polygon by ray casting) and the three routes. `POST /api/robots` stores `{mac: (id, x, y, received_t)}` for now. `GET /api/scan` returns `{"rooms": [{"name", "status": "unknown", "confidence": 0}], "robots": [], "detections": [], "warnings": [...]}`.
- [ ] **Step 4: Add a `--host` CLI option** to `presence_server.py` (default `127.0.0.1`). `--host 0.0.0.0` lets the robot team's laptop reach the API. Print the machine's LAN IP at startup.
- [ ] **Step 5: Write `docs/robot-team-api.md`:** the three endpoints, payload examples, coordinate rules and the 15 s stop convention, so the robot team can code against it today. Include:
  - which sensing board MAC goes on which robot,
  - how to find the base computer's IP,
  - a 10-line Python example that POSTs positions.
- [ ] **Step 6: Run** `test_floorplan.py`, `test_detector.py`, `test_localizer.py`, `test_ruview_bridge.py`. Expected: all print `ok`.

### Task 2: ESPectre link adapter and replay gate (decides the detector)

**Files:**
- Create: `host/espectre_link.py`, `host/test_espectre_link.py`, `host/tools/replay_compare.py`, `docs/replay-results.md`

**Interfaces:**
- Consumes: ESPectre `micro_espectre.device_utils.normalize_ht20_csi_payload`, `prepare_ht20_detector_input`, `lib/temporal_replay.TemporalReplayController`, `apply_temporal_admission`, `lib/high_accuracy_detector.HighAccuracyDetector` (paths: `third_party/espectre/src/python`, `third_party/espectre/tools/lib`)
- Produces:
  - `amplitudes_to_csi_bytes(amp: np.ndarray) -> bytes`: for 57 amplitudes, 114 bytes as (imag=0, real=min(127, round(a))) pairs, which then go through ESPectre's own normalizer (pad 4 left / 3 right → 128 B, DC on bin 32)
  - `class EspectreLinkDetector` with `push(t_s: float, csi: bytes | np.ndarray) -> None`, `update(now_s: float) -> None`, `snapshot(now_s: float) -> dict` with the same keys as `LinkDetector.snapshot` (`ratio = probability / threshold`, `calibrated = is_ready()`, `calib_left = None`, `motion = probability`, `threshold = 0.5`), `reset_calibration() -> None`; constructor `(target_pps: int = 100, window_ms: int = 1000, eval_ms: int = 250)`

- [ ] **Step 1: Write failing tests** in `host/test_espectre_link.py`:
  - `test_layout`: `amplitudes_to_csi_bytes(np.arange(57))` normalized by ESPectre gives 128 bytes. The real byte of bin 32 (= our index 28, the DC bin) equals 28, and bins 0–3 and 61–63 are zero.
  - `test_quiet_then_motion`: synthetic 50 pps stream, 20 s of constant amplitudes plus small white noise, then 10 s where bands of subcarriers swing ±30% at 1 Hz. Asserts mean ratio over the quiet part < 1 and the max over the motion part > 1 (target_pps=50).
  - `test_slow_link_not_ready` (Review Focus 3): a 5 pps stream gives `snapshot()["ratio"] is None`.
- [ ] **Step 2: Run** `test_espectre_link.py`. Expected: FAIL (module missing).
- [ ] **Step 3: Implement `espectre_link.py`.** Timestamps go to the sampler in µs. Every admitted packet goes through `apply_temporal_admission` then `process_packet`; `update_state()` runs every `eval_ms`. Probability comes from `get_motion_metric()`.
- [ ] **Step 4: Run** `test_espectre_link.py`. Expected: `ok`.
- [ ] **Step 5: Write `tools/replay_compare.py <recording.log> [--target-pps N]`.** It replays raw `CSI,` lines (old format, real I/Q) and `CSIA,`/`RLY,` lines (amplitude format) per link through both `LinkDetector` and `EspectreLinkDetector`. It prints a per-10 s table of ratio median/max for each, plus the % of time each is "detected".
- [ ] **Step 6: Run it** on `recordings/20261024_010613_COM13.log` and `recordings/20261024_010614_COM15.log` (walks at about 70–110 s and 120–145 s, quiet 0–60 s), and on the latest mesh recordings, at `--target-pps 100` and `50`. Also run the raw I/Q recordings converted to rounded amplitudes, to confirm the amplitude firmware loses nothing.
- [ ] **Step 7: Record the decision** in `docs/replay-results.md` and the design's "How detection works":
  - **ESPectre wins** if it catches both walks on both links (max ratio > 1 within each walk), with false detection ≤ 10% of quiet time and **no calibration**.
  - Otherwise **fallback**: keep `LinkDetector` and run Task 7.
  - Note whether 100 pps mattered. If it did, run Task 8.

### Task 3: Selectable detector backend per link

**Files:**
- Modify: `host/presence_server.py` (`Link.__init__`, `Link.push`, `Settings`, CLI `--detector espectre|denoised`)
- Test: `host/test_links.py` (create)

**Interfaces:**
- Consumes: `EspectreLinkDetector` (Task 2), `LinkDetector`
- Produces: `Settings.detector: str` (`"espectre"` or `"denoised"`, default from the Task 2 decision); `Link.detector` is either class, with an identical `snapshot()` shape

- [ ] **Step 1: Write failing tests:**
  - `test_backend_selected`: `Link("aa", "bb", Settings(10, detector="espectre")).detector` is an `EspectreLinkDetector`.
  - `test_ratio_none_adds_nothing`: `Localizer` with one link whose snapshot ratio is None produces no pair and `person is None` (Review Focus 3).
- [ ] **Step 2: Run** `test_links.py`. Expected: FAIL.
- [ ] **Step 3: Implement.** `Link.push` hands raw bytes or amplitudes to the backend. `CSI,` lines keep raw I/Q bytes for ESPectre; `CSIA`/`RLY` pass amplitudes.
- [ ] **Step 4: Run** all host tests. Expected: all `ok`.

### Task 4: Robot positions and stop gating

**Files:**
- Create: `host/robots.py`, `host/test_robots.py`
- Modify: `host/presence_server.py` (`POST /api/robots` → `RobotTracker.update`; `Link.push` drops samples unless both ends are stopped; `Localizer` uses robot positions over `layout.json` for robot MACs)

**Interfaces:**
- Produces:
  - `RobotTracker.update(robots: list[dict], now: float) -> None`
  - `RobotTracker.position(mac: str, now: float) -> tuple[float, float] | None`: None if older than **1 s**
  - `RobotTracker.is_stopped(mac: str, now: float) -> bool`: moved < 0.05 m over the last 2 s, stopped for ≥ 1 s settle, and fresh
  - `RobotTracker.stop_started(mac) -> float | None`
  - `RobotTracker.warnings(known_macs: set[str]) -> list[str]`
  - `RobotTracker.snapshot(now) -> list[dict]` (`id, mac, x, y, stopped, stop_s`)
- Fixed boards (`FloorPlan.boards`) always count as stopped. **Sentry mode** = listing the parked robot boards in `FloorPlan.boards` too: they then work with no tracking at all. A fresh tracker position for a MAC overrides its floor-plan entry.

- [ ] **Step 1: Write failing tests** in `test_robots.py`:
  - `test_stop_detection`: positions moving 10 cm/s for 3 s give not stopped. The same position held for 3.2 s gives stopped (2 s window + 1 s settle).
  - `test_stale_positions` (Review Focus 1): last update 1.5 s ago gives `position() is None` and `is_stopped() is False`.
  - `test_unknown_mac_warning` (Review Focus 2): robot posted with a MAC no board reports, and a board MAC with no robot or floor-plan entry, each give one warning string naming the MAC.
  - `test_gating`: a `Link` between a moving robot and the anchor ignores pushes (`detector` receives no samples). Once both are stopped, it receives them.
  - `test_sentry_static_boards`: with no `/api/robots` posts at all, two MACs listed in `FloorPlan.boards` count as stopped, and their link receives samples.
- [ ] **Step 2: Run** `test_robots.py`. Expected: FAIL.
- [ ] **Step 3: Implement** `robots.py` and the wiring. When a stop starts, clear that link's detector window, so evidence from the previous spot can't leak in.
- [ ] **Step 4: Run** all host tests. Expected: all `ok`.

### Task 5: Room evidence, `/api/scan` and `/api/stops`

**Files:**
- Create: `host/rooms.py`, `host/test_rooms.py`
- Modify: `host/presence_server.py` (a background loop at 4 Hz feeds `RoomEvidence` from stopped links; the scan endpoint returns real statuses)

**Interfaces:**
- Consumes: `FloorPlan`, `RobotTracker`, link snapshots (`tx, rx, ratio, detected`), positions
- Produces:
  - `RoomEvidence(plan: FloorPlan, ellipse_m: float = 0.35, half_life_s: float = 120)`
  - `RoomEvidence.add(a: tuple, b: tuple, ratio: float, now: float, dt: float) -> None`
  - `RoomEvidence.status(now) -> list[dict]` (`name, status, confidence, coverage_s`)
  - Rule: a room's coverage = the fraction of its area (sampled on a 0.1 m grid) inside the link ellipse (excess path < `ellipse_m`).
  - Evidence += coverage × dt × (ratio − 1) when ratio ≥ 1; −= coverage × dt × 0.5 when ratio < 0.8.
  - Decays with `half_life_s`. Status is **unknown** if `coverage_s` < 5 s; **occupied** if evidence > +1.5; **empty** if evidence < −1.5; otherwise unknown. `confidence = min(1, |evidence| / 3)`.

- [ ] **Step 1: Write failing tests** in `test_rooms.py`, using the example plan:
  - `test_link_through_room_a`: 20 s of ratio 2.5 on a link from (0.2, 1) to (1.8, 1) gives Room A **occupied** and Room B **unknown** (Review Focus 4).
  - `test_quiet_link_marks_empty`: 20 s of ratio 0.4 through Room B gives Room B **empty**.
  - `test_decay`: after the occupied case, 600 s with no links gives Room A no longer occupied.
  - `test_scan_shape`: `/api/scan` has keys `rooms, robots, detections, warnings`, and `detections` lists `"Person likely in Room A"` with confidence.
- [ ] **Step 2: Run** `test_rooms.py`. Expected: FAIL.
- [ ] **Step 3: Implement** `rooms.py` and the loop. `stop_done` per robot is true when stopped ≥ the current stop's `listen_s`, or when every link touching it has ≥ 10 s of ready detector data.
- [ ] **Step 4: Run** all host tests. Expected: all `ok`.

### Task 6: Mission views, 3D first (the judge-facing screen)

**Files:**
- Modify: `host/static/scene3d.html` (becomes the 3D mission view at `/3d`, still working without a floor plan)
- Create: `host/static/mission.html` (2D backup at `/mission`)
- Modify: `host/presence_server.py` (`GET /mission`; `/events` state includes `scan`, `robots`, `floorplan`)

**Interfaces:**
- Consumes: the `/events` state with `floorplan` (rooms, walls, boards, stops), `scan`, `robots`, `links`

- [ ] **Step 0: Build the 3D mission view** in `scene3d.html`:
  - floor-plan walls extruded as cardboard-coloured boxes (1 m high, 0.03 m thick);
  - each room's floor tinted by status (occupied = pulsing red glow with the person figure standing at the room's centre, or the localizer estimate if it falls inside that room; empty = soft green; unknown = grey);
  - the two robots as small 3-wheel discs with an ID label, moving smoothly to live `/api/robots` positions, with a ring while stopped and listening;
  - the anchor and gateway on posts; links as beams between boards that brighten with ratio;
  - an overlay with "Person likely in Room A — 82%" and the current stop.
  - Without a floor plan it falls back to today's board-layout view.
  - Verify with the fake server at `device_scale_factor=1.25`. Expected: walls visible, Room A glowing with a figure, robots at their posted positions, no console errors.
- [ ] **Step 1: Build the 2D backup page** `mission.html`.
  - Canvas 2D top-down map scaled to the floor plan, sized with the `dataset.cssHeight` pattern from `index.html`, so it doesn't grow at DPR ≠ 1.
  - Rooms filled by status: occupied = critical `#d03b3b` with a person icon; empty = good `#0ca30c`; unknown = neutral grey hatched.
  - Walls; the fixed boards; robots as labelled circles with stopped/moving; active links as lines coloured by ratio.
  - A side list "Person likely in Room A — 82%", the stop plan with the current stop highlighted, and warnings.
  - Same dark/light tokens as `index.html`.
- [ ] **Step 2: Verify with the fake server pattern** (scratchpad `fake_mesh_server.py` extended with a floor plan, two robots and Room A occupied). Screenshot with Playwright at `device_scale_factor=1.25`, 1229×768. Expected: Room A red, Room B green or grey, page height stable over 5 s, no console errors.
- [ ] **Step 3: Verify live** with the real boards placed by hand around two "rooms". Expected: walking in one room turns it occupied within about 15 s of a stop.

### Task 7 (only if Task 2 chose the fallback): Survey mode

**Files:**
- Create: `host/survey.py`, `host/test_survey.py`
- Modify: `host/presence_server.py` (`POST /api/survey/start`, `POST /api/survey/stop`; `LinkDetector` thresholds come from the survey table by stop name, or from the RSSI fit for unknown spots)

- [ ] **Step 1: Write failing tests:** `test_survey_baseline_used` (a link at a surveyed stop gets that stop's threshold) and `test_rssi_fit` (an unsurveyed spot uses `threshold = f(rssi)` from a least-squares fit over the survey table).
- [ ] **Step 2: Implement:** survey = for each stop, the median/MAD of the quiet score per link pair, stored in `host/survey.json`.
- [ ] **Step 3: Run** all host tests. Expected: all `ok`.

### Task 8 (only if Task 2 showed 100 pps matters): 100 Hz beacons

**Files:**
- Modify: `firmware/c5_presence_node/main/main.c` (`PN_TX_HZ` 100; `PN_RELAY_MIN_US` 20000)

- [ ] **Step 1: Build and flash** all four boards one at a time (`firmware\build_flash.ps1 -All`).
- [ ] **Step 2: Measure** with `peek_mesh.py` (in the scratchpad). Expected: own links ≥ 70 pps at ≥ -60 dBm, `dropped` stays 0, USB ≤ 30 KB/s.

### Task 9: Demo hardening and handover

**Files:**
- Modify: `run_dashboard.ps1` (open `/mission`), `README.md`, `docs/robot-team-api.md`

- [ ] **Step 1: Failure drills,** each with an expected result:
  - Unplug and replug the gateway: links resume within 5 s.
  - Power-cycle a robot board: its links resume and evidence keeps building.
  - Restart the server: rooms return to unknown, then rebuild.
  - Stop the tracking script: robots go stale and no new evidence is added.
  - Check the router's WiFi channel: it must not be channel 6 (the sensing mesh's channel); change the router if it is.
- [ ] **Step 2: Document** the judging flow in the README: placement, optional survey, sentry mode, how to run the recorded mission.
