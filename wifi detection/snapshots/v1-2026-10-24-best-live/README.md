# WiFi detection

WiFi sensing for the rescue robots. ESP32-C5 boards on the robots, plus a fixed anchor, find which rooms of a building have a person in them, and the base station shows it in 3D.

Design: `docs/rescue-robot-sensing-design.md` · Robot-team interface: `docs/robot-team-api.md` · Detector comparison: `docs/replay-results.md`

## How it works

- **Boards** (`firmware/c5_presence_node`, ESP-NOW on 2.4 GHz channel 6, no router needed): each board broadcasts 50 packets/s and records the CSI of every other board's packets. It also relays its own measurements in its packets, so one board on USB at the base computer receives every link. RuView's `edge_processing.c` is copied in unmodified; it runs on the boards, but its on-device flag isn't used for alarms.
- **Per-link detector** (`host/espectre_link.py`): ESPectre's calibration-free "High Accuracy" model (third_party/espectre, GPLv3), with its slot rate matched to each link's real packet rate. It needs no quiet-room calibration.
- **Robots** (`host/robots.py`): the robot team posts positions from the overhead camera. A link is only used while both of its ends have stood still for 3 s.
- **Rooms** (`host/rooms.py`, `host/floorplan.py`): each stationary link adds evidence to the floor-plan points inside its sensing ellipse. A room is **occupied** when ESPectre's debounced motion covers part of it, **empty** only after 30 s of quiet listening over most of it with no motion anywhere (confidence capped at 90%), and **unknown** otherwise. A link only counts for the one room holding at least 75% of its ellipse; links split across rooms (diagonals) only feed the x,y estimate.

## Run

```powershell
.\run_dashboard.ps1              # starts the server, opens the 3D view
.\run_dashboard.ps1 -Network     # also lets the robot team's laptop reach /api/robots (prints the URL)
.\run_dashboard.ps1 -Record      # also saves raw CSI to host\recordings\
```

Pages: **/3d** is the 3D mission view, for the judges. **/mission** is a 2D backup map. **/** is the sensor dashboard. **/ruview-3d** is RuView's own Observatory.

Tests: from `host\`, run each `test_*.py` with `..\.venv\Scripts\python.exe` (each prints `ok`).

## Rescue mission mode (floor plan)

1. Copy `host/floorplan.example.json` to `host/floorplan.json` and edit it:
   - room polygons and walls, in metres from the agreed corner of the cardboard house,
   - `boards`: the fixed anchor and the USB gateway positions,
   - `stops`: where robot A and robot B park for each room. Put them on opposite sides of the room, so the link between them crosses it.
2. Start with `-Network`, and give the robot team the printed URL and `docs/robot-team-api.md`.
3. The robots drive the stops from `/api/stops`, both park at each stop for about 15 s, and the 3D view marks rooms occupied, clear or not checked.

**Sentry mode** (the safe live demo): park both robots around the house, list all 4 board MACs with their positions under `boards` in `floorplan.json`, and skip the camera entirely. Rooms update continuously, so a judge can step into one and watch it turn red. Robot positions from the camera override `boards` entries while they're fresh. `host/floorplan.json` is set up for the open 4.5 × 4.0 m test area: the station gateway (`63:ac`, USB) at the door at the front centre, a breadcrumb board (`66:2c`, power bank) 4 m away at the far end, and robots B (left) and A (right) parked 4.5 m apart, halfway in. Boards at chest height and a 1.5-2 m clear zone around the area matter more than any setting (see `docs/replay-results.md`). With these positions, a simulation ranked the breadcrumb at the far end best (every spot covered, median error about 0.1 m with idealised links); a second board at the station barely helped. `host/floorplan.house.json` is the two-room cardboard house version. Both set `"motion_half_life_s": 8` so a room clears about 20 s after the person leaves; without that key, motion is remembered for minutes, as a robot mission needs.

Judging flow: rehearse with the cardboard house empty (both rooms should read "clear"), then with a person moving in one room. Keep onlookers about 1 m away from the outer walls during a run, because links near the walls react to people standing right outside.

## Sensor dashboard (single link, older mode)

Without `floorplan.json` the server uses `host/layout.json` (auto-created board corners). The sensor dashboard at **/** still shows the two-board view. The **Recalibrate** button and **Sensitivity** slider apply to the older `--detector denoised` mode, which learns a 10 s background first.

## Multi-board mesh and 3D view

Every board measures the link from every other board. Each board also includes its latest measurements in the packets it broadcasts, so a single board on USB delivers every link. With N boards that's N×(N−1) one-way links, and links between boards that are only powered are included. The server runs a motion detector on each link. It then combines all links into a position heatmap (radio tomographic imaging): a disturbed link adds weight along the ellipse around its path, and a quiet link subtracts.

- **http://localhost:8000/3d** shows your real setup in 3D, seen from the door side: boards at their positions, links that turn red when disturbed, the floor heatmap, and a red floor marker at the estimated x,y, smoothed by a Kalman filter (`host/tracking.py`). The marker shows while at least two board pairs detect motion and for 1 s after, so short gaps don't make it flicker. Rooms keep their "recent motion" colour for a few seconds after that. Position accuracy is roughly ±0.5–1 m with 4 boards in a few-metre space; in the 1.7 × 1.2 m test area a simulation gives a median of about 0.1 m with idealised links.
- **`host/layout.json`** holds each board's position in metres, keyed by MAC address. It's created with default square corners the first time boards are seen. Edit it to match where the boards actually are; the server reloads it within a second.
- **http://localhost:8000/ruview-3d** is RuView's own Observatory, fed by the same data through `host/ruview_bridge.py`. Its router, WiFi waves and furniture are RuView's built-in scenery, not your setup.

Walking registers best on links you cross directly. Links at weak signal (below about -65 dBm) deliver fewer measurements, so they react a little more slowly.

## Rebuild / reflash

```powershell
.\firmware\build_flash.ps1 -All               # every connected ESP32-C5
.\firmware\build_flash.ps1 -Ports COM10,COM11 # specific ports
```

`-All` checks each board's chip first. It skips boards that aren't ESP32-C5s and ports the dashboard server is using. The server only finds boards when it starts, so restart it after connecting new ones.

This needs ESP-IDF v5.5.6 at `C:\esp\esp-idf`. ESP-IDF can't build in a path containing spaces, so the script copies the project to `C:\esp\work` and builds it there.

## Findings so far (2026-10-23)

- The boards receive about 47 CSI frames/s each, with 57 subcarriers per frame (HT20).
- In a ~5 minute recording, the host detector on both links switched on at the same moments (+95 s, +103 s, +129 s) and stayed quiet otherwise. Two independent links agreeing suggests real motion, not noise.
- RuView's on-device presence flag was on 8–13% of the time, scattered, and did not line up with those events. It tracks the raw phase of one subcarrier, which drifts from packet to packet on ESP-NOW frames, so the alarm uses the host detector by default. RuView's flag is still shown, and the "Alarm from" menu can switch to it.
- Two boards form one link. That tells you whether someone is near the link, not where they are. Locating a person needs 3 or more boards; see the multi-board mesh above.
- 2026-10-24: at distance (-60 to -73 dBm) the original motion score was swamped by noise; walking only raised it 1.2–1.7× above quiet. Averaging subcarriers in bands of 8 and smoothing over 4 packets raised that to 1.9–3.6× on the same recordings.
- "Detect still person" (shift from the calibrated baseline) is experimental. The baseline drifts within minutes in a busy room.
- 2026-10-24: ESPectre, with no calibration, caught both walks on both recorded links, with 0% false detection in the quiet stretch. The denoised detector false-alarmed 31–37% of the time in two quiet 10 s blocks. ESPectre is now the default; details in `docs/replay-results.md`.
