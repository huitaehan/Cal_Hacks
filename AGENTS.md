# Agent guide: WiFi rescue robots (CalHacks 13.0)

Four ESP32-C5 boards around an area sense a moving person through the WiFi signals between them (CSI, channel
state information), with no camera on anyone. A laptop server turns that into "someone is here, at (x, y)", and a
3-wheel omni robot driven by a Raspberry Pi goes to them. An overhead camera only tracks the robots (it stands in
for the SLAM a real robot would use). Read this before changing anything; each folder's README has the details.

## Layout

| Path | What it is |
|---|---|
| `wifi detection/firmware/c5_presence_node/` | ESP-IDF firmware for every sensing board (same image on all). Boards ping each other over ESP-NOW on 2.4 GHz channel 6; the board on the laptop's USB (the gateway) prints its own measurements and relays everyone else's. Flash: `wifi detection\firmware\build_flash.ps1 -All`. |
| `wifi detection/host/presence_server.py` | Reads the gateway, runs the detector per link, locates the person, serves the dashboards and `GET /api/state` on port 8000. Start: `wifi detection\run_dashboard.ps1 -Record`. |
| `wifi detection/host/wander.py` | The motion detector (one per link). Tuning lives here; see "Decisions". |
| `wifi detection/host/floorplan.json` | Search area, board positions (by MAC) and walls, in metres. The server reloads it live. Written by `robot pipeline/wifi_area.py`. |
| `wifi detection/host/static/` | `index.html` (2D dashboard, `/`), `scene3d.html` (3D view, `/3d`), `mission.html`. |
| `wifi detection/host/recordings/` | Raw recordings from `-Record`, used to replay and tune. New ones are git-ignored (too big). |
| `robot pipeline/` | Camera tracking of the robots' colour tops, path planning around walls, mission logic, driving, the Pi controller and the setup tools. `run.py` runs it all. |
| `robot pipeline/pi_robot.py`, `pi/` | The robot's controller on the Raspberry Pi (port of `firmware/robot_motor`, same UDP protocol). Installed as `pi-robot.service`. |
| `wifi detection/RuView`, `wifi detection/third_party/espectre` | Upstream projects (git links, not our code). |
| `wifi detection/snapshots/` | Frozen copy of an earlier best version. Don't edit. |

## How it runs

```
4 boards --ESP-NOW--> gateway board --USB--> presence_server.py --GET /api/state (4 Hz)--> run.py
camera (C922, index 1) --> robot x, y, heading ----------------------------------------------> run.py
run.py --UDP 4210 (D seq speed dir turn | S)--> Pi on robot A --> 3 motors   (Pi announces HELLO on UDP 4211)
```

1. Boards placed and powered, gateway `2e:30` on the laptop's USB.
2. `cd "wifi detection"; .\run_dashboard.ps1 -Record`. Keep everyone 1 m back from the tape for 30 s.
3. Laptop and Pi on the phone hotspot `rescuebots` (ask the team for passwords; never commit them).
4. `cd "robot pipeline"; .\.venv\Scripts\python.exe run.py`. Keys: right-click floor = manual move, p pause,
   r recall, c clear found marks, q quit. Events also go to `mission_log.txt`.

Setup tools, all in `robot pipeline` with `.\.venv\Scripts\python.exe`:
- `setup_camera.py`: camera over the taped zone (aim, colours, click the 4 tape corners, check). It also moves the
  WiFi boards in the floor plan to the new zone size and prints their spots.
- `wifi_area.py <width cm> <depth cm> [--out cm] [--layout corners|edges]`: WiFi board spots only.
- `calibrate.py A`: which way robot A drives relative to its top (window; SPACE starts). Redo only if the top is
  remounted.
- `copy_to_pi.ps1`: puts `pi_robot.py` and `pi/` on a Pi (SD card or `-Ssh robot@robota.local`); then
  `sudo bash .../install.sh A`.

## Current setup (2026-10-25)

- Frame: origin = front-left tape corner, x to the right, y away from the laptop side, metres.
- Area: WiFi floor plan 1.97 x 2.38 m; camera zone in `config.json` 1.97 x 2.45 m. **Open: these disagree; measure
  the tape and fix one** (`wifi_area.py 197 245` or rerun `setup_camera.py` with 238).
- Boards on the corners, waist height, 18 cm out from each tape corner in x and y:

  | Board MAC | Corner | Note |
  |---|---|---|
  | `d0:cf:13:e1:2e:30` | front-left | gateway, on the laptop's USB (COM15) |
  | `d0:cf:13:e1:63:30` | front-right | |
  | `d0:cf:13:e1:66:2c` | back-right | |
  | `d0:cf:13:e1:63:ac` | back-left | |

- Robot A only. Raspberry Pi 4, hostname `robota`, user `robot`, Raspberry Pi OS 13 (Trixie). Motor driver on
  GPIO 17/27 (motor A), 22/23 (B), 24/25 (C). Wheels: A front-right, B front-left, C back. No IMU: heading comes
  from the camera. Its top is the 2 pink + 2 green pattern, so `config.json` has A and B's patterns swapped
  relative to the original plan. `calibration.json`: direction 0 = pattern heading +178 deg, mirrored.
- The Pi joins the iPhone hotspot (5 GHz, WPA2/WPA3 mixed) only with NetworkManager profile `rescuebots` set to
  `wifi-sec.key-mgmt wpa-psk` and `wifi-sec.pmf 2`; plain WPA3 association fails on the Pi 4's chip.
- Robot B has no Pi yet; walls are not built yet.

## Where to change what

| To change | Edit |
|---|---|
| Detection sensitivity | `wander.py` `THRESHOLD` (log of wander over the link's calm level; 1.4 = 4x) |
| Robot speeds | `config.json` `"drive"`: `speed_min`, `speed_max`, `slow_m`, `arrive_m` |
| Follow behaviour | `config.json` `"mission"`: `follow`, `follow_lost_s`; defaults in `mission.py` `MissionConfig` |
| Area size / board spots | `setup_camera.py` (both) or `wifi_area.py` (WiFi only) |
| Walls | `"walls"` in `floorplan.json`: `[[x1, y1], [x2, y2]]` per wall, metres; the planner keeps 13 cm clear |
| Robot pins | `pi_robot.py --pins` (BCM numbers, order motor A fwd/back, B, C) |
| Robot command protocol | `drive.py` `RobotLink` and `pi_robot.py` together |
| Which top is which robot | `config.json` `"pattern"` per robot |

## Decisions (don't undo without new data)

- **`THRESHOLD = 1.4`** (was 1.8). On the labelled test recording (Oct 24), any one board pair above it caught
  98-99 % of someone moving inside, with 1-4 % false alarms with nobody near and about a third for someone walking
  0.5-1 m outside the tape. The team chose fewer misses over fewer false alarms; 1.5 is the fallback.
- **`TARGET_RATE = 14`** packets/s in `wander.py`: a link arriving faster keeps a fixed every-k-th packet. The
  gateway measures its own links at 40-46/s, relayed links arrive at about 16/s, and judged on every packet the
  gateway's links read numb (Oct 25: front edge fired 2.8 % of the time vs 17-24 %; with thinning 10.8 %). The
  choice is sticky on purpose: a per-window or flipping pick added noise and false alarms.
- **Follow mode** (`"follow": true`): the robot heads straight for 45 cm short of each WiFi position and re-aims
  every 25 cm of movement; no settling, holding or listening countdowns. Back to its spot after 8 s without
  motion. A detection also ends a manual move.
- **`run.py` uses `http://127.0.0.1:8000`**, not `localhost`: on Windows `localhost` tries IPv6 first and each
  request took 0.55 s instead of 0.02 s.
- **Boards on the corners** cover the zone's corners better than edge middles (100 % of the zone seen by 2+ links
  vs 86 %), but pick up about 4x more of the area around the tape (40 % vs 11 % of a 20 cm-1 m band). Keep
  onlookers 1 m back.

## Rules and gotchas

- Don't edit `wifi detection/host/static/index.html`: it's the owner's working 2D dashboard.
- The camera only tracks robots. Never use it to find people; that's the point of the project.
- Use each folder's `.venv\Scripts\python.exe`. Plain `python` on the dev laptop is the Microsoft Store Python,
  which can't open cameras, and OpenCV 5 can't open cameras by index on Windows (the venvs pin 4.x).
- Only one program can use the camera at a time (`run.py`, `tracker.py`, `setup_camera.py`, `calibrate.py`).
- ESP-IDF can't build in a path with spaces or an apostrophe; `build_flash.ps1` builds in `C:\esp\work`.
- A board plugged into USB can sit silent until reset; the server resets boards when it starts and on Recalibrate.
- Never commit WiFi or Pi passwords. `firmware/robot_motor/wifi_config.h` holds placeholders only.
- Strong 2.4/5.8 GHz transmitters break it (a DJI drone's video link stopped the demo once).

## Known limits and open work

- Detects moving people only; position is good to about 0.5-1 m; two people show as one.
- The gateway's pairs are measured in one direction only. Pairs measured both ways (averaged) are steadier; a
  firmware change relaying the gateway's beacons as measured by the others would help most.
- One "drove the wrong way" stop at the back-left corner, farthest from the camera, likely a misread heading.
  Check `setup_camera.py` step 4's heading percentage there.
- No labelled test yet in the current area: walk set spots with `-Record` on, then replay to tune.
- Robot B needs a second Pi (`copy_to_pi.ps1`, `install.sh B`) and calibration.

## Tests

From each folder, run every `test_*.py` with that folder's `.venv\Scripts\python.exe`; each prints `ok`
(`robot pipeline`: 12 files; `wifi detection/host`: 11 files). `test_pipeline.py` runs the whole robot side end to
end with a simulated camera, robots and WiFi server.
