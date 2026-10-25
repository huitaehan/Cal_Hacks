# Robot pipeline

Connects the WiFi detection (`..\wifi detection`) to the robots. Fixed WiFi sensing boards watch the
search area while the robots wait on their spots. When the WiFi system detects someone moving, the nearest
robot drives over, going around the cardboard walls, stops next to them and listens. If the WiFi
detection confirms it, the spot is marked **FOUND**, the robot stays with the person for 20 s, and then
everyone returns to their spots.

```
overhead camera ─ robot colour patterns ─> robot x, y, heading ─────────────────────┐
                                                                                     v
WiFi server person estimate (GET /api/state) ──> mission ──> path around the walls ──> drive commands (UDP) ──> robots
```

The camera only looks for the robots' colour patterns. It never looks for people: people are found only
by the WiFi sensing.

## What's left (needs you)

1. **Robot controller:** a Raspberry Pi on robot A, on the `rescuebots` hotspot, running `pi_robot.py`
   (setup step 4). The 4 ESP32-C5s are all sensing boards. Robot B comes later.
2. **Calibrate** each robot (setup step 5).
3. **Walls:** sketch the cardboard layout inside the taped zone, with each wall's ends in cm from the
   front-left corner. I'll put the walls in the WiFi floor plan and set the robots' parking spots. Then
   build it as in [Making the cardboard match the floor plan](#making-the-cardboard-match-the-floor-plan).
   Keep the walls no taller than the robots' tops, so the camera always sees over them.
4. **Boards:** fixed, at waist height, 25 cm outside the middle of each taped edge (the spots in
   `..\wifi detection\host\floorplan.json`), measured from the same corner marks as the walls. If boards
   ever ride on the robots, rename each robot's `"board_if_carried"` to `"sensing_mac"` in `config.json`.
5. **Guided WiFi test:** confirms the detector and measures crouch, sit and lie detection, plus false alarms
   from onlookers.

| File | What it does |
|---|---|
| `colour_tracker.py` | The robots' SSL-Vision patterns: blue centre blob plus 4 dots. The dot colours say which robot it is (set by `pattern` in `config.json`: now A = 2 pink + 2 green, the top on the robot with the Pi; B = 4 pink), and the dot layout gives the heading (the SSL front dots are wider apart than the back ones). |
| `tracker.py` | Camera thread and drawing. Uses colour tracking, or printed ArUco markers if `"tracking": "aruco"` in `config.json`. |
| `setup_camera.py` | Camera setup over the taped zone in one window: aim, tune the colours, click the 4 tape corners and map the camera onto the robots' tops, then check the border, grid and how reliably each robot is read. Any camera angle works. |
| `planner.py` | Paths around the cardboard walls (A* on a 5 cm grid, kept 13 cm clear of walls), and driving along them. |
| `drive.py` | Drive-to-point for omni robots (no turning needed), with safety stops. UDP link to the robots; robots find the laptop by broadcasting `HELLO`. |
| `mission.py` | sentry → dispatch → verify → found → return; plus pause, recall and manual moves. |
| `wifi_link.py` | Reads the WiFi server's person estimate. Robots carrying a sensing board also report their position, smoothed so camera jitter doesn't look like movement. |
| `run.py` | Everything together, in one operator window. |
| `calibrate.py` | Works out which way each robot drives relative to its pattern. |
| `markers.py` | Only for ArUco mode: writes `markers/print.html`, the markers to print. |
| `firmware/robot_motor/` | The team's motion code (`move`, `dir_move`) plus a WiFi/USB command receiver and a 300 ms safety stop. |
| `pi_robot.py` | The same firmware in Python for a Raspberry Pi, for a robot without a working ESP32: same UDP commands, HELLO and 300 ms stop, so the laptop side doesn't change. Wiring and usage are at the top of the file. |
| `config.json` | Robots (dot pattern, sentry spot), where robots may drive (`area`), robot size, camera, server URL. Same floor frame as `..\wifi detection\host\floorplan.json`. |

Python: use this folder's `.venv\Scripts\python.exe`, a standalone Python 3.12 with numpy and OpenCV 4.x
(`requirements.txt`). Two things that broke the camera before: the Microsoft Store Python can't use cameras,
and OpenCV 5.0 can't open cameras by number on Windows.

## Setup, in order

1. **Camera:** mount it as high as you can, looking over the taped zone. An angle is fine; it doesn't need
   to be straight down. The C922 is camera 1 (the laptop's own webcam is 0).
   - If the sensing boards ride on the robots, mount each stick at the edge of the robot's top, between
     dots, so it doesn't hide the blue centre or a dot.
2. **Camera setup:** run `python setup_camera.py` with robots A and B in view (no other robot tops: a
   second top with B's colours confuses which one is B). It asks, in **cm**, the taped zone's width and
   depth and the robots' height (floor to the dots), then walks through 4 steps in one window, saving as it
   goes:
   1. **Aim:** tilt the camera until all the tape is in view, with about 15 cm spare all round.
   2. **Colours:** press `b` and click the blue centre, `p` and click pink dots, `g` and click green dots,
      a few times each, near and far. Each robot should show 1 blue and 4 dot circles, and nothing else
      should light up. Saves `colours.json`.
   3. **Corners:** click the 4 taped corners in order front-left, front-right, back-right, back-left
      (front = your side); a magnifier follows the mouse. Corners clicked in the wrong order or mirrored
      are refused. The measured depth is checked two ways: by the tape's shape in the image (needs a
      tilted camera) and by the robots' dots, which look squashed when the depth is wrong. Press `z` to
      re-enter the sizes. Saves `floor.json`, and the zone's size as `area` in `config.json` (parking
      spots outside it are moved in).
   4. **Check:** the zone's border (green, on the tape, corners labelled), a 0.5 m grid and each robot's
      position. Stand a robot centred on a tape corner: it should read that corner within about 2 cm. Move
      each robot along the edge farthest from the camera, where the dots look smallest: its "read" share
      should stay near 100 %.
   - Redo step 3 if the camera moves.
   - Why the robots' height: the camera sees the dots on the robots' tops, in line with a floor point
     farther from the camera, so the floor mapping alone would put robots several cm too far out (about
     8 cm at the far side with a 10 cm robot and the camera 1.3 m up). The script works out where the
     camera is from the tape, using the C922's focal length (1360 px at 1920 wide; set
     `"focal_px"` under `"camera"` for another camera), and maps the robots at their own height.
   - If robots A and B come out swapped, swap their `pattern` entries in `config.json`. The robots use the
     SSL dot layout (checked on robot B: front pink pair 1.57 times wider apart than the back green pair;
     the SSL standard is 1.56), so `"pattern_layout": "ssl"`.
3. **Live tracking view, any time:** `python tracker.py`. It shows each robot's arrow, position and heading,
   the zone's border, and the walls.
4. **Robot controller (Raspberry Pi):**
   1. **Network:** a phone hotspot named `rescuebots`, on 5 GHz if the phone offers it (the sensing boards
      use 2.4 GHz channel 6). The laptop joins it too. If the phone can't, the laptop's Windows Mobile
      Hotspot with the same name and password works the same.
   2. **SD card:** Raspberry Pi Imager, Raspberry Pi OS Lite (64-bit). In its settings: hostname `robota`,
      a username and password, WiFi `rescuebots` with its password, WiFi country, and SSH on.
   3. **Files:** with the card still in the laptop, `.\copy_to_pi.ps1` copies `pi_robot.py` and `pi\` onto
      it. Then boot the Pi, `ssh <user>@robota.local`, and run `sudo bash /boot/firmware/robot/install.sh A`
      once: it starts robot A now and at every boot, and turns off WiFi power saving. (Or, on a running Pi:
      `.\copy_to_pi.ps1 -Ssh <user>@robota.local` copies and installs in one go.)
   4. **Wiring:** the motor driver's 6 inputs from the ESP32's pins to the Pi's, as at the top of
      `pi_robot.py`, plus a Pi GND. Check with the wheels off the floor: `sudo systemctl stop pi-robot;
      sudo python3 /opt/robot/pi_robot.py A --spin; sudo systemctl start pi-robot` turns each motor forward
      then back.
   5. **Bench test from the laptop:** `python drive.py A 40 0 1.0` drives robot A at speed 40 in its
      direction 0 for 1 s. If Windows asks, allow Python through the firewall on private networks.
   - The ESP32 version of the same program is `firmware\robot_motor` (`flash_robot.ps1`), for a robot
     with its own ESP32.
5. **Calibrate each robot:** run `python calibrate.py A`, then `python calibrate.py B`, with 0.5 m of free
   floor around each robot. It drives briefly in two directions, saves `calibration.json`, then drives back
   to its start as a check. This also absorbs any difference between the pattern's front and the
   robot's own direction 0. Until a robot is calibrated, `run.py` won't drive it.
6. **Walls:** the walls are the `"walls"` line segments in `..\wifi detection\host\floorplan.json`, in
   metres: `[[x1, y1], [x2, y2]]` per wall, with gaps for the openings. Build and check them as in the
   next section.
7. **Run:**
   1. Start the WiFi server: `..\wifi detection\run_dashboard.ps1`.
   2. Start the pipeline: `python run.py`.
   3. Keep the area clear while the robots reach their spots and settle (about 10 s).

## Making the cardboard match the floor plan

The walls, the camera mapping and the sensing boards are all measured from the same 4 tape marks, so they
can't disagree with each other.

1. **Tape the area's 4 corners first.** Front-left is (0, 0); x runs to the right as seen from the front (your side),
   y away from the door. Check the rectangle is square: both diagonals must be the same length (for
   W × D, √(W² + D²); 6.02 m for 4.5 × 4.0 m).
2. **Each wall is two ends in cm from the front-left mark.** Leave openings of at least 40 cm: the robot is
   18 cm across and keeps 13 cm from every wall line, so it needs 26 cm, and 40 cm leaves room for error.
   Keep walls 20–30 cm tall so the angled camera can see robots behind them.
3. **Tape each wall's ends on the floor**, measuring x along the front edge and y straight in from it.
   For a wall parallel to an edge, measure its offset at both ends so it stays parallel. Stand the cardboard
   with the tape line along the middle of its thickness, and tape or weigh it down so a robot brushing it
   doesn't shift it.
4. **The camera uses the same marks:** in `setup_camera.py` step 3 you click the 4 tape corners.
5. **The sensing boards too:** put each at its floor-plan spot, measured from the same marks, at waist
   height.
6. **Check with the camera:** `python tracker.py` draws the floor-plan walls in red and shows each robot's
   distance to the nearest wall. Push a robot flat against each wall, near both ends: it should read about
   its radius, 9 cm. Within ±3 cm is fine. More than that means the cardboard or its numbers are off;
   at 4 cm off, the robots start touching that wall.

## Operator window (`run.py`)

- It shows the camera view with each robot's arrow, its planned path (yellow), the walls (red), the WiFi
  motion estimate (red circle), found spots (green), the mission state and recent events.
- **Right-click the floor** to send the nearest robot there (manual). A left click does nothing, so
  clicking to focus the window is safe.
- **p** pauses and stops every robot; press it again to resume. While paused, clicks and **r** are ignored.
- **r** recalls all robots to their spots.
- **c** clears the found list.
- **q** quits, which stops every robot.

## How a detection is handled

| Step | Default | Why |
|---|---|---|
| Settle after all robots reach their spots | 5 s | WiFi links from a robot that just stopped need a few still seconds |
| A WiFi position must persist | 2 s | Ignores one-off blips |
| Stop short of the position | 0.45 m | Don't drive into the person |
| Position outside the robots' area or on a wall | nearest reachable free floor | The robot gets as close as it can |
| Blocked on the way (no 5 cm of progress) | 4 s | It listens from where it is instead of pushing |
| Listen next to it | 8 s, ignoring the first 3.5 s | The WiFi server only uses a robot's links once it has been still for 3 s |
| Confirmed if | motion within 1.5 m for 30 % of the listening time | |
| Stay with the person | 20 s | Then all robots return |
| Ignore a found spot | 60 s, within 1 m | So the same person doesn't keep pulling the robot back |
| A robot not back on its spot | 20 s | It's parked where it is, and the others keep watching |

All of these can be overridden under `"mission"` in `config.json`. Driving speeds can be set under
`"drive"` (`speed_min`, `speed_max`, `arrive_m`, `slow_m`); robot size and wall clearance with
`robot_radius_m` and `wall_margin_m`.

## Safety stops

A robot is stopped, not driven, when:
- the camera hasn't seen it in the last 0.2 s, or hasn't seen which way it faces in the last 0.5 s;
- it isn't calibrated;
- it ends up 25 cm farther from its goal, along its path, than it had got (likely a wrong calibration);
- there's no path to its goal around the walls;
- `run.py` stops or the robot's WiFi drops: the firmware stops the motors after 0.3 s without commands.

## Limits to know

- Positions assume the robot height you entered in `setup_camera.py`, and a camera close to the C922's focal
  length; expect about 1 cm of error, more near the image edges where the lens bends straight lines.
- Tall walls hide robots from an angled camera; a robot the camera can't see is stopped.
- The WiFi position is roughly ±0.5–1 m, and two people show up as one.

## Tests

From this folder, run each `test_*.py` with `.venv\Scripts\python.exe`. Each prints `ok`.
`test_pipeline.py` runs everything end to end without hardware: a rendered overhead camera, robots that
obey the real UDP commands, and a fake WiFi server reporting a person behind a wall.
