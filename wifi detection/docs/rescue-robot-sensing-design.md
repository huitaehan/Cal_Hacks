# WiFi sensing for the rescue robots: design

Status: for approval · 2026-10-24 · deadline: demo ready by Sunday 07:00 (about 29 h from writing)

## Intent

Firefighters arrive at a building and don't know who is inside or where. Two small robots go in, stop
in and around rooms, and use WiFi to sense whether each room holds a person. The base station outside
shows a floor plan with rooms marked as occupied or empty, so the team knows where to go. WiFi matters
because it passes through walls, smoke and darkness, where cameras fail.

This document covers the **WiFi sensing and base-station side** only. The team already owns the robots
(omnidirectional 3-wheel, ESP32-C5 motor controllers, driven over UDP from a laptop through a router)
and the overhead-camera tracking and navigation.

## Demo scenario (as decided)

- A small **cardboard building with about 2 rooms**, not built yet. There are **no doors**: robots can
  drive anywhere, including into rooms. One or two people are in the rooms; other rooms are empty.
- People in rooms **move at least some of the time** (shifting, gesturing, waving for help). Detecting a
  perfectly still person is a stretch goal.
- Two robots start at the entrance. Navigation uses an overhead camera tracking the robots (colour now,
  AprilTags later). For the judges, this stands in for the LiDAR a real robot would use. The floor plan
  is known in advance.
- Robots **drive, stop, listen for about 15 s, and drive on**. Sensing happens only while stopped.
- A judged run lasts **about 3 minutes**. The team will **record a full mission on video beforehand**
  and show something live when judges arrive; a full live rescue is not required.
- Success: the base station's map shows the right rooms as occupied or empty.

## Deliverables, in priority order

1. **Live "sentry" demo for judges:** robots parked at fixed spots around and inside the building. Each
   room's status updates continuously, so a judge can step into a room and watch it light up. This is
   nearly what works today and is the safest thing to have at 07:00.
2. **Recorded mission:** robots visit planned stops, rooms get marked one by one on the mission map, and
   the result is screen-recorded alongside the overhead camera.
3. Full live mission during judging, if everything is stable.

## Goals

1. Decide **occupied / empty / unknown for each room** from WiFi alone.
2. A live **mission map** page on the base computer: floor plan, robots, anchor, active links, each
   room's status and confidence, and a plain list such as "Person likely in Room B (82%)".
3. A **simple interface** for the team: robot positions in, planned stops and room status out.
4. Demo-reliable: survives a board reset, a robot drop-out, or a server restart without manual fiddling.

## Non-goals (for this hackathon)

- Exact position inside a room, body pose or skeletons.
- Detecting a completely still, unconscious person (stretch only).
- Sensing while robots drive.
- Self-driving computer vision on the Raspberry Pis.

## Anti-goals (these would make it a failure even if it "works")

- **Using the camera to find people.** The overhead camera may only track robots.
- Changing the robots' motor firmware or their UDP control path. Sensing boards are separate.
- Claiming a room is "empty" with confidence when no link passed through it. Unvisited rooms show
  **unknown**.

## Hardware and radio layout

- **4 ESP32-C5 sensing boards**, all on the existing multi-board firmware (`firmware/c5_presence_node`,
  ESP-NOW on channel 6, every board measures every other board and shares its measurements):
  - one on each robot, a separate board next to the motor ESP32, powered from the robot,
  - one **anchor** on a phone charger at a fixed spot chosen for coverage (e.g. a far corner, so its
    links to the robots cross the rooms),
  - one **gateway** on USB at the base computer. It also senses, but its main job is to receive every
    link through the shared measurements. *(My call: this frees the anchor from the USB cable. The
    gateway must be within radio range of the building, which a hackathon table easily is.)*
- The router stays the robots' control network only, not a sensing node. The sensing channel (6) must
  differ from the router's channel.

## Coordinate frame and interface with the robot team

Everything is in **floor-plan metres** (x, y) with the same origin as the floor plan. The camera team
converts its pixel tracking to metres.

1. **Floor plan file** (`host/floorplan.json`), written by the WiFi side once the cardboard building
   exists: rooms as named polygons, anchor and gateway positions, walls for drawing, and the **stop
   plan** below.
2. **Stop plan, chosen by the WiFi side** *(my call: only the sensing side knows which positions make
   links cross a room)*: a short ordered list of **paired stops**, each giving a position for robot A and
   for robot B, with both robots parked at the same time. Typically each room gets A and B on opposite
   sides, or one inside and one outside, so the A↔B link and the anchor links cross it. With 2 rooms
   that's about 2–4 stops, which fits a 3-minute run. Navigation reads the list from
   `GET /api/stops` and drives the robots there; it never has to compute sensing geometry.
3. **Robot positions**, streamed by the tracking code at 5–10 Hz:
   `POST /api/robots` with `{"robots": [{"id": "A", "mac": "<sensing board MAC>", "x": 1.2, "y": 3.4}]}`.
   The server decides by itself whether each robot is stopped (moved less than ~5 cm over 2 s) and uses
   link data only while both robots are stopped, after 1 s of settle time.
4. **Stop length:** about 15 s. Navigation may poll `GET /api/scan` for `stop_done` to leave early or on
   time; it's optional.

## Outputs

- `GET /api/scan`: per room `{status: occupied|empty|unknown, confidence}`, per robot
  `{stopped, stop_done}`, and current detections.
- Mission map at `http://localhost:8000/mission`; the existing 2D dashboard and 3D view keep working,
  using live robot positions instead of `layout.json`.

## How detection works (outline; details left to the builder)

- **Preferred per-link detector: ESPectre's calibration-free "High Accuracy" model**
  (github.com/francescopace/espectre, GPLv3). It's a 529-parameter neural network on 8 gain-invariant
  features of the subcarrier amplitudes, with a fixed trained threshold and **no quiet-room calibration**.
  It reports 99.1–99.7% recall and 0–0.1% false positives on ESP32-C5. It uses the same 56 live bins our
  C5 boards produce. The host runs it on every link's amplitude stream.
  - Its numbers come from fixed sensors; moving boards and through-wall links are untested.
  - It expects about **100 packets per second**; our links run at 8–50. If it needs the rate, raise the
    boards' beacon rate to 100 Hz (firmware change; 3–4 boards keep airtime around 15%).
  - GPLv3: the project becomes GPL if distributed.
- **Fallback detector:** our own score (amplitude variation, denoised by bands of 8 subcarriers and
  4-packet smoothing) judged against a **quiet model from a quick on-site survey**: with the building
  empty, a **Survey** button runs the stop plan once (1–2 min) and records quiet baselines per stop, plus
  a "quiet score vs signal strength" fit. It is re-run wherever judging happens. *(For the judges: a
  real building can't be surveyed first; say so.)*
- **Room evidence:** each stopped window of each link adds evidence to the rooms its sensing ellipse
  passes through, weighted by overlap: positive if disturbed, negative if clearly quiet. Evidence
  decays slowly. Rooms no link has crossed stay unknown.

## Risks, tested first

1. **Stops without a fresh baseline.** First, replay tonight's recordings (walks through the 4-board
   square, with known quiet stretches) through ESPectre's detector at our packet rates: does it catch
   the walks with no calibration? Then a live experiment (about 2 h, boards carried by hand, any space
   split into two "rooms"): place boards at new spots and test a person moving in one room against an
   empty room, measuring hits and false alarms for both detectors. If neither holds up at new spots,
   fall back to the sentry demo, where boards never move and calibrate once.
2. **Onlookers outside thin cardboard walls** will disturb links that run near the outer walls. Mitigation:
   choose stops whose links stay inside the building, and ask onlookers to stand back during a run.

## Rough schedule (my proposal)

| When (Sat → Sun) | WiFi side (me + you) | Robot team |
|---|---|---|
| 02:00–05:00 | Experiment with boards by hand; `/api/robots` input, stop detection | camera → metres, navigation |
| 05:00–10:00 | Room evidence, survey mode, `/api/scan`, `/api/stops`; sentry mode working | mount boards, power; build cardboard house |
| 10:00–15:00 | Mission map page; `floorplan.json` and stop plan for the real house | drive the stop plan from `/api/stops` |
| 15:00–22:00 | Integration runs with robots; tune; **record the mission video** | same |
| 22:00–05:00 | Rehearse survey and judging flow; failure drills | same |
| 05:00–07:00 | Freeze; spare board flashed; backup video ready | same |

## Alternatives considered

- **RuView** (current base): its on-device presence flag fired at random on our ESP-NOW links, and its
  3D Observatory draws a fixed router and canned figures. We keep its edge code on the boards and its 3D
  view as an extra, but detection runs on the host.
- **diehard79/WiFi-People-Detection-System:** rejected. Its "99%" comes from simulated data or a cited
  paper, it uses router signal strength only, and it's an early personal project.
- **ESPectre as a whole** (one fixed board per room listening to a router): doesn't fit moving robots,
  and it can't prove a room is empty. We adopt only its detector, above.

## Left to the builder

Exact thresholds, decay and evidence weighting; how the quiet fit is computed; mission map visual
design; whether the 3D view is part of the pitch; logging and recording format.
