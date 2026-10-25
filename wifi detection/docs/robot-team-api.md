# WiFi sensing ↔ robot team: what you need to know

The WiFi sensing finds people. Your side drives the robots and tells the sensing server where they are.
Everything below happens over plain HTTP + JSON on the base computer.

## 1. Hardware

- Each robot carries one extra **ESP32-C5 sensing board**, separate from its motor board. It only needs
  power: USB from the robot's 5 V or a small power bank, about 150 mA. Don't wire it to the motor board
  or connect it to the router.

  | Board (MAC) | Goes on |
  |---|---|
  | `d0:cf:13:e1:63:30` | robot **A** |
  | `d0:cf:13:e1:2e:30` | robot **B** |
  | `d0:cf:13:e1:66:2c` | **breadcrumb** at the far end of the search area (power bank, position in `floorplan.json`) |
  | `d0:cf:13:e1:63:ac` | **gateway**, USB to the base computer at the station outside the door |

- Both robots start at the door next to the station. In the open test area their listening spots (`/api/stops`)
  are on the left and right, so the link between them crosses the middle.

- The sensing boards use WiFi **channel 6**. Put the robot router on **another channel** (1 or 11).

## 2. Coordinates

**Metres**, on the floor plan: x to the right and y up as seen by the overhead camera, origin at the agreed
corner of the cardboard house. Use the same origin as `host/floorplan.json`.

## 3. Find the base computer

On the base computer, start the server with `--host 0.0.0.0`. It prints a line like
`Robot team API: http://192.168.1.23:8000/api/robots`. Use that IP below as `<base-ip>`. Windows may ask
to allow Python through the firewall; allow it on private networks.

## 4. Send robot positions (5–10 times per second)

`POST http://<base-ip>:8000/api/robots`

```json
{"robots": [{"id": "A", "mac": "d0:cf:13:e1:63:30", "x": 1.20, "y": 0.80},
            {"id": "B", "mac": "d0:cf:13:e1:2e:30", "x": 2.50, "y": 1.90}]}
```

Python example:

```python
import json, time, urllib.request

BASE = "http://192.168.1.23:8000"   # <base-ip>

def send_positions(a_xy, b_xy):
    body = {"robots": [{"id": "A", "mac": "d0:cf:13:e1:63:30", "x": a_xy[0], "y": a_xy[1]},
                       {"id": "B", "mac": "d0:cf:13:e1:2e:30", "x": b_xy[0], "y": b_xy[1]}]}
    req = urllib.request.Request(BASE + "/api/robots", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    urllib.request.urlopen(req, timeout=0.5).read()
```

If positions stop arriving for more than 1 s, the server treats that robot as moving and ignores its data.

## 5. Drive the stop plan

`GET http://<base-ip>:8000/api/stops` returns:

```json
{"stops": [{"name": "Room A", "a": [1.0, 0.3], "b": [1.0, 1.7], "listen_s": 15.0}, ...]}
```

For each stop, in order:

1. Drive robot **A** to `a` and robot **B** to `b`.
2. **Both robots hold completely still** for `listen_s` + 3 seconds (motors off if you can). The server works
   out by itself when they've stopped, and needs 3 s of stillness before it starts listening.
3. Optionally, poll `GET http://<base-ip>:8000/api/scan` and leave as soon as both robots show
   `"stop_done": true`.
4. Go to the next stop.

## 6. Read the result (optional)

`GET /api/scan`:

```json
{"rooms": [{"name": "Room A", "status": "occupied", "confidence": 0.82}, {"name": "Room B", "status": "empty", "confidence": 0.7}],
 "robots": [{"id": "A", "stopped": true, "stop_done": false}],
 "detections": ["Person likely in Room A (82%)"],
 "warnings": []}
```

`status` is `occupied`, `empty` or `unknown` (not checked yet). A room only reads `empty` after at least
30 s of quiet listening across most of it, so a single 15 s stop can report a person but never "clear".
Bad payloads (missing `mac`, non-numeric or NaN `x`/`y`) get HTTP 400 with the reason.

## Rule

The camera only tracks the robots. It must never be used to find people.
