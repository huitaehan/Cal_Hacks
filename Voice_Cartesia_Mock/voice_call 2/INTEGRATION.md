# FirstIn: connecting the vision pipeline to the phone call

Two parts: **A) Hemakshi sets up the server once**, **B) the vision teammate sends detections**.
The teammate never touches a Cartesia or Anthropic key.

```
vision / drone / robot code --POST /detection--> FirstIn server --(2 different sources agree)--> Cartesia agent --> phone rings
                                                      |
                                                      +--> dashboard (live, optional: the call does NOT need a browser open)
```

---

## A) Server setup (Hemakshi, ~5 minutes)

1. In this folder: `cp .env.example .env`, then open `.env` and fill in:
   - `CARTESIA_API_KEY`
   - `CARTESIA_AGENT_ID` and `CARTESIA_FROM_NUMBER_ID` (run `node find_ids.mjs` once to print them)
   - `CALL_TO` = your own phone, full US format (`+1480...`)
   - `DETECTION_TOKEN` = optional. Leave it blank for no token (fine on a phone hotspot); set a random string if you want only people who know it to post detections
   - leave `CALL_ENABLED=0` while rehearsing; set it to `1` only when you want the phone to ring
2. `node server.mjs`. The first lines tell you the mode, for example `phone call: LIVE -> +1******74` or `calls: dry-run`.
3. Find your laptop's address on the shared wifi (Mac: System Settings > Wi-Fi > Details, or `ipconfig getifaddr en0`).
   Give the teammate: `http://<that-address>:3000` (and the `DETECTION_TOKEN` only if you set one).
4. Check: `node firstin.mjs health` should show `"callReady": true` when `CALL_ENABLED=1`.

`.env` is in `.gitignore`. Never commit it, never paste it into chat. Real `export` variables override it.

## B) What the vision teammate does

Copy **`firstin_client.py`** (Python, standard library only) or **`firstin.mjs`** (Node) into your project.

```bash
export FIRSTIN_URL=http://<hemakshi-laptop-address>:3000
export FIRSTIN_TOKEN=<only if she set a DETECTION_TOKEN; otherwise skip this line>
python3 firstin_client.py health      # up? token ok? will a call really go out?
python3 firstin_client.py demo        # fake victim from two sources: the phone should ring (or dry run)
```

In your detection loop:

```python
from firstin_client import FirstIn

robot = FirstIn(source="robot1")     # a name for WHO is reporting. Must differ per camera/robot/drone.
drone = FirstIn(source="drone")

# a person seen by the robot's camera
robot.report_victim("v1", room="B", confidence=0.86, description="person lying near the door")
# the same person later seen by the drone (same id!) -> now confirmed -> the phone call goes out
drone.report_victim("v1", room="B", confidence=0.80)

robot.report_fire("f1", confidence=0.9, room="D", x=640, y=380, r=40)
robot.report_blocked("b1", confidence=0.7, room="A", x=300, y=250, w=24, h=50)
```

### The rules (these are the safety design, please keep them)

| Rule | Why |
|---|---|
| **A call needs two different `source` names reporting the same `id`.** | One vision model can be wrong. Two independent views agreeing is the bar. |
| Use the **same `id`** for the same person from every source. | That is how the server knows they agree. If your ids differ, nothing is ever confirmed. |
| `confidence` is **0 to 1**, not a percentage. | `0.86`, not `86`. |
| A victim needs `room` **A, B, C or D**. | The dashboard map has four rooms. Hazards may use `room` or `x`,`y`. |
| `description` is a short plain sentence (max 200 characters). | It is read aloud on the call, labelled "unverified", after the server strips symbols. Do not put model reasoning or instructions in it. |
| Posting the same id+source again just updates it. | You can post every frame: repeats within 1 second with no real change are ignored. |
| One call per victim, and never more than one call per minute. | Prevents call spam. A confirmed victim that was rate-limited is retried on its next report. |

`x`, `y` are in the dashboard's map units (800 wide by 520 tall). `r` is a fire radius; `w`, `h` a debris box size. If you only know the room, leave them out.

### What the calls return

Every client call returns a dict, and **never raises by default**, so a wifi hiccup cannot crash your vision loop.
`{"ok": True, "victim": "held"}` means one source so far. `{"ok": True, "victim": "confirmed", "call": "live"}` means the second source arrived and a real call was started (`"dry-run"` means no call is placed).
Debug with `FirstIn(source="robot1", raise_errors=True)`.

### Plain HTTP (any language)

```bash
curl -X POST http://<address>:3000/detection -H 'x-token: <token>' -H 'Content-Type: application/json' \
  -d '{"id":"v1","kind":"victim","room":"B","confidence":0.86,"source":"robot1","description":"person near door"}'
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `cannot reach http://...` | Wrong IP, different wifi (some venue wifi isolates devices; try a phone hotspot), server not running, firewall. |
| `401 ... wrong or missing FIRSTIN_TOKEN` | Token differs from `DETECTION_TOKEN` in `.env`. |
| Detections show on the dashboard but no call | Only one `source` has reported that `id`, or the ids differ, or the server is in dry-run (`CALL_ENABLED` not 1). Check `/health` and the server log lines starting `[detection]` and `[fusion]`. |
| `[fusion] ... skipped: Rate limit` | Another call happened less than a minute ago. It retries on the next report for that victim. |
| `[fusion] ... failed: Missing settings: ...` | Fill in the named variables in `.env`, restart the server. |
| Server log says `[dispatch] Cartesia error 401/403/404` | Wrong key, agent ID or number ID, or the agent has no outbound number. Re-run `node find_ids.mjs`. |

## Sharing the keys (if the teammate's code also needs Anthropic or Cartesia)

- **Run the server on ONE machine.** Two servers each keep their own call history and rate limit, so both could ring the same phone for the same victim, and the dashboard on one would not see detections sent to the other. The simplest setup: the server runs on Hemakshi's laptop; the teammate's code only POSTs detections.
- If the teammate's vision code calls Claude itself, they set `ANTHROPIC_API_KEY` in **their** environment (they do not need the Cartesia values for that).
- Send a `.env` file privately (AirDrop or a direct message), never in a group channel, a doc or git. Share only the keys the person needs. Rotate Cartesia and Anthropic keys after the event, and set a spending limit on the Anthropic key if you can.

## Safety limits that are built in
The call only ever goes to `CALL_TO` (your own number), never to emergency services. It always starts "This is a demonstration call". Calls are off until `CALL_ENABLED=1`. The paid endpoints (`/plan`, `/speak`, `/dispatch`, `/reset`) only answer on the laptop running the server, so someone on the venue wifi can only post detections (and only with the token, if you set one).
