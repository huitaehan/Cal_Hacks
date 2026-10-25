# FirstIn mock-up (Cartesia + SpacetimeDB)

> **Quick start (new):** copy `.env.example` to `.env`, fill it in, run `node server.mjs`.
> Settings no longer need `export` commands (they still work and win over `.env`).
> To connect the vision teammate's code to the phone call, read **INTEGRATION.md** and give them
> `firstin_client.py` (Python) or `firstin.mjs` (Node). The server fuses two sources and places the
> call itself, so the dashboard does not need to be open. `.env` is gitignored: never commit it.
> Note: we are not using SpacetimeDB; `spacetimedb-module/` is unused.

## 1. Cartesia voice (server.mjs)

1. Get a key at https://play.cartesia.ai/keys
2. Pick two different voices in the playground and copy their IDs.
3. Run:

```bash
export CARTESIA_API_KEY=...
export VOICE_SCOUT1=<voice id>
export VOICE_SCOUT2=<voice id>
node server.mjs
```

Calls `POST https://api.cartesia.ai/tts/bytes` with model `sonic-3.5`. Key stays on the
server. The terminal prints per-line latency (quote this in the Cartesia pitch). If a
request is rejected, check the `Cartesia-Version` header in `server.mjs` against docs.cartesia.ai.

`server.mjs` also serves `dashboard.html` at http://localhost:3000 (the simulated map UI).
Add that file to this folder, or skip it and test voice with:

```bash
curl -s -X POST localhost:3000/speak -d '{"robot":"scout1","text":"Victim marker in room B."}' -o test.wav
```

## 2. SpacetimeDB (spacetimedb-module/index.ts)

```bash
curl -sSf https://install.spacetimedb.com | sh
spacetime dev --template basic-ts firstin
```

Replace `firstin/spacetimedb/src/index.ts` with `spacetimedb-module/index.ts`, then:

```bash
spacetime call --server local firstin reportDetection v1 victim 1.2 3.4 0.86 scout1
spacetime sql  --server local firstin "SELECT * FROM detection"
spacetime logs --server local firstin
```

Not compiled yet. Reducer names may need adjusting. Real test: open two clients,
call a reducer from one, confirm the other updates without polling.

---

# Phone dispatch (stretch)

When two independent sources have confirmed a victim, the dashboard asks `server.mjs` (`POST /dispatch`) to
call a teammate through Cartesia Agents. **Off by default: without `CALL_ENABLED=1` it is a dry
run** (the call script appears in the Dispatch panel, no call is placed).

Rules built in: only victims confirmed by two independent sources, one call per victim, one call per minute,
US numbers only, the script is built only from logged facts and starts with "This is a
demonstration call". Call only a teammate's own phone, with their OK. **Never an emergency number.**
You are responsible for following the rules that apply to calls (Cartesia's docs mention the TCPA
and its Acceptable Use Policy).

## One-time setup in the Cartesia playground
1. Create an agent. Set its **welcome message** to exactly: `{{brief}}`
2. Instruction prompt (suggested): "You are the FirstIn dispatch voice. You just read an incident
   brief to the person who answered. Answer questions using only the brief; if something is not in
   it, say you don't have that information. When they acknowledge, say 'Acknowledged, thank you.'
   and end the call."
3. Phone Numbers -> Add Number -> Cartesia Number, assign it to that agent.
   (Docs: US numbers only, outbound calls reach US numbers only, small per-minute add-on. I could not
   confirm whether free accounts can place outbound calls. Ask at the Cartesia table.)

## Find the IDs
```bash
export CARTESIA_API_KEY=...
node find_ids.mjs        # prints CARTESIA_AGENT_ID and CARTESIA_FROM_NUMBER_ID
```

## Run with real calls
```bash
export CALL_ENABLED=1
export CARTESIA_AGENT_ID=...
export CARTESIA_FROM_NUMBER_ID=...
export CALL_TO=+14805551234     # teammate's own phone, E.164
node server.mjs
```
`search the building` produces two confirmed victims; only the first dials (one call per minute).
Fallback if the call fails on stage: the Dispatch panel still shows the exact script.

---

# Dashboard and live detections (for the vision teammate)

`node server.mjs`, then open http://localhost:3000. The page has: a voice strip (what the commander
said, what the agent said, Cartesia latency), an **agent trace** (each command, the Claude tool call it
produced, and whether the grounding check passed or blocked it), **evidence cards** (a victim needs two
independent sources before any call), the dispatch panel, an audit log, a metrics row, and a **failure
drill** (kill the planner; inject a single-source detection; add its second source).

Without any keys it runs as a simulation with built-in rules and a dry-run dispatch.

## Sending detections from the vision pipeline

One POST per source per sighting. Two different `source` values for the same `id` confirm a victim
and trigger the dispatch (a dry run unless `CALL_ENABLED=1`).

```bash
curl -X POST http://<this-laptop-ip>:3000/detection -H 'x-token: <if DETECTION_TOKEN is set>' -d '{
  "id": "v1", "kind": "victim", "room": "B", "confidence": 0.86,
  "source": "robot1", "description": "person lying near door"
}'
# later, from another source (drone, second robot, second camera):
curl -X POST http://<this-laptop-ip>:3000/detection -d '{"id":"v1","kind":"victim","room":"B","confidence":0.8,"source":"drone"}'
```

Fields: `id` (stable per victim), `kind` (`victim`, `fire`, `blocked`), `room` (A-D, required for victims),
`confidence` 0-1, `source` (who saw it), optional `description`, `x`/`y` in the dashboard's 800x520 map
coordinates (default: room centre), `r` for fire, `x y w h` for blocked. Reports from the same source
for the same `id` just update it. The dashboard shows them as they arrive and replays them on refresh.

Optional env: `DETECTION_TOKEN` (require an `x-token` header; use it on shared wifi),
`CAMERA_URL` (an MJPEG or image URL for the Camera panel; or open the page with `?cam=<url>`).
The Reset button also clears the replay backlog.

Note: the map is still the simulated floor plan. Real robot poses are not drawn yet.
