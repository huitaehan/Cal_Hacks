// Phone dispatch: when two independent sources have confirmed a victim, place ONE call to a
// teammate's phone through Cartesia Agents (POST /agents/calls).
//
// Safe by default: nothing is dialed unless CALL_ENABLED=1. Without it you get a "dry run":
// the call script is built and returned, but no call is placed.
//
// Settings (put them in .env, see .env.example):
//   CALL_ENABLED=1
//   CARTESIA_API_KEY, CARTESIA_AGENT_ID, CARTESIA_FROM_NUMBER_ID
//   CALL_TO=+1XXXXXXXXXX   <- a teammate's own phone, with their OK. NEVER an emergency number.
//
// Two ways in: the server calls dispatchVictim() itself when a victim has two sources (works with
// no browser open), and the dashboard may POST /dispatch for the same victim. Both end up here,
// and the second request for the same victim gets the first one's result, never a second call.

import './env.mjs';

const ENABLED = process.env.CALL_ENABLED === '1';
const KEY = process.env.CARTESIA_API_KEY;
const AGENT_ID = process.env.CARTESIA_AGENT_ID;
const FROM_ID = process.env.CARTESIA_FROM_NUMBER_ID;
const TO = process.env.CALL_TO;
const API = process.env.CARTESIA_API_BASE || 'https://api.cartesia.ai';
const AGENTS_VERSION = process.env.CARTESIA_AGENTS_VERSION || '2026-08-14';
const MIN_GAP_MS = 60_000;      // never more than one call a minute
const RETRY_AFTER_FAIL_MS = 15_000;

const results = new Map();  // victim_id -> result of the call (or dry run) already made
const inflight = new Map(); // victim_id -> promise, so two simultaneous requests share one call
let lastCall = 0;

const json = (res, code, obj) => {
  res.writeHead(code, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify(obj));
};
const upper = (x) => String(x || '').toUpperCase();
const isRoom = (x) => /^[A-D]$/.test(x);
const maskedTo = () => (TO ? `${TO.slice(0, 2)}******${TO.slice(-2)}` : '(not set)');
const missingSettings = () =>
  [['CARTESIA_API_KEY', KEY], ['CARTESIA_AGENT_ID', AGENT_ID], ['CARTESIA_FROM_NUMBER_ID', FROM_ID], ['CALL_TO', TO]]
    .filter(([, x]) => !x).map(([n]) => n);

// What /health reports. Never includes a key, an ID or the full phone number.
export function dispatchInfo() {
  const missing = missingSettings();
  const badNumber = Boolean(TO) && !/^\+1[2-9]\d{2}[2-9]\d{6}$/.test(TO);
  return {
    callMode: ENABLED ? 'live' : 'dry-run',
    callTo: maskedTo(),
    missing: ENABLED ? missing : [],
    problem: ENABLED && badNumber ? 'CALL_TO must be a full US number such as +14805551234.' : '',
    ready: ENABLED && !missing.length && !badNumber,
  };
}

// Forget which victims were called (used by the dashboard's Reset button during rehearsals).
export function resetDispatch() {
  results.clear();
  lastCall = 0;
}

// Anything that came from a vision model is text we cannot trust: keep only plain characters
// and a short length before it is read aloud. It is labelled as an unverified description.
const cleanNote = (s) => String(s || '').replace(/[^A-Za-z0-9 ,.'-]/g, ' ').replace(/\s+/g, ' ').trim().slice(0, 120);

// The script is built only from structured fields (plus the cleaned camera note). notSearched is
// null when the caller does not know which rooms were searched; then that sentence is left out.
export function buildBrief({ room, confidence, hazards, notSearched, note }) {
  const pct = Math.round(confidence * 100);
  const where = (r) => (r === 'corridor' ? 'in the corridor' : r === 'unknown' ? 'at an unknown location' : 'in room ' + r);
  const hz = hazards.map((h) => `${h.kind === 'fire' ? 'fire' : 'debris'} ${where(h.room)}`);
  return (
    `This is a demonstration call from FirstIn. Two independent sources have confirmed a victim in room ${room}, ${pct} percent confidence.` +
    (note ? ` Camera description, unverified: ${note}.` : '') +
    (hz.length ? ` Known hazards: ${hz.join(', ')}.` : '') +
    (notSearched === null
      ? ''
      : notSearched.length
        ? ` Not yet searched: ${notSearched.length > 1 ? 'rooms' : 'room'} ${notSearched.join(', ')}.`
        : ' All rooms are searched.') +
    ' Please acknowledge.'
  );
}

// Core logic. Takes a plain object, returns {mode, brief?, note, agent_call_id?}.
//   mode: rejected | dry-run | placed | failed | skipped
export function dispatchVictim(v) {
  const id = String((v && v.victim_id) || '').slice(0, 40);
  if (id && inflight.has(id)) return inflight.get(id);
  const p = run(v, id).finally(() => inflight.delete(id));
  if (id) inflight.set(id, p);
  return p;
}

async function run(v, id) {
  v = v || {};
  const room = upper(v.room);
  const confirmers = new Set((Array.isArray(v.confirmed_by) ? v.confirmed_by : []).map((s) => String(s).toLowerCase().trim()));
  const confidence = Number(v.confidence);
  if (!id) return { mode: 'rejected', note: 'victim_id is required.' };
  if (!isRoom(room) || confirmers.size < 2 || !(confidence >= 0 && confidence <= 1)) {
    return { mode: 'rejected', note: 'Only a victim confirmed by two independent sources can trigger a call.' };
  }
  if (results.has(id)) return { ...results.get(id), note: `${results.get(id).note} (already handled for this victim)` };

  const hazards = (Array.isArray(v.hazards) ? v.hazards : [])
    .filter((h) => h && (h.kind === 'fire' || h.kind === 'blocked'))
    .map((h) => ({ kind: h.kind, room: isRoom(upper(h.room)) ? upper(h.room) : upper(h.room) === 'CORRIDOR' ? 'corridor' : 'unknown' }));
  const notSearched = Array.isArray(v.rooms_not_searched) ? v.rooms_not_searched.map(upper).filter(isRoom) : null;
  const brief = buildBrief({ room, confidence, hazards, notSearched, note: cleanNote(v.camera_note) });

  if (!ENABLED) {
    const out = { mode: 'dry-run', brief, note: 'CALL_ENABLED is not set, so no call was placed.' };
    results.set(id, out);
    return out;
  }
  const missing = missingSettings();
  if (missing.length) return { mode: 'failed', brief, note: `Missing settings: ${missing.join(', ')}.` };
  if (!/^\+1[2-9]\d{2}[2-9]\d{6}$/.test(TO)) {
    return { mode: 'failed', brief, note: 'CALL_TO must be a full US number such as +14805551234.' };
  }
  if (Date.now() - lastCall < MIN_GAP_MS) return { mode: 'skipped', brief, note: 'Rate limit: one call per minute.' };
  lastCall = Date.now(); // set before the network call so a second request cannot slip in

  const failed = (note) => {
    lastCall = Date.now() - MIN_GAP_MS + RETRY_AFTER_FAIL_MS; // allow one retry after a short wait
    return { mode: 'failed', brief, note };
  };
  try {
    const t0 = Date.now();
    const r = await fetch(`${API}/agents/calls`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${KEY}`,
        'X-API-Key': KEY,
        'Cartesia-Version': AGENTS_VERSION,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        from_number_id: FROM_ID,
        agent_id: AGENT_ID,
        ringing_timeout_seconds: 30,
        max_call_duration_minutes: 2,
        outbound_calls: [{ to_number: TO, dynamic_variables: { brief, room, confidence_percent: Math.round(confidence * 100) } }],
      }),
    });
    const text = await r.text();
    if (!r.ok) {
      console.log(`[dispatch] Cartesia error ${r.status}`);
      return failed(`Cartesia error ${r.status}: ${text.slice(0, 200)}`);
    }
    const item = (JSON.parse(text).calls || [])[0] || {};
    if (item.error || !item.agent_call_id) return failed(`Call not started: ${JSON.stringify(item.error || item).slice(0, 200)}`);
    console.log(`[dispatch] call ${item.agent_call_id} to ${maskedTo()} accepted in ${Date.now() - t0} ms`);
    const out = { mode: 'placed', brief, agent_call_id: item.agent_call_id, note: `Calling ${maskedTo()}.` };
    results.set(id, out);
    return out;
  } catch (e) {
    return failed(String(e).slice(0, 200));
  }
}

// HTTP wrapper used by the dashboard (POST /dispatch).
export async function handleDispatch(res, raw) {
  let v;
  try { v = JSON.parse(raw); } catch { return json(res, 400, { mode: 'rejected', note: 'Bad JSON.' }); }
  const out = await dispatchVictim(v);
  return json(res, out.mode === 'rejected' ? 400 : 200, out);
}
