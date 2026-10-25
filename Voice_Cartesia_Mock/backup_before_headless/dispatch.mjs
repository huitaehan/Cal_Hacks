// Phone dispatch (stretch goal): when two scouts have confirmed a victim, optionally place ONE
// call to a teammate's phone through Cartesia Agents (POST /agents/calls).
//
// Safe by default: nothing is dialed unless CALL_ENABLED=1. Without it you get a "dry run":
// the call script is built and shown on the dashboard, but no call is placed.
//
// Env for real calls (see README):
//   CALL_ENABLED=1
//   CARTESIA_API_KEY, CARTESIA_AGENT_ID, CARTESIA_FROM_NUMBER_ID
//   CALL_TO=+1XXXXXXXXXX   <- a teammate's own phone, with their OK. NEVER an emergency number.

const ENABLED = process.env.CALL_ENABLED === '1';
const KEY = process.env.CARTESIA_API_KEY;
const AGENT_ID = process.env.CARTESIA_AGENT_ID;
const FROM_ID = process.env.CARTESIA_FROM_NUMBER_ID;
const TO = process.env.CALL_TO;
const API = process.env.CARTESIA_API_BASE || 'https://api.cartesia.ai';
const AGENTS_VERSION = process.env.CARTESIA_AGENTS_VERSION || '2026-08-14';
const MIN_GAP_MS = 60_000;

const called = new Set(); // one call per victim
let lastCall = 0;         // and never more than one a minute

const json = (res, code, obj) => {
  res.writeHead(code, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify(obj));
};
const upper = (x) => String(x || '').toUpperCase();
const isRoom = (x) => /^[A-D]$/.test(x);

// The script is built only from fields the dashboard logged. Nothing here is free-form.
export function buildBrief({ room, confidence, hazards, notSearched }) {
  const pct = Math.round(confidence * 100);
  const hz = hazards.map((h) => `${h.kind === 'fire' ? 'fire' : 'debris'} in ${h.room === 'corridor' ? 'the corridor' : 'room ' + h.room}`);
  return (
    `This is a demonstration call from FirstIn. Two independent sources have confirmed a victim in room ${room}, ${pct} percent confidence.` +
    (hz.length ? ` Known hazards: ${hz.join(', ')}.` : '') +
    (notSearched.length ? ` Not yet searched: ${notSearched.length > 1 ? 'rooms' : 'room'} ${notSearched.join(', ')}.` : ' All rooms are searched.') +
    ' Please acknowledge.'
  );
}

export async function handleDispatch(res, raw) {
  let v;
  try { v = JSON.parse(raw); } catch { return json(res, 400, { mode: 'rejected', note: 'Bad JSON.' }); }

  const room = upper(v.room);
  const confirmers = new Set((Array.isArray(v.confirmed_by) ? v.confirmed_by : []).map(String));
  const confidence = Number(v.confidence);
  if (!isRoom(room) || confirmers.size < 2 || !(confidence >= 0 && confidence <= 1)) {
    return json(res, 400, { mode: 'rejected', note: 'Only a victim confirmed by two independent sources can trigger a call.' });
  }
  const hazards = (Array.isArray(v.hazards) ? v.hazards : [])
    .filter((h) => h && (h.kind === 'fire' || h.kind === 'blocked'))
    .map((h) => ({ kind: h.kind, room: isRoom(upper(h.room)) ? upper(h.room) : 'corridor' }));
  const notSearched = (Array.isArray(v.rooms_not_searched) ? v.rooms_not_searched : []).map(upper).filter(isRoom);
  const brief = buildBrief({ room, confidence, hazards, notSearched });
  const id = String(v.victim_id || '').slice(0, 40);

  if (!ENABLED) {
    return json(res, 200, { mode: 'dry-run', brief, note: 'CALL_ENABLED is not set, so no call was placed.' });
  }
  const missing = [['CARTESIA_API_KEY', KEY], ['CARTESIA_AGENT_ID', AGENT_ID], ['CARTESIA_FROM_NUMBER_ID', FROM_ID], ['CALL_TO', TO]]
    .filter(([, x]) => !x).map(([n]) => n);
  if (missing.length) return json(res, 200, { mode: 'failed', brief, note: `Missing settings: ${missing.join(', ')}.` });
  if (!/^\+1[2-9]\d{2}[2-9]\d{6}$/.test(TO)) {
    return json(res, 200, { mode: 'failed', brief, note: 'CALL_TO must be a full US number such as +14805551234.' });
  }
  if (called.has(id)) return json(res, 200, { mode: 'skipped', brief, note: 'Already called for this victim.' });
  if (Date.now() - lastCall < MIN_GAP_MS) return json(res, 200, { mode: 'skipped', brief, note: 'Rate limit: one call per minute.' });
  called.add(id); lastCall = Date.now();

  const masked = `${TO.slice(0, 2)}******${TO.slice(-2)}`;
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
      return json(res, 200, { mode: 'failed', brief, note: `Cartesia error ${r.status}: ${text.slice(0, 200)}` });
    }
    const item = (JSON.parse(text).calls || [])[0] || {};
    if (item.error || !item.agent_call_id) {
      return json(res, 200, { mode: 'failed', brief, note: `Call not started: ${JSON.stringify(item.error || item).slice(0, 200)}` });
    }
    console.log(`[dispatch] call ${item.agent_call_id} to ${masked} accepted in ${Date.now() - t0} ms`);
    return json(res, 200, { mode: 'placed', brief, agent_call_id: item.agent_call_id, note: `Calling ${masked}.` });
  } catch (e) {
    return json(res, 200, { mode: 'failed', brief, note: String(e).slice(0, 200) });
  }
}
