// Tiny server: serves the dashboard and proxies text -> speech to Cartesia,
// so your API key never reaches the browser. Node 18+, no dependencies.
//
//   export CARTESIA_API_KEY=...
//   export VOICE_SCOUT1=<voice id>   (pick two different voices at play.cartesia.ai)
//   export VOICE_SCOUT2=<voice id>
//   export ANTHROPIC_API_KEY=...     (optional: turns on the Claude planner; otherwise built-in rules are used)
//   node server.mjs   ->  http://localhost:3000

import http from 'node:http';
import { readFile } from 'node:fs/promises';
import { handleDispatch } from './dispatch.mjs';

const KEY = process.env.CARTESIA_API_KEY;
const VOICES = {
  scout1: process.env.VOICE_SCOUT1,
  scout2: process.env.VOICE_SCOUT2,
  commander: process.env.VOICE_COMMANDER || process.env.VOICE_SCOUT1,
};
const MODEL = process.env.CARTESIA_MODEL || 'sonic-3.5';
const VERSION = process.env.CARTESIA_VERSION || '2025-04-16'; // check docs.cartesia.ai if requests are rejected
const PORT = process.env.PORT || 3000;

// ---- Claude planner (optional) ----
const ANTHROPIC_KEY = process.env.ANTHROPIC_API_KEY;
const ANTHROPIC_URL = process.env.ANTHROPIC_BASE_URL || 'https://api.anthropic.com';
const CLAUDE_MODEL = process.env.CLAUDE_MODEL || 'claude-sonnet-5-5';

const PLANNER_SYSTEM = `You are the incident planner for a team of two small search robots (scout1, scout2) scouting a building during a fire emergency. A human incident commander speaks to you. You choose tool calls; you never drive motors.

Rules:
- Only state facts that appear in the MISSION STATE. Never invent victims, hazards, or room conditions. If nothing is known, say so.
- A room that is not searched is UNEXPLORED, not clear. Never call an unsearched room safe or empty.
- A victim with verified=false is unconfirmed; say so.
- Floor plan you may rely on: rooms A (northwest) and B (southwest) connect to the corridor by the west door; rooms C (northeast) and D (southeast) by the east door; the building exit is at the south of the corridor. East wing = C and D. West wing = A and B.
- Spoken text is under 25 words, plain speech, no markdown.
- If the request is unclear, ask one short clarifying question with answer_question.
- You may call several tools in one turn.`;

const PLANNER_TOOLS = [
  {
    name: 'assign_task',
    description: 'Send one scout to search one room. Use when the commander names a robot or a single room for one robot.',
    input_schema: {
      type: 'object',
      properties: {
        robot_id: { type: 'string', enum: ['scout1', 'scout2', 'any'] },
        room: { type: 'string', enum: ['A', 'B', 'C', 'D'] },
      },
      required: ['robot_id', 'room'],
    },
  },
  {
    name: 'search_rooms',
    description: 'Add rooms to the shared search queue; free scouts pick them up. Use for wings, several rooms, or the whole building.',
    input_schema: {
      type: 'object',
      properties: { rooms: { type: 'array', items: { type: 'string', enum: ['A', 'B', 'C', 'D'] } } },
      required: ['rooms'],
    },
  },
  {
    name: 'stop_all',
    description: 'Halt every robot immediately.',
    input_schema: { type: 'object', properties: {} },
  },
  {
    name: 'answer_question',
    description: 'Speak a short answer or clarifying question in a scout voice. Use only facts from the mission state.',
    input_schema: {
      type: 'object',
      properties: {
        robot_id: { type: 'string', enum: ['scout1', 'scout2'] },
        text: { type: 'string', description: 'Under 25 words, plain speech.' },
        based_on: { type: 'array', items: { type: 'string' }, description: 'IDs of detections this answer relies on; empty array if none.' },
      },
      required: ['robot_id', 'text', 'based_on'],
    },
  },
];

const readBody = (req) =>
  new Promise((resolve) => {
    let s = '';
    req.on('data', (c) => {
      s += c;
      if (s.length > 200_000) req.destroy(); // detections are tiny; refuse anything huge
    });
    req.on('end', () => resolve(s));
  });

// ---- Live detections from the vision pipeline (POST /detection -> dashboard over SSE) ----
// Contract (one POST per source per sighting):
//   { id, kind: "victim"|"fire"|"blocked", room: "A".."D", confidence: 0..1, source: "robot1",
//     description?, x?, y?, r? (fire), w?, h? (blocked) }
// The dashboard fuses sources: a victim needs two DIFFERENT sources before anything is dispatched.
const DETECTION_TOKEN = process.env.DETECTION_TOKEN;           // optional shared secret (x-token header)
const CAMERA_URL = process.env.CAMERA_URL || '';               // optional MJPEG/image URL shown on the dashboard
const CALL_ENABLED = process.env.CALL_ENABLED === '1';
const backlog = [];                                            // replayed to dashboards that (re)connect
const sseClients = new Set();
const broadcast = (obj) => {
  const line = `data: ${JSON.stringify(obj)}\n\n`;
  for (const c of sseClients) c.write(line);
};
const num = (v) => (v === undefined || v === null || v === '' ? undefined : Number(v));
function cleanDetection(d) {
  if (!d || typeof d !== 'object') return { error: 'Body must be a JSON object.' };
  const id = String(d.id || '').slice(0, 40);
  const kind = String(d.kind || '');
  const source = String(d.source || '').slice(0, 30);
  const confidence = Number(d.confidence);
  const room = String(d.room || '').toUpperCase();
  if (!id || !source) return { error: 'id and source are required.' };
  if (!['victim', 'fire', 'blocked'].includes(kind)) return { error: 'kind must be victim, fire or blocked.' };
  if (!(confidence >= 0 && confidence <= 1)) return { error: 'confidence must be between 0 and 1.' };
  if (kind === 'victim' && !/^[A-D]$/.test(room)) return { error: 'victim needs room A, B, C or D.' };
  const out = { id, kind, source, confidence, room: /^[A-D]$/.test(room) ? room : undefined, ts: Date.now() };
  out.description = String(d.description || '').replace(/[\u0000-\u001f]/g, ' ').slice(0, 200);
  for (const k of ['x', 'y', 'r', 'w', 'h']) {
    const n = num(d[k]);
    if (n !== undefined) { if (!Number.isFinite(n)) return { error: `${k} must be a number.` }; out[k] = n; }
  }
  return { detection: out };
}

http
  .createServer(async (req, res) => {
    try {
      if (req.method === 'GET' && (req.url === '/' || req.url === '/index.html')) {
        const html = await readFile(new URL('./dashboard.html', import.meta.url));
        res.writeHead(200, { 'Content-Type': 'text/html' });
        return res.end(html);
      }

      if (req.method === 'GET' && req.url === '/config') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        return res.end(JSON.stringify({
          voiceReady: Boolean(KEY && VOICES.scout1 && VOICES.scout2),
          plannerReady: Boolean(ANTHROPIC_KEY),
          plannerModel: ANTHROPIC_KEY ? CLAUDE_MODEL : '',
          callMode: CALL_ENABLED ? 'live' : 'dry-run',
          cameraUrl: CAMERA_URL,
        }));
      }

      // Dashboard subscribes here; every accepted detection is pushed as it arrives.
      if (req.method === 'GET' && req.url === '/events') {
        res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', Connection: 'keep-alive' });
        res.write('retry: 2000\n\n');
        for (const d of backlog) res.write(`data: ${JSON.stringify(d)}\n\n`);
        sseClients.add(res);
        const ping = setInterval(() => res.write(': ping\n\n'), 20000);
        req.on('close', () => { clearInterval(ping); sseClients.delete(res); });
        return;
      }

      if (req.method === 'POST' && req.url === '/detection') {
        if (DETECTION_TOKEN && req.headers['x-token'] !== DETECTION_TOKEN) {
          res.writeHead(401, { 'Content-Type': 'application/json' });
          return res.end(JSON.stringify({ ok: false, error: 'Missing or wrong x-token header.' }));
        }
        let parsed;
        try { parsed = JSON.parse(await readBody(req)); } catch { parsed = null; }
        const { detection, error } = cleanDetection(parsed);
        if (error) {
          res.writeHead(400, { 'Content-Type': 'application/json' });
          return res.end(JSON.stringify({ ok: false, error }));
        }
        backlog.push(detection);
        if (backlog.length > 200) backlog.shift();
        broadcast(detection);
        console.log(`[detection] ${detection.source}: ${detection.kind} ${detection.id}${detection.room ? ' room ' + detection.room : ''} conf ${detection.confidence}`);
        res.writeHead(200, { 'Content-Type': 'application/json' });
        return res.end(JSON.stringify({ ok: true, dashboards: sseClients.size }));
      }

      // The dashboard's Reset button clears the replay backlog so old test data does not come back.
      if (req.method === 'POST' && req.url === '/reset') {
        backlog.length = 0;
        res.writeHead(200, { 'Content-Type': 'application/json' });
        return res.end('{"ok":true}');
      }

      if (req.method === 'POST' && req.url === '/plan') {
        if (!ANTHROPIC_KEY) {
          res.writeHead(503);
          return res.end('Set ANTHROPIC_API_KEY to enable the Claude planner');
        }
        const { transcript, snapshot } = JSON.parse(await readBody(req));
        const t0 = Date.now();
        const r = await fetch(`${ANTHROPIC_URL}/v1/messages`, {
          method: 'POST',
          headers: {
            'x-api-key': ANTHROPIC_KEY,
            'anthropic-version': '2023-06-01',
            'content-type': 'application/json',
          },
          body: JSON.stringify({
            model: CLAUDE_MODEL,
            max_tokens: 400,
            system: PLANNER_SYSTEM,
            tools: PLANNER_TOOLS,
            tool_choice: { type: 'any' },
            messages: [{
              role: 'user',
              content: `MISSION STATE (the only facts you may use):\n${JSON.stringify(snapshot)}\n\nCOMMANDER SAID: "${String(transcript).slice(0, 300)}"`,
            }],
          }),
        });
        if (!r.ok) {
          res.writeHead(502);
          return res.end(`Claude error ${r.status}: ${await r.text()}`);
        }
        const data = await r.json();
        const calls = (data.content || []).filter((b) => b.type === 'tool_use').map((b) => ({ name: b.name, input: b.input }));
        console.log(`[plan] "${transcript}" -> ${calls.map((c) => c.name).join(', ') || 'no tool call'} in ${Date.now() - t0} ms`);
        res.writeHead(200, { 'Content-Type': 'application/json' });
        return res.end(JSON.stringify({ calls }));
      }

      if (req.method === 'POST' && req.url === '/dispatch') {
        return handleDispatch(res, await readBody(req));
      }

      if (req.method === 'POST' && req.url === '/speak') {
        const { robot, text } = JSON.parse(await readBody(req));
        const voice = VOICES[robot];
        if (!KEY || !voice) {
          res.writeHead(503);
          return res.end('Set CARTESIA_API_KEY and VOICE_SCOUT1 / VOICE_SCOUT2');
        }
        const t0 = Date.now();
        const r = await fetch('https://api.cartesia.ai/tts/bytes', {
          method: 'POST',
          headers: {
            'X-API-Key': KEY,
            'Cartesia-Version': VERSION,
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            model_id: MODEL,
            transcript: String(text).slice(0, 300),
            voice: { mode: 'id', id: voice },
            output_format: { container: 'wav', encoding: 'pcm_s16le', sample_rate: 24000 },
          }),
        });
        if (!r.ok) {
          res.writeHead(502);
          return res.end(`Cartesia error ${r.status}: ${await r.text()}`);
        }
        const buf = Buffer.from(await r.arrayBuffer());
        console.log(`[tts] ${robot}: "${text}" -> ${buf.length} bytes in ${Date.now() - t0} ms`);
        res.writeHead(200, { 'Content-Type': 'audio/wav' });
        return res.end(buf);
      }

      res.writeHead(404);
      res.end('not found');
    } catch (e) {
      console.error(e);
      if (!res.headersSent) res.writeHead(500);
      res.end(String(e));
    }
  })
  .listen(PORT, () => {
    console.log(`FirstIn mock-up on http://localhost:${PORT}`);
    console.log(`[config] calls: ${CALL_ENABLED ? 'LIVE' : 'dry-run'} | planner: ${ANTHROPIC_KEY ? CLAUDE_MODEL : 'off (built-in rules)'} | voice: ${KEY && VOICES.scout1 && VOICES.scout2 ? 'on' : 'off'}`);
    if (!DETECTION_TOKEN) console.log('[config] POST /detection is open to your network. Set DETECTION_TOKEN=<secret> on shared wifi.');
  });
