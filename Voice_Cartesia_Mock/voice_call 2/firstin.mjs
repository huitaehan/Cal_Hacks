// FirstIn client for Node (18+, no dependencies). Same idea as firstin_client.py.
//
//   import { FirstIn } from './firstin.mjs';
//   const robot = new FirstIn({ source: 'robot1' });   // url/token from FIRSTIN_URL / FIRSTIN_TOKEN
//   const drone = new FirstIn({ source: 'drone' });
//   await robot.reportVictim('v1', { room: 'B', confidence: 0.86, description: 'person near the door' });
//   await drone.reportVictim('v1', { room: 'B', confidence: 0.8 });   // 2nd source -> the call goes out
//
// Results are plain objects and never throw (unless raiseErrors: true): check result.ok.
//
//   node firstin.mjs health     # server up? token right? will a call go out?
//   node firstin.mjs demo       # one victim from two sources

import './env.mjs';

export class FirstIn {
  constructor({ source, url, token, timeoutMs = 3000, raiseErrors = false } = {}) {
    if (!source || !String(source).trim()) throw new Error("source is required, e.g. 'robot1' or 'drone'");
    this.source = String(source).trim();
    this.url = (url || process.env.FIRSTIN_URL || 'http://localhost:3000').replace(/\/$/, '');
    this.token = token || process.env.FIRSTIN_TOKEN || process.env.DETECTION_TOKEN || '';
    this.timeoutMs = timeoutMs;
    this.raiseErrors = raiseErrors;
    this._lastWarn = 0;
  }

  reportVictim(id, { room, confidence, description = '', x, y }) {
    return this._detect('victim', id, { room, confidence, description, x, y });
  }
  reportFire(id, { confidence, room, description = '', x, y, r }) {
    return this._detect('fire', id, { room, confidence, description, x, y, r });
  }
  reportBlocked(id, { confidence, room, description = '', x, y, w, h }) {
    return this._detect('blocked', id, { room, confidence, description, x, y, w, h });
  }
  health() { return this._request('GET', '/health'); }

  _detect(kind, id, f) {
    const confidence = Number(f.confidence);
    if (!(confidence >= 0 && confidence <= 1)) return this._fail('confidence must be between 0 and 1 (not a percentage)');
    if (kind === 'victim' && !/^[A-D]$/i.test(String(f.room || ''))) return this._fail("a victim needs room 'A', 'B', 'C' or 'D'");
    const body = { id: String(id), kind, confidence, source: this.source };
    if (f.room) body.room = String(f.room).toUpperCase();
    if (f.description) body.description = String(f.description).slice(0, 200);
    for (const k of ['x', 'y', 'r', 'w', 'h']) if (f[k] !== undefined && f[k] !== null) body[k] = f[k];
    return this._request('POST', '/detection', body);
  }

  async _request(method, path, body) {
    const headers = { 'Content-Type': 'application/json' };
    if (this.token) headers['x-token'] = this.token;
    try {
      const r = await fetch(this.url + path, { method, headers, body: body ? JSON.stringify(body) : undefined, signal: AbortSignal.timeout(this.timeoutMs) });
      const out = await r.json().catch(() => ({}));
      if (!r.ok) return this._fail(`server said ${r.status}: ${out.error || ''}${r.status === 401 ? ' (wrong or missing FIRSTIN_TOKEN?)' : ''}`);
      return out;
    } catch (e) {
      return this._fail(`cannot reach ${this.url}: ${e.message || e}`);
    }
  }

  _fail(message) {
    if (this.raiseErrors) throw new Error(message);
    if (Date.now() - this._lastWarn > 5000) { console.error(`[firstin] ${message}`); this._lastWarn = Date.now(); }
    return { ok: false, error: message };
  }
}

// Command line: node firstin.mjs health | demo [room]
if (import.meta.url === `file://${process.argv[1]}` || process.argv[1]?.endsWith('firstin.mjs')) {
  const [cmd, arg] = process.argv.slice(2);
  if (cmd === 'health') {
    console.log(JSON.stringify(await new FirstIn({ source: 'cli' }).health(), null, 2));
  } else if (cmd === 'demo') {
    const id = 'demo-' + String(Date.now()).slice(-6), room = (arg || 'B').toUpperCase();
    console.log('robot1 reports:', await new FirstIn({ source: 'robot1' }).reportVictim(id, { room, confidence: 0.86, description: 'person lying near the door' }));
    await new Promise((r) => setTimeout(r, 1200));
    console.log('drone  reports:', await new FirstIn({ source: 'drone' }).reportVictim(id, { room, confidence: 0.8, description: 'person visible from above' }));
    console.log('If the server is live, your phone should ring in a few seconds (dry run otherwise).');
  } else {
    console.log('usage: node firstin.mjs health | demo [room]');
  }
}
