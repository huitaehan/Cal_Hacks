// SpacetimeDB module skeleton for FirstIn (TypeScript).
// Written against the 2.0 quickstart pattern: schema({...}), table(opts, columns),
// and exported reducers. NOT compiled -- run `spacetime dev` and fix any API drift
// against https://spacetimedb.com/docs before relying on it.
//
// One writer per table:
//   robot       -> perception (pose) + robot workers (status)
//   map_cell    -> perception / world model
//   detection   -> perception
//   task        -> planner (Claude)
//   event       -> everyone
//   command_log -> voice agent

import { schema, table, t } from 'spacetimedb/server';

const spacetimedb = schema({
  robot: table(
    { public: true },
    {
      id: t.string().primaryKey(),        // "scout1"
      name: t.string(),                   // "Scout 1"
      x: t.f64(),                         // meters, arena frame
      y: t.f64(),
      theta: t.f64(),                     // radians
      status: t.string(),                 // idle | moving | searching | stopped
    }
  ),

  map_cell: table(
    { public: true },
    {
      key: t.string().primaryKey(),       // "12,7"
      x: t.i32(),
      y: t.i32(),
      state: t.string(),                  // unknown | clear | blocked | fire
      observed_by: t.string(),            // robot id, "" if never observed
    }
  ),

  detection: table(
    { public: true },
    {
      id: t.string().primaryKey(),
      kind: t.string(),                   // victim | fire | blocked
      x: t.f64(),
      y: t.f64(),
      confidence: t.f64(),
      source: t.string(),                 // which robot / sensor
      verified: t.bool(),
    }
  ),

  task: table(
    { public: true },
    {
      id: t.u64().primaryKey().autoInc(),
      robot_id: t.string(),
      kind: t.string(),                   // goto | search_sector | hold
      target: t.string(),                 // "roomB" or "x,y"
      status: t.string(),                 // pending | active | done | failed
    }
  ),

  event: table(
    { public: true },
    {
      id: t.u64().primaryKey().autoInc(),
      robot_id: t.string(),
      kind: t.string(),                   // detection | blocked | arrived | stopped
      message: t.string(),                // what the voice layer is allowed to say
    }
  ),

  command_log: table(
    { public: true },
    {
      id: t.u64().primaryKey().autoInc(),
      transcript: t.string(),
      intent: t.string(),
    }
  ),
});

export default spacetimedb;

// ---- Reducers -------------------------------------------------------------

export const upsertRobot = spacetimedb.reducer(
  { id: t.string(), name: t.string(), x: t.f64(), y: t.f64(), theta: t.f64(), status: t.string() },
  (ctx, args) => {
    const existing = ctx.db.robot.id.find(args.id);
    if (existing) ctx.db.robot.id.update({ ...existing, ...args });
    else ctx.db.robot.insert(args);
  }
);

export const reportDetection = spacetimedb.reducer(
  { id: t.string(), kind: t.string(), x: t.f64(), y: t.f64(), confidence: t.f64(), source: t.string() },
  (ctx, a) => {
    if (a.confidence < 0 || a.confidence > 1) throw new Error('confidence must be 0..1');
    if (!['victim', 'fire', 'blocked'].includes(a.kind)) throw new Error('unknown kind');
    const row = { ...a, verified: false };
    const existing = ctx.db.detection.id.find(a.id);
    if (existing) ctx.db.detection.id.update({ ...existing, ...row });
    else ctx.db.detection.insert(row);
    // The voice layer speaks from this row, never from the model's imagination.
    ctx.db.event.insert({
      id: 0n,
      robot_id: a.source,
      kind: 'detection',
      message: `${a.kind} at (${a.x.toFixed(1)}, ${a.y.toFixed(1)}), confidence ${a.confidence.toFixed(2)}`,
    });
  }
);

export const markCell = spacetimedb.reducer(
  { x: t.i32(), y: t.i32(), state: t.string(), observed_by: t.string() },
  (ctx, a) => {
    // Safety rule: a cell can only be "clear" if a robot observed it.
    if (a.state === 'clear' && a.observed_by === '') throw new Error('cannot mark clear without an observer');
    const key = `${a.x},${a.y}`;
    const existing = ctx.db.map_cell.key.find(key);
    if (existing) ctx.db.map_cell.key.update({ ...existing, ...a });
    else ctx.db.map_cell.insert({ key, ...a });
  }
);

export const assignTask = spacetimedb.reducer(
  { robot_id: t.string(), kind: t.string(), target: t.string() },
  (ctx, a) => {
    ctx.db.task.insert({ id: 0n, robot_id: a.robot_id, kind: a.kind, target: a.target, status: 'pending' });
  }
);

export const completeTask = spacetimedb.reducer(
  { id: t.u64(), status: t.string() },
  (ctx, a) => {
    const row = ctx.db.task.id.find(a.id);
    if (!row) throw new Error('no such task');
    ctx.db.task.id.update({ ...row, status: a.status });
  }
);

export const logCommand = spacetimedb.reducer(
  { transcript: t.string(), intent: t.string() },
  (ctx, a) => {
    ctx.db.command_log.insert({ id: 0n, ...a });
  }
);

export const logEvent = spacetimedb.reducer(
  { robot_id: t.string(), kind: t.string(), message: t.string() },
  (ctx, a) => {
    ctx.db.event.insert({ id: 0n, ...a });
  }
);
