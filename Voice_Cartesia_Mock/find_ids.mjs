// Prints your Cartesia agent IDs and the phone number IDs attached to them.
//   export CARTESIA_API_KEY=...
//   node find_ids.mjs
const KEY = process.env.CARTESIA_API_KEY;
if (!KEY) { console.error('Set CARTESIA_API_KEY first.'); process.exit(1); }
const r = await fetch('https://api.cartesia.ai/agents', {
  headers: { Authorization: `Bearer ${KEY}`, 'Cartesia-Version': process.env.CARTESIA_AGENTS_VERSION || '2026-08-14' },
});
if (!r.ok) { console.error(`Cartesia error ${r.status}: ${await r.text()}`); process.exit(1); }
const { summaries = [] } = await r.json();
if (!summaries.length) console.log('No agents found. Create one in the Cartesia playground first.');
for (const a of summaries) {
  console.log(`\nAgent: ${a.name}\n  CARTESIA_AGENT_ID=${a.id}`);
  const nums = a.phone_numbers || [];
  if (!nums.length) console.log('  (no phone number assigned: assign a Cartesia number to this agent in the playground)');
  for (const n of nums) console.log(`  CARTESIA_FROM_NUMBER_ID=${n.id}   (${n.number})`);
}
