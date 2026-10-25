// Tiny .env loader (no dependencies). Import this FIRST in any file that reads process.env.
//
// Reads ".env" next to this file. Real environment variables win over the file, so
// `CALL_ENABLED=0 node server.mjs` still overrides whatever .env says.
// Format: KEY=value, one per line. "#" starts a comment. Quotes around values are optional.
// The .env file holds secrets: it is in .gitignore and must never be pasted into chat or committed.

import { readFileSync } from 'node:fs';

try {
  const text = readFileSync(new URL('./.env', import.meta.url), 'utf8');
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    const eq = line.indexOf('=');
    if (eq < 1) continue;
    const key = line.slice(0, eq).trim().replace(/^export\s+/, '');
    let val = line.slice(eq + 1).trim();
    if ((val.startsWith('"') && val.endsWith('"')) || (val.startsWith("'") && val.endsWith("'"))) val = val.slice(1, -1);
    else val = val.replace(/\s+#.*$/, ''); // trailing comment on an unquoted value
    if (key && process.env[key] === undefined) process.env[key] = val;
  }
} catch {
  /* no .env file: fine, plain environment variables still work */
}
