// Runs before `npm start` and `npm run build`. Reads only committed files (api/config.json and
// api/data/*.json): no doxygen, no network.
//
//   1. checks every <ApiFunction> tag against the snapshot of the docs version it sits in
//   2. writes api/generated/data.json: for each docs version, just the functions its pages
//      place, so the browser bundle stays small
//
// With DOCS_TARGET=website it also refuses to build unless "current" is a released tag, so
// the live site cannot show text from a branch or a local folder.

import fs from 'node:fs';
import path from 'node:path';
import {ROOT, checkPages, readConfig, readSnapshot} from './check.mjs';

function fail(problems) {
  console.error(`[api] ${problems.length} problem${problems.length > 1 ? 's' : ''} with the API docs:\n\n${problems.join('\n\n')}\n`);
  process.exit(1);
}

let config;
try {
  config = readConfig();
} catch (e) {
  fail([e.message]);
}

const {errors, placed} = checkPages(config);

if (process.env.DOCS_TARGET === 'website') {
  const snapshot = readSnapshot(config.current);
  if (snapshot && !(/^v\d/.test(config.current) && snapshot.source.kind === 'tag' && snapshot.source.ref === config.current)) {
    errors.push(
      `api/config.json "current" is "${config.current}" (${snapshot.source.kind} ${snapshot.source.ref}) and DOCS_TARGET=website is set.\n` +
        `    The live site can only use a release snapshot. On website-dev run npm run api:update -- vX.Y.Z --use, merge that, then promote.`,
    );
  }
}
if (errors.length) fail(errors);

const data = {};
for (const [version, byAnchor] of placed) {
  data[version] = [...byAnchor.values()].sort((a, b) => (a.anchor < b.anchor ? -1 : 1));
}
const out = path.join(ROOT, 'api', 'generated');
fs.mkdirSync(out, {recursive: true});
fs.writeFileSync(path.join(out, 'data.json'), JSON.stringify(data, null, 2) + '\n');

const snapshotOf = (version) => (version === "current" ? config.current : config.versions[version]);
const summary = [...placed].map(([version, byAnchor]) => `${version} (${snapshotOf(version)}): ${byAnchor.size}`);
console.log(`[api] snapshots in use, functions placed: ${summary.join(', ')}`);
