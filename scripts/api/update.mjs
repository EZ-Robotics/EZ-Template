// npm run api:update -- <source> [--use]
//
// The only thing here that runs doxygen. It saves the function text from one version of the
// library's headers as a snapshot file, api/data/<name>.json, which gets committed. The site
// build reads only those files, so editing pages needs no doxygen.
//
//   <source>  a release tag (v4.0.0-beta.6), a branch (dev), or a local checkout (../EZ-Template)
//   <name>    the tag or branch name, or "local" for a folder
//   --use     also point the current docs at this snapshot (api/config.json "current")
//
// Without --use the file is only written, so you can look at it first. The same source always
// gives a byte-identical file: no timestamps, absolute paths or line numbers are saved.

import {execFileSync, spawnSync} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {extractFunctions} from './doxygen.mjs';
import {CONFIG_PATH, ROOT, checkPages, dataPath, readConfig, readSnapshot} from './check.mjs';
import {signatureOf} from './resolve.mjs';

const REPO = 'https://github.com/EZ-Robotics/EZ-Template.git';
const posix = (p) => p.split(path.sep).join('/');

let tmp = null;

function die(message) {
  console.error(`[api:update] ${message}`);
  if (tmp) fs.rmSync(tmp, {recursive: true, force: true});
  process.exit(1);
}

const args = process.argv.slice(2);
const use = args.includes('--use');
const source = args.find((a) => !a.startsWith('--'));
if (!source) die('usage: npm run api:update -- <tag | branch | folder> [--use]');
if (source.startsWith('-')) die(`"${source}" is not a tag, branch or folder.`);

const probe = spawnSync('doxygen', ['--version'], {encoding: 'utf8'});
if (probe.error || probe.status !== 0) {
  die('doxygen is not installed or not on PATH. Install it: macOS `brew install doxygen`, Windows `winget install DimitriVanHeesch.Doxygen` (then open a new terminal).');
}
const doxygenVersion = probe.stdout.trim().split(/\s+/)[0];

tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'ez-api-'));
try {
  // ---------- where the headers come from ----------
  let srcDir;
  let name;
  let kind;
  if (fs.existsSync(source) && fs.statSync(source).isDirectory()) {
    srcDir = path.resolve(source);
    name = 'local';
    kind = 'local';
  } else {
    let isTag;
    try {
      isTag = execFileSync('git', ['ls-remote', '--tags', REPO, `refs/tags/${source}`], {encoding: 'utf8'}).trim() !== '';
    } catch (e) {
      die(`could not reach ${REPO}: ${e.message}`);
    }
    srcDir = path.join(tmp, 'src');
    try {
      execFileSync('git', ['-c', 'advice.detachedHead=false', 'clone', '--quiet', '--depth', '1', '--branch', source, REPO, srcDir], {stdio: 'inherit'});
    } catch {
      die(`"${source}" is not a folder, and is not a tag or branch of ${REPO}.`);
    }
    name = source.replace(/[^A-Za-z0-9._-]/g, '-');
    kind = isTag ? 'tag' : 'branch';
  }
  if (!fs.existsSync(path.join(srcDir, 'include', 'EZ-Template'))) die(`${srcDir} has no include/EZ-Template folder.`);

  const gitOut = (a) => {
    try {
      return execFileSync('git', ['-C', srcDir, ...a], {encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore']}).trim();
    } catch {
      return '';
    }
  };

  // ---------- doxygen ----------
  const out = path.join(tmp, 'out');
  fs.mkdirSync(out);
  const run = spawnSync('doxygen', [path.join(ROOT, 'api', 'Doxyfile')], {
    cwd: ROOT,
    encoding: 'utf8',
    env: {...process.env, EZ_API_INPUT: posix(srcDir), EZ_API_OUTPUT: posix(out)},
  });
  if (run.error || run.status !== 0) die(`doxygen failed (exit ${run.status}).\n${run.stdout || ''}${run.stderr || ''}${run.error || ''}`);
  const warningsFile = path.join(out, 'doxygen-warnings.txt');
  const warningCount = fs.existsSync(warningsFile) ? fs.readFileSync(warningsFile, 'utf8').split(/\r?\n/).filter((l) => /warning:/.test(l)).length : 0;

  const {functions, unsupportedNotes} = extractFunctions(path.join(out, 'xml'), posix(srcDir));
  const snapshot = {
    schema: 1,
    source: {ref: kind === 'local' ? 'local' : source, kind, commit: gitOut(['rev-parse', 'HEAD']) || 'unknown'},
    doxygen: doxygenVersion,
    functions,
  };

  // ---------- what this changes ----------
  const hasConfig = fs.existsSync(CONFIG_PATH);
  const config = hasConfig ? readConfig() : {current: name, versions: {}};
  const inUse = hasConfig ? readSnapshot(config.current) : null;

  console.log(`[api:update] ${name} (${kind}, ${snapshot.source.commit.slice(0, 7)}), doxygen ${doxygenVersion}: ${functions.length} public functions, ${warningCount} doxygen warnings`);
  if (inUse) {
    const key = (f) => signatureOf(f);
    const before = new Map(inUse.functions.map((f) => [key(f), JSON.stringify(f)]));
    const after = new Map(functions.map((f) => [key(f), JSON.stringify(f)]));
    const added = [...after.keys()].filter((k) => !before.has(k));
    const removed = [...before.keys()].filter((k) => !after.has(k));
    const changed = [...after.keys()].filter((k) => before.has(k) && before.get(k) !== after.get(k));
    const list = (title, items) => {
      console.log(`  ${title}: ${items.length}`);
      for (const i of items.slice(0, 40)) console.log(`    ${i}`);
      if (items.length > 40) console.log(`    ...and ${items.length - 40} more`);
    };
    console.log(`[api:update] compared with the snapshot in use, "${config.current}" (${inUse.source.ref}, ${inUse.source.commit.slice(0, 7)}):`);
    list('added', added);
    list('removed', removed);
    list('changed text or signature', changed);
  } else {
    console.log('[api:update] no snapshot in use to compare with.');
  }

  const {errors} = checkPages({...config, current: name}, (n) => (n === name ? snapshot : readSnapshot(n)));
  if (errors.length) console.log(`[api:update] placed tags that would break with this snapshot as current: ${errors.length}\n\n${errors.join('\n\n')}\n`);
  else console.log('[api:update] placed tags that would break with this snapshot as current: none');
  for (const [tag, fns] of unsupportedNotes) console.log(`[api:update] note: <${tag}> is not rendered (${fns.length} functions, first: ${fns[0]})`);

  // ---------- write ----------
  fs.mkdirSync(path.dirname(dataPath(name)), {recursive: true});
  fs.writeFileSync(dataPath(name), JSON.stringify(snapshot, null, 2) + '\n');
  console.log(`[api:update] wrote api/data/${name}.json (${(fs.statSync(dataPath(name)).size / 1024).toFixed(0)} KB)`);
  if (use) {
    fs.writeFileSync(CONFIG_PATH, JSON.stringify({...config, current: name}, null, 2) + '\n');
    console.log(`[api:update] api/config.json: current is now "${name}"`);
  } else {
    console.log('[api:update] api/config.json not changed (pass --use to point the current docs at this snapshot)');
  }
} finally {
  fs.rmSync(tmp, {recursive: true, force: true});
}
