// What the build and `api:update` both need: which snapshot each docs version uses
// (api/config.json), the snapshot files (api/data/), and a check that every <ApiFunction> tag
// on the pages resolves against the snapshot of the docs version it sits in.
//
// Reads committed files only. No doxygen, no network.

import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {ApiResolveError, resolveFunction, signatureOf} from './resolve.mjs';

export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
export const CONFIG_PATH = path.join(ROOT, 'api', 'config.json');
export const dataPath = (name) => path.join(ROOT, 'api', 'data', `${name}.json`);
const relRoot = (p) => path.relative(ROOT, p).split(path.sep).join('/');

const PAGES_DIR = 'ez-template-docs';
const VERSIONED_DIR = 'versioned_docs';

export function readConfig() {
  let config;
  try {
    config = JSON.parse(fs.readFileSync(CONFIG_PATH, 'utf8'));
  } catch (e) {
    throw new Error(`Cannot read api/config.json: ${e.message}`);
  }
  if (typeof config.current !== 'string' || config.current === '') throw new Error('api/config.json needs "current": "<snapshot name>".');
  if (config.versions === undefined) config.versions = {};
  if (typeof config.versions !== 'object' || Object.values(config.versions).some((v) => typeof v !== 'string')) {
    throw new Error('api/config.json "versions" must map a docs version to a snapshot name, like {"4.0.0": "v4.0.0"}.');
  }
  return config;
}

// A snapshot file parsed, or null when it does not exist.
export function readSnapshot(name) {
  const file = dataPath(name);
  return fs.existsSync(file) ? JSON.parse(fs.readFileSync(file, 'utf8')) : null;
}

function walk(dir) {
  if (!fs.existsSync(dir)) return [];
  return fs.readdirSync(dir, {withFileTypes: true}).flatMap((e) => {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) return e.name === 'node_modules' ? [] : walk(p);
    return /\.mdx?$/.test(e.name) ? [p] : [];
  });
}

// Blanks out fenced code (same length, so offsets and line numbers hold) so a tag shown as an example is not a placed tag.
function maskFences(text) {
  let fence = null;
  return text
    .split('\n')
    .map((line) => {
      const m = /^\s*(`{3,}|~{3,})/.exec(line);
      if (fence) {
        if (m && m[1][0] === fence[0] && m[1].length >= fence.length && line.trim() === m[1]) fence = null;
        return ' '.repeat(line.length);
      }
      if (m) {
        fence = m[1];
        return ' '.repeat(line.length);
      }
      return line;
    })
    .join('\n');
}

const decode = (s) => s.replace(/&quot;/g, '"').replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&amp;/g, '&');
const TAG_RE = /<ApiFunction\b((?:"[^"]*"|'[^']*'|\{[^}]*\}|[^>"'{}])*)>/g;
const ATTR_RE = /([A-Za-z_:][\w:.-]*)\s*=\s*(?:"([^"]*)"|'([^']*)'|(\{[^}]*\}))/g;

function findTags(file) {
  const text = fs.readFileSync(file, 'utf8');
  const masked = maskFences(text);
  const tags = [];
  for (const m of masked.matchAll(TAG_RE)) {
    const line = masked.slice(0, m.index).split('\n').length;
    const attrs = {};
    const problems = [];
    for (const a of m[1].matchAll(ATTR_RE)) {
      const key = a[1];
      let value = a[2] ?? a[3];
      if (value === undefined) {
        const lit = /^\{\s*(["'])(.*)\1\s*\}$/.exec(a[4]);
        if (!lit) {
          problems.push(`${key} must be a plain string, like ${key}="...".`);
          continue;
        }
        value = lit[2];
      }
      if (key !== 'name' && key !== 'args') problems.push(`unknown attribute "${key}" (only name and args are used).`);
      else attrs[key] = decode(value);
    }
    tags.push({line, attrs, problems});
  }
  return tags;
}

// loadSnapshot(name) -> parsed snapshot or null; `api:update` passes one that serves the new data.
// Returns the problems found (file:line messages) and, per docs version, the placed functions by anchor.
export function checkPages(config, loadSnapshot = readSnapshot) {
  const errors = [];
  const placed = new Map();
  const fail = (file, line, message) => errors.push(`${relRoot(file)}:${line}: ${message.replace(/\n/g, '\n    ')}`);

  const locations = [
    {version: 'current', dir: PAGES_DIR, name: config.current},
    ...Object.entries(config.versions).map(([version, name]) => ({version, dir: `${VERSIONED_DIR}/version-${version}`, name})),
  ];

  for (const {version, dir, name} of locations) {
    const snapshot = loadSnapshot(name);
    if (!snapshot) {
      errors.push(
        `api/config.json uses the snapshot "${name}" for ${version === 'current' ? 'the current docs' : `docs version ${version}`}, but api/data/${name}.json does not exist.\n` +
          `    Run: npm run api:update -- ${name}`,
      );
      continue;
    }
    const byAnchor = new Map();
    placed.set(version, byAnchor);
    for (const file of walk(path.join(ROOT, dir))) {
      const seen = new Set();
      for (const {line, attrs, problems} of findTags(file)) {
        for (const p of problems) fail(file, line, `<ApiFunction> ${p}`);
        if (problems.length) continue;
        try {
          const fn = resolveFunction(snapshot.functions, attrs);
          if (seen.has(fn.anchor)) fail(file, line, `${signatureOf(fn)} is placed twice on this page; the anchor #${fn.anchor} would repeat.`);
          seen.add(fn.anchor);
          byAnchor.set(fn.anchor, fn);
        } catch (e) {
          if (!(e instanceof ApiResolveError)) throw e;
          fail(file, line, e.message);
        }
      }
    }
  }

  // Frozen docs versions that are not mapped have no API data to draw on.
  const versionedRoot = path.join(ROOT, VERSIONED_DIR);
  const dirs = fs.existsSync(versionedRoot) ? fs.readdirSync(versionedRoot, {withFileTypes: true}).filter((e) => e.isDirectory()) : [];
  for (const d of dirs) {
    const version = d.name.replace(/^version-/, '');
    if (config.versions[version] !== undefined) continue;
    for (const file of walk(path.join(versionedRoot, d.name))) {
      for (const {line} of findTags(file)) {
        fail(file, line, `docs version ${version} is not mapped in api/config.json, so <ApiFunction> cannot be used here.\n    Write the entry by hand, or map "${version}" to a snapshot under "versions".`);
      }
    }
  }
  return {errors, placed};
}
