// Reads the XML that doxygen writes and turns every public function into a plain record:
// prototype, summary, details, params, returns and examples as a small tree the <ApiFunction>
// component renders. Only scripts/api/update.mjs uses this; the site build never runs doxygen.
//
// The records carry no line numbers, absolute paths or doxygen ids, so a snapshot only changes
// when the wording or the signature changes.

import fs from 'node:fs';
import path from 'node:path';
import {DOMParser} from '@xmldom/xmldom';

const parseXml = (file) =>
  new DOMParser({
    onError: (level, message) => {
      if (level !== 'warning') throw new Error(`${file}: ${message}`);
    },
  }).parseFromString(fs.readFileSync(file, 'utf8'), 'text/xml');

const kids = (n) => Array.from(n.childNodes);
const els = (n, tag) => kids(n).filter((c) => c.nodeType === 1 && (!tag || c.tagName === tag));
const first = (n, tag) => els(n, tag)[0];
const textOf = (n) => (n ? n.textContent : '');
const squash = (s) => s.replace(/\s+/g, ' ');

const SYMBOLS = {ndash: '–', mdash: '—', nonbreakablespace: ' ', sp: ' ', lsquo: '‘', rsquo: '’', ldquo: '“', rdquo: '”', hellip: '…', rarr: '→', larr: '←', times: '×', deg: '°', plusmn: '±'};
const ADMONITIONS = {
  note: ['note', null],
  remark: ['note', 'Remark'],
  warning: ['warning', null],
  attention: ['warning', 'Attention'],
  see: ['info', 'See also'],
  pre: ['note', 'Precondition'],
  post: ['note', 'Postcondition'],
};

// ctx collects what a description holds besides running text: params, returns, examples.
function newCtx() {
  return {paramDocs: new Map(), returns: [], examples: [], unsupported: new Set()};
}

// Drops empty strings, merges neighbours and trims the edges; null when nothing is left.
function tidy(inline) {
  const out = [];
  for (const n of inline) {
    if (typeof n === 'string') {
      if (n === '') continue;
      if (typeof out[out.length - 1] === 'string') out[out.length - 1] += n;
      else out.push(n);
    } else out.push(n);
  }
  if (typeof out[0] === 'string') out[0] = out[0].replace(/^\s+/, '');
  if (typeof out[out.length - 1] === 'string') out[out.length - 1] = out[out.length - 1].replace(/\s+$/, '');
  const cleaned = out.filter((n) => n !== '');
  return cleaned.length ? cleaned : null;
}

function inlineOf(node, ctx) {
  const out = [];
  for (const c of kids(node)) {
    if (c.nodeType === 3 || c.nodeType === 4) out.push(squash(c.data));
    else if (c.nodeType === 1) out.push(...inlineElement(c, ctx));
  }
  return out;
}

function inlineElement(c, ctx) {
  switch (c.tagName) {
    case 'computeroutput':
      return [{t: 'code', c: squash(textOf(c))}];
    case 'bold':
      return [{t: 'b', c: inlineOf(c, ctx)}];
    case 'emphasis':
      return [{t: 'i', c: inlineOf(c, ctx)}];
    case 'ulink': {
      const href = c.getAttribute('url') || '';
      return /^(https?:|mailto:)/.test(href) ? [{t: 'a', href, c: inlineOf(c, ctx)}] : inlineOf(c, ctx);
    }
    case 'linebreak':
      return [{t: 'br'}];
    case 'anchor':
      return [];
    case 'ref':
    case 'strike':
    case 'underline':
    case 'superscript':
    case 'subscript':
      return inlineOf(c, ctx);
    default:
      if (SYMBOLS[c.tagName]) return [SYMBOLS[c.tagName]];
      ctx.unsupported.add(c.tagName);
      return [squash(textOf(c))];
  }
}

function codeText(node) {
  let out = '';
  for (const c of kids(node)) {
    if (c.nodeType === 3 || c.nodeType === 4) out += c.data;
    else if (c.nodeType === 1) {
      if (c.tagName === 'sp') out += ' '.repeat(Number(c.getAttribute('value')) || 1);
      else if (c.tagName === 'codeline') out += codeText(c) + '\n';
      else out += codeText(c);
    }
  }
  return out;
}

function parseBlocks(container, ctx) {
  const blocks = [];
  if (!container) return blocks;
  for (const c of els(container)) {
    if (c.tagName === 'para') blocks.push(...parsePara(c, ctx));
    else if (c.tagName !== 'title') ctx.unsupported.add(c.tagName);
  }
  return blocks;
}

function parsePara(para, ctx) {
  const blocks = [];
  let run = [];
  const flush = () => {
    const inline = tidy(run);
    if (inline) blocks.push({t: 'p', c: inline});
    run = [];
  };
  for (const c of kids(para)) {
    if (c.nodeType === 3 || c.nodeType === 4) {
      run.push(squash(c.data));
      continue;
    }
    if (c.nodeType !== 1) continue;
    switch (c.tagName) {
      case 'itemizedlist':
      case 'orderedlist':
        flush();
        blocks.push({t: c.tagName === 'itemizedlist' ? 'ul' : 'ol', items: els(c, 'listitem').map((li) => parseBlocks(li, ctx))});
        break;
      case 'programlisting': {
        flush();
        const lang = (c.getAttribute('filename') || '').replace(/^\./, '') || 'cpp';
        ctx.examples.push({lang, code: codeText(c).replace(/\s+$/, '')});
        break;
      }
      case 'verbatim':
      case 'preformatted':
        flush();
        blocks.push({t: 'pre', code: textOf(c).replace(/\s+$/, '')});
        break;
      case 'parameterlist':
        flush();
        if (c.getAttribute('kind') === 'param') {
          for (const item of els(c, 'parameteritem')) {
            const desc = parseBlocks(first(item, 'parameterdescription'), ctx);
            for (const n of els(first(item, 'parameternamelist'), 'parametername')) ctx.paramDocs.set(textOf(n).trim(), desc);
          }
        } else ctx.unsupported.add(`parameterlist kind=${c.getAttribute('kind')}`);
        break;
      case 'simplesect': {
        flush();
        const kind = c.getAttribute('kind');
        if (kind === 'return') ctx.returns.push(...parseBlocks(c, ctx));
        else if (ADMONITIONS[kind]) blocks.push({t: 'admonition', type: ADMONITIONS[kind][0], title: ADMONITIONS[kind][1], c: parseBlocks(c, ctx)});
        else ctx.unsupported.add(`simplesect kind=${kind}`);
        break;
      }
      case 'xrefsect': {
        flush();
        const title = textOf(first(c, 'xreftitle')).trim();
        if (title === 'Deprecated') blocks.push({t: 'admonition', type: 'danger', title: 'Deprecated', c: parseBlocks(first(c, 'xrefdescription'), ctx)});
        else ctx.unsupported.add(`xrefsect ${title}`);
        break;
      }
      case 'hruler':
        flush();
        break;
      default:
        run.push(...inlineElement(c, ctx));
    }
  }
  flush();
  return blocks;
}

// Doxygen spaces types out ("std::vector< int > &"); the headers do not.
function tidyType(type) {
  return squash(type)
    .replace(/\s*<\s*/g, '<')
    .replace(/\s*>/g, '>')
    .replace(/,(?=\S)/g, ', ')
    .replace(/\s+([&*])/g, '$1')
    .trim();
}

// Text after the parameter list's closing parenthesis: " const", " override", " = 0".
function afterParams(argsstring) {
  let depth = 0;
  for (let i = 0; i < argsstring.length; i++) {
    if (argsstring[i] === '(') depth++;
    else if (argsstring[i] === ')' && --depth === 0) return argsstring.slice(i + 1).replace(/\s+/g, ' ').trimEnd();
  }
  return '';
}

function buildPrototype(m, name, params) {
  const tp = first(m, 'templateparamlist');
  const template = tp ? `template <${els(tp, 'param').map((p) => tidyType(textOf(first(p, 'type')))).join(', ')}>\n` : '';
  const prefix = [m.getAttribute('static') === 'yes' && 'static', m.getAttribute('virt') !== 'non-virtual' && 'virtual', m.getAttribute('explicit') === 'yes' && 'explicit']
    .filter(Boolean)
    .map((w) => w + ' ')
    .join('');
  const ret = tidyType(textOf(first(m, 'type')));
  const list = params.map((p) => `${p.type}${p.name ? ' ' + p.name : ''}${p.array}${p.defval ? ' = ' + p.defval : ''}`).join(', ');
  return `${template}${prefix}${ret ? ret + ' ' : ''}${name}(${list})${afterParams(textOf(first(m, 'argsstring')))};`;
}

function readFunction(m, qualifier, unsupportedNotes, srcPosix) {
  const name = textOf(first(m, 'name')).trim();
  const params = els(m, 'param').map((p) => ({
    type: tidyType(textOf(first(p, 'type'))),
    name: textOf(first(p, 'declname')).trim() || textOf(first(p, 'defname')).trim(),
    array: textOf(first(p, 'array')).trim(),
    defval: squash(textOf(first(p, 'defval'))).trim(),
  }));

  const ctx = newCtx();
  const briefBlocks = parseBlocks(first(m, 'briefdescription'), ctx);
  const brief = tidy(briefBlocks.flatMap((b, i) => (b.t === 'p' ? [...(i ? [' '] : []), ...b.c] : [])));
  const details = parseBlocks(first(m, 'detaileddescription'), ctx);
  for (const tag of ctx.unsupported) {
    if (!unsupportedNotes.has(tag)) unsupportedNotes.set(tag, []);
    unsupportedNotes.get(tag).push(`${qualifier}::${name}`);
  }

  const loc = first(m, 'location');
  return {
    qualifier,
    cls: qualifier.split('::').pop(),
    name,
    params,
    returnType: tidyType(textOf(first(m, 'type'))),
    prototype: buildPrototype(m, name, params),
    brief,
    details,
    paramDocs: params.filter((p) => p.name && ctx.paramDocs.has(p.name)).map((p) => ({name: p.name, desc: ctx.paramDocs.get(p.name)})),
    returns: ctx.returns,
    examples: ctx.examples,
    // Header order decides overload numbers. Only used here, never saved.
    order: [loc.getAttribute('file').split(String.fromCharCode(92)).join('/').replace(srcPosix + '/', ''), Number(loc.getAttribute('line'))],
  };
}

const byText = (a, b) => (a < b ? -1 : a > b ? 1 : 0);

// xmlDir: doxygen's xml/ folder. srcPosix: the checkout root with forward slashes.
// Returns the functions in a stable order, plus the doxygen elements this does not render yet.
export function extractFunctions(xmlDir, srcPosix) {
  if (!fs.existsSync(path.join(xmlDir, 'index.xml'))) {
    throw new Error('doxygen finished but wrote no xml/index.xml. Is GENERATE_XML on in api/Doxyfile?');
  }
  const unsupportedNotes = new Map();
  const functions = [];
  for (const compound of els(parseXml(path.join(xmlDir, 'index.xml')).documentElement, 'compound')) {
    if (!['class', 'struct', 'namespace'].includes(compound.getAttribute('kind'))) continue;
    const doc = parseXml(path.join(xmlDir, compound.getAttribute('refid') + '.xml'));
    const def = first(doc.documentElement, 'compounddef');
    const qualifier = textOf(first(def, 'compoundname')).trim();
    if (qualifier.split('::').includes('detail') || qualifier.includes('@')) continue;
    for (const section of els(def, 'sectiondef')) {
      for (const m of els(section, 'memberdef')) {
        if (m.getAttribute('kind') === 'function' && m.getAttribute('prot') === 'public') {
          functions.push(readFunction(m, qualifier, unsupportedNotes, srcPosix));
        }
      }
    }
  }

  // Overloads are numbered in header order; anchors are api-<Class>-<name>, with -<n> when overloaded.
  functions.sort((a, b) => byText(a.order[0], b.order[0]) || a.order[1] - b.order[1]);
  const groups = new Map();
  for (const f of functions) {
    const key = `${f.qualifier}::${f.name}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(f);
  }
  const anchors = new Set();
  for (const group of groups.values()) {
    group.forEach((f, i) => {
      f.overloadIndex = i + 1;
      f.overloadCount = group.length;
      const suffix = group.length > 1 ? `-${i + 1}` : '';
      let anchor = `api-${f.cls}-${f.name}`;
      if (anchors.has(anchor + suffix)) anchor = `api-${f.qualifier.replace(/::/g, '-')}-${f.name}`;
      anchors.add(anchor + suffix);
      f.anchor = anchor + suffix;
    });
  }

  // Stable output order that does not depend on where a function sits in a header.
  const records = functions
    .map(({order, ...rest}) => rest)
    .sort((a, b) => byText(a.qualifier, b.qualifier) || byText(a.name, b.name) || a.overloadIndex - b.overloadIndex);
  return {functions: records, unsupportedNotes};
}
