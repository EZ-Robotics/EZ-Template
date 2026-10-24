// Finds one function in the generated API data from what a docs page asks for.
//
// Shared by scripts/api/generate.mjs (checks every tag before the build) and by
// the <ApiFunction> component (picks the entry to render), so both agree on what
// a tag means. It is bundled for the browser build: no node: imports here.
//
//   <ApiFunction name="pid_wait" />
//   <ApiFunction name="Drive::pid_wait" />                  when the name is in more than one class
//   <ApiFunction name="pid_drive_set" args="double, int" /> to pick one overload
//   <ApiFunction name="a" args="int | int, bool" />         one entry that shows several overloads of a
//   <ApiFunction name="a" also="a_forward, a_backward" />  one entry that also covers its siblings

export class ApiResolveError extends Error {
  constructor(message) {
    super(message);
    this.name = 'ApiResolveError';
  }
}

// "std::vector<std::pair<int, int>>, bool" -> ["std::vector<std::pair<int, int>>", "bool"]
export function splitArgs(text) {
  const out = [];
  let depth = 0;
  let cur = '';
  for (const ch of text) {
    if (ch === '<' || ch === '(' || ch === '[') depth++;
    else if (ch === '>' || ch === ')' || ch === ']') depth--;
    if (ch === ',' && depth === 0) {
      out.push(cur);
      cur = '';
    } else {
      cur += ch;
    }
  }
  if (cur.trim() !== '' || out.length > 0) out.push(cur);
  return out.map((a) => a.trim());
}

// `const`, `ez::`, `std::` and spacing do not tell overloads apart, so a page can say
// "QLength, int" or "const ez::QLength, int" and get the same overload.
export function normalizeType(type) {
  return type
    .replace(/\bconst\b/g, '')
    .replace(/\b(?:ez|std)::/g, '')
    .replace(/\s+/g, '');
}

export function argsKey(types) {
  return types.map(normalizeType).join(',');
}

export function signatureOf(fn) {
  const params = fn.params.map((p) => p.type + (p.name ? ' ' + p.name : '')).join(', ');
  return `${fn.qualifier}::${fn.name}(${params})`;
}

function argsAttr(fn) {
  return `args="${fn.params.map((p) => p.type).join(', ')}"`;
}

function editDistance(a, b) {
  const prev = Array.from({length: b.length + 1}, (_, i) => i);
  for (let i = 1; i <= a.length; i++) {
    let diag = prev[0];
    prev[0] = i;
    for (let j = 1; j <= b.length; j++) {
      const tmp = prev[j];
      prev[j] = Math.min(prev[j] + 1, prev[j - 1] + 1, diag + (a[i - 1] === b[j - 1] ? 0 : 1));
      diag = tmp;
    }
  }
  return prev[b.length];
}

function suggestNames(functions, wanted) {
  const names = [...new Set(functions.map((f) => f.name))];
  const limit = Math.max(2, Math.floor(wanted.length / 3));
  return names
    .map((n) => ({n, d: editDistance(n.toLowerCase(), wanted.toLowerCase())}))
    .filter((x) => x.d <= limit)
    .sort((a, b) => a.d - b.d || a.n.localeCompare(b.n))
    .slice(0, 4)
    .map((x) => x.n);
}

function listCandidates(cands) {
  return cands.map((f) => `    ${signatureOf(f)}`).join('\n');
}

// args may list several overloads separated by "|", like args="ez::QLength, int | ez::QLength, int, bool, bool".
// The entry shows all of them, in the order listed. Its text comes from the one with the most parameters (the first
// of those if they tie), which is the most complete form.
export function resolveOverloads(functions, query) {
  const parts = query.args === undefined ? [undefined] : query.args.split('|');
  const shown = parts.map((args) => resolveFunction(functions, {name: query.name, args}));
  if (shown.some((f, i) => shown.indexOf(f) !== i)) {
    throw new ApiResolveError(`<ApiFunction name="${query.name}" args="${query.args}"> lists the same overload twice.`);
  }
  const primary = shown.reduce((best, f) => (f.params.length > best.params.length ? f : best), shown[0]);
  return {primary, shown};
}

// The functions an entry also covers (also="a, b"). They must be in the same class as the primary, and when a name is
// overloaded the one with the same parameter types as the primary is used.
export function resolveAlso(functions, primary, also) {
  const names = also.split(',').map((n) => n.trim()).filter(Boolean);
  if (names.length === 0) throw new ApiResolveError(`also="${also}" lists no function names.`);
  const key = argsKey(primary.params.map((p) => p.type));
  return names.map((name) => {
    const cands = functions.filter((f) => f.qualifier === primary.qualifier && f.name === name);
    if (cands.length === 0) {
      const hint = suggestNames(functions.filter((f) => f.qualifier === primary.qualifier), name);
      throw new ApiResolveError(`also="${name}": no function named "${name}" in ${primary.qualifier}.` + (hint.length ? `\n  Did you mean: ${hint.join(', ')}?` : ''));
    }
    const same = cands.filter((f) => argsKey(f.params.map((p) => p.type)) === key);
    if (same.length === 1) return same[0];
    if (cands.length === 1) return cands[0];
    throw new ApiResolveError(`also="${name}" is overloaded and no overload has the parameter types of ${signatureOf(primary)}. The overloads are:\n${listCandidates(cands)}`);
  });
}

// query: { name: string, args?: string }. Returns the matching function record or throws ApiResolveError.
export function resolveFunction(functions, query) {
  const {name, args} = query;
  if (typeof name !== 'string' || name.trim() === '') {
    throw new ApiResolveError('<ApiFunction> needs a name="..." attribute.');
  }
  if (args !== undefined && typeof args !== 'string') {
    throw new ApiResolveError(`<ApiFunction name="${name}"> has an args attribute that is not a string.`);
  }

  const parts = name.trim().split('::');
  const fname = parts.pop();
  const scope = parts.join('::');
  const where = args === undefined ? `"${name}"` : `"${name}" with args="${args}"`;

  let cands = functions.filter((f) => f.name === fname);
  if (scope) cands = cands.filter((f) => f.qualifier === scope || f.qualifier.endsWith('::' + scope));

  if (cands.length === 0) {
    const hint = suggestNames(functions, fname);
    throw new ApiResolveError(
      `No function named "${name}" in the API data.` +
        (hint.length ? `\n  Did you mean: ${hint.join(', ')}?` : '') +
        `\n  Check the spelling, and that the function is declared in the headers of the version being built.`,
    );
  }

  const scopes = [...new Set(cands.map((f) => f.qualifier))];
  if (!scope && scopes.length > 1) {
    throw new ApiResolveError(
      `${where} is ambiguous: it is declared in ${scopes.length} places.\n${listCandidates(cands)}\n` +
        `  Use name="Class::${fname}" (for example name="${scopes[0].split('::').pop()}::${fname}"), plus args="..." if it is also overloaded.`,
    );
  }

  if (args !== undefined) {
    const wanted = argsKey(splitArgs(args));
    const hits = cands.filter((f) => argsKey(f.params.map((p) => p.type)) === wanted);
    if (hits.length === 1) return hits[0];
    if (hits.length === 0) {
      throw new ApiResolveError(
        `${where} matches no overload. The overloads are:\n${cands
          .map((f) => `    ${signatureOf(f)}\n      -> ${argsAttr(f)}`)
          .join('\n')}\n  args must list every parameter type, including ones that have defaults.`,
      );
    }
    throw new ApiResolveError(`${where} still matches ${hits.length} overloads.\n${listCandidates(hits)}`);
  }

  if (cands.length > 1) {
    throw new ApiResolveError(
      `${where} is overloaded; say which one with args="...".\n${cands
        .map((f) => `    ${signatureOf(f)}\n      -> ${argsAttr(f)}`)
        .join('\n')}`,
    );
  }
  return cands[0];
}
