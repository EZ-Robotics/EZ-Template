/**
 * Dev-only floating panel for live-previewing the themes/ layer combinations
 * from THEME-TRYOUT.md without touching a terminal. Injects one <style> tag
 * per layer category, swapped on selection; the always-on foundation
 * (themes/base.css) still comes from the normal src/css/custom.css build,
 * this only overrides the swappable layers on top of it.
 *
 * Only runs when NODE_ENV !== 'production', so it never ships to the
 * deployed site — it's a local tool for picking a combo, not a feature.
 */

const LAYERS = {
  density: {
    none: '',
    'a-roomy': `
:root {
  --ifm-font-size-base: 16.5px;
  --ifm-line-height-base: 1.75;
  --ifm-h1-font-size: 2.1rem;
  --ifm-h2-font-size: 1.5rem;
  --ifm-h3-font-size: 1.15rem;
  --ifm-navbar-height: 60px;
  --doc-sidebar-width: 250px;
}
.menu { font-size:.92rem; padding:1rem .8rem !important; }
.menu__link { padding:7px 11px; }
h2 { margin-top:2.6rem; }
.markdown > p, .markdown > ul, .markdown > ol { max-width:70ch; }
`,
    'b-middle': `
:root {
  --ifm-font-size-base: 15.5px;
  --ifm-line-height-base: 1.62;
  --ifm-h1-font-size: 1.95rem;
  --ifm-h2-font-size: 1.4rem;
  --ifm-h3-font-size: 1.08rem;
  --ifm-navbar-height: 54px;
  --doc-sidebar-width: 232px;
}
.menu { font-size:.86rem; padding:.8rem .65rem !important; }
.menu__link { padding:5px 10px; }
h2 { margin-top:2rem; }
.markdown > p, .markdown > ul, .markdown > ol { max-width:76ch; }
`,
    'c-compact': `
:root {
  --ifm-font-size-base: 14.5px;
  --ifm-line-height-base: 1.5;
  --ifm-h1-font-size: 1.78rem;
  --ifm-h2-font-size: 1.28rem;
  --ifm-h3-font-size: 1rem;
  --ifm-navbar-height: 48px;
  --doc-sidebar-width: 215px;
}
.menu { font-size:.82rem; padding:.6rem .55rem !important; }
.menu__link { padding:3px 9px; }
h2 { margin-top:1.5rem; }
.markdown > p, .markdown > ul, .markdown > ol { max-width:82ch; }
table td, table th { padding:.42rem .7rem; font-size:.9rem; }
`,
  },

  temperature: {
    none: '',
    cool: `
:root {
  --ez-ground:#ffffff; --ez-surface:#f7f7fa; --ez-raise:#eeeef4;
  --ez-line:#e3e3ea; --ez-line-soft:#eeeef3;
  --ez-ink:#3f3f4a; --ez-head:#16161d; --ez-muted:#6a6a78;
}
html[data-theme='dark'] {
  --ez-ground:#0d0f14; --ez-surface:#12151b; --ez-raise:#191d25;
  --ez-line:#242a34; --ez-line-soft:#1c212a;
  --ez-ink:#c3c9d4; --ez-head:#ffffff; --ez-muted:#858d9c;
}
`,
  },

  palette: {
    none: '',
    'l1-asis': `
:root {
  --ez-ground:#ffffff; --ez-surface:#f7f7fa; --ez-raise:#eeeef4;
  --ez-line:#e3e3ea; --ez-line-soft:#eeeef3;
  --ez-ink:#3f3f4a; --ez-head:#16161d; --ez-muted:#6a6a78;
}
`,
    'l4-slate': `
:root {
  --ez-ground:#fbfcfe; --ez-surface:#e7ecf4; --ez-raise:#dde4ef;
  --ez-line:#c9d2e0; --ez-line-soft:#dde3ed;
  --ez-ink:#2f3846; --ez-head:#0b1119; --ez-muted:#596374;
}
`,
  },

  hybrid: {
    off: '',
    on: `
:root {
  --ez-ground:#ffffff;
  --ez-nav:#e7ecf4;
  --ez-surface:#f7f7fa;
  --ez-raise:#f0f0f4;
  --ez-line:#dcdfe8; --ez-line-soft:#ebedf2;
  --ez-ink:#2f3846; --ez-head:#0b1119; --ez-muted:#596374;
  --ifm-navbar-background-color: var(--ez-nav);
  --ifm-background-color: var(--ez-ground);
  --ifm-background-surface-color: var(--ez-surface);
}
html[data-theme='dark'] {
  --ez-ground:#0d0f14;
  --ez-nav:#12151b;
  --ez-surface:#1a1b1e;
  --ez-raise:#191d25;
  --ez-line:#242a34; --ez-line-soft:#1c212a;
  --ez-ink:#c3c9d4; --ez-head:#ffffff; --ez-muted:#858d9c;
  --ifm-navbar-background-color: var(--ez-nav);
}
.navbar { background: var(--ez-nav) !important; border-bottom:1px solid var(--ez-line); }
.theme-doc-sidebar-container { background: var(--ez-surface) !important; }
.footer { background: var(--ez-nav); }
`,
  },

  sidebar: {
    none: '',
    s1: `:root { --ez-surface:#f7f7fa; --ez-line:#dcdfe8; --ifm-background-surface-color:var(--ez-surface); }
.theme-doc-sidebar-container { background:var(--ez-surface) !important; }`,
    s2: `:root { --ez-surface:#f0f3f9; --ez-line:#dbe1ec; --ifm-background-surface-color:var(--ez-surface); }
.theme-doc-sidebar-container { background:var(--ez-surface) !important; }`,
    s3: `:root { --ez-surface:#edf1f8; --ez-line:#d6dde9; --ifm-background-surface-color:var(--ez-surface); }
.theme-doc-sidebar-container { background:var(--ez-surface) !important; }`,
    s4: `:root { --ez-surface:#e9eef6; --ez-line:#d2dae8; --ifm-background-surface-color:var(--ez-surface); }
.theme-doc-sidebar-container { background:var(--ez-surface) !important; }`,
  },

  depth: {
    none: '',
    d1: `html[data-theme='dark'] { --ez-ground:#1b1b1d; --ez-nav:#12151b; --ez-surface:#1a1b1e;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    d2: `html[data-theme='dark'] { --ez-ground:#16181e; --ez-nav:#12151b; --ez-surface:#191d25;
  --ez-raise:#1d222b; --ez-line:#262c37;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    d3: `html[data-theme='dark'] { --ez-ground:#121419; --ez-nav:#171b23; --ez-surface:#1b1f28;
  --ez-raise:#1f2530; --ez-line:#282f3b;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    d4: `html[data-theme='dark'] { --ez-ground:#0d0f14; --ez-nav:#12151b; --ez-surface:#191d25;
  --ez-raise:#1d222c; --ez-line:#242a34;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    i1: `html[data-theme='dark'] { --ez-ground:#14171d; --ez-nav:#0f1116; --ez-surface:#0f1116;
  --ez-raise:#1b1f27; --ez-line:#242a34;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    i2: `html[data-theme='dark'] { --ez-ground:#171b23; --ez-nav:#0d0f14; --ez-surface:#0d0f14;
  --ez-raise:#1e242e; --ez-line:#262d38;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    i3: `html[data-theme='dark'] { --ez-ground:#1b2029; --ez-nav:#0d0f14; --ez-surface:#0d0f14;
  --ez-raise:#232a36; --ez-line:#2b333f;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
    i4: `html[data-theme='dark'] { --ez-ground:#202634; --ez-nav:#0b0d12; --ez-surface:#0b0d12;
  --ez-raise:#28303f; --ez-line:#333c4c;
  --ifm-background-color:var(--ez-ground); --ifm-background-surface-color:var(--ez-surface); }`,
  },
};

// Matches the combo currently baked into src/css/custom.css so picking these
// values back in the panel is a visual no-op.
const DEFAULTS = {
  density: 'b-middle',
  temperature: 'cool',
  palette: 'l4-slate',
  hybrid: 'off',
  sidebar: 's4',
  depth: 'd4',
};

const CATEGORY_LABELS = {
  density: 'Density',
  temperature: 'Temperature',
  palette: 'Light palette',
  hybrid: 'Hybrid chrome/sidebar split',
  sidebar: 'Light sidebar tint',
  depth: 'Dark page depth',
};

const STORAGE_KEY = 'ez-theme-panel-v2';

function recipeArgs(state) {
  const args = [];
  if (state.density !== 'none') args.push(state.density);
  if (state.temperature !== 'none') args.push(state.temperature);
  if (state.palette !== 'none') args.push(state.palette);
  if (state.hybrid === 'on') args.push('hybrid');
  if (state.sidebar !== 'none') args.push(state.sidebar);
  if (state.depth !== 'none') args.push(state.depth);
  return args;
}

function loadState() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
    return { ...DEFAULTS, ...saved };
  } catch (e) {
    return { ...DEFAULTS };
  }
}

function saveState(state) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
  } catch (e) {
    /* ignore */
  }
}

function applyState(state, active) {
  Object.keys(LAYERS).forEach((category) => {
    const tagId = `ez-theme-panel-${category}`;
    let tag = document.getElementById(tagId);
    if (!tag) {
      tag = document.createElement('style');
      tag.id = tagId;
      document.head.appendChild(tag);
    }
    tag.textContent = active ? LAYERS[category][state[category]] || '' : '';
  });
}

function buildPanel() {
  if (document.getElementById('ez-theme-panel-root')) return;

  const state = loadState();
  let panelActive = true;

  const root = document.createElement('div');
  root.id = 'ez-theme-panel-root';
  root.style.cssText = `
    position: fixed; bottom: 16px; right: 16px; z-index: 999999;
    font-family: system-ui, sans-serif; font-size: 13px;
    background: #1a1a1c; color: #e4e4e7; border: 1px solid #3a3a42;
    border-radius: 8px; box-shadow: 0 8px 24px rgba(0,0,0,.4);
    width: 260px; overflow: hidden;
  `;

  const header = document.createElement('div');
  header.style.cssText = `
    display:flex; align-items:center; justify-content:space-between;
    padding: 8px 10px; background:#232326; cursor:pointer;
    font-weight:600; user-select:none;
  `;
  header.innerHTML = `<span>🎨 Theme panel</span><span id="ez-theme-panel-caret">▾</span>`;

  const body = document.createElement('div');
  body.style.cssText = 'padding: 10px; display:flex; flex-direction:column; gap:8px;';

  const masterRow = document.createElement('label');
  masterRow.style.cssText = 'display:flex; align-items:center; gap:6px; margin-bottom:2px;';
  masterRow.innerHTML = `<input type="checkbox" id="ez-theme-panel-active" checked /> Panel overrides active`;
  body.appendChild(masterRow);

  const selects = {};
  Object.keys(LAYERS).forEach((category) => {
    const row = document.createElement('label');
    row.style.cssText = 'display:flex; flex-direction:column; gap:2px;';
    const labelText = document.createElement('span');
    labelText.style.cssText = 'color:#a1a1aa; font-size:11px; text-transform:uppercase; letter-spacing:.04em;';
    labelText.textContent = CATEGORY_LABELS[category];
    const select = document.createElement('select');
    select.style.cssText = `
      background:#0d0f14; color:#e4e4e7; border:1px solid #3a3a42;
      border-radius:4px; padding:4px 6px; font-size:13px;
    `;
    Object.keys(LAYERS[category]).forEach((optionName) => {
      const opt = document.createElement('option');
      opt.value = optionName;
      opt.textContent = optionName;
      if (state[category] === optionName) opt.selected = true;
      select.appendChild(opt);
    });
    select.addEventListener('change', () => {
      state[category] = select.value;
      saveState(state);
      applyState(state, panelActive);
      updateRecipeText();
    });
    selects[category] = select;
    row.appendChild(labelText);
    row.appendChild(select);
    body.appendChild(row);
  });

  const recipeLabel = document.createElement('div');
  recipeLabel.style.cssText = 'color:#a1a1aa; font-size:11px; text-transform:uppercase; letter-spacing:.04em; margin-top:4px;';
  recipeLabel.textContent = 'Equivalent apply.sh command';
  body.appendChild(recipeLabel);

  const recipeBox = document.createElement('code');
  recipeBox.style.cssText = `
    display:block; background:#0d0f14; border:1px solid #3a3a42; border-radius:4px;
    padding:6px 8px; font-size:11px; word-break:break-all; line-height:1.4;
  `;
  body.appendChild(recipeBox);

  const copyBtn = document.createElement('button');
  copyBtn.textContent = 'Copy command';
  copyBtn.style.cssText = `
    background:#ff93d5; color:#1a1a1c; border:none; border-radius:4px;
    padding:6px 8px; font-weight:600; cursor:pointer; font-size:12px;
  `;
  copyBtn.addEventListener('click', () => {
    navigator.clipboard.writeText(recipeBox.textContent).then(() => {
      copyBtn.textContent = 'Copied!';
      setTimeout(() => { copyBtn.textContent = 'Copy command'; }, 1200);
    });
  });
  body.appendChild(copyBtn);

  function updateRecipeText() {
    recipeBox.textContent = `./themes/apply.sh ${recipeArgs(state).join(' ')}`;
  }
  updateRecipeText();

  const activeCheckbox = masterRow.querySelector('#ez-theme-panel-active');
  activeCheckbox.addEventListener('change', () => {
    panelActive = activeCheckbox.checked;
    applyState(state, panelActive);
  });

  let collapsed = false;
  header.addEventListener('click', () => {
    collapsed = !collapsed;
    body.style.display = collapsed ? 'none' : 'flex';
    document.getElementById('ez-theme-panel-caret').textContent = collapsed ? '▸' : '▾';
  });

  root.appendChild(header);
  root.appendChild(body);
  document.body.appendChild(root);

  applyState(state, panelActive);
}

if (typeof document !== 'undefined' && process.env.NODE_ENV !== 'production') {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', buildPanel);
  } else {
    buildPanel();
  }
}
