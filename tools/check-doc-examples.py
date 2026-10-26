#!/usr/bin/env python3
"""Checks the \\code ... \\endcode examples and \\param tags in include/EZ-Template/**/*.hpp.

The docs site builds its function pages from these comments, so a team pastes these examples
into a project. Each example is checked three ways.

  compile  It compiles on its own, as if pasted into a fresh example project. It is wrapped in
           the example project's main.h setup and checked with -fsyntax-only by the ARM
           compiler against the real PROS headers in include/, not the host test stubs.
  format   It equals clang-format output with the repo's .clang-format but ColumnLimit 100,
           so every line is 100 columns or fewer, and with EXAMPLE_STYLE below: no function,
           loop or if body on one line, and braces on every if, else, for and while.  The width
           is that of the example text, not counting the comment's " * " or "/// " prefix.
  params   Every \\param name is a parameter of the function the comment documents.

Usage: python3 tools/check-doc-examples.py [--fix] [--skip compile,format,params] [--jobs N]
  --fix   rewrites the examples that fail "format" in place, then runs the other checks
  --skip  leaves out checks. CI runs "format" and "params" with clang-format on the runner and
          "compile" inside the pros-build image, which has the ARM compiler.
Every failure prints as file:line: function: what is wrong. Exit status is 1 if any example or
tag fails.

Needs Python 3 and the standard library. "format" needs clang-format 18.1.8 (CLANG_FORMAT sets
the path). "compile" needs arm-none-eabi-g++ (ARM_GXX sets the path). Run it from anywhere inside
the checkout.

include/EZ-Template/version.hpp is left out. tools/gen-version.sh generates it, and its examples
are preprocessor snippets, not function docs.
"""

import argparse
import concurrent.futures
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HEADER_DIR = "include/EZ-Template"
SKIP_FILES = {"include/EZ-Template/version.hpp"}
FORMAT_COLUMNS = 100
# What the examples change from the repo's .clang-format. Examples are read by students, so a
# function body or a loop body is never joined onto one line, and an if, else, for or while
# always has braces.
EXAMPLE_STYLE = {
    "ColumnLimit": str(FORMAT_COLUMNS),
    "AllowShortFunctionsOnASingleLine": "None",
    "AllowShortIfStatementsOnASingleLine": "Never",
    "AllowShortLoopsOnASingleLine": "false",
    "AllowShortBlocksOnASingleLine": "Never",
    "InsertBraces": "true",
}
CHECKS = ("compile", "format", "params")

# The example project's main.h, which every team's code sees.
PREAMBLE = """\
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#include "api.h"
#include "EZ-Template/api.hpp"
extern ez::Drive chassis;
extern "C" {
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
}
"""

# Flags from common.mk and the Makefile, minus the optimizer and warnings.
COMPILE_FLAGS = [
    "-fsyntax-only", "-mcpu=cortex-a9", "-mfpu=neon-fp16", "-mfloat-abi=hard", "-mthumb",
    "-D_POSIX_THREADS", "-D_UNIX98_THREAD_MUTEX_ATTRIBUTES", "-D_POSIX_TIMERS", "-D_POSIX_MONOTONIC_CLOCK",
    "-D_PROS_INCLUDE_LIBLVGL_LLEMU_H", "-D_PROS_INCLUDE_LIBLVGL_LLEMU_HPP",
    "--std=gnu++26", "-Wno-psabi", "-w", "-fmax-errors=0", "-fno-diagnostics-color", "-iquote", "include",
]


class Example:
    def __init__(self, path, function, start, lines, prefixes):
        self.path = path          # repo relative, forward slashes
        self.function = function  # name of the declaration the comment documents
        self.start = start        # 1-based line of the first example line in the file
        self.lines = lines        # example text, comment prefix removed
        self.prefixes = prefixes  # the removed prefixes, one per line

    @property
    def text(self):
        return "\n".join(self.lines) + "\n"

    def where(self, offset=0):
        return "%s:%d" % (self.path, self.start + offset)


class Failure:
    def __init__(self, rule, where, function, message, key=None):
        self.rule, self.where, self.function, self.message = rule, where, function, message
        self.key = key or where  # what is counted: one example, or one \param tag

    def __str__(self):
        return "%s: %s: [%s] %s" % (self.where, self.function, self.rule, self.message)


# ---------------------------------------------------------------------------------------------
# Reading the headers
# ---------------------------------------------------------------------------------------------

def split_comment_line(raw):
    """Returns (prefix, text) for a line inside a doc comment, or None if it isn't a comment line."""
    m = re.match(r"^(\s*///)( ?)(.*)$", raw)
    if m:
        return m.group(1), m.group(3)
    m = re.match(r"^(\s*\*)( ?)(.*)$", raw)
    if m and not raw.strip().startswith("*/"):
        return m.group(1), m.group(3)
    return None


def find_comment_blocks(lines):
    """Yields (first_index, last_index) of each doc comment: a /** ... */ or a run of ///."""
    i = 0
    while i < len(lines):
        s = lines[i].strip()
        if s.startswith("/**"):
            j = i
            while "*/" not in lines[j] and j + 1 < len(lines):
                j += 1
            yield i, j
            i = j + 1
        elif s.startswith("///"):
            j = i
            while j + 1 < len(lines) and lines[j + 1].strip().startswith("///"):
                j += 1
            yield i, j
            i = j + 1
        else:
            i += 1


def comment_text(raw, first_line):
    """Text of one line of a /** */ or /// comment without its prefix, plus that prefix."""
    s = raw.strip()
    if s.startswith("///"):
        parts = split_comment_line(raw)
        return parts[0], parts[1]
    if first_line:
        m = re.match(r"^(\s*/\*\*)( ?)(.*?)(\*/)?\s*$", raw)
        return m.group(1), m.group(3)
    if s.startswith("*/"):
        return raw[: raw.index("*/")], ""
    parts = split_comment_line(raw)
    if parts is None:
        return "", s
    text = parts[1]
    if text.rstrip().endswith("*/"):
        text = text.rstrip()[:-2].rstrip()
    return parts[0], text


def strip_balanced(s, open_c, close_c):
    depth = 0
    out = []
    for c in s:
        if c == open_c:
            depth += 1
        elif c == close_c and depth:
            depth -= 1
        elif depth == 0:
            out.append(c)
    return "".join(out)


def read_declaration(lines, after):
    """Text of the declaration that starts at lines[after], up to its ';' or '{', or None."""
    k = after
    while k < len(lines) and not lines[k].strip():
        k += 1
    if k >= len(lines):
        return None
    if lines[k].lstrip().startswith("#define"):
        return lines[k].strip()
    out = []
    depth = 0
    for row in lines[k : k + 40]:
        stripped = re.sub(r"//.*$", "", row)
        for c in stripped:
            if c in "([":
                depth += 1
            elif c in ")]":
                depth -= 1
            elif c in ";{" and depth <= 0:
                return "".join(out) + " "
            out.append(c)
        out.append(" ")
    return "".join(out)


def parse_declaration(decl):
    """Returns (name, [parameter names]). parameters is None when the declaration has no (...)."""
    if decl is None:
        return "?", None
    decl = decl.strip()
    m = re.match(r"#define\s+(\w+)\s*(\(([^)]*)\))?", decl)
    if m:
        names = [p.strip() for p in (m.group(3) or "").split(",") if p.strip()] if m.group(2) else None
        return m.group(1), names
    decl = re.sub(r"\[\[[^\]]*\]\]", " ", decl)
    if re.match(r"\s*template\s*<", decl):
        decl = decl[re.match(r"\s*template\s*", decl).end():]
        depth = 0
        for n, c in enumerate(decl):
            if c == "<":
                depth += 1
            elif c == ">":
                depth -= 1
                if depth == 0:
                    decl = decl[n + 1:]
                    break
    # The first '(' that isn't inside template arguments.
    angle = 0
    paren = -1
    for n, c in enumerate(decl):
        if c == "<" and n and (decl[n - 1].isalnum() or decl[n - 1] == "_") and not decl[:n].rstrip().endswith("operator"):
            angle += 1
        elif c == ">" and angle and decl[n - 1] != "-":
            angle -= 1
        elif c == "(" and angle == 0:
            paren = n
            break
    head = decl[: paren if paren >= 0 else len(decl)]
    head = re.split(r"=|\{", head)[0] if paren < 0 else head
    names = re.findall(r"operator\s*[^\s\w(]+|~?\w+", head)
    name = names[-1] if names else "?"
    if re.match(r"^\s*(class|struct|enum|union|namespace)\b", head) and paren < 0:
        words = re.findall(r"\w+", head)
        words = [w for w in words if w not in ("class", "struct", "enum", "union", "namespace")]
        name = words[0] if words else "?"
    if paren < 0:
        return name, None
    depth = 0
    end = None
    for n in range(paren, len(decl)):
        if decl[n] == "(":
            depth += 1
        elif decl[n] == ")":
            depth -= 1
            if depth == 0:
                end = n
                break
    inner = decl[paren + 1 : end if end is not None else len(decl)]
    # Split on top level commas.
    parts, cur, depth, angle = [], [], 0, 0
    for n, c in enumerate(inner):
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "<" and n and (inner[n - 1].isalnum() or inner[n - 1] == "_"):
            angle += 1
        elif c == ">" and angle and inner[n - 1] != "-":
            angle -= 1
        if c == "," and depth == 0 and angle == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(c)
    parts.append("".join(cur))
    params = []
    for p in parts:
        p = p.strip()
        if not p or p == "void" or p == "...":
            continue
        d = 0
        for n, c in enumerate(p):  # drop the default value
            if c in "(<{[":
                d += 1
            elif c in ")>}]":
                d -= 1
            elif c == "=" and d == 0:
                p = p[:n]
                break
        p = re.sub(r"\[[^\]]*\]\s*$", "", p.strip())
        fp = re.search(r"\(\s*[*&]\s*(\w+)", p)
        if fp:
            params.append(fp.group(1))
            continue
        ids = re.findall(r"[A-Za-z_]\w*", strip_balanced(p, "<", ">"))
        if len(ids) >= 2:
            params.append(ids[-1])
        else:
            params.append(None)
    return name, params


def read_header(path):
    """Returns (examples, tags) for one header. tags are (where, function, param_names, declared)."""
    with open(os.path.join(ROOT, path), encoding="utf-8", newline="") as f:
        lines = f.read().replace("\r\n", "\n").split("\n")
    examples, tags = [], []
    for first, last in find_comment_blocks(lines):
        decl_text = read_declaration(lines, last + 1)
        function, declared = parse_declaration(decl_text)
        in_code = None
        for idx in range(first, last + 1):
            prefix, text = comment_text(lines[idx], idx == first)
            code_tag = re.match(r"^\s*[\\@]code(\{[^}]*\})?\s*$", text)
            if code_tag:
                in_code = []
                continue
            if re.match(r"^\s*[\\@]endcode\s*$", text) and in_code is not None:
                block = in_code
                while block and not block[-1][1].strip():
                    block.pop()
                if block:
                    examples.append(Example(path, function, block[0][0] + 1, [t for _, t, _ in block], [p for _, _, p in block]))
                in_code = None
                continue
            if in_code is not None:
                in_code.append((idx, text.rstrip(), prefix))
                continue
            m = re.match(r"^\s*[\\@]param(?:\[[^\]]*\])?\s+(\w+(?:\s*,\s*\w+)*)", text)
            if m:
                names = [n.strip() for n in m.group(1).split(",")]
                tags.append(("%s:%d" % (path, idx + 1), function, names, declared))
    return examples, tags


def read_all():
    examples, tags = [], []
    base = os.path.join(ROOT, HEADER_DIR)
    for dirpath, _, files in os.walk(base):
        for name in sorted(files):
            if not name.endswith(".hpp"):
                continue
            rel = os.path.relpath(os.path.join(dirpath, name), ROOT).replace(os.sep, "/")
            if rel in SKIP_FILES:
                continue
            e, t = read_header(rel)
            examples += e
            tags += t
    examples.sort(key=lambda e: (e.path, e.start))
    tags.sort(key=lambda t: t[0])
    return examples, tags


# ---------------------------------------------------------------------------------------------
# params
# ---------------------------------------------------------------------------------------------

def check_params(tags):
    failures = []
    for where, function, names, declared in tags:
        for name in names:
            if declared is None:
                failures.append(Failure("params", where, function, "\\param %s is on a declaration with no parameter list" % name))
            elif name not in declared:
                real = ", ".join(n for n in declared if n) or "none"
                failures.append(Failure("params", where, function, "\\param %s is not a parameter (parameters: %s)" % (name, real)))
    return failures


# ---------------------------------------------------------------------------------------------
# format
# ---------------------------------------------------------------------------------------------

def clang_format_path():
    path = os.environ.get("CLANG_FORMAT") or "clang-format"
    found = shutil.which(path)
    if not found:
        sys.exit("Cannot run '%s'. Install clang-format 18.1.8 (pip install clang-format==18.1.8) or set CLANG_FORMAT." % path)
    return found


def format_examples(examples, tmp):
    """Returns the formatted text of each example, in order."""
    cf = clang_format_path()
    with open(os.path.join(ROOT, ".clang-format"), encoding="utf-8") as f:
        config = f.read()
    for key, value in EXAMPLE_STYLE.items():
        config, n = re.subn(r"(?m)^%s:.*$" % key, "%s: %s" % (key, value), config)
        if n == 0:
            config += "\n%s: %s\n" % (key, value)
        elif n > 1:
            sys.exit(".clang-format has %s more than once" % key)
    cfg = os.path.join(tmp, "clang-format-100.yaml")
    with open(cfg, "w", encoding="utf-8", newline="\n") as f:
        f.write(config)
    version = subprocess.run([cf, "--version"], capture_output=True, text=True).stdout.strip()
    if "18." not in version:
        print("Warning: %s. CI uses 18.1.8 and other versions may format differently." % version, file=sys.stderr)

    def one(e):
        r = subprocess.run([cf, "--style=file:" + cfg, "--assume-filename=example.cpp"], input=e.text.encode("utf-8"), capture_output=True)
        if r.returncode != 0:
            sys.exit("clang-format failed on %s: %s" % (e.where(), r.stderr.decode("utf-8", "replace")))
        return r.stdout.decode("utf-8").replace("\r\n", "\n")

    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        return list(pool.map(one, examples))


def check_format(examples, formatted):
    failures = []
    for e, out in zip(examples, formatted):
        for offset, line in enumerate(e.lines):
            if len(line) > FORMAT_COLUMNS:
                failures.append(Failure("format", e.where(offset), e.function, "line is %d columns, the limit is %d" % (len(line), FORMAT_COLUMNS), e.where()))
        if out != e.text:
            want = out.rstrip("\n").split("\n")
            have = e.lines
            offset = next((n for n in range(min(len(want), len(have))) if want[n] != have[n]), min(len(want), len(have)))
            failures.append(Failure("format", e.where(offset), e.function, "not clang-format output at 100 columns (python3 tools/check-doc-examples.py --fix)", e.where()))
    return failures


def fix_format(examples, formatted):
    """Rewrites the examples that differ from clang-format output. Returns how many changed."""
    by_file = {}
    for e, out in zip(examples, formatted):
        if out != e.text:
            by_file.setdefault(e.path, []).append((e, out.rstrip("\n").split("\n")))
    changed = 0
    for path, items in by_file.items():
        full = os.path.join(ROOT, path)
        with open(full, encoding="utf-8", newline="") as f:
            raw = f.read()
        crlf = "\r\n" in raw
        lines = raw.replace("\r\n", "\n").split("\n")
        for e, new in sorted(items, key=lambda it: -it[0].start):
            prefix = e.prefixes[0]
            new_lines = [prefix + (" " + t if t else "") for t in new]
            lines[e.start - 1 : e.start - 1 + len(e.lines)] = new_lines
            changed += 1
        with open(full, "w", encoding="utf-8", newline="") as f:
            f.write(("\r\n" if crlf else "\n").join(lines))
    return changed


# ---------------------------------------------------------------------------------------------
# compile
# ---------------------------------------------------------------------------------------------

def arm_gxx():
    path = os.environ.get("ARM_GXX") or "arm-none-eabi-g++"
    found = shutil.which(path)
    if not found:
        sys.exit("Cannot run '%s'. Install the PROS toolchain (or use the pros-build image) or set ARM_GXX." % path)
    return found


def run_gxx(gxx, source, tmp, name):
    path = os.path.join(tmp, name)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(source)
    r = subprocess.run([gxx] + COMPILE_FLAGS + [path], cwd=ROOT, capture_output=True, text=True, errors="replace")
    return r.returncode, r.stderr


ERROR_RE = re.compile(r"^(ex(\d+)\.cpp):(\d+):(\d+): (?:fatal )?error: (.*)$")


def one_example_source(e, ident, wrap):
    body = "#line 1 \"ex%d.cpp\"\n%s" % (ident, e.text)
    if wrap:
        body = "namespace ex_%d {\n%s}\n" % (ident, body)
        # The #line has to come after the namespace line so error lines match the example.
    return body


def compile_units(gxx, tmp, chunk, label, wrap):
    """Compiles chunk, [(ident, example)], as one translation unit. Returns ({ident: (line,
    message)}, clean) where clean is False when an error couldn't be traced to an example."""
    source = PREAMBLE + "".join(one_example_source(e, i, wrap) for i, e in chunk)
    code, stderr = run_gxx(gxx, source, tmp, "unit_%s.cpp" % label)
    ids = {i for i, _ in chunk}
    errors, clean = {}, True
    for line in stderr.splitlines():
        if " error" not in line:
            continue
        m = ERROR_RE.match(line)
        if m and int(m.group(2)) in ids:
            errors.setdefault(int(m.group(2)), (int(m.group(3)), m.group(5)))
        else:
            clean = False
    if code != 0 and not errors:
        clean = False
    return errors, clean


def compile_chunk(gxx, tmp, chunk, label):
    """chunk is [(ident, example)]. Returns {ident: (line, first error message)}. Examples
    without preprocessor lines share a translation unit, each in its own namespace. A chunk
    with any error is split in half until the failing examples are alone, because one
    unbalanced brace would otherwise blame the examples after it."""
    plain = [(i, e) for i, e in chunk if not re.search(r"^\s*#", e.text, re.M)]
    errors = {}

    def bisect(part, tag):
        errs, clean = compile_units(gxx, tmp, part, tag, True)
        if clean and not errs:
            return
        if len(part) == 1:
            ident = part[0][0]
            errors[ident] = errs.get(ident, (1, "does not compile"))
            return
        half = len(part) // 2
        bisect(part[:half], tag + "a")
        bisect(part[half:], tag + "b")

    if plain:
        bisect(plain, label)
    for i, e in chunk:
        if (i, e) in plain:
            continue
        errs, clean = compile_units(gxx, tmp, [(i, e)], "%s_%d" % (label, i), False)
        if errs or not clean:
            errors[i] = errs.get(i, (1, "does not compile"))
    return errors


def check_compile(examples, tmp, jobs):
    gxx = arm_gxx()
    numbered = list(enumerate(examples))
    size = max(1, -(-len(numbered) // (jobs * 2)))
    chunks = [numbered[n : n + size] for n in range(0, len(numbered), size)]
    failures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        results = list(pool.map(lambda args: compile_chunk(gxx, tmp, args[1], str(args[0])), enumerate(chunks)))
    for errors in results:
        for ident, (line, message) in errors.items():
            e = examples[ident]
            failures.append(Failure("compile", e.where(line - 1), e.function, message, e.where()))
    return failures


# ---------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Check the \\code examples and \\param tags in include/EZ-Template.")
    ap.add_argument("--fix", action="store_true", help="rewrite examples that fail the format check")
    ap.add_argument("--skip", default="", help="comma separated checks to leave out: " + ", ".join(CHECKS))
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4, help="parallel compiler runs")
    args = ap.parse_args()
    skip = {s.strip() for s in args.skip.split(",") if s.strip()}
    if skip - set(CHECKS):
        sys.exit("--skip takes: " + ", ".join(CHECKS))

    examples, tags = read_all()
    print("Found %d examples and %d \\param tags." % (len(examples), len(tags)))
    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        if "format" not in skip:
            formatted = format_examples(examples, tmp)
            if args.fix:
                print("Rewrote %d examples." % fix_format(examples, formatted))
                examples, tags = read_all()
                formatted = format_examples(examples, tmp)
            failures += check_format(examples, formatted)
        elif args.fix:
            sys.exit("--fix needs the format check")
        if "params" not in skip:
            failures += check_params(tags)
        if "compile" not in skip:
            failures += check_compile(examples, tmp, max(1, args.jobs))

    failures.sort(key=lambda f: (f.where.rsplit(":", 1)[0], int(f.where.rsplit(":", 1)[1]), f.rule))
    for f in failures:
        print(f)

    def count(rule):
        return len({f.key for f in failures if f.rule == rule})

    print()
    for rule in CHECKS:
        if rule not in skip:
            print("%-8s %d failing %s" % (rule, count(rule), "tags" if rule == "params" else "examples"))
    if failures:
        print("%d failures." % len(failures))
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
