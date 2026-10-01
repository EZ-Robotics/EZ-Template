#!/bin/sh
# Formats EZ-Template's own C/C++ with .clang-format. CI runs the --check form, so this is
# the one place that decides what is formatted.
# Usage: sh tools/format.sh           format every in-scope file in place
#        sh tools/format.sh --check   change nothing, show what would change, exit 1 if anything would
# Set CLANG_FORMAT to use a clang-format that is not on PATH. Works in Git Bash on Windows.
# Vendored and generated files are left out on purpose: include/pros, include/liblvgl,
# include/EZ-Units, include/api.h, include/main.h, firmware/, test/doctest.h, test/fixtures/,
# test/stub/ and include/EZ-Template/version.hpp (written by tools/gen-version.sh).
cd "$(dirname "$0")/.." || exit 1

mode=fix
case "$1" in
  "") ;;
  --check) mode=check ;;
  *) echo "Usage: sh tools/format.sh [--check]"; exit 2 ;;
esac

cf="${CLANG_FORMAT:-clang-format}"
version=$("$cf" --version 2>/dev/null) || { echo "Cannot run '$cf'. Install clang-format (pip install clang-format==18.1.8) or set CLANG_FORMAT."; exit 1; }
echo "$version"
major=$(echo "$version" | sed -n 's/.*version \([0-9][0-9]*\)\..*/\1/p' | head -n 1)
if [ -n "$major" ] && [ "$major" -lt 18 ]; then
  echo "Warning: clang-format $major is older than 18, it may format differently from CI."
fi

files=$(git ls-files -- src include/EZ-Template include/autons.hpp include/subsystems.hpp test |
  grep -E '\.(c|cpp|h|hpp)$' |
  grep -v -E '^(include/EZ-Template/version\.hpp|test/doctest\.h|test/fixtures/|test/stub/)')
[ -n "$files" ] || { echo "No files found. Run this from inside the EZ-Template git checkout."; exit 1; }

if [ "$mode" = fix ]; then
  echo "$files" | xargs -n 40 "$cf" -i --style=file --fallback-style=none || exit 1
  echo "Formatted $(echo "$files" | wc -l | tr -d ' ') files."
  exit 0
fi

tmp=$(mktemp) || exit 1
trap 'rm -f "$tmp"' EXIT
bad=0
for f in $files; do
  "$cf" --style=file --fallback-style=none "$f" > "$tmp" || { echo "clang-format failed on $f"; exit 1; }
  if ! cmp -s "$f" "$tmp"; then
    bad=$((bad + 1))
    echo "Needs formatting: $f"
    diff -u "$f" "$tmp" | tail -n +3
  fi
done
if [ "$bad" -ne 0 ]; then
  echo "$bad file(s) need formatting."
  echo "Run: sh tools/format.sh"
  exit 1
fi
echo "All $(echo "$files" | wc -l | tr -d ' ') files are formatted."
