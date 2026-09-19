#!/bin/sh
# Tests tools/gen-version.sh and the include/EZ-Template/version.hpp it writes:
#   1. which VERSION strings it accepts and rejects
#   2. that the generated macros are right, sort in the right order, and compile clean
#   3. that the pattern the header comment tells other libraries to use works with and
#      without the macros (EZ-Template 3.x has none)
#
# Usage: sh version_test.sh "<compiler>"      (VERBOSE=1 to list every check)
# Run from the test/ directory. CI runs it; nothing else needs to. Compile only
# (-fsyntax-only), every check is a static_assert or #error, so nothing runs on the host.
CXX="${1:-g++}"
GEN=../tools/gen-version.sh
FLAGS="-std=c++17 -Wall -Wextra -Werror -fsyntax-only"

WORK="$(mktemp -d)"
LOG="$WORK/out.log"
trap 'rm -rf "$WORK"' EXIT
fail=0
n=0
checks=0

# quiet unless something fails
pass() { checks=$((checks + 1)); [ -z "$VERBOSE" ] || echo "ok    $1"; return 0; }
bad() { checks=$((checks + 1)); echo "FAIL  $1"; fail=1; }
section() { [ -z "$VERBOSE" ] || echo "== $1"; return 0; }

# gen <version> <dir>: writes <dir>/EZ-Template/version.hpp, prints the script's output
gen() {
  mkdir -p "$WORK/$2/EZ-Template"
  sh "$GEN" "$1" "$WORK/$2/EZ-Template/version.hpp"
}

# accept <version>: no output, and a header appears
accept() {
  msg=$(gen "$1" "h/$1")
  f="$WORK/h/$1/EZ-Template/version.hpp"
  if [ -z "$msg" ] && [ -s "$f" ] && [ ! -e "$f.tmp" ]; then pass "accepts '$1'"; else bad "accepts '$1' ($msg)"; fi
}

# reject <version>: prints a message, and a header that was already there is left alone
reject() {
  n=$((n + 1)); f="$WORK/rej$n.hpp"
  echo sentinel > "$f"
  msg=$(sh "$GEN" "$1" "$f")
  if [ -z "$msg" ]; then bad "rejects '$1' (printed nothing)"
  elif [ "$(cat "$f")" != sentinel ] || [ -e "$f.tmp" ]; then bad "rejects '$1' (touched the output file)"
  else pass "rejects '$1': $msg"; fi
}

# compiles <name> <file> [compiler flags]  /  fails_to_compile <name> <file> [compiler flags]
compiles() {
  name="$1"; src="$2"; shift 2
  # shellcheck disable=SC2086
  if $CXX $FLAGS "$@" "$src" >"$LOG" 2>&1; then pass "$name"; else bad "$name"; sed -n 1,8p "$LOG"; fi
}
fails_to_compile() {
  name="$1"; src="$2"; shift 2
  # shellcheck disable=SC2086
  if $CXX $FLAGS "$@" "$src" >"$LOG" 2>&1; then bad "$name (expected a compile error, but it compiled)"; else pass "$name (fails as documented)"; fi
}

section "gen-version.sh accepts"
for v in 4.0.0-beta.2 4.0.0-rc.1 4.0.0 4.1.0-alpha.3 10.99.99-rc.99 0.1.0 4.10.0 4.0.10 4.0.0-alpha.0 4.0.0-beta.10; do accept "$v"; done

section "gen-version.sh rejects"
for v in 4.0.0-dev 4.0 4.0.0-beta 4.0.0.1 4.100.0 4.0.100 4.0.0-beta.100 4.0.0-beta.2.3 4.0.0-beta.2-x \
         4.010.0 4.08.0 04.0.0 4.0.0-beta.08 4.0.0- 4.0.0-.1 4.0.0-beta.x 4.x.0 v4.0.0 ""; do reject "$v"; done

section "build metadata"
# LemLib/pros-build rewrites VERSION to 4.0.0-beta.2+<sha> in CI before it runs make template.
# That must neither fail the build nor change the header.
gen 4.0.0-beta.2 meta/plain >/dev/null
for v in 4.0.0-beta.2+3f9a1c 4.0.0-beta.2+3f9a1c.dirty; do
  msg=$(gen "$v" meta/with)
  if [ -n "$msg" ]; then bad "accepts '$v' ($msg)"
  elif ! cmp -s "$WORK/meta/plain/EZ-Template/version.hpp" "$WORK/meta/with/EZ-Template/version.hpp"; then bad "'$v' gives a different header than 4.0.0-beta.2"
  else pass "accepts '$v', header identical to 4.0.0-beta.2"; fi
done
msg=$(gen 4.0.0+3f9a1c meta/rel)
if [ -z "$msg" ] && grep -q 'VERSION_STRING "4.0.0"$' "$WORK/meta/rel/EZ-Template/version.hpp"; then pass "accepts '4.0.0+3f9a1c' as 4.0.0"; else bad "accepts '4.0.0+3f9a1c' ($msg)"; fi

section "header is only rewritten when it changes"
mkdir -p "$WORK/mtime"
touch "$WORK/mtime/ref"
sh "$GEN" 4.0.0 "$WORK/mtime/version.hpp"
touch -t 200001010000 "$WORK/mtime/version.hpp"
touch "$WORK/mtime/ref"
sh "$GEN" 4.0.0 "$WORK/mtime/version.hpp"
if [ "$WORK/mtime/version.hpp" -ot "$WORK/mtime/ref" ]; then pass "same version leaves the header alone"; else bad "same version rewrote the header"; fi
sh "$GEN" 4.0.1 "$WORK/mtime/version.hpp"
if grep -q 'VERSION_STRING "4.0.1"' "$WORK/mtime/version.hpp"; then pass "new version rewrites the header"; else bad "new version did not rewrite the header"; fi

section "default paths, no arguments, and the root version file"
# A copy of the repo layout, so the real Makefile, version file and header are never touched.
fake="$WORK/fake"
mkdir -p "$fake/tools" "$fake/include/EZ-Template"
cp "$GEN" "$fake/tools/gen-version.sh"
fgen() { sh "$fake/tools/gen-version.sh" "$@"; }
fake_header="$fake/include/EZ-Template/version.hpp"
expect_version_file() { printf '%s' "$1" > "$WORK/expected"; cmp -s "$WORK/expected" "$fake/version"; }

printf 'LIBNAME:=EZ-Template\nVERSION:=4.0.0-rc.1\n' > "$fake/Makefile"
printf '%s' 4.0.0-beta.2 > "$fake/version"
msg=$(fgen)
if [ -z "$msg" ] && grep -q 'VERSION_STRING "4.0.0-rc.1"' "$fake_header" && expect_version_file 4.0.0-rc.1; then pass "no arguments: reads VERSION from the Makefile, header and version file follow"; else bad "no arguments ($msg)"; fi

msg=$(fgen 4.0.0-rc.1+3f9a1c)
if [ -z "$msg" ] && expect_version_file 4.0.0-rc.1; then pass "build metadata does not reach the version file"; else bad "build metadata reached the version file ($msg)"; fi

msg=$(fgen 4.1.0 "$WORK/elsewhere.hpp")
if [ -z "$msg" ] && expect_version_file 4.0.0-rc.1; then pass "an explicit output path leaves the version file alone"; else bad "an explicit output path changed the version file"; fi

msg=$(fgen 4.0.0-dev)
if [ -n "$msg" ] && expect_version_file 4.0.0-rc.1 && grep -q 'VERSION_STRING "4.0.0-rc.1"' "$fake_header"; then pass "a bad version leaves the header and version file alone"; else bad "a bad version changed something ($msg)"; fi

printf 'VERSION:=4.0.0-dev\n' > "$fake/Makefile"
msg=$(fgen)
if [ -n "$msg" ]; then pass "a bad VERSION in the Makefile is reported: $msg"; else bad "a bad VERSION in the Makefile printed nothing"; fi

# Windows checkouts have CRLF in the Makefile
printf 'VERSION:=4.0.0\r\n' > "$fake/Makefile"
msg=$(fgen)
if [ -z "$msg" ] && expect_version_file 4.0.0; then pass "a CRLF Makefile works"; else bad "a CRLF Makefile ($msg)"; fi

rm "$fake/version"
printf 'VERSION:=4.2.0\n' > "$fake/Makefile"
msg=$(fgen)
if [ -z "$msg" ] && [ ! -e "$fake/version" ] && grep -q 'VERSION_STRING "4.2.0"' "$fake_header"; then pass "without a version file, none is created"; else bad "without a version file ($msg)"; fi

section "the macros for each accepted version"
# compile_case <version> <major> <minor> <patch> <stage> <num> <prerelease string>
compile_case() {
  v="$1"
  cat > "$WORK/case.cpp" <<CPP
#include "h/$v/EZ-Template/version.hpp"

constexpr bool streq(const char* a, const char* b) { return *a == *b && (*a == 0 || streq(a + 1, b + 1)); }

static_assert(EZ_TEMPLATE_VERSION_MAJOR == $2, "major");
static_assert(EZ_TEMPLATE_VERSION_MINOR == $3, "minor");
static_assert(EZ_TEMPLATE_VERSION_PATCH == $4, "patch");
static_assert(EZ_TEMPLATE_VERSION_STAGE == EZ_TEMPLATE_STAGE_$5, "stage");
static_assert(EZ_TEMPLATE_VERSION_PRERELEASE_NUM == $6, "prerelease number");
static_assert(streq(EZ_TEMPLATE_VERSION_PRERELEASE, "$7"), "prerelease string");
static_assert(streq(EZ_TEMPLATE_VERSION_STRING, "$v"), "version string");
static_assert(EZ_TEMPLATE_VERSION == EZ_TEMPLATE_VERSION_ENCODE($2, $3, $4, EZ_TEMPLATE_STAGE_$5, $6), "encoded version");

// AT_LEAST ignores the prerelease: true for its own major.minor.patch, false for anything above
static_assert(EZ_TEMPLATE_VERSION_AT_LEAST($2, $3, $4), "at least itself");
static_assert(!EZ_TEMPLATE_VERSION_AT_LEAST($2, $3, $4 + 1), "at least the next patch");
static_assert(!EZ_TEMPLATE_VERSION_AT_LEAST($2 + 1, 0, 0), "at least the next major");

// and it has to work in #if, which is where other libraries use it
#if !EZ_TEMPLATE_VERSION_AT_LEAST($2, $3, $4)
#error "AT_LEAST is false in #if for its own version"
#endif
#if EZ_TEMPLATE_VERSION_AT_LEAST($2 + 1, 0, 0)
#error "AT_LEAST is true in #if for the next major"
#endif
CPP
  compiles "macros for $v compile clean and are right" "$WORK/case.cpp"
}
compile_case 4.0.0-beta.2 4 0 0 BETA 2 beta.2
compile_case 4.0.0-rc.1 4 0 0 RC 1 rc.1
compile_case 4.0.0 4 0 0 RELEASE 0 ""
compile_case 4.1.0-alpha.3 4 1 0 ALPHA 3 alpha.3
compile_case 10.99.99-rc.99 10 99 99 RC 99 rc.99

section "ordering"
# One translation unit that includes the header generated for every version in turn, keeps
# each EZ_TEMPLATE_VERSION and whether AT_LEAST(4, 0, 0) held, and #undefs the macros so
# the next header can define them again.
ORDER="3.2.2 3.9.9 4.0.0-alpha.1 4.0.0-alpha.99 4.0.0-beta.0 4.0.0-beta.2 4.0.0-beta.10 4.0.0-rc.1 4.0.0-rc.99 4.0.0 4.0.1-alpha.1 4.0.1 4.0.99 4.1.0-alpha.0 4.1.0 10.0.0-alpha.0"
i=0; prev=""
: > "$WORK/order.cpp"
for v in $ORDER; do
  gen "$v" "ord/$i" >/dev/null
  # the undef list comes from the generated header, so new macros are picked up automatically
  sed -n 's/^#define \(EZ_TEMPLATE_[A-Za-z_]*\).*/#undef \1/p' "$WORK/ord/$i/EZ-Template/version.hpp" > "$WORK/ord/$i/undef.hpp"
  {
    echo "#include \"ord/$i/EZ-Template/version.hpp\""
    echo "constexpr long v$i = EZ_TEMPLATE_VERSION;"
    echo "constexpr bool at400_$i = EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0);"
    echo "#include \"ord/$i/undef.hpp\""
    [ -n "$prev" ] && echo "static_assert(v$((i - 1)) < v$i, \"$prev < $v\");"
    case "$v" in
      [0-3].*) echo "static_assert(!at400_$i, \"AT_LEAST(4, 0, 0) is false on $v\");" ;;
      *)       echo "static_assert(at400_$i, \"AT_LEAST(4, 0, 0) is true on $v\");" ;;
    esac
  } >> "$WORK/order.cpp"
  prev="$v"; i=$((i + 1))
done
compiles "$(echo $ORDER | sed 's/ / < /g')" "$WORK/order.cpp"

section "using the macros from another library"
# EZ-Template 3.x has api.hpp but no version macros, so the 3.x stand-in is an api.hpp that
# is empty. The 4.x stand-ins include the generated header, like the real api.hpp does.
if grep -q '#include "EZ-Template/version.hpp"' ../include/EZ-Template/api.hpp; then pass "api.hpp includes version.hpp"; else bad "api.hpp does not include version.hpp"; fi
mkdir -p "$WORK/v3/EZ-Template"
: > "$WORK/v3/EZ-Template/api.hpp"
for v in 4.0.0-beta.2 4.0.0-rc.1 4.0.0; do
  mkdir -p "$WORK/v4-$v/EZ-Template"
  cp "$WORK/h/$v/EZ-Template/version.hpp" "$WORK/v4-$v/EZ-Template/"
  echo '#include "EZ-Template/version.hpp"' > "$WORK/v4-$v/EZ-Template/api.hpp"
done

# The pattern from the header comment, verbatim
cat > "$WORK/downstream.cpp" <<'CPP'
#include "EZ-Template/api.hpp"

#ifndef EZ_TEMPLATE_VERSION_AT_LEAST
#define EZ_TEMPLATE_VERSION_AT_LEAST(major, minor, patch) 0
#endif

#if EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)
constexpr int api_level = 4;
#else
constexpr int api_level = 3;
#endif

#ifdef EZ_TEMPLATE_VERSION
#if EZ_TEMPLATE_VERSION >= EZ_TEMPLATE_VERSION_ENCODE(4, 0, 0, EZ_TEMPLATE_STAGE_RC, 1)
constexpr int rc_or_newer = 1;
#else
constexpr int rc_or_newer = 0;
#endif
#else
constexpr int rc_or_newer = 0;
#endif

// Passing "it compiles" isn't enough: a fallback that wins on 4.x would compile too.
static_assert(api_level == EXPECT_API, "took the wrong AT_LEAST branch");
static_assert(rc_or_newer == EXPECT_RC, "took the wrong exact-version branch");
CPP
compiles "3.x (no macros): fallback, takes the 3.x branch" "$WORK/downstream.cpp" -I "$WORK/v3" -DEXPECT_API=3 -DEXPECT_RC=0
compiles "4.0.0-beta.2: takes the 4.x branch, before rc.1" "$WORK/downstream.cpp" -I "$WORK/v4-4.0.0-beta.2" -DEXPECT_API=4 -DEXPECT_RC=0
compiles "4.0.0-rc.1: takes the 4.x branch, is rc.1" "$WORK/downstream.cpp" -I "$WORK/v4-4.0.0-rc.1" -DEXPECT_API=4 -DEXPECT_RC=1
compiles "4.0.0: takes the 4.x branch, is past rc.1" "$WORK/downstream.cpp" -I "$WORK/v4-4.0.0" -DEXPECT_API=4 -DEXPECT_RC=1

# Why the header comment says not to do this: on 3.x it is a preprocessor syntax error, even
# though defined() is false, because the preprocessor still parses the second half.
cat > "$WORK/defined_and.cpp" <<'CPP'
#include "EZ-Template/api.hpp"
#if defined(EZ_TEMPLATE_VERSION_MAJOR) && EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)
#endif
CPP
fails_to_compile "defined() && AT_LEAST on 3.x" "$WORK/defined_and.cpp" -I "$WORK/v3"

# The opt-in prerelease warning from the header comment: silent on 3.x and on a release, and a
# warning (so an error under -Werror) on a prerelease.
cat > "$WORK/prerelease.cpp" <<'CPP'
#include "EZ-Template/api.hpp"

#ifdef EZ_TEMPLATE_VERSION_STAGE
#if EZ_TEMPLATE_VERSION_STAGE != EZ_TEMPLATE_STAGE_RELEASE
#warning "Built against an EZ-Template prerelease, the API may change"
#endif
#endif
CPP
compiles "prerelease warning is silent on 3.x" "$WORK/prerelease.cpp" -I "$WORK/v3"
compiles "prerelease warning is silent on 4.0.0" "$WORK/prerelease.cpp" -I "$WORK/v4-4.0.0"
for v in 4.0.0-beta.2 4.0.0-rc.1; do
  if $CXX $FLAGS -I "$WORK/v4-$v" "$WORK/prerelease.cpp" >"$LOG" 2>&1; then bad "prerelease warning fires on $v (it compiled clean)"
  elif grep -q 'Built against an EZ-Template prerelease' "$LOG"; then pass "prerelease warning fires on $v"
  else bad "prerelease warning on $v failed for another reason"; sed -n 1,8p "$LOG"; fi
done

if [ "$fail" -eq 0 ]; then echo "version tests passed ($checks checks)"; else echo "version tests FAILED"; fi
exit "$fail"
