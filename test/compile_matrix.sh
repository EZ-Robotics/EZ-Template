#!/bin/sh
# What does upgrading look like from a user's project? Compiles
# compile/user_project.cpp (a stand-in for main.h + autons.cpp) under each
# combination of "is okapilib installed", "does main.h still have the old
# `using namespace okapi::literals;` line" and "does the project include okapi's
# unit headers itself", and checks the result is the documented one.
#
# Usage: compile_matrix.sh "<compiler and flags>" "<extra flags>"
# Run from the test/ directory (the Makefile does).
CXX="${1:-g++}"
FLAGS="$2 -fsyntax-only"

SRC=compile/user_project.cpp
WORK="$(mktemp -d)"
LOG="$WORK/out.log"
trap 'rm -rf "$WORK"' EXIT
fail=0

# Only EZ-Template's own headers go on the include path. Whether okapi is
# "installed" must be decided by the -I fixtures flag below, not by whatever
# else happens to be sitting in ../include.
mkdir "$WORK/include"
cp -R ../include/EZ-Template ../include/EZ-Units "$WORK/include/"
INC="-I stub -I $WORK/include -iquote $WORK/include/EZ-Template -I ."

# check <name> <expected: pass|fail> <error regex, "-" if none> <defines and include flags>
check() {
  name="$1"; want="$2"; pattern="$3"; extra="$4"
  # shellcheck disable=SC2086
  $CXX $FLAGS $INC $extra $SRC >"$LOG" 2>&1
  rc=$?
  if [ "$want" = pass ]; then
    if [ $rc -eq 0 ]; then echo "ok    $name"; else echo "FAIL  $name (expected to compile)"; sed -n 1,8p "$LOG"; fail=1; fi
  else
    if [ $rc -eq 0 ]; then
      echo "FAIL  $name (expected a compile error, but it compiled)"; fail=1
    elif [ "$pattern" != "-" ] && ! grep -qiE "$pattern" "$LOG"; then
      echo "FAIL  $name (failed, but not with /$pattern/)"; sed -n 1,8p "$LOG"; fail=1
    else
      echo "ok    $name (fails as documented)"
    fi
  fi
}

OKAPI="-I fixtures"
STALE="-DSTALE_MAIN_H_LINE"
USES="-DUSER_INCLUDES_OKAPI_UNITS"

check "no okapi anywhere"                                   pass -            ""
check "okapilib installed, main.h line removed"             pass -            "$OKAPI"
check "okapilib installed, stale main.h line (common case)" pass -            "$OKAPI $STALE"
check "okapi units included, main.h line removed"           pass -            "$OKAPI $USES"
check "okapi units included, stale main.h line"             fail "ambiguous"  "$OKAPI $USES $STALE"
check "okapilib removed, stale main.h line"                 fail "okapi"      "$STALE"


[ $fail -eq 0 ] && echo "upgrade compile matrix: all as documented"
exit $fail
