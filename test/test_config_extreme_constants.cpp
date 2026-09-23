// Step 3 finding #11 (high, verifier-narrowed): small_error/big_error set larger than the
// motion's actual starting distance gives a near-instant SMALL_EXIT/BIG_EXIT (a clean,
// interfered=false "success"). This is a tolerance-based settle check doing exactly what it's
// configured to do -- EZ-Template's philosophy is raw, unguarded constants (see WAIT_BEHAVIOR_
// SPEC.md's JC-7, the identical precedent for zeroed exit constants: "no new hard ceiling --
// the team's choice to make", already signed off). No code change here; this locks in and
// documents current behavior so a future reader doesn't mistake an untested characterization
// gap for a live bug, and so any future change to this settle-check semantics has to
// deliberately break this test rather than silently drift.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID: small_error wider than the actual motion gives a near-instant SMALL_EXIT") {
  PID pid;
  pid.exit_condition_set(10, 500.0, 0, 0);  // small_error (500) far exceeds any real motion
  pid.error = 24.0;                         // a real, substantial remaining distance

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 5);  // don't hang the suite if this regresses
    result = pid.exit_condition();
  }
  CHECK(result == SMALL_EXIT);
  CHECK(pass == 2);  // small_exit_time=10ms / DELAY_TIME=10ms -> fires on the 2nd pass
}

TEST_CASE("PID: big_error wider than the actual motion gives a near-instant BIG_EXIT") {
  PID pid;
  pid.exit_condition_set(10, 0.001, 20, 500.0);  // big_error (500) far exceeds any real motion
  pid.error = 24.0;

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 5);
    result = pid.exit_condition();
  }
  CHECK(result == BIG_EXIT);
}
