// PID::exit_condition()'s BIG_EXIT branch calls timers_reset() before returning, exactly like its
// SMALL_EXIT and VELOCITY_EXIT siblings (VELOCITY_EXIT's own re-arm is already pinned by
// test_velocity_exit_rearm.cpp). No existing test polls the same PID again after a BIG_EXIT with no
// intervening setter call, so nothing currently protects that one call.
//
// A caller that keeps polling exit_condition() on the same PID after a BIG_EXIT, with no pid_*_set()
// in between, is a real, reachable path: pid_wait_until() ending on a failsafe exit, immediately
// followed by pid_wait() to finish out the same motion. Without the reset, the big-error timer stays
// above big_exit_time, so the very next exit_condition() call re-fires BIG_EXIT immediately instead of
// getting a fresh settling window -- unlike every other exit type, which all reset on their own return.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID exit_condition automatically re-arms after a BIG_EXIT with no explicit timers_reset() call, same as SMALL_EXIT and VELOCITY_EXIT already do") {
  PID pid;
  pid.exit_condition_set(0, 0.0, 250, 3.0, 0, 0);  // big only: 250ms/3in, small/velocity/mA off

  // A real compute_error() call every pass, not a direct `.error =` write -- the big exit timer only
  // credits `error` when a real compute has landed since it last checked (see PID.cpp), so "held" has
  // to mean a real compute repeatedly landing on the same value, the same as pid_wait_until() ->
  // pid_wait() on the same motion would have ez_auto_task doing throughout this whole sequence.
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 40);  // don't hang the suite if this regresses; real fire is pass 26
    pid.compute_error(2.0, 0.0);  // inside big_error, outside small (small is off anyway)
    result = pid.exit_condition();
  }
  CHECK(result == BIG_EXIT);

  // No explicit pid.timers_reset() call here -- the real reachable path (a caller that keeps
  // polling exit_condition() on the same PID after a BIG_EXIT, e.g. pid_wait_until() -> pid_wait()
  // on the same motion). The very next call must NOT immediately re-fire BIG_EXIT; it needs a
  // fresh big_exit_time countdown, same as after an explicit timers_reset().
  pid.compute_error(2.0, 0.0);
  CHECK(pid.exit_condition() == RUNNING);

  // And the fresh countdown behaves identically to a real motion start: the same passes before it
  // can fire again.
  int pass2 = 1;  // the call just above already consumed one pass of the fresh countdown
  result = RUNNING;
  while (result == RUNNING) {
    pass2++;
    REQUIRE(pass2 <= 40);
    pid.compute_error(2.0, 0.0);
    result = pid.exit_condition();
  }
  CHECK(result == BIG_EXIT);
  CHECK(pass2 == pass);  // identical settling time to the first, real BIG_EXIT
}
