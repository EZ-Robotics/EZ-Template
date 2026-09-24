// Coverage gap found by mutation testing PID.cpp's timers_reset(): it zeroes hold_timer along
// with the other exit-condition bookkeeping. hold_timer counts how long velocity_exit_hold_set()
// has continuously held the velocity exit open; past VELOCITY_EXIT_HOLD_FALLBACK ms of continuous
// hold it's ignored as a safety valve (see PID::exit_condition()). If timers_reset() didn't zero
// it, a caller that keeps velocity_exit_hold on across an exit-and-restart of the SAME PID object
// (e.g. Drive's turn-bias hold spanning a SMALL_EXIT into the next motion) would inherit however
// much of that 2-second budget the previous motion had already burned, and the fallback could
// release the hold -- and fire a false VELOCITY_EXIT -- far sooner than the caller intended.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID timers_reset() clears hold_timer so a held velocity exit gets a fresh fallback budget") {
  PID pid;
  pid.velocity_sensor_main_exit_set(0.5);
  pid.exit_condition_set(20, 1.0, 0, 0.0, 50, 0);  // small: 20ms/1.0, velocity: 50ms, big/mA off
  pid.velocity_exit_hold_set(true);

  // Arm the velocity channel and burn most (1900 ms of 2000 ms) of the hold's fallback budget.
  // error stays outside small_error(1.0) the whole time so no small exit fires; derivative is 0
  // (stopped) so, without the hold, the velocity exit would fire almost immediately -- the hold
  // is the only thing keeping this RUNNING here.
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms the velocity channel
  pid.derivative = 0.0;
  for (int pass = 0; pass < 189; pass++) {
    INFO("burn pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // Force a SMALL_EXIT, which internally calls timers_reset() -- the line under test. error comes
  // inside small_error; small_exit_time(20ms) is crossed well before hold_timer's own fallback
  // (2000 ms) could ever fire on its own, so this always ends in a SMALL_EXIT, not a VELOCITY_EXIT.
  pid.error = 0.5;
  exit_output result = RUNNING;
  int small_pass = 0;
  while (result == RUNNING) {
    small_pass++;
    REQUIRE(small_pass <= 10);
    result = pid.exit_condition();
  }
  REQUIRE(result == SMALL_EXIT);

  // Re-arm and move back outside small_error so no further small exit competes with what we're
  // testing below, keeping velocity_exit_hold on throughout (it was never turned off).
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);
  pid.derivative = 0.0;

  // Correct code: hold_timer restarted at 0 by timers_reset(), so held stays true (and the
  // velocity exit's own k counter never even starts, since the whole branch is skipped while
  // held) for close to another 2000 ms/200 passes. Buggy code: hold_timer resumed near 1900+10,
  // crosses VELOCITY_EXIT_HOLD_FALLBACK(2000) within about 10 more passes, after which k starts
  // counting and VELOCITY_EXIT fires within velocity_exit_time(50 ms/5 passes) more -- well
  // inside 60 passes total. Sampling 60 passes here cleanly separates the two: correct code must
  // still be RUNNING at the end of it.
  for (int pass = 0; pass < 60; pass++) {
    INFO("post-reset pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
}
