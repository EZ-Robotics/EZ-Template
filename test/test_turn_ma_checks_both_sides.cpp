// Coverage gap found by mutation testing exit_conditions.cpp's pid_wait() TURN branch: the mA
// check polls both_sides(left_motors, right_motors), not just one side, because a turn drives
// both sides and a jam or pin can land on either one. If a stall lands on a motor this check
// didn't poll, turnPID's mA timer would never accumulate at all, and the turn would have to fall
// all the way back to the much slower progress-based stuck backstop (SingleStuckWatch) -- or hang,
// if that backstop were also disabled -- instead of catching the stall at its configured mA_timeout.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}
}  // namespace

TEST_CASE("pid_wait() TURN mA exit catches a stall on either side, not just one") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // mA exit only (100 ms/10 passes); small/big/velocity off, so a genuine over-current on any
  // polled motor is the only thing that can end this turn quickly. The SingleStuckWatch progress
  // backstop is still live (window_ falls back to mA_timeout when velocity_exit_time is 0) but
  // only fires after its own much longer 1000 ms start allowance plus window -- comfortably later
  // than 100 ms, so a fast return here can only be the mA exit.
  chassis.pid_turn_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_turn_set(90.0, 100);

  // Right side only -- left never reads as over current.
  chassis.right_motors[0].fake().over_current = true;

  test_stub::g_clock.delay_calls_until_stop = 15;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " interfered=", chassis.interfered);

  // Correct code: mA_timeout(100ms) is crossed at pass 11, well inside the 15-pass budget, via
  // the right motor alone.
  REQUIRE(returned);
  CHECK(chassis.interfered);
}
