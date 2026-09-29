// wait_until_turn_swing_internal()'s TURN branch has two separate places that decide whether a
// wait_until() call's target really is the motion's own final aim, so that settling inside
// big_error counts as an ordinary settle instead of a stuck robot (see
// test_wait_until_turn_swing_latched_side_recheck.cpp and test_wait_until_turn_to_point_final_target_gate.cpp):
//
//   - The stuck-detected path (the no-progress watch firing while still RUNNING) uses
//     turn_at_final_target, which requires mode == TURN.
//   - The already-latched path (turn_exit already SMALL_EXIT/BIG_EXIT on entry to the else branch)
//     uses turn_recheck_settle_ok, a plain numeric comparison against turnPID's own target with no
//     mode restriction at all -- deliberately, per that variable's own comment, because restricting
//     it the way turn_at_final_target is restricted "would report interfered=true on every ordinary
//     turn-to-point settle, chained or not".
//
// That reasoning applies just as much to the stuck-detected path, but turn_at_final_target was never
// given the same treatment. So a TURN_TO_POINT wait_until() call whose target is exactly the
// motion's own resolved aim (unchained, the normal case) still gets reported as interfered=true if
// it happens to settle inside big_error via the no-progress watch instead of a clean small/big exit
// latch -- something that returns interfered=false for a plain TURN settling the exact same way.
//
// This test is a minimal, mechanical variant of
// test_wait_until_turn_swing_latched_side_recheck.cpp's own passing control case ("hovering settled
// at the final target without a clean latch still returns interfered=false"): identical hover
// script, identical exit constants, identical wait_until() target choice (the motion's own resolved
// aim) -- the only difference is pid_turn_set() being given a point to face instead of a plain
// angle, which is enough to flip the result.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Hovers across small_error(3deg) every pass -- comfortably inside big_error(7deg) the whole time --
// by writing turnPID.error/derivative directly instead of through compute_error(), so PID.cpp's
// error_fresh gate never sees a real compute land and the small/big exit timers never advance, let
// alone latch. The only thing that can end this wait is the no-progress watch. Same script as
// test_wait_until_turn_swing_latched_side_recheck.cpp's hover_script.
void hover_script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  double e = (g_pass % 2 == 0) ? 2.7 : 3.3;
  c.turnPID.error = e;
  c.turnPID.derivative = (g_pass % 2 == 0) ? 0.3 : -0.3;
}
}  // namespace

TEST_CASE("pid_wait_until() TURN control: hovering settled at the final target without a clean latch returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(90, 100);
  double final_target = chassis.turnPID.target_get();

  g_chassis = &chassis;
  g_pass = 0;
  hover_script();
  test_stub::g_clock.on_delay = hover_script;
  test_stub::g_clock.delay_calls_until_stop = 400;

  bool returned = true;
  try {
    chassis.pid_wait_until(final_target * ez::degree);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " passes=", g_pass);
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until() TURN_TO_POINT: hovering settled at the motion's own resolved aim without a clean latch is wrongly reported as interfered") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);

  chassis.pid_turn_set({10.0, 0.0, 0.0}, ez::fwd, 100);
  REQUIRE(chassis.mode == TURN_TO_POINT);
  double final_target = chassis.turnPID.target_get();

  g_chassis = &chassis;
  g_pass = 0;
  hover_script();
  test_stub::g_clock.on_delay = hover_script;
  test_stub::g_clock.delay_calls_until_stop = 400;

  bool returned = true;
  try {
    chassis.pid_wait_until(final_target * ez::degree);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " passes=", g_pass);
  REQUIRE(returned);
  // A plain TURN hovering the exact same way, at its own final target, returns interfered=false
  // (the control case above, and test_wait_until_turn_swing_latched_side_recheck.cpp's own control
  // case). TURN_TO_POINT settling exactly the same way, at exactly its own resolved aim, should not
  // be treated any differently -- but turn_at_final_target (exit_conditions.cpp) unconditionally
  // requires mode == TURN, so this currently comes back interfered=true instead.
  CHECK_FALSE(chassis.interfered);
}
