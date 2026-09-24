// Coverage gap found by mutation testing exit_conditions.cpp's pid_wait() PURE_PURSUIT branch:
// the loop that runs before the last point caches current_a_odomPID's exit
// result (a_exit) per pp_index, and resets it to RUNNING whenever pp_index changes -- except for
// the single-step transition directly onto the LAST point, which never re-enters that loop's body
// (the while condition `pp_index != movements.size()-1` goes false before the body runs for the
// final index). The line right after the loop:
//
//   if (pp_index != a_exit_index) a_exit = RUNNING;  // the final move onto the last point never
//                                                     // re-enters the loop above
//
// exists specifically to catch that transition. Without it, a_exit can carry a SMALL_EXIT latched
// at an earlier, unrelated point straight onto the final point, where the real heading requirement
// can be completely different -- pid_wait() would then finish once xy alone converges, without
// ever actually waiting for the robot to face the final heading.
//
// This is a realistic shape, not a contrived one: a mostly-straight path where the second-to-last
// leg is straight (heading error settles to 0 and latches there) and the final leg needs an actual
// turn is exactly the geometry of an L-shaped or hooked path ending in a turn.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// xy: small_error 1in/90ms, everything else off (no velocity/mA) so StuckWatch's window_ is 0 and
// it can never fire -- isolates this test to the xy_exit/a_exit RUNNING bookkeeping alone.
// angle: small_error 3deg/90ms, everything else off, same reason.
void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 0, 0.0, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

// ANGLE_NOT_SET thetas (a plain pure-pursuit path, not boomerang): pid_odom_pp_set() only injects
// an extra carrot/parent point after an entry with an EXPLICIT theta, so this stays a 3-point path
// plus the one pose it always prepends -- but the exact count is never hardcoded below; it's read
// back from pp_movements() after the fact, the same way test_pp_wait_stuck.cpp's last_index() does.
void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110}});
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 0, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

int last_index(Drive& chassis) { return (int)DriveTestAccess::pp_movements(chassis).size() - 1; }

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_last = 0;

// Pass timeline (g_last computed from the real path, never hardcoded):
//  1-20:        pp_index 0, angle aligned (error 0), xy far (error 5in, not converging).
//  21-45:       pp_index (last-1) -- angle stays aligned (error 0) long enough (>9 passes = 90ms)
//               to latch SMALL_EXIT at THIS index. xy still far.
//  46+:         pp_index jumps straight to `last` (single-step transition, skips the loop body)
//               with a REAL 60 degree heading mismatch, and xy starts converging immediately
//               (small-exits by ~pass 55). Angle is held at 60 degrees (not converging) until
//               pass 150, where it is finally driven to 0 and allowed to small-exit for real.
void script() {
  ++g_pass;
  Drive& c = *g_chassis;
  int& idx = DriveTestAccess::pp_index(c);
  if (g_pass <= 20) {
    idx = 0;
    c.current_a_odomPID.error = 0.0;
    c.xyPID.error = 5.0;
  } else if (g_pass <= 45) {
    idx = g_last - 1;
    c.current_a_odomPID.error = 0.0;
    c.xyPID.error = 5.0;
  } else {
    idx = g_last;  // jumps directly from (last-1) to last -- never visits the loop body at `last`
    c.xyPID.error = 0.0;                              // converges quickly after the jump
    c.current_a_odomPID.error = g_pass < 150 ? 60.0 : 0.0;  // the real final-heading mismatch
  }
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  double final_angle_error;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  g_last = last_index(chassis);
  REQUIRE(g_last >= 2);  // needs at least 3 points for a distinct "second-to-last" index
  script();  // pass 0 / initial state before the wait's first delay
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.final_angle_error = chassis.current_a_odomPID.error;
  return o;
}
}  // namespace

TEST_CASE("pid_wait does not finish on the final point using an angle exit latched at an earlier point") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);

  Outcome o = run_wait(chassis, 400);

  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  // xy alone converges by ~pass 55 (46 + 9 passes for the small exit timer). If pid_wait() finished
  // there, it would be relying on a_exit's SMALL_EXIT latched back at pp_index (last-1) -- stale,
  // since the real heading error at the final point (60 degrees) never actually converged yet.
  // The real, current angle error must be small when the wait returns.
  CHECK(std::fabs(o.final_angle_error) < 3.0);
  // The wait can only correctly finish once angle is driven back to 0 starting at pass 150, plus its
  // own 90ms/9-pass small exit timer.
  CHECK(o.passes >= 150 + 9);
}
