// Coverage gap found by mutation testing set_odom_pid.cpp's pid_odom_pp_set(): starting a new
// pure-pursuit/point-to-point path resets xyPID's exit-condition timers before the new path
// begins. Without it, a path that leaves xyPID's small-exit timer partway accumulated (e.g.
// superseded by another pid_odom_pp_set() call before it ever actually exits) would hand that
// leftover count straight to the next path, which could then fire an exit almost immediately
// instead of getting its own full small_exit_time.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;

void hold_errors() {
  // A real compute_error() call every pass, not a direct `.error =` write -- the small exit timer
  // this test is pinning only credits `error` when a real compute has landed since it last checked
  // (see PID.cpp), so the accumulation this test relies on needs a real compute behind it, the same
  // as a real ez_auto_task pass would provide. `current` held fixed is fine here (nothing in this
  // test reads derivative).
  g_chassis->xyPID.compute_error(0.5, g_chassis->xyPID.cur);              // inside xy's small_error(1.0) throughout
  g_chassis->current_a_odomPID.compute_error(0.0, g_chassis->current_a_odomPID.cur);  // always inside angle's (huge) small_error
}

void start_single_point_path(Drive& chassis, double y) {
  chassis.pid_odom_pp_set({{{0.0, y, ANGLE_NOT_SET}, fwd, 100}});
  // Skip straight to the "last point" logic -- what happens before that (pp_index advancing
  // through intermediate points) is a different code path, not what this test is about.
  DriveTestAccess::pp_index(chassis) = (int)DriveTestAccess::pp_movements(chassis).size() - 1;
}
}  // namespace

TEST_CASE("pid_odom_pp_set() resets xyPID's timer so a new path gets a fresh exit window") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  // xy: small exit only (100 ms/1 in). angle: small_exit_time so short (10 ms/1 pass) with a huge
  // small_error that it resolves to SMALL_EXIT on the very first pass and stays out of the way --
  // isolating this test to xyPID's own timer.
  chassis.pid_odom_drive_exit_condition_set(100, 1.0, 0, 0.0, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(10, 1000.0, 0, 0.0, 0, 0);

  start_single_point_path(chassis, 24.0);
  g_chassis = &chassis;
  hold_errors();
  test_stub::g_clock.on_delay = hold_errors;

  // Accumulate xyPID's small-exit timer to 80ms (8 passes) without ever letting it fire -- safely
  // under the 100ms threshold. One exit_condition() call happens per delay() call here (the
  // budget below counts delay() calls, one more than the exit_condition() calls it produces,
  // matching the leading pros::delay() at the top of pid_wait()).
  test_stub::g_clock.delay_calls_until_stop = 9;
  bool first_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    first_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  REQUIRE_FALSE(first_returned);  // must not have actually exited yet

  // A second, different path -- simulates a superseded path immediately followed by a fresh one.
  start_single_point_path(chassis, 48.0);

  test_stub::g_clock.delay_calls_until_stop = 5;  // budget far short of a fresh 100ms/10-pass window
  bool second_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    second_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("second_returned=", second_returned);

  // Correct code: xyPID's small-exit timer was reset to 0 by the second pid_odom_pp_set(), so the
  // new path needs its own fresh ~10 passes and must NOT have returned within 5.
  CHECK_FALSE(second_returned);
}
