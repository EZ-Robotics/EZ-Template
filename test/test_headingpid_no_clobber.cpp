// pid_wait()'s odom branch writes headingPID right after its loop exits: odom_target_start's final theta, resolved to the equivalent angle
// nearest the IMU.  This lands a concurrent retarget in the trailing pros::delay() of the pass both xyPID/current_a_odomPID's exits were
// already latched on.  The loop only checks for a new motion at the top of a pass, and it doesn't run another pass once both exits have
// latched, so the wait returns on the original motion, which did finish, without looking at the new one.  The closing write then reads
// odom_target_start, which already belongs to the new motion, so headingPID gets the new motion's final heading while it runs: the value
// a wait on the new motion leaves when it ends.  That is 3.2.2's behavior.  The replaced motion's final heading must not land there.
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
bool g_retargeted = false;

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  if (g_pass == 2) {
    // Lands during the exact trailing delay of the pass both exits latched on (small_exit_time=0 below makes that the very first pass
    // this wait's loop runs), before the loop condition finds both exits latched and falls out to the headingPID write.
    g_chassis->pid_odom_ptp_set({{0.0, 90.0, 45.0}, fwd, 100});
    g_retargeted = true;
  }
}
}  // namespace

TEST_CASE("pid_wait() odom: a retarget landing in the exit-then-delay gap returns on the finished motion and leaves the new motion's heading") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: SMALL_EXIT fires on the first in-tolerance pass
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  // A real compute_error() call puts both errors at 0, inside small_error; nothing later touches xyPID/current_a_odomPID
  chassis.xyPID.compute_error(0.0, 0.0);
  chassis.current_a_odomPID.compute_error(0.0, 0.0);

  g_chassis = &chassis;
  g_pass = 0;
  g_retargeted = false;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered, " heading=", chassis.headingPID.target_get());

  REQUIRE(returned);
  REQUIRE(g_retargeted);
  // Leading delay (pass 1), the poll that latches both small exits, its trailing delay (pass 2, where the retarget lands), then out.
  CHECK(g_pass == 2);
  CHECK_FALSE(chassis.interfered);  // the original motion finished on its small exits
  CHECK(chassis.mode == POINT_TO_POINT);
  // The original motion's final theta is 0 and the new one's is 45: headingPID holds the new motion's resolved final heading
  double new_final = DriveTestAccess::new_turn_target_compute(chassis, 45.0, chassis.drive_angle_get(), shortest);
  double old_final = DriveTestAccess::new_turn_target_compute(chassis, 0.0, chassis.drive_angle_get(), shortest);
  REQUIRE(new_final != old_final);
  CHECK(chassis.headingPID.target_get() == new_final);
}
