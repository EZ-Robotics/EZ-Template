// ptp_task()'s turn bias (pid_tasks.cpp) zeroes xy_out entirely once heading error passes the zero
// crossing of 1 - ((1 - cos(a_error)) / odom_turn_bias_amount) -- about 84.26 degrees at the default
// 0.9 bias. xy_delta_fake then reads ~0 because the robot genuinely isn't translating, not because
// it's stalled, and xyPID's velocity exit can't tell those apart from the reading alone.
//
// xy_translation_bias_gated (set every ptp_task() pass) is what Drive::xy_exit_condition() reads to
// ignore xyPID's velocity exit while that's happening: ptp_task marks bias-gated once heading error
// passes the zero crossing and not on an ordinary heading error.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void configure_chassis(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 50, 0);  // short velocity_exit_time to keep the tests fast
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.odom_look_ahead_set(7.0);
}

void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {}
  test_stub::g_clock.delay_calls_until_stop = -1;
}
}  // namespace

TEST_CASE("ptp_task marks xy translation bias-gated once heading error passes the turn-bias zero crossing") {
  Drive chassis = make_chassis();
  configure_chassis(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();

  // Target directly behind the robot: initial heading error is ~180 degrees, well past the
  // ~84.26 degree zero crossing (arccos(0.1)) at the default 0.9 turn bias. Nothing in this host
  // stub turns the fake imu, so the error stays there for the whole test.
  odom movement{{0.0, -24.0}, fwd, 60};
  chassis.pid_odom_ptp_set(movement);
  run_one_auto_task_pass(chassis);

  CHECK(DriveTestAccess::xy_translation_bias_gated(chassis));
}

TEST_CASE("ptp_task does not mark xy translation bias-gated on an ordinary heading error") {
  Drive chassis = make_chassis();
  configure_chassis(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();

  // Target straight ahead: no meaningful heading error, scale stays at 1.0.
  odom movement{{0.0, 24.0}, fwd, 60};
  chassis.pid_odom_ptp_set(movement);
  run_one_auto_task_pass(chassis);

  CHECK_FALSE(DriveTestAccess::xy_translation_bias_gated(chassis));
}
