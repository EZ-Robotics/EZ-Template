// Competition state (follow-up to GitHub issue #365): when the drive task stops commanding the drive because
// the field disabled the robot, or autonomous ended, it has to stop the motors too. Setting the mode to
// DISABLE alone leaves the last autonomous voltage latched on every drive motor until something else writes
// to them, which for a driver control that is slow to reach its first drive call is a robot driving on its own.
//
// Most cases run a real motion so the drive PID is what puts a voltage on the motors, move the fake field
// status, and check the motors read 0 from then on, however many passes of the drive task go by. Entering
// autonomous is the control: it must still leave a motion running (test_tracking.cpp), and a stop must happen
// once, when the mode changes, not on every pass.
#include <algorithm>
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// One pass of ez_auto_task(); see test_tracking.cpp.
void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}

void run_passes(Drive& chassis, int passes) {
  for (int i = 0; i < passes; i++) run_one_auto_task_pass(chassis);
}

// The largest voltage on any drive motor, either direction.
double max_voltage(Drive& chassis) {
  double biggest = 0.0;
  for (auto& m : chassis.left_motors) biggest = std::max(biggest, std::fabs(m.fake().voltage));
  for (auto& m : chassis.right_motors) biggest = std::max(biggest, std::fabs(m.fake().voltage));
  return biggest;
}

// Runs `passes` passes and reports whether every motor read 0 after every one of them.
bool stays_stopped(Drive& chassis, int passes) {
  bool stopped = true;
  for (int i = 0; i < passes; i++) {
    run_one_auto_task_pass(chassis);
    if (max_voltage(chassis) != 0.0) stopped = false;
  }
  return stopped;
}

void set_field(bool disabled, bool autonomous) {
  test_stub::g_competition.disabled = disabled;
  test_stub::g_competition.autonomous = autonomous;
}

void latch_voltage(Drive& chassis, double voltage) {
  for (auto& m : chassis.left_motors) m.fake().voltage = voltage;
  for (auto& m : chassis.right_motors) m.fake().voltage = voltage;
}

// Starts a long drive in autonomous and lets the PID get the wheels going.
void start_autonomous_drive(Drive& chassis) {
  set_field(false, true);
  run_one_auto_task_pass(chassis);  // syncs last_was_autonomous to true
  chassis.pid_drive_set(72.0, 110);
  run_passes(chassis, 10);
  REQUIRE(chassis.drive_mode_get() == DRIVE);
  REQUIRE(max_voltage(chassis) > 0.0);  // the PID, not the test, is driving the motors
}
}  // namespace

TEST_CASE("competition state: autonomous -> driver with no disabled gap stops the drive motors and keeps them stopped") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  start_autonomous_drive(chassis);

  // Straight to driver control, no disabled period in between, and a driver control that takes 3 s
  // (300 passes) to reach its first drive call.
  set_field(false, false);

  CHECK(stays_stopped(chassis, 300));
  CHECK(chassis.drive_mode_get() == DISABLE);
}

TEST_CASE("competition state: autonomous -> disabled stops the drive motors and keeps them stopped") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  start_autonomous_drive(chassis);

  set_field(true, false);

  CHECK(stays_stopped(chassis, 300));
  CHECK(chassis.drive_mode_get() == DISABLE);
}

TEST_CASE("competition state: a motion running in driver control is stopped when the robot is disabled") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  set_field(false, false);
  run_one_auto_task_pass(chassis);
  chassis.pid_drive_set(72.0, 110);
  run_passes(chassis, 10);
  REQUIRE(chassis.drive_mode_get() == DRIVE);
  REQUIRE(max_voltage(chassis) > 0.0);

  set_field(true, false);

  CHECK(stays_stopped(chassis, 300));
  CHECK(chassis.drive_mode_get() == DISABLE);
}

TEST_CASE("competition state: entering autonomous does not stop the first motion") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  set_field(true, false);
  run_one_auto_task_pass(chassis);

  // Field control moves to autonomous, and the autonomous task's first setter runs before the drive task's
  // next pass sees the status change.
  set_field(false, true);
  chassis.pid_drive_set(72.0, 110);
  run_passes(chassis, 10);

  CHECK(chassis.drive_mode_get() == DRIVE);
  CHECK(max_voltage(chassis) > 0.0);
}

TEST_CASE("competition state: a whole match leaves driver control in charge of the motors") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  set_field(true, false);  // disabled before the match
  run_one_auto_task_pass(chassis);
  CHECK(max_voltage(chassis) == doctest::Approx(0.0));

  start_autonomous_drive(chassis);

  set_field(true, false);  // autonomous ends, robot disabled
  CHECK(stays_stopped(chassis, 50));

  set_field(false, false);  // driver control starts
  CHECK(stays_stopped(chassis, 50));

  // Driver control reaches its first drive call. The drive task must leave what it writes alone.
  chassis.drive_set(64, 64);
  double driven = max_voltage(chassis);
  REQUIRE(driven > 0.0);
  run_passes(chassis, 100);
  CHECK(max_voltage(chassis) == doctest::Approx(driven));
}

TEST_CASE("competition state: motors written by something else while already disabled are left alone") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  set_field(true, false);
  run_one_auto_task_pass(chassis);  // mode is already DISABLE, or is set to it here

  // The stop happens once, when the mode changes to DISABLE, not on every pass: a user's own driver control
  // code writing the motors while the drive task idles must not be overwritten with 0 every 10 ms.
  latch_voltage(chassis, 6000.0);
  run_one_auto_task_pass(chassis);

  for (auto& m : chassis.left_motors) CHECK(m.fake().voltage == doctest::Approx(6000.0));
  for (auto& m : chassis.right_motors) CHECK(m.fake().voltage == doctest::Approx(6000.0));
}
