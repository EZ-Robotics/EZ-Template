// Competition state (follow-up to GitHub issue #365): when the drive task stops commanding the drive because
// the field disabled the robot, or autonomous ended, it has to stop the motors too. Setting the mode to
// DISABLE alone leaves the last autonomous voltage latched on every drive motor until something else writes
// to them, which for a driver control that is slow to reach its first drive call is a robot driving on its own.
//
// Each case puts a known non-zero voltage on every drive motor, moves the fake field status, runs one pass of
// the drive task, and checks the motors read 0. Entering autonomous is the control: it must still leave a
// motion running (test_tracking.cpp).
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

void latch_voltage(Drive& chassis, double voltage) {
  for (auto& m : chassis.left_motors) m.fake().voltage = voltage;
  for (auto& m : chassis.right_motors) m.fake().voltage = voltage;
}

void check_motors_stopped(Drive& chassis) {
  for (auto& m : chassis.left_motors) CHECK(m.fake().voltage == doctest::Approx(0.0));
  for (auto& m : chassis.right_motors) CHECK(m.fake().voltage == doctest::Approx(0.0));
}
}  // namespace

TEST_CASE("competition state: autonomous -> driver with no disabled gap stops the drive motors") {
  Drive chassis = make_chassis();

  test_stub::g_competition.disabled = false;
  test_stub::g_competition.autonomous = true;
  run_one_auto_task_pass(chassis);  // syncs last_was_autonomous to true

  chassis.pid_drive_set(12.0, 100);
  latch_voltage(chassis, 6000.0);
  REQUIRE(chassis.drive_mode_get() == DRIVE);

  // Straight to driver control, no disabled period in between.
  test_stub::g_competition.autonomous = false;
  run_one_auto_task_pass(chassis);

  CHECK(chassis.drive_mode_get() == DISABLE);
  check_motors_stopped(chassis);
}

TEST_CASE("competition state: autonomous -> disabled stops the drive motors") {
  Drive chassis = make_chassis();

  test_stub::g_competition.disabled = false;
  test_stub::g_competition.autonomous = true;
  run_one_auto_task_pass(chassis);

  chassis.pid_drive_set(12.0, 100);
  latch_voltage(chassis, 6000.0);
  REQUIRE(chassis.drive_mode_get() == DRIVE);

  test_stub::g_competition.disabled = true;
  test_stub::g_competition.autonomous = false;
  run_one_auto_task_pass(chassis);

  CHECK(chassis.drive_mode_get() == DISABLE);
  check_motors_stopped(chassis);
}

TEST_CASE("competition state: a motion started while disabled is stopped, motors included") {
  Drive chassis = make_chassis();

  test_stub::g_competition.disabled = true;
  test_stub::g_competition.autonomous = false;
  run_one_auto_task_pass(chassis);

  chassis.pid_drive_set(12.0, 100);
  latch_voltage(chassis, 6000.0);
  REQUIRE(chassis.drive_mode_get() == DRIVE);

  run_one_auto_task_pass(chassis);

  CHECK(chassis.drive_mode_get() == DISABLE);
  check_motors_stopped(chassis);
}

TEST_CASE("competition state: motors written by something else while already disabled are left alone") {
  Drive chassis = make_chassis();

  test_stub::g_competition.disabled = true;
  test_stub::g_competition.autonomous = false;
  run_one_auto_task_pass(chassis);  // mode is already DISABLE, or is set to it here

  // The stop happens once, when the mode changes to DISABLE, not on every pass: a user's own driver control
  // code writing the motors while the drive task idles must not be overwritten with 0 every 10 ms.
  latch_voltage(chassis, 6000.0);
  run_one_auto_task_pass(chassis);

  for (auto& m : chassis.left_motors) CHECK(m.fake().voltage == doctest::Approx(6000.0));
  for (auto& m : chassis.right_motors) CHECK(m.fake().voltage == doctest::Approx(6000.0));
}
