// check_imu_task() judges a redundant IMU "stuck" by watching for translation (the drive
// physically moving) while that IMU's heading reading stays frozen. But an ordinary straight
// leg IS translation with genuinely no rotation -- a healthy IMU correctly reports a flat
// heading the whole time, indistinguishable by that check from a real frozen sensor. The fix
// must gate suspicion on the drive actually ROTATING (encoders disagreeing), using the same
// drive_sensor_left()/right() the function already reads, not just moving.
//
// The bug is mode-agnostic: check_imu_task() runs unconditionally from ez_auto_task() every
// pass, with no look at pros::competition::is_autonomous(). Two scenarios are still tested
// separately (an active auton drive motion vs. idle/DISABLE mode, standing in for a driver
// simply steering with the joystick) because the finding explicitly calls out that this fires
// outside auton too, and a reader should not have to take that on faith from the mechanism
// alone.
#include <algorithm>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// Drive has no usable copy/move constructor (it owns a mutex), so it can't be built by a
// factory that returns it by value -- construct it directly in each TEST_CASE and use this
// only to reset the fake-hardware registries beforehand and flip on calibration afterward.
void reset_before_construction() { test_stub::reset_all(); }
void mark_calibrated(Drive& chassis) { DriveTestAccess::imu_calibration_complete(chassis) = true; }

// Drives both sides' fake encoders forward together by the same amount -- a straight leg.
// Neither IMU's fake_rotation is touched: on a real straight leg a healthy IMU's heading
// genuinely does not change, so leaving both flat is the honest simulation, not a shortcut.
void drive_straight_one_pass(Drive& c) {
  c.left_motors.front().fake().position += 50;   // ~1.02in at this chassis's tick/inch
  c.right_motors.front().fake().position += 50;
  DriveTestAccess::check_imu_task(c);
}

// Drives the fake encoders apart -- a genuine in-place turn -- and advances the "healthy"
// IMU's rotation to match, the way a real IMU would while the robot actually turns.
void turn_one_pass(Drive& c, pros::Imu* healthy) {
  c.left_motors.front().fake().position += 50;
  c.right_motors.front().fake().position -= 50;
  healthy->fake_rotation += 2.0;
  DriveTestAccess::check_imu_task(c);
}
}  // namespace

TEST_CASE("check_imu_task(): a long straight-line drive in AUTON mode does not eject a healthy redundant IMU") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(500, 100);  // an active auton drive motion, matching how this is really invoked

  REQUIRE(chassis.good_imus.size() == 2);
  for (int pass = 0; pass < 500; pass++) drive_straight_one_pass(chassis);

  CHECK(chassis.good_imus.size() == 2);
}

TEST_CASE("check_imu_task(): the same straight-line scenario in DRIVER CONTROL (opcontrol) mode does not eject a healthy redundant IMU") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  // No pid_*_set call at all -- mode stays DISABLE, standing in for a driver steering the robot
  // straight by joystick with no auton motion active. check_imu_task() runs regardless.
  REQUIRE(chassis.mode == ez::DISABLE);
  REQUIRE(chassis.good_imus.size() == 2);

  for (int pass = 0; pass < 500; pass++) drive_straight_one_pass(chassis);

  CHECK(chassis.good_imus.size() == 2);
}

TEST_CASE("check_imu_task(): a genuinely frozen IMU during an actual turn is still ejected") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);

  pros::Imu* frozen = chassis.good_imus[0];   // port 5, never updated below -- the broken sensor
  pros::Imu* healthy = chassis.good_imus[1];  // port 6, updated every pass -- turning for real

  REQUIRE(chassis.good_imus.size() == 2);
  for (int pass = 0; pass < 500 && chassis.good_imus.size() == 2; pass++) turn_one_pass(chassis, healthy);

  CHECK(chassis.good_imus.size() == 1);
  CHECK(std::find(chassis.good_imus.begin(), chassis.good_imus.end(), frozen) == chassis.good_imus.end());
  CHECK(chassis.good_imus.front() == healthy);
}
