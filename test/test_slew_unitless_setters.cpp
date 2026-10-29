// The slew constant setters each have a unitless version that means the same as its unit version, and swing
// slew only ever measures heading, so its constants are degrees.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void check_constants(slew& s, double distance, int min_speed) {
  CHECK(s.constants_get().distance_to_travel == doctest::Approx(distance));
  CHECK(s.constants_get().min_speed == min_speed);
}
}  // namespace

TEST_CASE("unitless drive slew constants are inches") {
  Drive chassis = make_chassis();

  chassis.slew_drive_constants_set(3, 70);
  check_constants(chassis.slew_forward, 3.0, 70);
  check_constants(chassis.slew_backward, 3.0, 70);

  chassis.slew_drive_constants_forward_set(5, 60);
  check_constants(chassis.slew_forward, 5.0, 60);
  check_constants(chassis.slew_backward, 3.0, 70);

  chassis.slew_drive_constants_backward_set(7, 50);
  check_constants(chassis.slew_forward, 5.0, 60);
  check_constants(chassis.slew_backward, 7.0, 50);

  // Same result as the unit version
  chassis.slew_drive_constants_set(2_in, 90);
  check_constants(chassis.slew_forward, 2.0, 90);
  check_constants(chassis.slew_backward, 2.0, 90);
}

TEST_CASE("unitless turn slew constants are degrees") {
  Drive chassis = make_chassis();

  chassis.slew_turn_constants_set(4, 65);
  check_constants(chassis.slew_turn, 4.0, 65);

  chassis.slew_turn_constants_set(10_deg, 75);
  check_constants(chassis.slew_turn, 10.0, 75);
}

TEST_CASE("unitless swing slew constants are degrees") {
  Drive chassis = make_chassis();

  chassis.slew_swing_constants_set(15, 80);
  check_constants(chassis.slew_swing_forward, 15.0, 80);
  check_constants(chassis.slew_swing_backward, 15.0, 80);

  chassis.slew_swing_constants_forward_set(20, 60);
  check_constants(chassis.slew_swing_forward, 20.0, 60);
  check_constants(chassis.slew_swing_backward, 15.0, 80);

  chassis.slew_swing_constants_backward_set(25, 50);
  check_constants(chassis.slew_swing_forward, 20.0, 60);
  check_constants(chassis.slew_swing_backward, 25.0, 50);

  // Same result as the unit version
  chassis.slew_swing_constants_set(30_deg, 90);
  check_constants(chassis.slew_swing_forward, 30.0, 90);
  check_constants(chassis.slew_swing_backward, 30.0, 90);
}

TEST_CASE("a new Drive swings with slew over 15 degrees starting at 80") {
  Drive chassis = make_chassis();

  check_constants(chassis.slew_swing_forward, 15.0, 80);
  check_constants(chassis.slew_swing_backward, 15.0, 80);
}

TEST_CASE("swing slew ramps by heading, not by how far a side of the drive has moved") {
  Drive chassis = make_chassis();
  chassis.pid_swing_constants_set(5.0, 0.0, 0.0, 0.0);
  chassis.slew_swing_constants_set(15, 80);
  chassis.imu->fake_rotation = 0.0;

  chassis.pid_swing_set(LEFT_SWING, 90.0, 127, 0, true);  // slew on

  // Nothing has turned yet, so slew is at its minimum speed
  DriveTestAccess::swing_pid_task(chassis);
  CHECK(chassis.slew_swing.output() == doctest::Approx(80.0));

  // The heading is halfway through the 15 degree ramp while the drive's encoders haven't moved
  chassis.imu->fake_rotation = 7.5;
  DriveTestAccess::swing_pid_task(chassis);
  CHECK(chassis.slew_swing.output() == doctest::Approx(103.5));

  // Past 15 degrees slew is done and gives back the full speed
  chassis.imu->fake_rotation = 16.0;
  DriveTestAccess::swing_pid_task(chassis);
  CHECK(chassis.slew_swing.output() == doctest::Approx(127.0));
}
