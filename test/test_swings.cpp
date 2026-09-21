// A swing started at speed 0 must leave the still side of the drive at 0 instead of sending it a garbage
// value, and a swing at a normal speed must still scale the still side by how much of the maximum speed the
// swing is using.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

void configure(Drive& chassis) {
  chassis.pid_swing_constants_set(1.0, 0.0, 0.0, 0.0);  // output equals error, so a 50 degree swing asks for 50
  chassis.slew_swing_set(false);
  chassis.imu->fake_rotation = 0.0;
}

// Voltage the drive was last told to run, converted back to the -127 to 127 scale. Loose comparisons are
// needed because move_voltage() takes whole millivolts.
double left_out(Drive& chassis) { return chassis.left_motors[0].fake().voltage / (12000.0 / 127.0); }
double right_out(Drive& chassis) { return chassis.right_motors[0].fake().voltage / (12000.0 / 127.0); }
}  // namespace

TEST_CASE("swings a left swing at speed 0 leaves the still side at 0") {
  Drive chassis = make_chassis();
  configure(chassis);
  chassis.pid_swing_set(LEFT_SWING, 50.0, 0, 50);  // speed 0, opposite speed 50

  DriveTestAccess::swing_pid_task(chassis);

  CHECK(left_out(chassis) == doctest::Approx(0.0).epsilon(0.01));
  CHECK(right_out(chassis) == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("swings a right swing at speed 0 leaves the still side at 0") {
  Drive chassis = make_chassis();
  configure(chassis);
  chassis.pid_swing_set(RIGHT_SWING, 50.0, 0, 50);

  DriveTestAccess::swing_pid_task(chassis);

  CHECK(left_out(chassis) == doctest::Approx(0.0).epsilon(0.01));
  CHECK(right_out(chassis) == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("swings a swing at speed 0 with slew on leaves the still side at 0") {
  Drive chassis = make_chassis();
  configure(chassis);
  chassis.pid_swing_set(LEFT_SWING, 50.0, 0, 50, true);  // slew on

  DriveTestAccess::swing_pid_task(chassis);

  CHECK(left_out(chassis) == doctest::Approx(0.0).epsilon(0.01));
  CHECK(right_out(chassis) == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("swings the still side is scaled by the fraction of max speed the swing is using") {
  Drive chassis = make_chassis();
  configure(chassis);
  chassis.pid_swing_set(LEFT_SWING, 50.0, 100, 50);  // error 50 with kp 1 is 50 of a max 100, so scale 0.5

  DriveTestAccess::swing_pid_task(chassis);

  CHECK(left_out(chassis) == doctest::Approx(50.0).epsilon(0.01));
  CHECK(right_out(chassis) == doctest::Approx(25.0).epsilon(0.01));  // 50 * 0.5
}

TEST_CASE("swings a right swing scales the still side and reverses its sign") {
  Drive chassis = make_chassis();
  configure(chassis);
  chassis.pid_swing_set(RIGHT_SWING, 50.0, 100, 50);

  DriveTestAccess::swing_pid_task(chassis);

  CHECK(left_out(chassis) == doctest::Approx(-25.0).epsilon(0.01));  // -(50 * 0.5)
  CHECK(right_out(chassis) == doctest::Approx(-50.0).epsilon(0.01));
}
