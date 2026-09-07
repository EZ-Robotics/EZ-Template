// The turn-min gate does not engage for an 88 -> 90 turn and does engage
// inside start_i for a 0 -> 90 turn.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("turns turn-min gate does not engage for an 88 -> 90 turn") {
  Drive chassis = make_chassis();
  chassis.pid_turn_constants_set(20.0, 0.5, 0.0, 5.0);  // start_i = 5
  chassis.pid_turn_min_set(20);
  chassis.slew_turn_set(false);

  chassis.imu->fake_rotation = 88.0;
  chassis.pid_turn_set(90.0, 127);  // chain distance = |90-88| = 2, not > start_i (5)

  DriveTestAccess::turn_pid_task(chassis);

  double gyro_out = chassis.left_motors[0].fake().voltage / (12000.0 / 127.0);
  // Unclamped: kp(20) * error(2) = 40. If the gate had wrongly engaged, this
  // would instead be clamped to +-20. Loose epsilon: move_voltage()'s
  // int32_t mV parameter quantizes the round-tripped value by ~0.006.
  CHECK(gyro_out == doctest::Approx(40).epsilon(0.01));
}

TEST_CASE("turns turn-min gate engages inside start_i for a 0 -> 90 turn") {
  Drive chassis = make_chassis();
  chassis.pid_turn_constants_set(20.0, 0.5, 0.0, 5.0);  // start_i = 5
  chassis.pid_turn_min_set(20);
  chassis.slew_turn_set(false);

  chassis.imu->fake_rotation = 0.0;
  chassis.pid_turn_set(90.0, 127);  // chain distance = |90-0| = 90, > start_i (5)

  chassis.imu->fake_rotation = 87.0;  // error = 3, inside start_i (5)
  DriveTestAccess::turn_pid_task(chassis);

  double gyro_out = chassis.left_motors[0].fake().voltage / (12000.0 / 127.0);
  // Unclamped this would be kp(20) * error(3) = 60; the gate clamps it to the
  // 20 min. Loose epsilon: see the note in the previous test case.
  CHECK(gyro_out == doctest::Approx(20).epsilon(0.01));
}
