// turns [H5, H6, M4]: with odom_theta_flip(true), pid_turn_set(90) then
// pid_turn_relative_set(45) leaves headingPID.target_get() at -135;
// pid_turn_set({24,24}, fwd) with x and theta flipped leaves it at -45;
// pid_swing_set(LEFT_SWING, 90, 110, 45) mirrored selects the forward slew
// constants; the turn-min gate does not engage for an 88 -> 90 turn and does
// engage inside start_i for a 0 -> 90 turn.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("turns [H5] pid_turn_set then pid_turn_relative_set with theta flipped") {
  Drive chassis = make_chassis();
  chassis.odom_theta_flip(true);

  chassis.pid_turn_set(90.0, 100);
  chassis.pid_turn_relative_set(45.0, 100);

  CHECK(chassis.headingPID.target_get() == doctest::Approx(-135));
}

TEST_CASE("turns [H6] pid_turn_set(pose) with x and theta flipped") {
  Drive chassis = make_chassis();
  chassis.odom_x_flip(true);
  chassis.odom_theta_flip(true);

  chassis.pid_turn_set(pose{24, 24}, fwd, 100);

  // Derived by running find_point_to_face()'s trig (look-ahead point
  // selection, then absolute_angle_to_point()) rather than hand-computed:
  // that path has no closed form worth re-deriving by inspection alone.
  CHECK(chassis.headingPID.target_get() == doctest::Approx(-45));
}

TEST_CASE("turns [M4] mirrored LEFT_SWING selects the forward slew constants") {
  Drive chassis = make_chassis();
  chassis.odom_theta_flip(true);  // mirrors LEFT_SWING <-> RIGHT_SWING

  chassis.slew_swing_constants_forward_set(6_in, 70);
  chassis.slew_swing_constants_backward_set(3_in, 40);

  chassis.pid_swing_set(LEFT_SWING, 90.0, 110, 45);

  // swing_set_internal() picks forward/backward slew constants by the sign
  // of (target - chain_sensor_start) in the *internal* (post-mirror) frame;
  // with theta flipped, LEFT_SWING becomes RIGHT_SWING and the target flips
  // sign too, and the two flips land on "forward" here. distance_to_travel
  // is in inches (6) since these were set via the QLength overload.
  DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, 90.0, 0.0);  // no-op sanity call, doesn't throw
  CHECK(chassis.slew_swing.constants_get().distance_to_travel == doctest::Approx(6));
  CHECK(chassis.slew_swing.constants_get().min_speed == doctest::Approx(70));
}

TEST_CASE("turns [M4] turn-min gate does not engage for an 88 -> 90 turn") {
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

TEST_CASE("turns [M4] turn-min gate engages inside start_i for a 0 -> 90 turn") {
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
