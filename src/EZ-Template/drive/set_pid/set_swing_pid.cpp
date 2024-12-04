/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "okapi/api/units/QAngle.hpp"

/////
// Sets swing constants
/////
// Max speed when i is enabled for large turns
void Drive::pid_swing_min_set(int min) { swing_min = abs(min); }
int Drive::pid_swing_min_get() { return swing_min; }

// PID Constants
void Drive::pid_swing_constants_set(double p, double i, double d, double p_start_i) {
  pid_swing_constants_forward_set(0.0, 0.0, 0.0, 0.0);
  pid_swing_constants_backward_set(0.0, 0.0, 0.0, 0.0);
  fwd_rev_swingPID.constants_set(p, i, d, p_start_i);
}
void Drive::pid_swing_constants_forward_set(double p, double i, double d, double p_start_i) {
  forward_swingPID.constants_set(p, i, d, p_start_i);
}
void Drive::pid_swing_constants_backward_set(double p, double i, double d, double p_start_i) {
  backward_swingPID.constants_set(p, i, d, p_start_i);
}
PID::Constants Drive::pid_swing_constants_forward_get() { return forward_swingPID.constants_get(); }
PID::Constants Drive::pid_swing_constants_backward_get() { return backward_swingPID.constants_get(); }
PID::Constants Drive::pid_swing_constants_get() {
  auto fwd_const = pid_swing_constants_forward_get();
  auto rev_const = pid_swing_constants_backward_get();
  if (!(fwd_const.kp == rev_const.kp && fwd_const.ki == rev_const.ki && fwd_const.kd == rev_const.kd && fwd_const.start_i == rev_const.start_i)) {
    printf("\nForward and Reverse constants are not the same!");
    return {-1, -1, -1, -1};
  }
  return fwd_const;
}

// Slew Constants
void Drive::slew_swing_constants_backward_set(okapi::QLength distance, int min_speed) {
  slew_swing_rev_using_angle = false;
  double dist = distance.convert(okapi::inch);
  slew_swing_backward.constants_set(dist, min_speed);
}
void Drive::slew_swing_constants_forward_set(okapi::QLength distance, int min_speed) {
  slew_swing_fwd_using_angle = false;
  double dist = distance.convert(okapi::inch);
  slew_swing_forward.constants_set(dist, min_speed);
}
void Drive::slew_swing_constants_set(okapi::QLength distance, int min_speed) {
  slew_swing_constants_forward_set(distance, min_speed);
  slew_swing_constants_backward_set(distance, min_speed);
}
void Drive::slew_swing_constants_backward_set(okapi::QAngle distance, int min_speed) {
  slew_swing_rev_using_angle = true;
  double dist = distance.convert(okapi::degree);
  slew_swing_backward.constants_set(dist, min_speed);
}
void Drive::slew_swing_constants_forward_set(okapi::QAngle distance, int min_speed) {
  slew_swing_fwd_using_angle = true;
  double dist = distance.convert(okapi::degree);
  slew_swing_forward.constants_set(dist, min_speed);
}
void Drive::slew_swing_constants_set(okapi::QAngle distance, int min_speed) {
  slew_swing_constants_forward_set(distance, min_speed);
  slew_swing_constants_backward_set(distance, min_speed);
}

// Global enables for swing slew
void Drive::slew_swing_set(bool slew_on) {
  global_forward_swing_slew_enabled = slew_on;
  global_backward_swing_slew_enabled = slew_on;
}
void Drive::slew_swing_forward_set(bool slew_on) { global_forward_swing_slew_enabled = slew_on; }
bool Drive::slew_swing_forward_get() { return global_forward_swing_slew_enabled; }
void Drive::slew_swing_backward_set(bool slew_on) { global_backward_swing_slew_enabled = slew_on; }
bool Drive::slew_swing_backward_get() { return global_backward_swing_slew_enabled; }
// Checks if slew is globally enabled or not
bool Drive::is_swing_slew_enabled(e_swing type, double target, double current) {
  int side = type == ez::LEFT_SWING ? 1 : -1;
  int direction = util::sgn((target - current) * side);
  return direction == 1 ? slew_swing_forward_get() : slew_swing_backward_get();
}

// Swing default behavior set
void Drive::pid_swing_behavior_set(ez::e_angle_behavior behavior) { default_swing_type = behavior; }
ez::e_angle_behavior Drive::pid_swing_behavior_get() { return default_swing_type; }

/////
// Set swing PID basic wrappers
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed) {
  bool slew_on = is_swing_slew_enabled(type, target, drive_imu_get());
  pid_swing_set(type, target, speed, 0, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed) {
  // Figure out if going forward or backward
  double absolute_heading = target + headingPID.target_get();
  bool slew_on = is_swing_slew_enabled(type, absolute_heading, drive_imu_get());
  pid_swing_relative_set(type, target, speed, 0, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed);
}

/////
// Set turn PID with only swing behavior
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior) {
  bool slew_on = is_swing_slew_enabled(type, target, drive_imu_get());
  pid_swing_set(type, target, speed, 0, behavior, slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed, behavior);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior) {
  // Figure out if going forward or backward
  double absolute_heading = target + headingPID.target_get();
  bool slew_on = is_swing_slew_enabled(type, absolute_heading, drive_imu_get());
  pid_swing_relative_set(type, target, speed, 0, behavior, slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, behavior);
}

/////
// Set turn PID with only opposite speed
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed, int opposite_speed) {
  bool slew_on = is_swing_slew_enabled(type, target, drive_imu_get());
  pid_swing_set(type, target, speed, opposite_speed, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  bool slew_on = is_swing_slew_enabled(type, target, drive_imu_get());
  pid_swing_set(type, target, speed, opposite_speed, slew_on);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed) {
  double absolute_heading = target + headingPID.target_get();
  bool slew_on = is_swing_slew_enabled(type, absolute_heading, drive_imu_get());
  pid_swing_relative_set(type, target, speed, opposite_speed, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, opposite_speed);
}

/////
// Set turn PID with only slew
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed, bool slew_on) {
  pid_swing_set(type, target, speed, 0, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed, slew_on);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, bool slew_on) {
  pid_swing_relative_set(type, target, speed, 0, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, slew_on);
}

/////
// Set turn PID with only opposite speed and swing behavior
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior) {
  bool slew_on = is_swing_slew_enabled(type, target, drive_imu_get());
  pid_swing_set(type, target, speed, opposite_speed, behavior, slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  bool slew_on = is_swing_slew_enabled(type, target, drive_imu_get());
  pid_swing_set(type, target, speed, opposite_speed, behavior);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior) {
  double absolute_heading = target + headingPID.target_get();
  bool slew_on = is_swing_slew_enabled(type, absolute_heading, drive_imu_get());
  pid_swing_relative_set(type, target, speed, opposite_speed, behavior, slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, opposite_speed, behavior);
}

/////
// Set turn PID with opposite speed and slew
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on) {
  pid_swing_set(type, target, speed, opposite_speed, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed, opposite_speed, slew_on);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.target_get() + target;
  if (print_toggle) printf("Relative ");
  pid_swing_set(type, absolute_target, speed, opposite_speed, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, opposite_speed, slew_on);
}

/////
// Set turn PID with swing behavior and slew
/////
// Absolute
void Drive::pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on) {
  pid_swing_set(type, target, speed, 0, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed, behavior, slew_on);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.target_get() + target;
  if (print_toggle) printf("Relative ");
  pid_swing_set(type, absolute_target, speed, 0, pid_swing_behavior_get(), slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, behavior, slew_on);
}

/////
// Set turn PID with opposite speed, swing behavior, and slew
/////
// Absolute
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed, opposite_speed, behavior, slew_on);
}
// Relative
void Drive::pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.target_get() + target;
  if (print_toggle) printf("Relative ");
  pid_swing_set(type, absolute_target, speed, opposite_speed, behavior, slew_on);
}
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, opposite_speed, behavior, slew_on);
}

/////
// Swing set base
/////
void Drive::pid_swing_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on) {
  swingPID.timers_reset();

  // Set turn behavior
  current_angle_behavior = behavior;

  // Compute new turn target based on new angle
  target = flip_angle_target(target);
  target = new_turn_target_compute(target, drive_imu_get(), current_angle_behavior);

  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %.2f\n", target);
  current_swing = type;
  chain_sensor_start = drive_imu_get();
  chain_target_start = target;
  used_motion_chain_scale = 0.0;

  // Figure out if going forward or backward
  int side = type == ez::LEFT_SWING ? 1 : -1;
  int direction = util::sgn((target - chain_sensor_start) * side);

  // Set constants according to the robots direction

  PID *new_drive_pid;
  PID *new_swing_pid;
  slew::Constants slew_consts;

  if (direction == -1) {
    new_drive_pid = &backward_drivePID;
    new_swing_pid = &backward_swingPID;
    slew_consts = slew_swing_backward.constants_get();
    slew_swing_using_angle = slew_swing_rev_using_angle;
    motion_chain_backward = true;
  } else {
    new_drive_pid = &forward_drivePID;
    new_swing_pid = &forward_swingPID;
    slew_consts = slew_swing_forward.constants_get();
    slew_swing_using_angle = slew_swing_fwd_using_angle;
    motion_chain_backward = false;
  }

  // Prioritize custom fwd/rev constants.  Otherwise, use the same for fwd and rev
  if (fwd_rev_drivePID.constants_set_check() && (!new_drive_pid->constants_set_check()))
    new_drive_pid = &fwd_rev_drivePID;
  if (fwd_rev_swingPID.constants_set_check() && !new_swing_pid->constants_set_check())
    new_swing_pid = &fwd_rev_swingPID;

  PID::Constants pid_drive_consts = new_drive_pid->constants_get();
  PID::Constants pid_swing_consts = new_swing_pid->constants_get();
  swingPID.constants_set(pid_swing_consts.kp, pid_swing_consts.ki, pid_swing_consts.kd, pid_swing_consts.start_i);
  leftPID.constants_set(pid_drive_consts.kp, pid_drive_consts.ki, pid_drive_consts.kd, pid_drive_consts.start_i);
  rightPID.constants_set(pid_drive_consts.kp, pid_drive_consts.ki, pid_drive_consts.kd, pid_drive_consts.start_i);
  slew_swing.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);

  // Set targets for the side that isn't moving
  leftPID.target_set(drive_sensor_left());
  rightPID.target_set(drive_sensor_right());

  // Set PID targets
  swingPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);
  swing_opposite_speed = opposite_speed;

  // Initialize slew
  double current = slew_swing_using_angle ? chain_sensor_start : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  double slew_tar = slew_swing_using_angle ? target : direction * 100;
  if (!slew_swing_using_angle) slew_tar += current;
  slew_swing.initialize(slew_on, max_speed, slew_tar, current);
  current_slew_on = slew_on;

  // Run task
  drive_mode_set(SWING);
}
