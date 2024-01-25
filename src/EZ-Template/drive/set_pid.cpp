/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "okapi/api/units/QAngle.hpp"

// Set PID constants
void Drive::pid_drive_constants_set(double p, double i, double d, double p_start_i) {
  pid_drive_forward_constants_set(p, i, d, p_start_i);
  pid_drive_backward_constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_drive_constants_get() {
  auto fwd_const = pid_drive_forward_constants_get();
  auto rev_const = pid_drive_backward_constants_get();
  if (!(fwd_const.kp == rev_const.kp && fwd_const.ki == rev_const.ki && fwd_const.kd == rev_const.kd && fwd_const.start_i == rev_const.start_i)) {
    printf("\nForward and Reverse constants are not the same!");
    return {-1, -1, -1, -1};
  }
  return fwd_const;
}

void Drive::pid_drive_forward_constants_set(double p, double i, double d, double p_start_i) {
  forward_drivePID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_drive_forward_constants_get() {
  return forward_drivePID.constants_get();
}

void Drive::pid_drive_backward_constants_set(double p, double i, double d, double p_start_i) {
  backward_drivePID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_drive_backward_constants_get() {
  return backward_drivePID.constants_get();
}

void Drive::pid_turn_constants_set(double p, double i, double d, double p_start_i) {
  turnPID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_turn_constants_get() {
  return turnPID.constants_get();
}

void Drive::pid_swing_constants_set(double p, double i, double d, double p_start_i) {
  swingPID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_swing_constants_get() {
  return swingPID.constants_get();
}

void Drive::pid_heading_constants_set(double p, double i, double d, double p_start_i) {
  headingPID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_heading_constants_get() {
  return headingPID.constants_get();
}

// Updates max speed
void Drive::pid_speed_max_set(int speed) {
  max_speed = abs(util::clamp(speed, 127, -127));
}

int Drive::pid_speed_max_get() {
  return max_speed;
}

void Drive::pid_targets_reset() {
  headingPID.target_set(0);
  leftPID.target_set(0);
  rightPID.target_set(0);
  forward_drivePID.target_set(0);
  backward_drivePID.target_set(0);
  turnPID.target_set(0);
}

void Drive::drive_angle_raw_set(double angle) {
  headingPID.target_set(angle);
  drive_imu_reset(angle);
}

void Drive::drive_angle_set(okapi::QAngle p_angle) {
  double angle = p_angle.convert(okapi::degree);  // Convert okapi unit to degree
  drive_angle_raw_set(angle);
}

void Drive::drive_mode_set(e_mode p_mode) { mode = p_mode; }
e_mode Drive::drive_mode_get() { return mode; }

void Drive::pid_turn_min_set(int min) { turn_min = abs(min); }
int Drive::pid_turn_min_get() { return turn_min; }

void Drive::pid_swing_min_set(int min) { swing_min = abs(min); }
int Drive::pid_swing_min_get() { return swing_min; }

// Set drive PID
void Drive::pid_drive_set(okapi::QLength p_target, int speed, bool slew_on, bool toggle_heading) {
  double target = p_target.convert(okapi::inch);  // Convert okapi unit to inches

  // Print targets
  if (print_toggle) printf("Drive Started... Target Value: %f in", target);
  if (slew_on && print_toggle) printf(" with slew");
  if (print_toggle) printf("\n");

  // Global setup
  pid_speed_max_set(speed);
  heading_on = toggle_heading;
  bool is_backwards = false;
  l_start = drive_sensor_left();
  r_start = drive_sensor_right();

  double l_target_encoder, r_target_encoder;

  // Figure actual target value
  l_target_encoder = l_start + target;
  r_target_encoder = r_start + target;

  // Figure out if going forward or backward
  if (l_target_encoder < l_start && r_target_encoder < r_start) {
    auto consts = backward_drivePID.constants_get();
    leftPID.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    rightPID.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = true;
  } else {
    auto consts = forward_drivePID.constants_get();
    leftPID.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    rightPID.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = false;
  }

  // Set PID targets
  leftPID.target_set(l_target_encoder);
  rightPID.target_set(r_target_encoder);

  // Initialize slew
  slew_initialize(left_slew, slew_on, max_speed, l_target_encoder, drive_sensor_left(), l_start, is_backwards);
  slew_initialize(right_slew, slew_on, max_speed, r_target_encoder, drive_sensor_right(), r_start, is_backwards);

  // Run task
  drive_mode_set(DRIVE);
}

// Raw Set Turn PID
void Drive::pid_turn_raw_set(double target, int speed) {
  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  turnPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);

  // Run task
  drive_mode_set(TURN);
}

// Set turn PID
void Drive::pid_turn_set(okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_raw_set(target, speed);
}

void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  // Compute absolute target by adding to current heading
  double absolute_target = turnPID.target_get() + target;
  pid_turn_raw_set(absolute_target, speed);

  if (print_toggle) printf("Relative ");
}

// Raw Set Swing PID
void Drive::pid_swing_raw_set(e_swing type, double target, int speed, int opposite_speed) {
  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %f\n", target);
  current_swing = type;

  // Set targets for the side that isn't moving
  auto consts = forward_drivePID.constants_get();
  leftPID.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
  rightPID.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
  leftPID.target_set(drive_sensor_left());
  rightPID.target_set(drive_sensor_right());

  // Set PID targets
  swingPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);
  swing_opposite_speed = opposite_speed;

  // Run task
  drive_mode_set(SWING);
}

// Set swing PID
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_raw_set(type, target, speed, opposite_speed);
}

// Set swing PID
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  // Compute absolute target by adding to current heading
  double absolute_target = swingPID.target_get() + target;
  pid_swing_raw_set(type, absolute_target, speed, opposite_speed);

  if (print_toggle) printf("Relative ");
}
