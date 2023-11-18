/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Set PID constants
void Drive::pid_drive_constants_set(double p, double i, double d, double p_start_i) {
  pid_drive_forward_constants_set(p, i, d, p_start_i);
  pid_drive_backward_constants_set(p, i, d, p_start_i);
}

void Drive::pid_drive_forward_constants_set(double p, double i, double d, double p_start_i) {
  forward_drivePID.constants_set(p, i, d, p_start_i);
}

void Drive::pid_drive_backward_constants_set(double p, double i, double d, double p_start_i) {
  backward_drivePID.constants_set(p, i, d, p_start_i);
}

void Drive::pid_turn_constants_set(double p, double i, double d, double p_start_i) {
  turnPID.constants_set(p, i, d, p_start_i);
}

void Drive::pid_swing_constants_set(double p, double i, double d, double p_start_i) {
  swingPID.constants_set(p, i, d, p_start_i);
}

void Drive::pid_heading_constants_set(double p, double i, double d, double p_start_i) {
  headingPID.constants_set(p, i, d, p_start_i);
}

// Updates max speed
void Drive::pid_speed_max_set(int speed) {
  max_speed = util::clamp(abs(speed), 127, -127);
}

void Drive::pid_targets_reset() {
  headingPID.target_set(0);
  leftPID.target_set(0);
  rightPID.target_set(0);
  forward_drivePID.target_set(0);
  backward_drivePID.target_set(0);
  turnPID.target_set(0);
}

void Drive::set_angle(okapi::QAngle p_angle) {
  double angle = p_angle.convert(okapi::degree);  // Convert okapi unit to degree

  headingPID.target_set(angle);
  drive_imu_reset(angle);
}

void Drive::drive_mode_set(e_mode p_mode) { mode = p_mode; }

void Drive::pid_turn_min_set(int min) { turn_min = abs(min); }
int Drive::get_turn_min() { return turn_min; }

void Drive::pid_swing_min_set(int min) { swing_min = abs(min); }
int Drive::pid_swing_min_get() { return swing_min; }

e_mode Drive::drive_mode_get() { return mode; }

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

// Set turn PID
void Drive::pid_turn_set(okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree

  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  turnPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);

  // Run task
  drive_mode_set(TURN);
}

void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  // Compute absolute target by adding to current heading
  double absolute_target = turnPID.target_get() + target;

  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", absolute_target);

  // Set PID targets
  turnPID.target_set(absolute_target);
  headingPID.target_set(absolute_target);
  pid_speed_max_set(speed);

  // Run task
  drive_mode_set(TURN);
}

// Set swing PID
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %f\n", target);
  current_swing = type;

  // Set PID targets
  swingPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);

  // Run task
  drive_mode_set(SWING);
}

// Set swing PID
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree

  // Compute absolute target by adding to current heading
  double absolute_target = swingPID.target_get() + target;

  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %f\n", absolute_target);
  current_swing = type;

  // Set PID targets
  swingPID.target_set(absolute_target);
  headingPID.target_set(absolute_target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);

  // Run task
  drive_mode_set(SWING);
}
