/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Set PID constants
void Drive::set_drive_pid_constants(double p, double i, double d, double p_start_i) {
  set_drive_forward_pid_constants(p, i, d, p_start_i);
  set_drive_backwards_pid_constants(p, i, d, p_start_i);
}

void Drive::set_drive_forward_pid_constants(double p, double i, double d, double p_start_i) {
  forward_drivePID.constants_set(p, i, d, p_start_i);
}

void Drive::set_drive_backwards_pid_constants(double p, double i, double d, double p_start_i) {
  backward_drivePID.constants_set(p, i, d, p_start_i);
}

void Drive::set_turn_pid_constants(double p, double i, double d, double p_start_i) {
  turnPID.constants_set(p, i, d, p_start_i);
}

void Drive::set_swing_pid_constants(double p, double i, double d, double p_start_i) {
  swingPID.constants_set(p, i, d, p_start_i);
}

void Drive::set_heading_pid_constants(double p, double i, double d, double p_start_i) {
  headingPID.constants_set(p, i, d, p_start_i);
}

// Updates max speed
void Drive::set_max_speed(int speed) {
  max_speed = util::clamp_number(abs(speed), 127, -127);
}

void Drive::reset_pid_targets() {
  headingPID.target_set(0);
  leftPID.target_set(0);
  rightPID.target_set(0);
  forward_drivePID.target_set(0);
  backward_drivePID.target_set(0);
  turnPID.target_set(0);
}

void Drive::set_angle(double angle) {
  headingPID.target_set(angle);
  reset_gyro(angle);
}

void Drive::set_mode(e_mode p_mode) { mode = p_mode; }

void Drive::set_turn_min(int min) { turn_min = abs(min); }
int Drive::get_turn_min() { return turn_min; }

void Drive::set_swing_min(int min) { swing_min = abs(min); }
int Drive::get_swing_min() { return swing_min; }

e_mode Drive::get_mode() { return mode; }

// Set drive PID
void Drive::set_drive_pid(okapi::QLength p_target, int speed, bool slew_on, bool toggle_heading) {
  double target = p_target.convert(okapi::inch);

  // Print targets
  if (print_toggle) printf("Drive Started... Target Value: %f in", target);
  if (slew_on && print_toggle) printf(" with slew");
  if (print_toggle) printf("\n");

  // Global setup
  set_max_speed(speed);
  heading_on = toggle_heading;
  bool is_backwards = false;
  l_start = left_sensor();
  r_start = right_sensor();

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
  slew_initialize(left_slew, slew_on, max_speed, l_target_encoder, left_sensor(), l_start, is_backwards);
  slew_initialize(right_slew, slew_on, max_speed, r_target_encoder, right_sensor(), r_start, is_backwards);

  // Run task
  set_mode(DRIVE);
}

// Set turn PID
void Drive::set_turn_pid(double target, int speed) {
  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  turnPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  set_max_speed(speed);

  // Run task
  set_mode(TURN);
}

void Drive::set_relative_turn_pid(double target, int speed) {
  // Compute absolute target by adding to current heading
  double absolute_target = turnPID.target_get() + target;

  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", absolute_target);

  // Set PID targets
  turnPID.target_set(absolute_target);
  headingPID.target_set(absolute_target);
  set_max_speed(speed);

  // Run task
  set_mode(TURN);
}

// Set swing PID
void Drive::set_swing_pid(e_swing type, double target, int speed) {
  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %f\n", target);
  current_swing = type;

  // Set PID targets
  swingPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  set_max_speed(speed);

  // Run task
  set_mode(SWING);
}
