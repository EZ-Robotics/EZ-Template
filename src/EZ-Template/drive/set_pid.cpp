/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Set PID constants
void Drive::pid_drive_constants_set(double p, double i, double d, double p_start_i) {
  pid_drive_forward_constants_set(p, i, d, p_start_i);
  pid_drive_backwards_constants_set(p, i, d, p_start_i);
}

void Drive::pid_drive_forward_constants_set(double p, double i, double d, double p_start_i) {
  pid_forward_drive.constants_set(p, i, d, p_start_i);
}

void Drive::pid_drive_backwards_constants_set(double p, double i, double d, double p_start_i) {
  pid_backward_drive.constants_set(p, i, d, p_start_i);
}

void Drive::pid_turn_constants_set(double p, double i, double d, double p_start_i) {
  pid_turn.constants_set(p, i, d, p_start_i);
}

void Drive::pid_swing_constants_set(double p, double i, double d, double p_start_i) {
  pid_swing.constants_set(p, i, d, p_start_i);
}

void Drive::pid_heading_constants_set(double p, double i, double d, double p_start_i) {
  pid_heading.constants_set(p, i, d, p_start_i);
}

// Updates max speed
void Drive::max_speed_set(int speed) {
  speed_max = util::clamp_number(abs(speed), 127, -127);
}

void Drive::pid_targets_reset() {
  pid_heading.target_set(0);
  pid_left.target_set(0);
  pid_right.target_set(0);
  pid_forward_drive.target_set(0);
  pid_backward_drive.target_set(0);
  pid_turn.target_set(0);
}

void Drive::angle_set(double angle) {
  pid_heading.target_set(angle);
  imu_reset(angle);
}

void Drive::mode_set(e_mode p_mode) { mode = p_mode; }

void Drive::turn_min_set(int min) { turn_min = abs(min); }
int Drive::turn_min_get() { return turn_min; }

void Drive::swing_min_set(int min) { swing_min = abs(min); }
int Drive::swing_min_get() { return swing_min; }

e_mode Drive::mode_get() { return mode; }

// Set drive PID
void Drive::pid_drive_set(okapi::QLength p_target, int speed, bool slew_on, bool toggle_heading) {
  double target = p_target.convert(okapi::inch);

  // Print targets
  if (print_toggle) printf("Drive Started... Target Value: %f in", target);
  if (slew_on && print_toggle) printf(" with slew");
  if (print_toggle) printf("\n");

  // Global setup
  max_speed_set(speed);
  heading_on = toggle_heading;
  bool is_backwards = false;
  start_left = sensor_left();
  start_right = sensor_right();

  double l_target_encoder, r_target_encoder;

  // Figure actual target value
  l_target_encoder = start_left + target;
  r_target_encoder = start_right + target;

  // Figure out if going forward or backward
  if (l_target_encoder < start_left && r_target_encoder < start_right) {
    auto consts = pid_backward_drive.constants_get();
    pid_left.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    pid_right.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = true;
  } else {
    auto consts = pid_forward_drive.constants_get();
    pid_left.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    pid_right.constants_set(consts.kp, consts.ki, consts.kd, consts.start_i);
    is_backwards = false;
  }

  // Set PID targets
  pid_left.target_set(l_target_encoder);
  pid_right.target_set(r_target_encoder);

  // Initialize slew
  slew_initialize(slew_left, slew_on, speed_max, l_target_encoder, sensor_left(), start_left, is_backwards);
  slew_initialize(slew_right, slew_on, speed_max, r_target_encoder, sensor_right(), start_right, is_backwards);

  // Run task
  mode_set(DRIVE);
}

// Set turn PID
void Drive::pid_turn_set(double target, int speed) {
  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  pid_turn.target_set(target);
  pid_heading.target_set(target);  // Update heading target for next drive motion
  max_speed_set(speed);

  // Run task
  mode_set(TURN);
}

void Drive::pid_relative_turn_set(double target, int speed) {
  // Compute absolute target by adding to current heading
  double absolute_target = pid_turn.target_get() + target;

  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", absolute_target);

  // Set PID targets
  pid_turn.target_set(absolute_target);
  pid_heading.target_set(absolute_target);
  max_speed_set(speed);

  // Run task
  mode_set(TURN);
}

// Set swing PID
void Drive::pid_swing_set(e_swing type, double target, int speed) {
  // Print targets
  if (print_toggle) printf("Swing Started... Target Value: %f\n", target);
  swing_current = type;

  // Set PID targets
  pid_swing.target_set(target);
  pid_heading.target_set(target);  // Update heading target for next drive motion
  max_speed_set(speed);

  // Run task
  mode_set(SWING);
}
