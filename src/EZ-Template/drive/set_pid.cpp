/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/util.hpp"
#include "main.h"
#include "okapi/api/units/QAngle.hpp"

// Set PID constants
void Drive::pid_drive_constants_set(double p, double i, double d, double p_start_i) {
  pid_drive_constants_forward_set(p, i, d, p_start_i);
  pid_drive_constants_backward_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_drive_constants_get() {
  auto fwd_const = pid_drive_constants_forward_get();
  auto rev_const = pid_drive_constants_backward_get();
  if (!(fwd_const.kp == rev_const.kp && fwd_const.ki == rev_const.ki && fwd_const.kd == rev_const.kd && fwd_const.start_i == rev_const.start_i)) {
    printf("\nForward and Reverse constants are not the same!");
    return {-1, -1, -1, -1};
  }
  return fwd_const;
}

void Drive::pid_drive_constants_forward_set(double p, double i, double d, double p_start_i) {
  forward_drivePID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_drive_constants_forward_get() {
  return forward_drivePID.constants_get();
}

void Drive::pid_drive_constants_backward_set(double p, double i, double d, double p_start_i) {
  backward_drivePID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_drive_constants_backward_get() {
  return backward_drivePID.constants_get();
}

void Drive::pid_turn_constants_set(double p, double i, double d, double p_start_i) {
  turnPID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_turn_constants_get() {
  return turnPID.constants_get();
}

void Drive::pid_swing_constants_set(double p, double i, double d, double p_start_i) {
  pid_swing_constants_forward_set(p, i, d, p_start_i);
  pid_swing_constants_backward_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_swing_constants_get() {
  auto fwd_const = pid_swing_constants_forward_get();
  auto rev_const = pid_swing_constants_backward_get();
  if (!(fwd_const.kp == rev_const.kp && fwd_const.ki == rev_const.ki && fwd_const.kd == rev_const.kd && fwd_const.start_i == rev_const.start_i)) {
    printf("\nForward and Reverse constants are not the same!");
    return {-1, -1, -1, -1};
  }
  return fwd_const;
}

void Drive::pid_swing_constants_forward_set(double p, double i, double d, double p_start_i) {
  forward_swingPID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_swing_constants_forward_get() {
  return forward_swingPID.constants_get();
}

void Drive::pid_swing_constants_backward_set(double p, double i, double d, double p_start_i) {
  backward_swingPID.constants_set(p, i, d, p_start_i);
}

PID::Constants Drive::pid_swing_constants_backward_get() {
  return backward_swingPID.constants_get();
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
  slew_left.speed_max_set(max_speed);
  slew_right.speed_max_set(max_speed);
  slew_turn.speed_max_set(max_speed);
  slew_swing.speed_max_set(max_speed);
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
  swingPID.target_set(0);
  forward_swingPID.target_set(0);
  backward_swingPID.target_set(0);
}

void Drive::drive_angle_set(double angle) {
  headingPID.target_set(angle);
  drive_imu_reset(angle);
}

void Drive::drive_angle_set(okapi::QAngle p_angle) {
  double angle = p_angle.convert(okapi::degree);  // Convert okapi unit to degree
  drive_angle_set(angle);
}

void Drive::drive_mode_set(e_mode p_mode) { mode = p_mode; }
e_mode Drive::drive_mode_get() { return mode; }

void Drive::pid_turn_min_set(int min) { turn_min = abs(min); }
int Drive::pid_turn_min_get() { return turn_min; }

void Drive::pid_swing_min_set(int min) { swing_min = abs(min); }
int Drive::pid_swing_min_get() { return swing_min; }

// Set drive PID raw
void Drive::pid_drive_set(double target, int speed, bool slew_on, bool toggle_heading) {
  leftPID.timers_reset();
  rightPID.timers_reset();

  // Print targets
  if (print_toggle) printf("Drive Started... Target Value: %.2f", target);
  if (slew_on && print_toggle) printf(" with slew");
  if (print_toggle) printf("\n");
  chain_target_start = target;
  chain_sensor_start = drive_sensor_left();
  used_motion_chain_scale = 0.0;

  // Global setup
  pid_speed_max_set(speed);
  heading_on = toggle_heading;
  l_start = drive_sensor_left();
  r_start = drive_sensor_right();

  double l_target_encoder, r_target_encoder;

  // Figure actual target value
  l_target_encoder = l_start + target;
  r_target_encoder = r_start + target;

  PID::Constants pid_consts;
  slew::Constants slew_consts;

  // Figure out if going forward or backward and set constants accordingly
  if (l_target_encoder < l_start && r_target_encoder < r_start) {
    pid_consts = backward_drivePID.constants_get();
    slew_consts = slew_backward.constants_get();
    motion_chain_backward = true;
  } else {
    pid_consts = forward_drivePID.constants_get();
    slew_consts = slew_forward.constants_get();
    motion_chain_backward = false;
  }
  leftPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
  rightPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
  slew_left.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);
  slew_right.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);

  // Set PID targets
  leftPID.target_set(l_target_encoder);
  rightPID.target_set(r_target_encoder);

  // Initialize slew
  slew_left.initialize(slew_on, max_speed, l_target_encoder, drive_sensor_left());
  slew_right.initialize(slew_on, max_speed, r_target_encoder, drive_sensor_right());

  // Run task
  drive_mode_set(DRIVE);
}

// Set drive PID
void Drive::pid_drive_set(okapi::QLength p_target, int speed, bool slew_on, bool toggle_heading) {
  double target = p_target.convert(okapi::inch);  // Convert okapi unit to inches
  pid_drive_set(target, speed, slew_on, toggle_heading);
}

// Raw Set Turn PID
void Drive::pid_turn_set(double target, int speed, bool slew_on) {
  turnPID.timers_reset();

  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %.2f\n", target);
  chain_sensor_start = drive_imu_get();
  chain_target_start = target;
  used_motion_chain_scale = 0.0;

  // Set PID targets
  turnPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);

  // Initialize slew
  slew_turn.initialize(slew_on, max_speed, target, chain_sensor_start);

  // Run task
  drive_mode_set(TURN);
}

// Set turn PID
void Drive::pid_turn_set(okapi::QAngle p_target, int speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_set(target, speed, slew_on);
}

// Set relative turn PID
void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_relative_set(target, speed, slew_on);
}

// Set relative turn PID
void Drive::pid_turn_relative_set(double target, int speed, bool slew_on) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.target_get() + target;
  if (print_toggle) printf("Relative ");
  pid_turn_set(absolute_target, speed, slew_on);
}

// Raw Set Swing PID
void Drive::pid_swing_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on) {
  swingPID.timers_reset();

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
  PID::Constants pid_consts;
  PID::Constants pid_swing_consts;
  slew::Constants slew_consts;

  if (direction == -1) {
    pid_consts = backward_drivePID.constants_get();
    pid_swing_consts = backward_swingPID.constants_get();
    slew_consts = slew_swing_backward.constants_get();
    slew_swing_using_angle = slew_swing_rev_using_angle;
    motion_chain_backward = true;

  } else {
    pid_consts = forward_drivePID.constants_get();
    pid_swing_consts = forward_swingPID.constants_get();
    slew_consts = slew_swing_forward.constants_get();
    slew_swing_using_angle = slew_swing_fwd_using_angle;
    motion_chain_backward = false;
  }

  // Set targets for the side that isn't moving
  swingPID.constants_set(pid_swing_consts.kp, pid_swing_consts.ki, pid_swing_consts.kd, pid_swing_consts.start_i);
  leftPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
  rightPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
  leftPID.target_set(drive_sensor_left());
  rightPID.target_set(drive_sensor_right());
  slew_swing.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);

  // Set PID targets
  swingPID.target_set(target);
  headingPID.target_set(target);  // Update heading target for next drive motion
  pid_speed_max_set(speed);
  swing_opposite_speed = opposite_speed;

  // Initialize slew
  double slew_tar = slew_swing_using_angle ? target : direction * 100;
  double current = slew_swing_using_angle ? chain_sensor_start : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  slew_swing.initialize(slew_on, max_speed, slew_tar, current);

  // Run task
  drive_mode_set(SWING);
}

// Set swing PID
void Drive::pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_set(type, target, speed, opposite_speed, slew_on);
}

// Set relative swing PID
void Drive::pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_swing_relative_set(type, target, speed, opposite_speed, slew_on);
}

void Drive::pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.target_get() + target;
  if (print_toggle) printf("Relative ");
  pid_swing_set(type, absolute_target, speed, opposite_speed, slew_on);
}