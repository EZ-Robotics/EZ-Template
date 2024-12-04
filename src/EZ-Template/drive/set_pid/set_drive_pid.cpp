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

// Slew constants
void Drive::slew_drive_constants_forward_set(okapi::QLength distance, int min_speed) {
  double dist = distance.convert(okapi::inch);
  slew_forward.constants_set(dist, min_speed);
}
void Drive::slew_drive_constants_backward_set(okapi::QLength distance, int min_speed) {
  double dist = distance.convert(okapi::inch);
  slew_backward.constants_set(dist, min_speed);
}
void Drive::slew_drive_constants_set(okapi::QLength distance, int min_speed) {
  slew_drive_constants_backward_set(distance, min_speed);
  slew_drive_constants_forward_set(distance, min_speed);
}
// Global enables for drive slew
void Drive::slew_drive_set(bool slew_on) {
  global_forward_drive_slew_enabled = slew_on;
  global_backward_drive_slew_enabled = slew_on;
}
void Drive::slew_drive_forward_set(bool slew_on) { global_forward_drive_slew_enabled = slew_on; }
bool Drive::slew_drive_forward_get() { return global_forward_drive_slew_enabled; }
void Drive::slew_drive_backward_set(bool slew_on) { global_backward_drive_slew_enabled = slew_on; }
bool Drive::slew_drive_backward_get() { return global_backward_drive_slew_enabled; }

// PID Constants
void Drive::pid_drive_constants_set(double p, double i, double d, double p_start_i) {
  pid_drive_constants_forward_set(0.0, 0.0, 0.0, 0.0);
  pid_drive_constants_backward_set(0.0, 0.0, 0.0, 0.0);
  fwd_rev_drivePID.constants_set(p, i, d, p_start_i);
}
void Drive::pid_drive_constants_forward_set(double p, double i, double d, double p_start_i) {
  forward_drivePID.constants_set(p, i, d, p_start_i);
}
void Drive::pid_drive_constants_backward_set(double p, double i, double d, double p_start_i) {
  backward_drivePID.constants_set(p, i, d, p_start_i);
}
void Drive::pid_heading_constants_set(double p, double i, double d, double p_start_i) {
  headingPID.constants_set(p, i, d, p_start_i);
}
void Drive::drive_angle_set(double angle) {
  headingPID.target_set(angle);
  drive_imu_reset(angle);
  central_pose.theta = angle;
  l_pose.theta = angle;
  r_pose.theta = angle;
  was_odom_just_set = true;
}
void Drive::drive_angle_set(okapi::QAngle p_angle) {
  double angle = p_angle.convert(okapi::degree);  // Convert okapi unit to degree
  drive_angle_set(angle);
}
PID::Constants Drive::pid_heading_constants_get() { return headingPID.constants_get(); }
PID::Constants Drive::pid_drive_constants_backward_get() { return backward_drivePID.constants_get(); }
PID::Constants Drive::pid_drive_constants_forward_get() { return forward_drivePID.constants_get(); }
PID::Constants Drive::pid_drive_constants_get() {
  auto fwd_const = pid_drive_constants_forward_get();
  auto rev_const = pid_drive_constants_backward_get();
  if (!(fwd_const.kp == rev_const.kp && fwd_const.ki == rev_const.ki && fwd_const.kd == rev_const.kd && fwd_const.start_i == rev_const.start_i)) {
    printf("\nForward and Reverse constants are not the same!");
    return {-1, -1, -1, -1};
  }
  return fwd_const;
}

/////
// Set drive PID
/////

// Set pid using global slew
void Drive::pid_drive_set(double target, int speed) {
  bool slew_on = util::sgn(target) >= 0 ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_drive_set(target, speed, slew_on);
}

// Set drive PID
void Drive::pid_drive_set(okapi::QLength p_target, int speed, bool slew_on, bool toggle_heading) {
  double target = p_target.convert(okapi::inch);  // Convert okapi unit to inches
  pid_drive_set(target, speed, slew_on, toggle_heading);
}

// Set drive PID with global slew and okapi units
void Drive::pid_drive_set(okapi::QLength p_target, int speed) {
  double target = p_target.convert(okapi::inch);  // Convert okapi unit to inches
  pid_drive_set(target, speed);
}

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

  PID *new_drive_pid;
  slew::Constants slew_consts;

  // Figure out if going forward or backward and set constants accordingly
  if (l_target_encoder < l_start && r_target_encoder < r_start) {
    new_drive_pid = &backward_drivePID;
    slew_consts = slew_backward.constants_get();
    motion_chain_backward = true;
  } else {
    new_drive_pid = &forward_drivePID;
    slew_consts = slew_forward.constants_get();
    motion_chain_backward = false;
  }

  // Prioritize custom fwd/rev constants.  Otherwise, use the same for fwd and rev
  if (fwd_rev_drivePID.constants_set_check() && !new_drive_pid->constants_set_check())
    new_drive_pid = &fwd_rev_drivePID;

  PID::Constants pid_drive_consts = new_drive_pid->constants_get();
  leftPID.constants_set(pid_drive_consts.kp, pid_drive_consts.ki, pid_drive_consts.kd, pid_drive_consts.start_i);
  rightPID.constants_set(pid_drive_consts.kp, pid_drive_consts.ki, pid_drive_consts.kd, pid_drive_consts.start_i);
  slew_left.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);
  slew_right.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);

  // Set PID targets
  leftPID.target_set(l_target_encoder);
  rightPID.target_set(r_target_encoder);

  // Initialize slew
  slew_left.initialize(slew_on, max_speed, l_target_encoder, drive_sensor_left());
  slew_right.initialize(slew_on, max_speed, r_target_encoder, drive_sensor_right());
  current_slew_on = slew_on;

  // Make sure we're using normal PID
  leftPID.exit = internal_leftPID.exit;
  rightPID.exit = internal_rightPID.exit;

  // Run task
  drive_mode_set(DRIVE);
}