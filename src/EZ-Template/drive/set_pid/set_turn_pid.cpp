/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "okapi/api/units/QAngle.hpp"

/////
// Sets turn constants
/////
void Drive::slew_turn_constants_set(okapi::QAngle distance, int min_speed) {
  double dist = distance.convert(okapi::degree);
  slew_turn.constants_set(dist, min_speed);
}
void Drive::pid_turn_constants_set(double p, double i, double d, double p_start_i) {
  turnPID.constants_set(p, i, d, p_start_i);
}
PID::Constants Drive::pid_turn_constants_get() { return turnPID.constants_get(); }
void Drive::pid_turn_min_set(int min) { turn_min = abs(min); }
int Drive::pid_turn_min_get() { return turn_min; }

// Sets the behavior of turning
void Drive::pid_turn_behavior_set(ez::e_angle_behavior behavior) { default_turn_type = behavior; }
ez::e_angle_behavior Drive::pid_turn_behavior_get() { return default_turn_type; }

// Global enables for turn slew
void Drive::slew_turn_set(bool slew_on) { global_turn_slew_enabled = slew_on; }
bool Drive::slew_turn_get() { return global_turn_slew_enabled; }

double Drive::flip_angle_target(double target) {
  int flip_theta = theta_flipped ? -1 : 1;
  double new_target = target;
  new_target *= flip_theta;
  return new_target;
}

/////
// Set turn PID basic wrappers
/////
// Absolute
void Drive::pid_turn_set(double target, int speed) {
  pid_turn_set(target, speed, pid_turn_behavior_get(), slew_turn_get());
}
void Drive::pid_turn_set(okapi::QAngle p_target, int speed) {
  pid_turn_set(p_target, speed, pid_turn_behavior_get(), slew_turn_get());
}
// Relative
void Drive::pid_turn_relative_set(double target, int speed) {
  pid_turn_relative_set(target, speed, pid_turn_behavior_get(), slew_turn_get());
}
void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed) {
  pid_turn_relative_set(p_target, speed, pid_turn_behavior_get(), slew_turn_get());
}

/////
// Set turn PID with only turn behavior
/////
// Absolute
void Drive::pid_turn_set(double target, int speed, e_angle_behavior behavior) {
  pid_turn_set(target, speed, behavior, slew_turn_get());
}
void Drive::pid_turn_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior) {
  pid_turn_set(p_target, speed, behavior, slew_turn_get());
}
// Relative
void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior) {
  pid_turn_relative_set(p_target, speed, behavior, slew_turn_get());
}
void Drive::pid_turn_relative_set(double target, int speed, e_angle_behavior behavior) {
  pid_turn_relative_set(target, speed, behavior, slew_turn_get());
}

/////
// Set turn PID with only slew
/////
// Absolute
void Drive::pid_turn_set(double target, int speed, bool slew_on) {
  pid_turn_set(target, speed, pid_turn_behavior_get(), slew_on);
}
void Drive::pid_turn_set(okapi::QAngle p_target, int speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_set(target, speed, pid_turn_behavior_get(), slew_on);
}
// Relative
void Drive::pid_turn_relative_set(double target, int speed, bool slew_on) {
  pid_turn_relative_set(target, speed, pid_turn_behavior_get(), slew_on);
}
void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_relative_set(target, speed, pid_turn_behavior_get(), slew_on);
}

/////
// Set turn PID with turn behavior and slew
/////
// Absolute
void Drive::pid_turn_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_set(target, speed, behavior, slew_on);
}
// Relative
void Drive::pid_turn_relative_set(double target, int speed, e_angle_behavior behavior, bool slew_on) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.target_get() + target;
  if (print_toggle) printf("Relative ");
  pid_turn_set(absolute_target, speed, behavior, slew_on);
}
void Drive::pid_turn_relative_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on) {
  double target = p_target.convert(okapi::degree);  // Convert okapi unit to degree
  pid_turn_relative_set(target, speed, behavior, slew_on);
}

/////
// Turn to angle base
/////
void Drive::pid_turn_set(double target, int speed, e_angle_behavior behavior, bool slew_on) {
  turnPID.timers_reset();

  // Set turn behavior
  current_angle_behavior = behavior;

  // Compute new turn target based on new angle
  target = flip_angle_target(target);
  target = new_turn_target_compute(target, drive_imu_get(), current_angle_behavior);

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
  current_slew_on = slew_on;

  // Run task
  drive_mode_set(TURN);
}

/////
// Turn to point wrappers
/////
// No units
void Drive::pid_turn_set(pose itarget, drive_directions dir, int speed) {
  pid_turn_set(itarget, dir, speed, default_turn_type, slew_turn_get());
}
void Drive::pid_turn_set(pose itarget, drive_directions dir, int speed, bool slew_on) {
  pid_turn_set(itarget, dir, speed, default_turn_type, slew_on);
}
void Drive::pid_turn_set(pose itarget, drive_directions dir, int speed, e_angle_behavior behavior) {
  pid_turn_set(itarget, dir, speed, behavior, slew_turn_get());
}
// Units
void Drive::pid_turn_set(united_pose p_itarget, drive_directions dir, int speed) {
  pid_turn_set(util::united_pose_to_pose(p_itarget), dir, speed);
}
void Drive::pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, bool slew_on) {
  pid_turn_set(util::united_pose_to_pose(p_itarget), dir, speed, slew_on);
}
void Drive::pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior) {
  pid_turn_set(util::united_pose_to_pose(p_itarget), dir, speed, behavior);
}
void Drive::pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on) {
  pid_turn_set(util::united_pose_to_pose(p_itarget), dir, speed, behavior, slew_on);
}

/////
// Turn to point base
/////
void Drive::pid_turn_set(pose itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on) {
  itarget = flip_pose(itarget);
  odom_imu_start = drive_imu_get();

  current_drive_direction = dir;
  current_angle_behavior = behavior;

  // Calculate the point to look at
  point_to_face = find_point_to_face(odom_pose_get(), {itarget.x, itarget.y}, current_drive_direction, true);

  double target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_pose_get());  // Calculate the point for angle to face

  // Compute new turn target based on new angle
  // angle_adder = (new_turn_target_compute(target, odom_imu_start, current_angle_behavior)) - target;
  // ANGLE_ADDER_WAS_RESET = false;

  if (print_toggle) printf("Turn to Point PID Started... Target Point: (%.2f, %.2f) \n", itarget.x, itarget.y);
  pid_turn_set(target, speed, behavior, slew_on);

  drive_mode_set(TURN_TO_POINT);
}
