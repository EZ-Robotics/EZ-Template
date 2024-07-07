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
  xyPID.target_set(0);
  aPID.target_set(0);
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

  // Make sure we're using normal PID
  leftPID.exit = internal_leftPID.exit;
  rightPID.exit = internal_rightPID.exit;

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

/////
// Odom Stuff
/////

// Set turn PID to point
void Drive::pid_turn_set(pose itarget, drive_directions dir, int speed, bool slew_on) {
  current_drive_direction = dir;

  // Calculate the point to look at
  point_to_face = find_point_to_face(odom_current, {itarget.x, itarget.y}, true);

  int add = current_drive_direction == REV ? 180 : 0;                                               // Decide if going fwd or rev
  double target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_current) + add;  // Calculate the point for angle to face
  turnPID.target_set(util::wrap_angle(target - drive_imu_get()));                                   // Constrain error to -180 to 180

  if (print_toggle) printf("Turn to Point PID Started... Target Point: (%.2f, %.2f) \n", itarget.x, itarget.y);
  pid_turn_set(target, speed, slew_on);

  drive_mode_set(TURN_TO_POINT);
}

// Raw move to point
void Drive::raw_pid_odom_ptp_set(odom imovement, bool slew_on) {
  // Update current turn type
  current_drive_direction = imovement.drive_direction;

  // Calculate the point to look at
  point_to_face = find_point_to_face(odom_current, {imovement.target.x, imovement.target.y}, true);

  // Set max speed
  pid_speed_max_set(imovement.max_xy_speed);

  // Set targets
  odom_target.x = imovement.target.x;
  odom_target.y = imovement.target.y;

  // Change constants if we're going fwd or rev
  PID::Constants pid_consts;
  slew::Constants slew_consts;
  PID::Constants angle_consts = headingPID.constants_get();
  if (current_drive_direction == REV) {
    pid_consts = backward_drivePID.constants_get();
    slew_consts = slew_backward.constants_get();

  } else {
    pid_consts = forward_drivePID.constants_get();
    slew_consts = slew_forward.constants_get();
  }

  // Set constants
  xyPID.constants_set(pid_consts.kp, pid_consts.ki, pid_consts.kd, pid_consts.start_i);
  aPID.constants_set(angle_consts.kp, angle_consts.ki, angle_consts.kd, angle_consts.start_i);
  slew_left.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);
  slew_right.constants_set(slew_consts.distance_to_travel, slew_consts.min_speed);

  bool is_current_boomerang = false;
  if (mode == PURE_PURSUIT)
    is_current_boomerang = pp_movements[pp_index].target.theta != ANGLE_NOT_SET ? true : false;
  if (print_toggle && !was_last_pp_mode_boomerang) {
    if (mode == PURE_PURSUIT)
      printf(" ");
    printf("Odom Motion Started... Target Coordinates: (%.2f, %.2f, %.2f) \n", imovement.target.x, imovement.target.y, imovement.target.theta);
  }
  if (mode == PURE_PURSUIT)
    was_last_pp_mode_boomerang = is_current_boomerang;

  // Get the starting point for if we're positive or negative.  This is used to find if we've past target
  past_target = util::sgn(is_past_target(odom_target, odom_current));

  // This is used for wait_until
  int dir = current_drive_direction == REV ? -1 : 1;  // If we're going backwards, add a -1
  leftPID.target_set(l_start + (LOOK_AHEAD * dir));
  rightPID.target_set(l_start + (LOOK_AHEAD * dir));
  leftPID.exit = xyPID.exit;  // Switch over to xy pid exits
  rightPID.exit = xyPID.exit;
}

// Move to point
void Drive::pid_odom_ptp_set(odom imovement, bool slew_on) {
  odom_second_to_last = odom_current;
  odom_target_start = imovement.target;
  odom_start = odom_current;

  xyPID.timers_reset();
  aPID.timers_reset();

  // This is used for wait_until and slew
  l_start = drive_sensor_left();
  r_start = drive_sensor_right();

  raw_pid_odom_ptp_set(imovement, slew_on);

  // Initialize slew
  int dir = current_drive_direction == REV ? -1 : 1;  // If we're going backwards, add a -1
  double dist_to_target = util::distance_to_point(odom_target, odom_current) * dir;
  slew_left.initialize(slew_on, max_speed, dist_to_target + l_start, l_start);
  slew_right.initialize(slew_on, max_speed, dist_to_target + r_start, r_start);

  drive_mode_set(POINT_TO_POINT);
}

// Raw pure pursuit
void Drive::raw_pid_odom_pp_set(std::vector<odom> imovements, bool slew_on) {
  odom_second_to_last = imovements[imovements.size() - 2].target;
  odom_target_start = imovements[imovements.size() - 1].target;
  odom_start = odom_current;

  was_last_pp_mode_boomerang = false;

  // Clear current list of targets
  pp_movements.clear();
  pp_index = 0;

  // Set new target
  pp_movements = imovements;

  raw_pid_odom_ptp_set(pp_movements[pp_index], slew_on);

  // This is used for wait_until and slew
  l_start = drive_sensor_left();
  r_start = drive_sensor_right();

  // Initialize slew
  int dir = current_drive_direction == REV ? -1 : 1;  // If we're going backwards, add a -1
  double dist_to_target = util::distance_to_point(pp_movements.end()->target, odom_current) * dir;
  slew_left.initialize(slew_on, max_speed, dist_to_target + l_start, l_start);
  slew_right.initialize(slew_on, max_speed, dist_to_target + r_start, r_start);

  drive_mode_set(PURE_PURSUIT);
}

// Pure pursuit
void Drive::pid_odom_pp_set(std::vector<odom> imovements, bool slew_on) {
  xyPID.timers_reset();
  aPID.timers_reset();

  std::vector<odom> input = imovements;
  input.insert(input.begin(), {{{odom_current.x, odom_current.y, ANGLE_NOT_SET}, imovements[0].drive_direction, imovements[0].max_xy_speed}});

  int t = 0;
  for (int i = 0; i < input.size() - 1; i++) {
    int j = i + t;
    j = i;
    if (input[j].target.theta != ANGLE_NOT_SET) {
      // Calculate the new point with known information: hypot and angle
      double angle_to_point = input[j].target.theta;
      int dir = input[j].drive_direction == REV ? -1 : 1;
      pose new_point = util::vector_off_point(LOOK_AHEAD * dir, {input[j].target.x, input[j].target.y, angle_to_point});
      new_point.theta = ANGLE_NOT_SET;

      input.insert(input.cbegin() + j + 1, {new_point, input[j].drive_direction, input[j].max_xy_speed});

      t++;
    }
  }

  // This is used for pid_wait_until_pp()
  injected_pp_index.clear();
  injected_pp_index.push_back(0);
  for (int i = 0; i < input.size(); i++) {
    if (i != 0 && input[i - 1].target.theta == ANGLE_NOT_SET)
      injected_pp_index.push_back(i);
  }

  if (print_toggle) printf("Pure Pursuit ");
  raw_pid_odom_pp_set(input, slew_on);
}

// Smooth injected pure pursuit
void Drive::pid_odom_injected_pp_set(std::vector<ez::odom> imovements, bool slew_on) {
  xyPID.timers_reset();
  aPID.timers_reset();

  if (print_toggle) printf("Injected ");
  std::vector<odom> input_path = inject_points(imovements);
  raw_pid_odom_pp_set(input_path, slew_on);
}

// Smooth injected pure pursuit
void Drive::pid_odom_smooth_pp_set(std::vector<odom> imovements, bool slew_on) {
  xyPID.timers_reset();
  aPID.timers_reset();

  if (print_toggle) printf("Smooth Injected ");
  std::vector<odom> input_path = smooth_path(inject_points(imovements), 0.75, 0.03, 0.0001);
  raw_pid_odom_pp_set(input_path, slew_on);
}

// Pose to Pose
void Drive::pid_odom_boomerang_set(odom imovement, bool slew_on) {
  if (print_toggle) printf("Boomerang ");
  pid_odom_pp_set({imovement}, slew_on);
}

void Drive::pid_odom_set(odom imovement, bool slew_on) {
  if (imovement.target.theta != ANGLE_NOT_SET)
    pid_odom_boomerang_set(imovement, slew_on);
  else
    pid_odom_injected_pp_set({imovement}, slew_on);
}

void Drive::pid_odom_set(std::vector<odom> imovements, bool slew_on) {
  pid_odom_smooth_pp_set(imovements, slew_on);
}