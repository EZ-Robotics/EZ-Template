/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/util.hpp"
#include "main.h"
#include "okapi/api/units/QAngle.hpp"

/////
// Set constants
/////
void Drive::pid_odom_behavior_set(ez::e_angle_behavior behavior) { default_odom_type = behavior; }
ez::e_angle_behavior Drive::pid_odom_behavior_get() { return default_odom_type; }
// Flip all inputs so it works internally
pose Drive::flip_pose(pose input) {
  int flip_x = x_flipped ? -1 : 1;
  int flip_y = y_flipped ? -1 : 1;

  pose new_pose = input;
  new_pose.x *= flip_x;
  new_pose.y *= flip_y;

  return new_pose;
}
std::vector<odom> Drive::set_odoms_direction(std::vector<odom> inputs) {
  std::vector<odom> output;

  for (int i = 0; i < inputs.size(); i++) {
    pose new_pose = flip_pose(inputs[i].target);
    output.push_back({new_pose,
                      inputs[i].drive_direction,
                      inputs[i].max_xy_speed,
                      inputs[i].turn_behavior});
  }

  return output;
}
odom Drive::set_odom_direction(odom input) {
  return set_odoms_direction({input})[0];
}
void Drive::odom_x_direction_flip(bool flip) { x_flipped = flip; }
bool Drive::odom_x_direction_get() { return x_flipped; }
void Drive::odom_y_direction_flip(bool flip) { y_flipped = flip; }
bool Drive::odom_y_direction_get() { return y_flipped; }

/////
// pid_odom_set
/////
// No units
void Drive::pid_odom_set(odom imovement) {
  bool slew_on = imovement.drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_set(imovement, slew_on);
}
void Drive::pid_odom_set(odom imovement, bool slew_on) {
  if (imovement.target.theta != ANGLE_NOT_SET)
    pid_odom_boomerang_set(imovement, slew_on);
  else
    pid_odom_injected_pp_set({imovement}, slew_on);
}
void Drive::pid_odom_set(std::vector<odom> imovements) {
  bool slew_on = imovements[0].drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_set(imovements, slew_on);
}
void Drive::pid_odom_set(std::vector<odom> imovements, bool slew_on) {
  pid_odom_smooth_pp_set(imovements, slew_on);
}
// Units
void Drive::pid_odom_set(united_odom p_imovement) {
  odom imovement = util::united_odom_to_odom(p_imovement);
  pid_odom_set(imovement);
}
void Drive::pid_odom_set(united_odom p_imovement, bool slew_on) {
  odom imovement = util::united_odom_to_odom(p_imovement);
  pid_odom_set(imovement, slew_on);
}
void Drive::pid_odom_set(std::vector<united_odom> p_imovements) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_set(imovements);
}
void Drive::pid_odom_set(std::vector<united_odom> p_imovements, bool slew_on) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_set(imovements, slew_on);
}

/////
// ptp
/////
// No units
void Drive::pid_odom_ptp_set(odom imovement) {
  bool slew_on = imovement.drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_ptp_set(imovement, slew_on);
}
// Units
void Drive::pid_odom_ptp_set(united_odom p_imovement) {
  odom imovement = util::united_odom_to_odom(p_imovement);
  pid_odom_ptp_set(imovement);
}
void Drive::pid_odom_ptp_set(united_odom p_imovement, bool slew_on) {
  odom imovement = util::united_odom_to_odom(p_imovement);
  pid_odom_ptp_set(imovement, slew_on);
}

/////
// pp
/////
// No units
void Drive::pid_odom_pp_set(std::vector<odom> imovements) {
  bool slew_on = imovements[0].drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_pp_set(imovements, slew_on);
}
// Units
void Drive::pid_odom_pp_set(std::vector<united_odom> p_imovements) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_pp_set(imovements);
}
void Drive::pid_odom_pp_set(std::vector<united_odom> p_imovements, bool slew_on) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_pp_set(imovements, slew_on);
}

/////
// injected pp
/////
// No units
void Drive::pid_odom_injected_pp_set(std::vector<ez::odom> imovements) {
  bool slew_on = imovements[0].drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_injected_pp_set(imovements, slew_on);
}
void Drive::pid_odom_injected_pp_set(std::vector<ez::odom> imovements, bool slew_on) {
  xyPID.timers_reset();
  aPID.timers_reset();

  if (print_toggle) printf("Injected ");
  std::vector<odom> input_path = inject_points(set_odoms_direction(imovements));
  raw_pid_odom_pp_set(input_path, slew_on);
}
// Units
void Drive::pid_odom_injected_pp_set(std::vector<ez::united_odom> p_imovements) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_injected_pp_set(imovements);
}
void Drive::pid_odom_injected_pp_set(std::vector<ez::united_odom> p_imovements, bool slew_on) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_injected_pp_set(imovements, slew_on);
}

/////
// smooth injected pp
/////
// No units
void Drive::pid_odom_smooth_pp_set(std::vector<odom> imovements) {
  bool slew_on = imovements[0].drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_smooth_pp_set(imovements, slew_on);
}
void Drive::pid_odom_smooth_pp_set(std::vector<odom> imovements, bool slew_on) {
  xyPID.timers_reset();
  aPID.timers_reset();

  if (print_toggle) printf("Smooth Injected ");
  std::vector<odom> input_path = smooth_path(inject_points(set_odoms_direction(imovements)), 0.75, 0.03, 0.0001);
  raw_pid_odom_pp_set(input_path, slew_on);
}
// Units
void Drive::pid_odom_smooth_pp_set(std::vector<united_odom> p_imovements) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_smooth_pp_set(imovements);
}
void Drive::pid_odom_smooth_pp_set(std::vector<united_odom> p_imovements, bool slew_on) {
  std::vector<odom> imovements = util::united_odoms_to_odoms(p_imovements);
  pid_odom_smooth_pp_set(imovements, slew_on);
}

/////
// boomerang
/////
// No units
void Drive::pid_odom_boomerang_set(odom imovement) {
  bool slew_on = imovement.drive_direction == fwd ? slew_drive_forward_get() : slew_drive_backward_get();
  pid_odom_boomerang_set(imovement, slew_on);
}
void Drive::pid_odom_boomerang_set(odom imovement, bool slew_on) {
  if (print_toggle) printf("Boomerang ");
  pid_odom_pp_set({imovement}, slew_on);
}
// Units
void Drive::pid_odom_boomerang_set(united_odom p_imovement) {
  odom imovement = util::united_odom_to_odom(p_imovement);
  pid_odom_boomerang_set(imovement);
}
void Drive::pid_odom_boomerang_set(united_odom p_imovement, bool slew_on) {
  odom imovement = util::united_odom_to_odom(p_imovement);
  pid_odom_boomerang_set(imovement, slew_on);
}

/////
// External base pure pursuit
/////
void Drive::pid_odom_pp_set(std::vector<odom> imovements, bool slew_on) {
  xyPID.timers_reset();
  aPID.timers_reset();

  std::vector<odom> input = set_odoms_direction(imovements);
  input.insert(input.begin(), {{{odom_current.x, odom_current.y, ANGLE_NOT_SET}, imovements[0].drive_direction, imovements[0].max_xy_speed}});

  int t = 0;
  for (int i = 0; i < input.size() - 1; i++) {
    // Inject new parent points for boomerang
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

  // Shift all the turn behaviors 1 parent point down
  for (int i = 0; i < input.size() - 1; i++) {
    input[i].turn_behavior = input[i + 1].turn_behavior;
  }
  input.back().turn_behavior = raw;

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

//////
// External base ptp
/////
void Drive::pid_odom_ptp_set(odom imovement, bool slew_on) {
  imovement = set_odom_direction(imovement);

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

/////
// Base pure pursuit
/////
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

/////
// Base point to point
/////
void Drive::raw_pid_odom_ptp_set(odom imovement, bool slew_on) {
  // Update current drive/turn behavior
  current_drive_direction = imovement.drive_direction;

  // Calculate the point to look at
  point_to_face = find_point_to_face(odom_current, {imovement.target.x, imovement.target.y}, current_drive_direction, true);
  double target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_current);  // Calculate the point for angle to face
  if (imovement.turn_behavior != raw) {
    odom_imu_start = drive_imu_get();
    current_angle_behavior = imovement.turn_behavior;
  }

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