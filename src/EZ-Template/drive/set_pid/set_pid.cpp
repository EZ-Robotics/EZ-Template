/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "okapi/api/units/QAngle.hpp"

// Updates max speed
void Drive::pid_speed_max_set(int speed) {
  max_speed = abs(util::clamp(speed, 127, -127));
  slew_left.speed_max_set(max_speed);
  slew_right.speed_max_set(max_speed);
  slew_turn.speed_max_set(max_speed);
  slew_swing.speed_max_set(max_speed);
}
int Drive::pid_speed_max_get() { return max_speed; }

// "turn bias" will bias either left or right, the user can decide
// the shortest path from 0.1 to 180 would be to go 179.9 degrees, but
// PID has some level of variance.  this allows the user to set a tolerance
// that will make the robot go left or right when it's within that tolerance
void Drive::pid_angle_behavior_bias_set(e_angle_behavior behavior) {
  if (behavior == ez::LEFT_TURN)
    turn_biased_left = true;
  else if (behavior == ez::RIGHT_TURN)
    turn_biased_left = false;
  else
    printf("Must input 'left' or 'right' for angle behavior bias!\n");
}
e_angle_behavior Drive::pid_angle_behavior_bias_get() { return turn_biased_left ? ez::LEFT_TURN : ez::RIGHT_TURN; }
void Drive::pid_angle_behavior_tolerance_set(double tolerance) { turn_tolerance = tolerance; }
void Drive::pid_angle_behavior_tolerance_set(okapi::QAngle p_tolerance) { pid_angle_behavior_tolerance_set(p_tolerance.convert(okapi::degree)); }
double Drive::pid_angle_behavior_tolerance_get() { return turn_tolerance; }

// Changes global default turn behavior to either:
//  - cw
//  - ccw
//  - shortest
//  - longest
//  - raw
void Drive::pid_angle_behavior_set(ez::e_angle_behavior behavior) {
  default_swing_type = behavior;
  default_turn_type = behavior;
  default_odom_type = behavior;
}

void Drive::pid_targets_reset() {
  headingPID.target_set(0);
  leftPID.target_set(0);
  rightPID.target_set(0);
  xyPID.target_set(0);
  current_a_odomPID.target_set(0);
  forward_drivePID.target_set(0);
  backward_drivePID.target_set(0);
  turnPID.target_set(0);
  swingPID.target_set(0);
  forward_swingPID.target_set(0);
  backward_swingPID.target_set(0);
}

void Drive::drive_mode_set(e_mode p_mode, bool stop_drive) {
  mode = p_mode;
  if (mode == DISABLE && stop_drive)
    private_drive_set(0, 0);
}
e_mode Drive::drive_mode_get() { return mode; }

// Toggle drive motors but still allow PID to run
void Drive::pid_drive_toggle(bool toggle) { drive_toggle = toggle; }
bool Drive::pid_drive_toggle_get() { return drive_toggle; }

// Don't print stuff
void Drive::pid_print_toggle(bool toggle) { print_toggle = toggle; }
bool Drive::pid_print_toggle_get() { return print_toggle; }
