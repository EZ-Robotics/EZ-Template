/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


// Updates max speed
void Drive::set_max_speed(int speed) {
  max_speed = abs(speed);
}

// Set drive PID
void Drive::set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading) {
  // Disable tasks
  turn_pid.suspend();
  swing_pid.suspend();

  TICK_PER_INCH = get_tick_per_inch();

  // Print targets
  printf("Drive Started... Target Value: %f (%f ticks)", target, target*TICK_PER_INCH);
  if (slew_on) printf(" with slew");
  printf("\n");

  // Global setup
  set_max_speed(speed);
  heading_on = toggle_heading;
  bool is_backwards = false;
  l_start = left_sensor();
  r_start = right_sensor();

  double l_target_encoder, r_target_encoder;

  // Figure actual target value
  l_target_encoder = l_start + (target*TICK_PER_INCH);
  r_target_encoder = r_start + (target*TICK_PER_INCH);

  // Figure out if going forward or backward
  if (l_target_encoder<l_start && r_target_encoder<r_start) {
    auto consts = backwardDrivePID.GetConstants();
    leftPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
    rightPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
    is_backwards = true;
  } else {
    auto consts = forwardDrivePID.GetConstants();
    leftPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
    rightPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
    is_backwards = false;
  }

  // Set PID targets
  leftPID. SetTarget(l_target_encoder);
  rightPID.SetTarget(r_target_encoder);

  // Initialize slew
  slew_initialize(left_slew,  slew_on, max_speed, l_target_encoder, left_sensor (), l_start, is_backwards);
  slew_initialize(right_slew, slew_on, max_speed, r_target_encoder, right_sensor(), r_start, is_backwards);

  // Run task
  drive_pid.resume();
}

// Set turn PID
void Drive::set_turn_pid(double target, int speed) {
  // Disable tasks
  swing_pid.suspend();
  drive_pid.suspend();

  // Print targets
  printf("Turn Started... Target Value: %f\n", target);

  // Set PID targets
  turnPID.SetTarget(target);
  headingPID.SetTarget(target); // Update heading target for next drive motion
  set_max_speed(speed);

  // Run task
  turn_pid.resume();
}

// Set swing PID
void Drive::set_swing_pid(e_swing type, double target, int speed) {
  // Disable tasks
  drive_pid.suspend();
  turn_pid.suspend();

  // Print targets
  printf("Swing Started... Target Value: %f\n", target);
  current_swing = type;

  // Set PID targets
  swingPID.SetTarget(target);
  headingPID.SetTarget(target); // Update heading target for next drive motion
  set_max_speed(speed);

  // Run task
  swing_pid.resume();
}
