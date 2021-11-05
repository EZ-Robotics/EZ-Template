/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


// Drive PID task
void Drive::drive_pid_task() {
  while (true) {

    // Compute PID
    leftPID. Compute(left_sensor());
    rightPID.Compute(right_sensor());
    headingPID.Compute(get_gyro());

    // Compute slew
    double l_slew_out = slew_calculate(l, left_sensor ());
    double r_slew_out = slew_calculate(l, right_sensor());

    // Clip leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
    double l_drive_out = ez::util::clip_num(leftPID. output, l_slew_out, -l_slew_out);
    double r_drive_out = ez::util::clip_num(rightPID.output, r_slew_out, -r_slew_out);

    // Toggle heading
    double gyro_out = heading_on ? headingPID.output : 0;

    // Combine heading and drive
    double l_out = l_drive_out + gyro_out;
    double r_out = r_drive_out - gyro_out;

    // Set motors
    set_tank(l_out, r_out);

    pros::delay(ez::util::DELAY_TIME);

  }
}

// Turn PID task
void Drive::turn_pid_task() {
  while (true) {

    // Compute PID
    turnPID.Compute(get_gyro());

    // Clip gyroPID to max speed
    double gyro_out = ez::util::clip_num(turnPID.output, max_speed, -max_speed);

    // Set motors
    set_tank(gyro_out, -gyro_out);

    pros::delay(ez::util::DELAY_TIME);

  }
}

// Swing PID task
void Drive::swing_pid_task() {
  while (true) {

    // Compute PID
    swingPID.Compute(get_gyro());

    // Clip swingPID to max speed
    double swing_out = ez::util::clip_num(swingPID.output, max_speed, -max_speed);

    // Check if left or right swing, then set motors accordingly
    if (current_swing == l_swing)
      set_tank(swing_out, 0);
    else if (current_swing == r_swing)
      set_tank(0, -swing_out);

    pros::delay(ez::util::DELAY_TIME);

  }
}
