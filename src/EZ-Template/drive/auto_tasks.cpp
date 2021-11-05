/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Drive task
void Drive::drive_pid_task() {
  while (true) {

    leftPID. Compute(left_sensor());
    rightPID.Compute(left_sensor());
    headingPID.Compute(get_gyro());

    double l_drive_out = ez::util::clip_num(leftPID. output, max_speed, -max_speed);
    double r_drive_out = ez::util::clip_num(rightPID.output, max_speed, -max_speed);

    double l_out = l_drive_out + headingPID.output;
    double r_out = r_drive_out - headingPID.output;

    set_tank(l_out, r_out);

    pros::delay(ez::util::DELAY_TIME);
  }
}

// Turn task
void Drive::turn_pid_task() {
  while (true) {

    pros::delay(ez::util::DELAY_TIME);
  }
}

// Swing task
void Drive::swing_pid_task() {
  while (true) {

    pros::delay(ez::util::DELAY_TIME);
  }
}
