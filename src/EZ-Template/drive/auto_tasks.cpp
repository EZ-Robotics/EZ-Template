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
    rightPID.Compute(right_sensor());
    headingPID.Compute(get_gyro());

    double l_drive_out = ez::util::clip_num(leftPID. output, slew_calculate(l, left_sensor ()), -slew_calculate(l, left_sensor ()));
    double r_drive_out = ez::util::clip_num(rightPID.output, slew_calculate(r, right_sensor()), -slew_calculate(r, right_sensor()));

    double gyro_out = heading_on ? headingPID.output : 0;

    double l_out = l_drive_out + gyro_out;
    double r_out = r_drive_out - gyro_out;

    set_tank(l_out, r_out);

    pros::delay(ez::util::DELAY_TIME);

  }
}

// Turn task
void Drive::turn_pid_task() {
  while (true) {

    turnPID.Compute(get_gyro());

    double gyro_out = ez::util::clip_num(turnPID.output, max_speed, -max_speed);

    set_tank(gyro_out, -gyro_out);

    pros::delay(ez::util::DELAY_TIME);

  }
}

// Swing task
void Drive::swing_pid_task() {
  while (true) {

    swingPID.Compute(get_gyro());

    double swing_out = ez::util::clip_num(swingPID.output, max_speed, -max_speed);

    if (current_swing == l_swing)
      set_tank(swing_out, 0);
    else if (current_swing == r_swing)
      set_tank(0, -swing_out);

    pros::delay(ez::util::DELAY_TIME);

  }
}
