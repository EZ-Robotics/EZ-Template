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

    //printf("%f   %f\n", leftPID.output, rightPID.output);

    set_tank(leftPID.output, rightPID.output);
    //set_tank(127, 127);

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
