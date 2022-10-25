/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "pros/misc.hpp"

using namespace ez;

void Drive::ez_auto_task() {
  while (true) {
    // Autonomous PID
    if (mode_get() == DRIVE)
      pid_drive_task();
    else if (mode_get() == TURN)
      pid_turn_task();
    else if (mode_get() == SWING)
      pid_swing_task();

    util::AUTON_RAN = mode_get() != DISABLE ? true : false;

    /*
    if (pros::competition::is_autonomous() && !util::AUTON_RAN)
      util::AUTON_RAN = true;
    else if (!pros::competition::is_autonomous())
      mode_set(DISABLE);
    */

    pros::delay(util::DELAY_TIME);
  }
}

// Drive PID task
void Drive::pid_drive_task() {
  // Compute PID
  pid_left.compute(sensor_left());
  pid_right.compute(sensor_right());

  pid_heading.compute(imu_get());

  // Compute slew
  double l_slew_out = slew_calculate(slew_left, sensor_left());
  double r_slew_out = slew_calculate(slew_right, sensor_right());

  // Clip pid_left and pid_right to slew (if slew is disabled, it returns speed_max)
  double l_drive_out = util::clamp_number(pid_left.output, l_slew_out, -l_slew_out);
  double r_drive_out = util::clamp_number(pid_right.output, r_slew_out, -r_slew_out);

  // Toggle heading
  double gyro_out = heading_on ? pid_heading.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + gyro_out;
  double r_out = r_drive_out - gyro_out;

  // Set motors
  if (drive_toggle)
    tank_set_private(l_out, r_out);
}

// Turn PID task
void Drive::pid_turn_task() {
  // Compute PID
  pid_turn.compute(imu_get());

  // Clip gyroPID to max speed
  double gyro_out = util::clamp_number(pid_turn.output, speed_max, -speed_max);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (pid_turn.constants.ki != 0 && (fabs(pid_turn.target_get()) > pid_turn.constants.start_i && fabs(pid_turn.error) < pid_turn.constants.start_i)) {
    if (turn_min_get() != 0)
      gyro_out = util::clamp_number(gyro_out, turn_min_get(), -turn_min_get());
  }

  // Set motors
  if (drive_toggle)
    tank_set_private(gyro_out, -gyro_out);
}

// Swing PID task
void Drive::pid_swing_task() {
  // Compute PID
  pid_swing.compute(imu_get());

  // Clip pid_swing to max speed
  double swing_out = util::clamp_number(pid_swing.output, speed_max, -speed_max);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (pid_swing.constants.ki != 0 && (fabs(pid_swing.target_get()) > pid_swing.constants.start_i && fabs(pid_swing.error) < pid_swing.constants.start_i)) {
    if (swing_min_get() != 0)
      swing_out = util::clamp_number(swing_out, swing_min_get(), -swing_min_get());
  }

  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (swing_current == LEFT_SWING)
      tank_set_private(swing_out, 0);
    else if (swing_current == RIGHT_SWING)
      tank_set_private(0, -swing_out);
  }
}
