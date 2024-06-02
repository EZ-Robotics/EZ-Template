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
    switch (drive_mode_get()) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN:
        turn_pid_task();
        break;
      case SWING:
        swing_pid_task();
        break;
      case DISABLE:
        break;
      default:
        break;
    }

    util::AUTON_RAN = drive_mode_get() != DISABLE ? true : false;

    pros::delay(util::DELAY_TIME);
  }
}

// Drive PID task
void Drive::drive_pid_task() {
  // Compute PID
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  headingPID.compute(drive_imu_get());

  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());

  // Left and Right outputs
  double l_drive_out = leftPID.output;
  double r_drive_out = rightPID.output;

  // Scale leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double max_slew_out = fmin(slew_left.output(), slew_right.output());
  double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));
  if (faster_side > max_slew_out) {
    l_drive_out = l_drive_out * (max_slew_out / faster_side);
    r_drive_out = r_drive_out * (max_slew_out / faster_side);
  }

  // Toggle heading
  double imu_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + imu_out;
  double r_out = r_drive_out - imu_out;

  // Vector scaling when combining drive and imo
  max_slew_out = fmin(slew_left.output(), slew_right.output());
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out = l_out * (max_slew_out / faster_side);
    r_out = r_out * (max_slew_out / faster_side);
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID
  turnPID.compute(drive_imu_get());

  // Compute slew
  slew_turn.iterate(drive_imu_get());

  // Clip gyroPID to max speed
  double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.target_get()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (pid_turn_min_get() != 0)
      gyro_out = util::clamp(gyro_out, pid_turn_min_get(), -pid_turn_min_get());
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(gyro_out, -gyro_out);
}

// Swing PID task
void Drive::swing_pid_task() {
  // Compute PID
  swingPID.compute(drive_imu_get());
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  // Compute slew
  double current = slew_swing_using_angle ? drive_imu_get() : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  slew_swing.iterate(current);

  // Clip swingPID to max speed
  double swing_out = util::clamp(swingPID.output, slew_swing.output(), -slew_swing.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (swingPID.constants.ki != 0 && (fabs(swingPID.target_get()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (pid_swing_min_get() != 0)
      swing_out = util::clamp(swing_out, pid_swing_min_get(), -pid_swing_min_get());
  }

  // Set the motors powers, and decide what to do with the "still" side of the drive
  double opposite_output = 0;
  double scale = swing_out / max_speed;
  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (current_swing == LEFT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? rightPID.output : (swing_opposite_speed * scale);
      private_drive_set(swing_out, opposite_output);
    } else if (current_swing == RIGHT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? leftPID.output : -(swing_opposite_speed * scale);
      private_drive_set(opposite_output, -swing_out);
    }
  }
}