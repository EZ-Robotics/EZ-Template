/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/misc.hpp"

using namespace ez;

void Drive::ez_auto_task() {
  while (true) {
    // Run odom
    ez_tracking_task();

    // Autonomous PID
    switch (drive_mode_get()) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN ... TURN_TO_POINT:
        turn_pid_task();
        break;
      case SWING:
        swing_pid_task();
        break;
      case POINT_TO_POINT:
        ptp_task();
        break;
      case PURE_PURSUIT:
        pp_task();
        break;
      case DISABLE:
        break;
      default:
        break;
    }

    // This is used to reset sensors for active braking
    util::AUTON_RAN = drive_mode_get() != DISABLE ? true : false;

    pros::delay(ez::util::DELAY_TIME);
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
  double max_slew_out = fmax(slew_left.output(), slew_right.output());
  double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));
  if (faster_side > max_slew_out) {
    l_drive_out *= (max_slew_out / faster_side);
    r_drive_out *= (max_slew_out / faster_side);
  }

  // Toggle heading
  double imu_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + imu_out;
  double r_out = r_drive_out - imu_out;

  // Vector scaling when combining drive and imu
  max_slew_out = fmax(slew_left.output(), slew_right.output());
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID if it's a normal turn
  if (mode == TURN) {
    turnPID.compute(drive_imu_get());
  }
  // Compute PID if we're turning to point
  else {
    double a_target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_pose_get());  // Calculate the point for angle to face
    a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
    double error = a_target - odom_theta_get();
    turnPID.compute_error(error, odom_theta_get());
  }

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

// Odom To Point Task
void Drive::ptp_task() {
  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());
  double max_slew_out = fmax(slew_left.output(), slew_right.output());

  // Decide if we've past the target or not
  double temp_target = is_past_target(odom_target, odom_pose_get());        // Use this instead of distance formula to fix impossible movements
  int dir = (current_drive_direction == REV ? -1 : 1);                      // If we're going backwards, add a -1
  int flipped = util::sgn(temp_target) != util::sgn(past_target) ? -1 : 1;  // Check if we've flipped directions to what we started

  // Compute xy PID
  new_current_fake += xy_delta_fake * ((dir * flipped));  // Create a "current sensor value" for the PID to calculate off of
  xyPID.compute_error(fabs(temp_target) * dir * flipped, new_current_fake);

  // Compute angle
  pose ptf = point_to_face[!ptf1_running];
  double a_target = util::absolute_angle_to_point(ptf, odom_pose_get());  // Calculate the point for angle to face
  a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
  double wrapped_a_target = a_target - odom_theta_get();
  current_a_odomPID.compute_error(wrapped_a_target, odom_theta_get());
  // printf("shortest_a_target: %.2f      error: %.2f\n", a_target, wrapped_a_target);

  // Prioritize turning by scaling xy_out down
  double xy_out = xyPID.output;
  xy_out = util::clamp(xy_out, max_slew_out);
  // double scale = cos(util::to_rad(current_a_odomPID.error)) / odom_turn_bias_amount;
  double scale = 1.0 - ((1.0 - cos(util::to_rad(current_a_odomPID.error))) / odom_turn_bias_amount);  // 1 - ((1-0.7)/0.75)
  if (odom_turn_bias_enabled())
    xy_out *= scale;
  double a_out = current_a_odomPID.output;
  // a_out = util::clamp(a_out, max_slew_out);

  // Scale xy_out and a_out to max speed
  // this ensures no data is lost that would otherwise by lost in clamping
  double faster_side = fmax(fabs(xy_out), fabs(a_out));
  if (faster_side > max_slew_out) {
    xy_out *= (max_slew_out / faster_side);
    a_out *= (max_slew_out / faster_side);
  }

  // Combine heading and drive
  double l_out = xy_out + a_out;
  double r_out = xy_out - a_out;

  // Vector scaling when combining drive and imu
  // this ensures no data is lost that would otherwise by lost in clamping
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  // printf("lr out (%.2f, %.2f)   xy/a(%.2f, %.2f)   lr slew (%.2f, %.2f)\n", l_out, r_out, xy_out, a_out, slew_left.output(), slew_right.output());
  // printf("max_slew_out %.2f      headingerr: %.2f\n", max_slew_out, aPID.error);
  // printf("lr(%.2f, %.2f)   xy_raw: %.2f   xy_out: %.2f   heading_out: %.2f      max_slew_out: %.2f\n", l_out, r_out, xyPID.output, xy_out, current_a_odomPID.output, max_slew_out);
  // printf("xy(%.2f, %.2f, %.2f)   xyPID: %.2f   aPID: %.2f     dir: %i   sgn: %i   past_target: %i    is_past_target: %i   is_past_using_xy: %i      fake_xy(%.2f, %.2f, %.2f)\n", odom_x_get(), odom_y_get(), odom_theta_get(), xyPID.target_get(), current_a_odomPID.target_get(), dir, flipped, past_target, (int)is_past_target(odom_target, odom_pose_get()), is_past_target_using_xy, fake_x, fake_y, util::to_deg(fake_angle));
  // printf("xy(%.2f, %.2f, %.2f)   xyPID: %.2f   aPID: %.2f   ptf:(%.2f, %.2f)   xy/a(%.2f, %.2f)   lr(%.2f, %.2f)   fake xy: %.2f\n", odom_x_get(), odom_y_get(), odom_theta_get(), xyPID.error, current_a_odomPID.error, ptf.x, ptf.y, xy_out, a_out, l_out, r_out, new_current_fake);

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);

  // This is for wait_until
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());
}

void Drive::boomerang_task() {
  int target_index = pp_index;
  pose target = pp_movements[target_index].target;

  // target.theta += current_drive_direction == REV ? 180 : 0;  // Decide if going fwd or rev
  int dir = current_drive_direction == REV ? -1 : 1;

  double h = util::distance_to_point(target, odom_pose_get()) * odom_boomerang_dlead_get();
  double max = max_boomerang_distance;
  h = h > max ? max : h;
  h *= dir;

  pose temp = util::vector_off_point(-h, pp_movements[target_index].target);
  temp.theta = target.theta;

  if (util::distance_to_point(target, odom_pose_get()) < odom_look_ahead_get() / 2.0) {
    temp = target;
  }

  if (odom_target.x != temp.x || odom_target.y != temp.y) {
    bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;
    raw_pid_odom_ptp_set({temp, pp_movements[target_index].drive_direction, pp_movements[target_index].max_xy_speed}, slew_on);
  }

  // printf("cur(%.2f, %.2f, %.2f)   tar(%.2f, %.2f, %.2f)   h %.2f  \n", odom_x_get(), odom_y_get(), odom_theta_get(), temp.x, temp.y, temp.theta, h);

  ptp_task();
}

void Drive::pp_task() {
  if (fabs(util::distance_to_point(pp_movements[pp_index].target, odom_pose_get())) < odom_look_ahead_get()) {
    if (pp_index < pp_movements.size() - 1) {
      pp_index = pp_index >= pp_movements.size() - 1 ? pp_index : pp_index + 1;
      bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;
      if (!current_slew_on) slew_on = false;
      raw_pid_odom_ptp_set(pp_movements[pp_index], slew_on);
    }
  }

  if (pp_movements[pp_index].target.theta != ANGLE_NOT_SET) {
    boomerang_task();
  } else {
    ptp_task();
  }
}