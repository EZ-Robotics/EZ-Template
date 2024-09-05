/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

void Drive::odom_pose_x_set(double x) { odom_current.x = x; }
void Drive::odom_pose_y_set(double y) { odom_current.y = y; }
void Drive::odom_pose_theta_set(double a) { drive_angle_set(a); }
void Drive::odom_reset() { odom_pose_set({0, 0, 0}); }
void Drive::drive_width_set(double input) { track_width = input; }
double Drive::drive_width_get() { return track_width; }
void Drive::drive_odom_enable(bool input) { odometry_enabled = input; }
void Drive::odom_pose_set(pose itarget) {
  odom_pose_theta_set(itarget.theta);
  odom_pose_x_set(itarget.x);
  odom_pose_y_set(itarget.y);
}

// Tracking based on https://wiki.purduesigbots.com/software/odometry
void Drive::ez_tracking_task() {
  if (!imu_calibration_complete || !odometry_enabled) {
    l_last = 0;
    r_last = 0;
    last_theta = 0;
    return;
  }
  // pros::delay(3000);
  // while (true) {
  // printf("x: %.2f   y: %.2f   a: %.2f\n", odom_current.x, odom_current.y, odom_current.theta);

  // float l_current = drive_sensor_left();
  float r_current = drive_sensor_right();
  // c_current = get_raw_center();

  // float l_ = l_current - l_last;
  float r_ = r_current - r_last;
  // float c = c_current - c_last;

  // l_last = l_current;
  r_last = r_current;
  // c_last = c_current;

  // diff between wheels for correcting turning
  float current_global_theta = ez::util::to_rad(drive_imu_get());
  float theta = current_global_theta - last_theta;  // imu theta
  last_theta = current_global_theta;

  float beta = 0.0;
  float h = r_;
  if (theta != 0) {
    float radius_r = r_ / theta;
    beta = theta / 2.0;
    h = ((radius_r + (track_width / 2.0)) * sin(beta)) * 2.0;
    // radius_c = c / theta;
    // h2 = (radius_c + CENTER_OFFSET) * 2.0 * sin(beta);
  }  // ///else {
     // ///h = l_;
     // ///beta = 0;
     // h2 = 0; // Used for perp wheel
  // ///}

  float alpha = angle_rad + beta;

  // Xx = h2 * cos(alpha);
  // Xy = h2 * -sin(alpha);
  float Yx = h * sin(alpha);
  float Yy = h * cos(alpha);

  odom_current.x += (/*Xx +*/ Yx);
  odom_current.y += (/*Xy +*/ Yy);
  angle_rad = current_global_theta;
  odom_current.theta = drive_imu_get();  // drive_imu_get();

  // pros::delay(1);
  // }
}