/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

// Sets and gets
void Drive::odom_x_set(double x) { odom_current.x = x; }
void Drive::odom_y_set(double y) { odom_current.y = y; }
void Drive::odom_theta_set(double a) { drive_angle_set(a); }
void Drive::odom_reset() { odom_pose_set({0, 0, 0}); }
void Drive::drive_width_set(double input) { track_width = input; }
double Drive::drive_width_get() { return track_width; }
void Drive::drive_odom_enable(bool input) { odometry_enabled = input; }
void Drive::odom_pose_set(pose itarget) {
  odom_theta_set(itarget.theta);
  odom_x_set(itarget.x);
  odom_y_set(itarget.y);
}

double Drive::odom_x_get() { return odom_current.x; }
double Drive::odom_y_get() { return odom_current.y; }
double Drive::odom_theta_get() { return odom_current.theta; }
pose Drive::odom_pose_get() { return odom_current; }

// Tracking based on https://wiki.purduesigbots.com/software/odometry
void Drive::ez_tracking_task() {
  // Don't let this function run if odom is disabled
  // and make sure all the "lasts" are 0
  if (!imu_calibration_complete || !odometry_enabled) {
    v_last = 0;
    h_last = 0;
    last_theta = 0;
    return;
  }

  // Figure out what sensor to use for the vertical tracking
  float vertical_current = 0.0;
  float vertical_track_width = 0.0;
  if (odom_right_tracker_enabled) {
    vertical_current = odom_right_tracker->get();
    vertical_track_width = odom_right_tracker->distance_to_center_get();
  } else if (odom_left_tracker_enabled) {
    vertical_current = odom_left_tracker->get();
    vertical_track_width = odom_left_tracker->distance_to_center_get();
  } else {
    vertical_current = drive_sensor_right();
    vertical_track_width = track_width / 2.0;
  }

  // Figure out what sensor to use for horizontal tracking
  float horizontal_current = 0.0;
  float horizontal_track_width = 0.0;
  if (odom_back_tracker_enabled) {
    horizontal_current = odom_back_tracker->get();
    horizontal_track_width = odom_back_tracker->distance_to_center_get();
  } else if (odom_front_tracker_enabled) {
    horizontal_current = odom_front_tracker->get();
    horizontal_track_width = odom_front_tracker->distance_to_center_get();
  }

  // Vertical sensor and velocity
  float v_ = vertical_current - v_last;
  v_last = vertical_current;

  // Horizontal sensor and velocity
  float c_ = horizontal_current - h_last;
  h_last = horizontal_current;

  // Angle and velocity
  float current_global_theta = ez::util::to_rad(drive_imu_get());
  float theta = current_global_theta - last_theta;
  last_theta = current_global_theta;

  // Figure out how far we've actually moved
  float beta = 0.0;
  float local_y = v_;
  float local_x = c_;
  if (theta != 0) {
    float v_radius = v_ / theta;
    beta = theta / 2.0;
    local_y = ((v_radius + vertical_track_width) * sin(beta)) * 2.0;
    float h_radius = c_ / theta;
    local_x = ((h_radius + horizontal_track_width) * sin(beta)) * 2.0;
  }

  float alpha = angle_rad + beta;

  float x = local_x * cos(alpha);
  x += local_y * sin(alpha);
  float y = local_x * -sin(alpha);
  y += local_y * cos(alpha);

  odom_current.x += x;
  odom_current.y += y;
  angle_rad = current_global_theta;
  odom_current.theta = drive_imu_get();
}