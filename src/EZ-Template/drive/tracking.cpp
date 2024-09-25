/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

// Sets and gets
void Drive::odom_x_set(double x) { odom_current.x = x; }
void Drive::odom_y_set(double y) { odom_current.y = y; }
void Drive::odom_theta_set(double a) { drive_angle_set(a); }
void Drive::odom_x_set(okapi::QLength p_x) { odom_x_set(p_x.convert(okapi::inch)); }
void Drive::odom_y_set(okapi::QLength p_y) { odom_y_set(p_y.convert(okapi::inch)); }
void Drive::odom_theta_set(okapi::QAngle p_a) { odom_theta_set(p_a.convert(okapi::degree)); }
void Drive::odom_reset() { odom_pose_set({0, 0, 0}); }
void Drive::drive_width_set(double input) { track_width = input; }
void Drive::drive_width_set(okapi::QLength p_input) { drive_width_set(p_input.convert(okapi::inch)); }
double Drive::drive_width_get() { return track_width; }
void Drive::odom_enable(bool input) { odometry_enabled = input; }
bool Drive::odom_enabled() { return odometry_enabled; }
void Drive::odom_pose_set(united_pose itarget) { odom_pose_set(util::united_pose_to_pose(itarget)); }
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
    t_last = 0;
    return;
  }

  // Figure out what sensor to use for the vertical tracking
  float v_current = 0.0;
  float v_track_width = 0.0;
  if (odom_right_tracker_enabled) {
    v_current = odom_right_tracker->get();
    v_track_width = odom_right_tracker->distance_to_center_get();
  } else if (odom_left_tracker_enabled) {
    v_current = odom_left_tracker->get();
    v_track_width = odom_left_tracker->distance_to_center_get();
  } else {
    // Use left side IME
    if (odom_ime_use_left) {
      v_current = drive_sensor_left();
      v_track_width = odom_ime_track_width_left;
    }
    // Use right side IME
    else {
      v_current = drive_sensor_right();
      v_track_width = odom_ime_track_width_right;
    }
  }

  // Figure out what sensor to use for horizontal tracking
  float h_current = 0.0;
  float h_track_width = default_center_distance;
  if (odom_back_tracker_enabled) {
    h_current = odom_back_tracker->get();
    h_track_width = odom_back_tracker->distance_to_center_get();
  } else if (odom_front_tracker_enabled) {
    h_current = odom_front_tracker->get();
    h_track_width = odom_front_tracker->distance_to_center_get();
  }

  // Vertical sensor and velocity
  float v_ = v_current - v_last;
  v_last = v_current;

  // Horizontal sensor and velocity
  float h_ = h_current - h_last;
  h_last = h_current;

  // Angle and velocity
  float t_current = -ez::util::to_rad(drive_imu_get());
  float t_ = t_current - t_last;
  t_last = t_current;

  // Figure out how far we've actually moved
  float local_y = h_;
  float local_x = v_;
  if (t_ != 0) {
    /*
    float v_radius = v_ / t_;
    beta = t_ / 2.0;
    local_y = ((v_radius + vertical_track_width) * sin(beta)) * 2.0;
    float h_radius = h_ / t_;
    local_x = ((h_radius + horizontal_track_width) * sin(beta)) * 2.0;
    */

    double i = sin(t_ / 2.0) * 2.0;
    local_x = (v_ / t_ - v_track_width) * i;
    local_y = (h_ / t_ + h_track_width) * i;
  }

  float alpha = angle_rad - t_ / 2.0;

  float x = 0.0, y = 0.0;

  x += cos(alpha) * local_x - sin(alpha) * local_y;
  y += sin(alpha) * local_x + cos(alpha) * local_y;

  // odom_current.x += x;
  // odom_current.y += y;
  odom_current.x -= y;
  odom_current.y += x;
  angle_rad = t_current;
  odom_current.theta = drive_imu_get();

  /*
  x += local_x * cos(alpha);
  x += local_y * sin(alpha);
  y += local_y * cos(alpha);
  y += local_x * -sin(alpha);
  */

  // LemLib localizer
  /*
  x += local_x * -cos(alpha);
  x += local_y * sin(alpha);
  y += local_y * cos(alpha);
  y += local_x * sin(alpha);
  */
}