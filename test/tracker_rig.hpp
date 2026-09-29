// Tracker odometry rig, shared by the tracking-wheel odometry tests.
//
// Moves a simulated tank robot through a series of small constant-twist steps and feeds
// the fake IMU, fake tracking wheels and fake drive motor encoders exactly what real ones
// would read, calling ez_tracking_task() after every step.  The robot's true pose is
// integrated alongside, so a test can compare the pose EZ-Template reports with the truth.
//
// Frames: x is to the robot's right at heading 0, y is forward, heading is degrees clockwise
// from +y (the same convention as the library's odom pose).
//
// Sensor conventions (the library's):
//   * vertical tracker / drive encoder: positive = forward
//   * horizontal tracker: positive = the robot moved LEFT
// A tracker's true offset is a distance from the turning center, always positive; a left
// vertical tracker sits at -offset (to the left), a right one at +offset, a front horizontal
// tracker at +offset (ahead), a back one at -offset (behind).  The offset the library is told
// (the constructor argument) is kept apart from the true one so mis-set offsets can be tested.
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>

#include "drive_test_access.hpp"

namespace ez::tracker_rig {

struct Cfg {
  // True offsets in inches; an empty optional means that tracker does not exist.
  std::optional<double> left, right, horiz;
  // The offset passed to each tracker's constructor.  Empty means "the true one".
  std::optional<double> set_left, set_right, set_horiz;
  bool horiz_front = false;       // false: back tracker, true: front tracker
  bool horiz_wired_left = true;   // true: reads positive when the robot moves left
  double drive_width = 12.0;      // physical distance between the drive wheels
  bool tell_drive_width = false;  // call drive_width_set(drive_width) like a team with IMEs would
  double start_x = 0.0, start_y = 0.0, start_heading = 0.0;
};

struct Rig {
  Drive chassis;
  std::unique_ptr<tracking_wheel> tl, tr, th;
  Cfg cfg;
  double acc_l = 0, acc_r = 0, acc_h = 0;    // inches each tracker has travelled
  double acc_iml = 0, acc_imr = 0;           // inches each drive side has travelled
  double tx, ty, tth;                        // true pose

  static Drive make_chassis() {
    test_stub::reset_all();
    return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
  }

  explicit Rig(const Cfg& c) : chassis(make_chassis()), cfg(c) {
    if (cfg.left) {
      tl = std::make_unique<tracking_wheel>(11, 2.75, cfg.set_left.value_or(*cfg.left));
      chassis.odom_tracker_left_set(tl.get());
    }
    if (cfg.right) {
      tr = std::make_unique<tracking_wheel>(12, 2.75, cfg.set_right.value_or(*cfg.right));
      chassis.odom_tracker_right_set(tr.get());
    }
    if (cfg.horiz) {
      th = std::make_unique<tracking_wheel>(13, 2.75, cfg.set_horiz.value_or(*cfg.horiz));
      if (cfg.horiz_front)
        chassis.odom_tracker_front_set(th.get());
      else
        chassis.odom_tracker_back_set(th.get());
    }
    if (cfg.tell_drive_width) chassis.drive_width_set(cfg.drive_width);

    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.odom_xyt_set(cfg.start_x, cfg.start_y, cfg.start_heading);
    chassis.drive_sensor_reset();  // primes every "last" value to the readings as they stand
    tx = cfg.start_x;
    ty = cfg.start_y;
    tth = cfg.start_heading;
  }

  // Moves the robot's center ds inches forward and dr inches to the right while turning dth_deg
  // clockwise, as one constant-twist step, then runs one tracking pass.
  void move(double ds, double dr, double dth_deg) {
    const double dth = dth_deg * M_PI / 180.0;
    const double half_w = cfg.drive_width / 2.0;

    acc_iml += ds + dth * half_w;
    acc_imr += ds - dth * half_w;
    if (cfg.left) acc_l += ds + dth * *cfg.left;
    if (cfg.right) acc_r += ds - dth * *cfg.right;
    if (cfg.horiz) {
      const double f = cfg.horiz_front ? *cfg.horiz : -*cfg.horiz;  // forward position of the tracker
      acc_h += -dr - dth * f;
    }

    const double half = dth / 2.0;
    const double sinc = half == 0.0 ? 1.0 : std::sin(half) / half;
    const double mid = (tth + dth_deg / 2.0) * M_PI / 180.0;
    tx += sinc * (ds * std::sin(mid) + dr * std::cos(mid));
    ty += sinc * (ds * std::cos(mid) - dr * std::sin(mid));
    tth += dth_deg;

    push();
    chassis.ez_tracking_task();
  }

  void push() {
    (*DriveTestAccess::all_imus(chassis).begin())->fake_rotation = tth;
    if (tl) tl->smart_encoder.fake_position = (std::int32_t)std::llround(acc_l * tl->ticks_per_inch());
    if (tr) tr->smart_encoder.fake_position = (std::int32_t)std::llround(acc_r * tr->ticks_per_inch());
    if (th) {
      const double reading = cfg.horiz_wired_left ? acc_h : -acc_h;
      th->smart_encoder.fake_position = (std::int32_t)std::llround(reading * th->ticks_per_inch());
    }
    const double tpi = chassis.drive_tick_per_inch();
    chassis.left_motors[0].fake().position = (std::int32_t)std::llround(acc_iml * tpi);
    chassis.right_motors[0].fake().position = (std::int32_t)std::llround(acc_imr * tpi);
  }

  static int steps_for(double amount, double per_step) { return std::max(1, (int)std::ceil(std::fabs(amount) / per_step)); }

  // Point turn, clockwise positive.
  void turn(double deg) {
    const int n = steps_for(deg, 0.25);
    for (int i = 0; i < n; i++) move(0.0, 0.0, deg / n);
  }
  // Straight drive, forward positive.
  void drive(double inches) {
    const int n = steps_for(inches, 0.1);
    for (int i = 0; i < n; i++) move(inches / n, 0.0, 0.0);
  }
  // Sideways slide, to the right positive, the robot's heading unchanged.
  void slide(double right_inches) {
    const int n = steps_for(right_inches, 0.1);
    for (int i = 0; i < n; i++) move(0.0, right_inches / n, 0.0);
  }
  // The center travels `inches` along a circular arc while the heading changes by `deg`.
  void arc(double inches, double deg) {
    const int n = std::max(steps_for(inches, 0.1), steps_for(deg, 0.25));
    for (int i = 0; i < n; i++) move(inches / n, 0.0, deg / n);
  }
  // Each drive side travels the given inches (left, right), one side may be 0 for a swing.
  void wheels(double left_in, double right_in) {
    const double ds = (left_in + right_in) / 2.0;
    const double deg = (left_in - right_in) / cfg.drive_width * 180.0 / M_PI;
    const int n = std::max(steps_for(std::fabs(left_in) > std::fabs(right_in) ? left_in : right_in, 0.1), steps_for(deg, 0.25));
    for (int i = 0; i < n; i++) move(ds / n, 0.0, deg / n);
  }
  // Swing by `deg` degrees clockwise (positive) or counter-clockwise (negative) with one side stopped.
  void swing(double deg, bool left_side_stopped) {
    const double travel = std::fabs(deg) * M_PI / 180.0 * cfg.drive_width;
    // Clockwise means the left side goes forward and the right side goes backward, relative to each other.
    if (deg >= 0.0)
      left_side_stopped ? wheels(0.0, -travel) : wheels(travel, 0.0);
    else
      left_side_stopped ? wheels(0.0, travel) : wheels(-travel, 0.0);
  }

  double err_x() { return chassis.odom_x_get() - tx; }
  double err_y() { return chassis.odom_y_get() - ty; }
  // Distance between the pose the library reports and the true pose.
  double err() { return std::hypot(err_x(), err_y()); }
  // Distance between the pose the library reports and where the robot started.
  double drift_from_start() { return std::hypot(chassis.odom_x_get() - cfg.start_x, chassis.odom_y_get() - cfg.start_y); }
};

}  // namespace ez::tracker_rig
