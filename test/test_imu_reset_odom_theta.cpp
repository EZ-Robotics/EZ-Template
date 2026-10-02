// drive_imu_reset() sets the pose heading too, so odom_theta_get() reads the new heading at once.
//
// drive_imu_reset() updated the IMUs, angle_rad, t_last and last_good_angle but not odom_current.theta or the
// per-tracker pose thetas, which were refreshed only on the next tracking pass. drive_angle_set() had the four pose
// writes (6ad3b59), so it was right and a direct drive_imu_reset() was not. Every odom setter seeds the angle PID with
// current_a_odomPID.motion_reset(odom_theta_get()), so an odom motion started right after drive_imu_reset() seeded from
// the old heading and its first pass computed a huge false derivative: about one pass of full turn power.
#include <algorithm>
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
// Turns the robot to about `deg` and lets the tracking task catch up, so the pose heading really is `deg`.
void turn_and_settle(Drive& chassis, double deg) {
  chassis.pid_turn_set(deg, 110, ez::raw);
  chassis.pid_wait();
  for (int i = 0; i < 5; i++) pros::delay(util::DELAY_TIME);
}

struct Passes {
  double first_diff;  // |left - right| commanded on the motion's first pass
  double mid_max;     // the largest |left - right| over passes 3 to 12, an ordinary stretch of the same motion
};

Passes reset_then_odom_motion() {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  turn_and_settle(chassis, 89.0);
  chassis.drive_imu_reset();
  chassis.pid_odom_set({{0_in, 24_in}, ez::fwd, 110});

  auto diff = [&] { return std::fabs(chassis.left_motors[0].fake().voltage - chassis.right_motors[0].fake().voltage); };
  pros::delay(util::DELAY_TIME);
  Passes p{diff(), 0.0};
  for (int i = 2; i <= 12; i++) {
    pros::delay(util::DELAY_TIME);
    if (i >= 3) p.mid_max = std::max(p.mid_max, diff());
  }
  return p;
}
}  // namespace

TEST_CASE("drive_imu_reset: odom_theta_get() reads 0 at once, before any tracking pass") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  turn_and_settle(chassis, 89.0);
  CHECK(chassis.odom_theta_get() == doctest::Approx(89.0).epsilon(0.02));
  chassis.drive_imu_reset();
  CHECK(chassis.drive_angle_get() == doctest::Approx(0.0));
  CHECK(chassis.odom_theta_get() == doctest::Approx(0.0));
  CHECK(DriveTestAccess::central_pose(chassis).theta == doctest::Approx(0.0));
}

TEST_CASE("drive_imu_reset(45): odom_theta_get() reads 45 at once") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  turn_and_settle(chassis, 89.0);
  chassis.drive_imu_reset(45.0);
  CHECK(chassis.drive_angle_get() == doctest::Approx(45.0));
  CHECK(chassis.odom_theta_get() == doctest::Approx(45.0));
  CHECK(DriveTestAccess::central_pose(chassis).theta == doctest::Approx(45.0));
}

TEST_CASE("drive_imu_reset then an odom motion at once: the first pass is no bigger than an ordinary pass") {
  Passes p = reset_then_odom_motion();
  MESSAGE("first pass |L-R|=", p.first_diff, " ordinary pass max=", p.mid_max);
  CHECK(p.first_diff <= p.mid_max * 1.5 + 200.0);
}

TEST_CASE("control: drive_angle_set reads the heading it set, unchanged") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.drive_angle_set(90.0);
  CHECK(chassis.odom_theta_get() == doctest::Approx(90.0));
  CHECK(chassis.headingPID.target_get() == doctest::Approx(90.0));
  chassis.odom_xyt_set(1_in, 2_in, -30_deg);
  CHECK(chassis.odom_theta_get() == doctest::Approx(-30.0));
}

TEST_CASE("control: the next tracking pass after the reset moves the pose only by what the robot really moved") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  turn_and_settle(chassis, 89.0);
  double x = chassis.odom_x_get(), y = chassis.odom_y_get();
  chassis.drive_imu_reset();
  for (int i = 0; i < 3; i++) pros::delay(util::DELAY_TIME);
  CHECK(chassis.odom_theta_get() == doctest::Approx(0.0).epsilon(0.01));
  CHECK(std::fabs(chassis.odom_x_get() - x) < 0.05);
  CHECK(std::fabs(chassis.odom_y_get() - y) < 0.05);
}

TEST_CASE("control: a redundant IMU recovering (check_imu_task) does not change the pose heading") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.drive_imu_reset(30.0);
  chassis.good_imus[0]->fake_rotation = 30.0;
  chassis.good_imus[1]->fake_rotation = 30.0;
  double before = DriveTestAccess::odom_current(chassis).theta;

  // Eject the second IMU, then let it read as healthy (changing values) until it is re-added
  pros::Imu* ejected = chassis.good_imus[1];
  chassis.good_imus.erase(chassis.good_imus.begin() + 1);
  for (int i = 0; i < 200; i++) {
    ejected->fake_rotation += 0.01;
    DriveTestAccess::check_imu_task(chassis);
  }
  CHECK(chassis.good_imus.size() == 2);  // it really did go through set_rotation and get re-added
  CHECK(DriveTestAccess::odom_current(chassis).theta == doctest::Approx(before));
  CHECK(chassis.drive_angle_get() == doctest::Approx(30.0));
}
