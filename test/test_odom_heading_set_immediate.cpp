// After odom_xyt_set() / drive_angle_set(), odom_theta_get() reads the heading that was just set, at once.
//
// drive_angle_set() updated the IMU, headingPID and the tracking poses' theta but not odom_current.theta, which is only
// refreshed on the next tracking pass. Every odom setter seeds the angle PID with
// current_a_odomPID.motion_reset(odom_theta_get()), so an odom motion started right after
// `odom_xyt_set(x, y, 180_deg)` -- the normal first two lines of an odom auton -- seeded from the old heading, and its
// first pass computed a huge false derivative: about -6600 / +6400 mV for one pass, about half a degree of
// twitch. One pass later it was normal.
#include <algorithm>
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
// The heading that already faces (0, -24) from (5.31, -4.13), so the motion has no turning to do and an ordinary pass is
// small; the stale-heading kick is then the only large thing on the first pass.
constexpr double kAimedHeading = -165.04;
struct Passes {
  double first_diff;  // |left - right| commanded on the motion's first pass
  double mid_max;     // the largest |left - right| over passes 3 to 12, an ordinary stretch of the same motion
};

Passes run(bool one_pass_between) {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});

  chassis.odom_xyt_set(5.31_in, -4.13_in, kAimedHeading * 1_deg);
  if (one_pass_between) pros::delay(util::DELAY_TIME);
  chassis.pid_odom_set({{0_in, -24_in}, ez::fwd, 110});

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

TEST_CASE("odom_xyt_set: odom_theta_get() reads the heading that was just set, before any tracking pass") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.odom_xyt_set(5.31_in, -4.13_in, 180_deg);
  CHECK(chassis.odom_theta_get() == doctest::Approx(180.0));
  CHECK(chassis.odom_x_get() == doctest::Approx(5.31));
  CHECK(chassis.odom_y_get() == doctest::Approx(-4.13));
  chassis.odom_theta_set(-90.0);
  CHECK(chassis.odom_theta_get() == doctest::Approx(-90.0));
}

TEST_CASE("odom_xyt_set then an odom motion at once: the first pass is no bigger than an ordinary pass") {
  Passes p = run(false);
  MESSAGE("first pass |L-R|=", p.first_diff, " ordinary pass max=", p.mid_max);
  // Bound derived from the motion itself: the first pass may be a little more than the later passes (the robot has
  // not started to turn yet), not thousands of mV more.
  CHECK(p.first_diff <= p.mid_max * 1.5 + 200.0);
}

TEST_CASE("control: the same with one pass between the set and the motion") {
  Passes p = run(true);
  MESSAGE("first pass |L-R|=", p.first_diff, " ordinary pass max=", p.mid_max);
  CHECK(p.first_diff <= p.mid_max * 1.5 + 200.0);
}
