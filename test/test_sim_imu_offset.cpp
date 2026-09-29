// The sim used to rewrite the fake IMU's rotation from its own physical heading on every tick, which
// silently undid drive_angle_set(), odom_xyt_set() and odom_pose_set() with a nonzero heading: a robot set to 180
// read 0 again one tick later. A write the sim did not make is now adopted as an offset it adds to its own heading.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

void tick(int n) {
  for (int i = 0; i < n; i++) pros::delay(util::DELAY_TIME);
}
}  // namespace

TEST_CASE("sim: a heading set before the first tick survives 50 ticks") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.drive_angle_set(180.0);
  tick(50);
  CHECK(chassis.drive_angle_get() == doctest::Approx(180.0).epsilon(1e-6));
  CHECK(sim.heading_deg() == doctest::Approx(0.0));  // the robot itself never moved
}

TEST_CASE("sim: a heading set after the sim has been running survives too") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  tick(20);
  chassis.drive_angle_set(-45.0);
  tick(50);
  CHECK(chassis.drive_angle_get() == doctest::Approx(-45.0).epsilon(1e-6));
}

TEST_CASE("sim: a turn from a set heading is measured from that heading") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.drive_angle_set(180.0);
  chassis.pid_turn_set(270.0, 90);
  test_stub::g_clock.delay_calls_until_stop = 3000;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  REQUIRE(returned);
  CHECK(std::fabs(chassis.drive_angle_get() - 270.0) < 3.0);
  CHECK(std::fabs(sim.heading_deg()) < 100.0);  // it turned about 90 physical degrees, not 270 or 0
}

TEST_CASE("sim: a value left in the fake IMU before the sim existed is still replaced by the first tick") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::all_imus(chassis)[0]->fake_rotation = 33.0;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  tick(5);
  CHECK(chassis.drive_angle_get() == doctest::Approx(0.0).epsilon(1e-6));
}
