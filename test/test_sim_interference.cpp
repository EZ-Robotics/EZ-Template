// The sim's interference hooks -- push (an external force for a time window), pin (held still for a time window) and wall
// (forward travel stops) -- all default off. Each does what it says, and nothing else.
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

void ticks(int n) {
  for (int i = 0; i < n; i++) pros::delay(util::DELAY_TIME);
}
}  // namespace

TEST_CASE("sim interference: with none set, an idle robot stays exactly where it is") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  ticks(100);
  CHECK(sim.left().position_in == 0.0);
  CHECK(sim.heading_deg() == 0.0);
}

TEST_CASE("sim interference: a push moves a free robot only while its window lasts, then it coasts to rest") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.push(-60.0, 200.0, 300.0);  // 60 N back, from 200 ms for 300 ms
  ticks(15);
  CHECK(sim.left().position_in == 0.0);  // before the window
  ticks(50);
  double during = sim.left().position_in;
  CHECK(during < -0.05);  // pushed back
  ticks(100);
  CHECK(sim.left().position_in <= during);  // never pushed the other way
  CHECK(std::fabs(sim.left().velocity_in_s) < 1.0);  // at rest again
}

TEST_CASE("sim interference: a pin holds the robot for its window, then lets go") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.drive_set(60, 60);  // the drive keeps pushing while pinned
  ticks(20);
  CHECK(sim.left().position_in > 0.0);
  sim.pin(sim.now_ms(), 500.0);
  ticks(2);
  double held = sim.left().position_in;
  ticks(40);
  CHECK(sim.left().position_in == doctest::Approx(held));
  CHECK(sim.left().velocity_in_s == 0.0);
  ticks(30);  // past the window
  CHECK(sim.left().position_in > held + 0.1);
}

TEST_CASE("sim interference: a wall stops forward travel and the drive stalls against it") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.wall(5.0);
  chassis.drive_set(127, 127);
  ticks(50);  // the sim's thermal model throttles current after about a second of stall, so look early
  CHECK(sim.left().position_in == doctest::Approx(5.0).epsilon(0.01));
  CHECK(chassis.left_motors[0].fake().over_current);
}
