// drive_angle_set() moves the heading PID's target along with the heading.
//
// pid_drive_set() holds whatever headingPID's target is, so after drive_angle_set(90_deg) -- the first line of an auton
// that starts facing a wall -- a straight drive has to hold 90, not steer back to wherever the target last was.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
struct Rig {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis;
  sim::SimRobot sim;
  Rig() : chassis(make()), sim(chassis, a, sim::NoiseConfig{false, 1}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
  }
  static Drive make() {
    test_stub::reset_all();
    auto a = sim::archetype_light_fast();
    return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  }
};
}  // namespace

TEST_CASE("drive_angle_set sets the heading PID target, so a straight drive that follows reads it") {
  for (double angle : {90.0, -135.0, 30.0}) {
    Rig r;
    r.chassis.drive_angle_set(angle);
    CAPTURE(angle);
    CHECK(r.chassis.headingPID.target_get() == doctest::Approx(angle));
    r.chassis.pid_drive_set(12_in, 100);
    CHECK(r.chassis.headingPID.target_get() == doctest::Approx(angle));
  }
}

TEST_CASE("drive_angle_set(90) then a straight pid_drive_set holds 90 degrees, no turn back toward the old heading") {
  Rig r;
  r.chassis.pid_turn_set(20_deg, 90);  // leaves the heading target somewhere other than 90
  r.chassis.pid_wait();
  r.chassis.drive_angle_set(90.0);
  r.chassis.pid_drive_set(24_in, 110);
  r.chassis.pid_wait();
  CHECK(std::fabs(r.chassis.drive_angle_get() - 90.0) < 2.0);
  CHECK_FALSE(r.chassis.interfered);
}
