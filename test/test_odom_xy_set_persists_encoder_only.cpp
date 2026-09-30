// odom_x_set() and odom_y_set() on a drive with no tracking wheels: the value survives the tracking passes that follow.
//
// With no vertical trackers odom_current is rewritten from central_pose on every tracking pass, so a setter that only
// updates odom_current (and the left and right poses) reads back correctly until the very next pass and then snaps
// back to where the encoders' own accumulation had the robot. The set has to reach central_pose too.
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
    sim.use_real_auto_task(true);  // the real task is what runs the tracking pass
  }
  static Drive make() {
    test_stub::reset_all();
    auto a = sim::archetype_light_fast();
    return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  }
};

void passes(int n) {
  for (int i = 0; i < n; i++) pros::delay(util::DELAY_TIME);
}
}  // namespace

TEST_CASE("odom_x_set on an encoder-only drive is still the reading after tracking passes with the robot still") {
  Rig r;
  passes(5);
  r.chassis.odom_x_set(12.0);
  CHECK(r.chassis.odom_x_get() == doctest::Approx(12.0));
  passes(10);
  CHECK(r.chassis.odom_x_get() == doctest::Approx(12.0));
  CHECK(r.chassis.odom_y_get() == doctest::Approx(0.0));
}

TEST_CASE("odom_y_set on an encoder-only drive is still the reading after tracking passes with the robot still") {
  Rig r;
  passes(5);
  r.chassis.odom_y_set(-7.5);
  CHECK(r.chassis.odom_y_get() == doctest::Approx(-7.5));
  passes(10);
  CHECK(r.chassis.odom_y_get() == doctest::Approx(-7.5));
  CHECK(r.chassis.odom_x_get() == doctest::Approx(0.0));
}

TEST_CASE("odom_x_set then driving: the robot's travel is added to the x that was set") {
  Rig r;
  passes(5);
  r.chassis.odom_xyt_set(12.0, 0.0, 90.0);  // facing +x, so forward travel moves x
  r.chassis.pid_drive_set(10_in, 110);
  r.chassis.pid_wait();
  CHECK(r.chassis.odom_x_get() == doctest::Approx(22.0).epsilon(0.03));
  CHECK(std::fabs(r.chassis.odom_y_get()) < 0.5);
}
