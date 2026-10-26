// A swing that finished reads clean when it settles inside its small error before a short mA exit can fire, and a turn pinned outside big_error
// still ends interfered promptly.  A sticky robot resting in the friction deadband short of its target draws over current, so an mA_timeout
// longer than the small exit time (300 ms against 100 ms here) lets the small exit end the wait first.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
template <typename F>
bool run_capped(F&& wait, int max_ticks) {
  test_stub::g_clock.delay_calls_until_stop = max_ticks;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

Drive make_drive(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

struct Rig {
  sim::SimArchetype a;
  Drive chassis;
  sim::SimRobot sim;
  Rig(const sim::SimArchetype& arch, int passes, bool noise = false, std::uint32_t seed = 1)
      : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{noise, seed}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim.passes_per_tick(passes);
  }
};
}  // namespace

TEST_CASE("a sticky robot's finished left swing to 90 is not interfered with short exits (100/1/1000/3/0/300)") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_swing_exit_condition_set(100_ms, 1_deg, 1000_ms, 3_deg, 0_ms, 300_ms);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK_FALSE(r.chassis.interfered);
  CHECK(std::fabs(r.chassis.drive_angle_get() - 90.0) < 5.0);
}

TEST_CASE("control: a turn pinned outside big_error reports interfered within 1600 ms") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_turn_exit_condition_set(250_ms, 1_deg, 250_ms, 3_deg, 100_ms, 100_ms);
  r.chassis.pid_turn_set(90_deg, 90);
  r.sim.pin(0, 5000);
  std::uint32_t t0 = pros::millis();
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK(r.chassis.interfered);
  CHECK(pros::millis() - t0 <= 1600);
}
