// A motion that finished must not be reported interfered because the stuck window is floored.
//
// The stuck watch's window is floored at 350 ms so a shove is not called stuck (see test_shove_recovery.cpp). Inside
// big_error, though, a stuck verdict is a clean "settled" return, so there is nothing for the floor to protect, and
// flooring it only delayed that clean return. A heavy or sticky robot that rests in the friction deadband short of its
// target draws over current, and with a short mA_timeout (2550R uses 100 ms) the mA exit then fired before the delayed
// settled verdict and the finished turn came back interfered. Inside big_error the watch keeps the team's own window.
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
  Rig(const sim::SimArchetype& arch, int passes, bool noise = false, std::uint32_t seed = 1) : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{noise, seed}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim.passes_per_tick(passes);
  }
};
}  // namespace

TEST_CASE("a heavy robot's finished 90 degree turn is not interfered with short exits (250/1/250/3/0/100), 2 and 3 passes per poll") {
  for (int passes : {2, 3}) {
    Rig r(sim::archetype_heavy_slow(), passes);
    r.chassis.pid_turn_exit_condition_set(250_ms, 1_deg, 250_ms, 3_deg, 0_ms, 100_ms);
    r.chassis.pid_turn_set(90_deg, 90);
    bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
    CAPTURE(passes);
    REQUIRE(returned);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(std::fabs(r.chassis.drive_angle_get() - 90.0) < 3.0);
  }
}

TEST_CASE("a sticky robot's finished left swing to 90 is not interfered with short exits (100/1/1000/3/0/300)") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_swing_exit_condition_set(100_ms, 1_deg, 1000_ms, 3_deg, 0_ms, 300_ms);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK_FALSE(r.chassis.interfered);
  CHECK(std::fabs(r.chassis.drive_angle_get() - 90.0) < 5.0);
}

TEST_CASE("control: a turn pinned outside big_error still waits out the floored window and reports interfered") {
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

// The same for an odom move. A sticky robot resting in the friction deadband at the point draws over current, and with
// 100 ms exits the odom stuck watch's floored window (350 ms) let the mA exit fire first on 24 of 60 seeded runs; a plain
// 350 ms window ended 8 of 60, and the shipped window before the floor ended 6 of 60 on mA. Inside big_error on both axes
// the watch keeps the team's own window, so a finished move is no more often reported interfered than it was.
TEST_CASE("a sticky robot's finished odom point move is no more often interfered with 100 ms exits than before the floor (60 seeds)") {
  int interfered = 0;
  for (std::uint32_t seed = 1; seed <= 60; seed++) {
    Rig r(sim::archetype_sticky_high_friction(), 1, /*noise=*/true, seed);
    r.chassis.pid_odom_drive_exit_condition_set(150_ms, 0.5_in, 250_ms, 1.5_in, 100_ms, 100_ms);
    r.chassis.pid_odom_turn_exit_condition_set(150_ms, 1.5_deg, 250_ms, 3_deg, 100_ms, 100_ms);
    r.chassis.pid_odom_set({{0_in, 24_in}, fwd, 100});
    bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
    REQUIRE(returned);
    interfered += r.chassis.interfered ? 1 : 0;
  }
  CAPTURE(interfered);
  CHECK(interfered <= 6);
}
