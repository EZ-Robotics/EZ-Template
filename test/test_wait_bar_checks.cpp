// The release bar for the wait code, as named controls: none of these may move.
//
//   1. A turn or swing pinned from the start ends within 1600 ms with interfered = true.
//        -> test_shove_recovery.cpp, "a turn or swing pinned from the start still ends interfered within 1600 ms"
//   2. A drive held back by a steady force ends within 5 s with interfered = true.
//        -> test_shove_recovery.cpp, "a drive held back by a steady force ends interfered within 5 s"
//   3. A heavy drive at speed 20 with default exits completes with interfered = false on light_fast.   (below)
//   4. pid_wait_until(24) then pid_wait() on a 24 in drive finishes the second wait within 500 ms of the first.   (below)
//   5. A competition disable mid-wait ends the wait on the next pass with interfered = true.   (below)
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

Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}
}  // namespace

TEST_CASE("bar 3: a heavy drive at speed 20 with default exits completes with interfered = false") {
  sim::SimArchetype heavy = sim::archetype_light_fast();
  heavy.name = "light_fast, 9 kg";
  heavy.mass_kg = 9.0;
  for (const auto& a : {sim::archetype_light_fast(), heavy, sim::archetype_sticky_high_friction()}) {
    Drive chassis = make_chassis(a);
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
    chassis.pid_drive_set(24_in, 20);
    bool returned = run_capped([&] { chassis.pid_wait(); }, 3000);
    CAPTURE(a.name);
    REQUIRE(returned);
    CHECK_FALSE(chassis.interfered);
    CHECK(std::fabs(sim.left().position_in - 24.0) < 3.0);
  }
}

TEST_CASE("bar 4: pid_wait_until(24) then pid_wait() on a 24 in drive finishes the second wait within 500 ms of the first") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.pid_drive_set(24_in, 110);
  REQUIRE(run_capped([&] { chassis.pid_wait_until(24_in); }, 3000));
  std::uint32_t first = pros::millis();
  REQUIRE(run_capped([&] { chassis.pid_wait(); }, 3000));
  CAPTURE(pros::millis() - first);
  CHECK(pros::millis() - first <= 500);
  CHECK_FALSE(chassis.interfered);
}

namespace {
void (*g_sim_hook)() = nullptr;
std::uint32_t g_disable_at = 0;
// Runs the sim's own tick, then disables the field once the fake clock reaches g_disable_at.
void disable_hook() {
  g_sim_hook();
  if (pros::millis() >= g_disable_at) test_stub::g_competition.disabled = true;
}
}  // namespace

TEST_CASE("bar 5: a competition disable mid-wait ends the wait on the next pass with interfered = true") {
  auto a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.use_real_auto_task(true);  // the disable is handled by ez_auto_task() itself
  test_stub::g_competition.autonomous = true;
  chassis.pid_drive_set(96_in, 60);
  std::uint32_t t0 = pros::millis();
  g_disable_at = t0 + 300;
  g_sim_hook = test_stub::g_clock.on_delay;
  test_stub::g_clock.on_delay = disable_hook;
  bool returned = run_capped([&] { chassis.pid_wait(); }, 3000);
  test_stub::g_clock.on_delay = g_sim_hook;
  REQUIRE(returned);
  CAPTURE(pros::millis() - g_disable_at);
  CHECK(pros::millis() - g_disable_at <= 3 * util::DELAY_TIME);  // the pass that disables, the pass that notices
  CHECK(chassis.interfered);
}
