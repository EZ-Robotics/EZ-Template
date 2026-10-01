// A shove during an odom point move is recovered from, and the odom stuck watch is not called stuck while it lands.
//
// test_shove_recovery.cpp covers the same thing for pid_drive_set. The odom wait has its own stuck watch, with its own
// channel for the distance to the point, and it needs both halves of the fix separately: a shove restarts its
// no-progress clock (when it latches and again when it peaks), and its window is floored at 350 ms so a team's 100 ms
// velocity_exit_time is not read as "stuck" while a shove is still landing. Either one missing ended these moves
// interfered, with the robot driving again and the next motion starting from the wrong place.
//
// 2550R's 90/1/200/3/100/100 exits, light_fast. Forces stay under the ~150 N where the shove leaves the motors over
// current longer than the team's own mA_timeout, which is a different exit and ends the wait on purpose.
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
  explicit Rig(const sim::SimArchetype& arch) : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{false, 1}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 200_ms, 3_deg, 100_ms, 100_ms);
  }
};

struct Outcome {
  bool returned;
  bool interfered;
  double y;
  int ms;
};

// One shove of `newtons` against the motion for `duration_ms`, starting 400 ms in. as_path: the same move as a two point path.
Outcome shoved(double newtons, double duration_ms, bool as_path) {
  Rig r(sim::archetype_light_fast());
  if (as_path)
    r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 80}, {{0_in, 27_in}, fwd, 80}}, true);
  else
    r.chassis.pid_odom_set({{0_in, 27_in}, fwd, 80});
  std::uint32_t t0 = pros::millis();
  r.sim.push(-newtons, r.sim.now_ms() + 400, duration_ms);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  return {returned, r.chassis.interfered, r.chassis.odom_y_get(), (int)(pros::millis() - t0)};
}
}  // namespace

// A shove that is short next to the floored window but a good deal of it: with a 100 ms window the wait gives up while
// the shove is still landing.
TEST_CASE("an odom point move on 90/1/200/3/100/100 exits recovers from a 60 to 90 N shove of 250 to 350 ms") {
  for (bool as_path : {false, true}) {
    for (double newtons : {60.0, 90.0}) {
      for (double ms : {250.0, 300.0, 350.0}) {
        Outcome o = shoved(newtons, ms, as_path);
        CAPTURE(as_path);
        CAPTURE(newtons);
        CAPTURE(ms);
        CAPTURE(o.y);
        REQUIRE(o.returned);
        CHECK_FALSE(o.interfered);
        CHECK(std::fabs(o.y - 27.0) < 3.0);
      }
    }
  }
}

// A shove that outlasts the floored window before the robot has recovered a step: only the clock restarting when the
// shove latches and peaks keeps the wait from calling it stuck while the robot is on its way back.
TEST_CASE("an odom point move on 90/1/200/3/100/100 exits recovers from a 120 N shove of 350 and 450 ms") {
  for (bool as_path : {false, true}) {
    for (double ms : {350.0, 450.0}) {
      Outcome o = shoved(120.0, ms, as_path);
      CAPTURE(as_path);
      CAPTURE(ms);
      CAPTURE(o.y);
      REQUIRE(o.returned);
      CHECK_FALSE(o.interfered);
      CHECK(std::fabs(o.y - 27.0) < 3.0);
    }
  }
}

// Controls that must not move: a robot that is held, not shoved, is still called stuck, and promptly.
TEST_CASE("control: an odom point move pinned mid-move ends interfered within a window of the pin plus the start allowance") {
  Rig r(sim::archetype_light_fast());
  r.chassis.pid_odom_set({{0_in, 60_in}, fwd, 60});
  std::uint32_t t0 = pros::millis();
  r.sim.pin(r.sim.now_ms() + 400, 60000);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK(r.chassis.interfered);
  CHECK(pros::millis() - t0 <= 400 + 1000 + 350 + 100);
}

TEST_CASE("control: an odom point move held back by a steady force ends interfered within 5 s") {
  Rig r(sim::archetype_light_fast());
  r.chassis.pid_odom_set({{0_in, 60_in}, fwd, 60});
  std::uint32_t t0 = pros::millis();
  r.sim.push(-100.0, r.sim.now_ms() + 400, 60000);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK(r.chassis.interfered);
  CHECK(pros::millis() - t0 <= 5000);
}
