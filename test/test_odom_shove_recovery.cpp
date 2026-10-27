// A shove during an odom point move: what the wait reports depends on where the shove holds the robot.
//
// test_shove_recovery.cpp covers the same thing for pid_drive_set. The xy velocity exit fires once the robot's speed (its
// movement over the last 100 ms) has stayed at or under 5 in/s for the team's velocity_exit_time. A shove that holds the
// robot still outside big_error for longer than that is a velocity exit by design, and ends the wait interfered. A shove
// the robot drives through, or one that leaves it stopped inside big_error, ends the wait clean at the point.
//
// The corpus 90/1/200/3/100/100 exits, light_fast. Forces stay under the ~150 N where the shove leaves the motors over
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
    // A team whose stop speed is 5 in/s and 5 deg/s: a robot recovering from a shove crawls back in under the 8 in/s default for longer than
    // the 100 ms velocity exit time and would be read as held
    chassis.pid_distance_exit_stop_speed_set(5.0);
    chassis.pid_angle_exit_stop_speed_set(5.0);
    chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
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

// 60 N, the robot drives through and arrives. 90 N for 250 ms or more stops the robot about 5.5 in
// short of the point, outside the 3 in big_error, and holds it there for more than the 100 ms velocity_exit_time: the
// velocity exit ends the wait interfered while the robot is still held. Promptly: by the end of the shove, one 100 ms speed
// window for the speed to read the stop, the velocity exit time, and a pass or two.
TEST_CASE("an odom point move on 90/1/200/3/100/100 exits recovers from a 60 to 90 N shove of 250 to 350 ms unless it is held outside big_error") {
  for (bool as_path : {false, true}) {
    for (double newtons : {60.0, 90.0}) {
      for (double ms : {250.0, 300.0, 350.0}) {
        Outcome o = shoved(newtons, ms, as_path);
        // A shorter speed window reads the stop sooner, so a 250 ms shove already holds the robot long enough to be read as held
        bool held_outside = newtons == 90.0 && ms >= 250.0;
        CAPTURE(as_path);
        CAPTURE(newtons);
        CAPTURE(ms);
        CAPTURE(o.y);
        CAPTURE(o.ms);
        REQUIRE(o.returned);
        if (held_outside) {
          CHECK(o.interfered);
          CHECK(std::fabs(o.y - 27.0) > 3.0);  // held outside the team's big_error
          CHECK(o.ms <= 400 + ms + 100 + 100 + 20);
        } else {
          CHECK_FALSE(o.interfered);
          CHECK(std::fabs(o.y - 27.0) < 3.0);
        }
      }
    }
  }
}

// 120 N pushes the robot back without holding it still outside big_error for the velocity exit time, and it drives back to
// the point.
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

// Controls that must not move: a robot that is held, not shoved, still ends the wait interfered, and promptly.
TEST_CASE("control: an odom point move pinned mid-move ends interfered within 1550 ms of the pin") {
  Rig r(sim::archetype_light_fast());
  r.chassis.pid_odom_set({{0_in, 60_in}, fwd, 60});
  std::uint32_t t0 = pros::millis();
  r.sim.pin(r.sim.now_ms() + 400, 60000);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK(r.chassis.interfered);
  CHECK(pros::millis() - t0 <= 400 + 1550);  // pinned at 400 ms
}

// A steady 100 N against the motion drags the robot back until it meets a wall, which holds it, and the wait ends there. The
// sim's walls only stop forward travel, so the move is driven in reverse and the force pushes the robot forward, back to the
// wall at its starting line (about 16 in of drag).
TEST_CASE("control: an odom point move held back by a steady force into a wall ends interfered within 5 s") {
  Rig r(sim::archetype_light_fast());
  r.sim.wall(0.0);
  r.chassis.pid_odom_set({{0_in, -60_in}, rev, 60});
  std::uint32_t t0 = pros::millis();
  r.sim.push(100.0, r.sim.now_ms() + 400, 60000);
  bool returned = run_capped([&] { r.chassis.pid_wait(); }, 3000);
  REQUIRE(returned);
  CHECK(r.chassis.interfered);
  CHECK(pros::millis() - t0 <= 5000);
}
