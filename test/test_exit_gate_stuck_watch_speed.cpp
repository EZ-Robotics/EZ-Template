// A wait must not call a robot settled while it is still moving, whatever the team's exit window is.
//
// Inside big_error the stuck watch returns "settled". It used to do that when the robot had not made a full step of new progress
// (1 in, or 3 degrees) within the team's velocity_exit_time, and that test hides a speed of step / window: at a 50 ms window
// anything slower than 20 in/s was "stopped". A robot decelerating through its last 3 in at 13 in/s came back clean 2.6 in short.
// Now it settles when the robot travelled less than the stop speed (1.5 in/s, 4 deg/s) times the window over that window, or when
// it made no progress for max(window, step / stop speed), which is what ends a robot hunting back and forth.
//
// These tests give the position exits a 5 s time (so they never fire, but small_error still sets the watch's step), so the stuck
// watch is the only thing that can end a wait and what it does is what is measured. The same motions with the position exits on
// are in test_exit_gate_small_big_stopped.cpp.
//
// Every speed here is the sim's own.
#include <cmath>
#include <cstdio>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {
// A robot "hunting": a P and I tuned to chatter about the target. light_fast with a P of 3000 and no D holds a sustained
// 0.43 in peak to peak oscillation at about 25 Hz around a 24 in target (measured in the sim: amplitude 0.43 in, 25 Hz).
void make_hunter(Drive& c) { c.pid_drive_constants_set(3000.0, 0.0, 0.0); }
}  // namespace

TEST_CASE("classroom robot, 50 ms window, stuck watch only: pid_wait() does not return while it is still moving (24 in drive)") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 50_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(50) < FLOOR_DISTANCE);
  CHECK(std::fabs(24.0 - r.trace.back().avg) < 1.0);
  CHECK(r.run_on(500).distance < 0.3);
}

TEST_CASE("light_fast, 50 ms window, stuck watch only: pid_wait() does not return while it is still moving (24 in drive)") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 50_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(50) < FLOOR_DISTANCE);
  CHECK(r.run_on(500).distance < 0.3);
}

TEST_CASE("light_fast, 50 ms window, stuck watch only: a 90 degree turn and a 90 degree left swing are done turning when they return") {
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_turn_exit_condition_set(5000_ms, 1_deg, 5000_ms, 3_deg, 50_ms, 500_ms);
    r.chassis.pid_turn_set(90_deg, 110);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.angle_speed_now() < 8.0);
    CHECK(r.angle_speed_over(50) < FLOOR_ANGLE);
    CHECK(r.run_on(500).angle < 1.0);
  }
  {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_swing_exit_condition_set(5000_ms, 1_deg, 5000_ms, 3_deg, 50_ms, 500_ms);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.angle_speed_now() < 8.0);
    CHECK(r.angle_speed_over(50) < FLOOR_ANGLE);
    CHECK(r.run_on(500).angle < 1.0);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_swing_exit_condition_set(5000_ms, 1_deg, 5000_ms, 3_deg, 50_ms, 500_ms);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.angle_speed_now() < 8.0);
    CHECK(r.angle_speed_over(50) < FLOOR_ANGLE);
    CHECK(r.run_on(500).angle < 1.0);
  }
}

TEST_CASE("a sticky robot, 100 ms window, stuck watch only: pid_wait() on a 24 in drive does not return while it is still moving") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 100_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(100) < FLOOR_DISTANCE);
  CHECK(r.run_on(500).distance < 0.3);
}

TEST_CASE("a robot hunting about its target is ended by the no-progress backstop, never hangs, at the shipped exits") {
  Rig r(sim::archetype_light_fast(), 1);
  make_hunter(r.chassis);
  r.chassis.pid_print_toggle(true);
  r.chassis.pid_drive_set(24_in, 110);
  double elapsed = 0;
  bool returned = false;
  std::string out = test_stub::capture_stdout([&] { returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed); });
  REQUIRE(returned);
  CHECK_FALSE(r.chassis.interfered);
  // It is oscillating faster than the floor when it ends, and that is the one allowed exception: it returns through the stuck
  // watch ("counted as settled"), not through a small or big exit
  std::printf("  [hunter] returned at %.0f ms at %.2f in, verdict %s: %s", elapsed, r.trace.back().avg, r.chassis.interfered ? "interfered" : "clean",
              out.empty() ? "\n" : out.substr(out.rfind('\n', out.size() - 2) + 1).c_str());
  CHECK(std::fabs(24.0 - r.trace.back().avg) < 1.0);
  CHECK(elapsed < 3000);
}

TEST_CASE("a robot stopped inside big_error with its encoders flickering a count every tick still settles at the team's window, not at the backstop") {
  for (int every : {1, 2}) {
    Rig r(sim::archetype_light_fast(), 1);
    // Only the stuck watch can end this: big exit off, the velocity exit's own result is not used by drive waits
    r.chassis.pid_drive_exit_condition_set(90_ms, 0.5_in, 0_ms, 3_in, 200_ms, 0_ms);
    r.chassis.pid_drive_set(24_in, 110);
    r.sim.wall(22.5);  // it stops 1.5 in short: inside big_error, outside small_error
    double count = 1.0 / r.chassis.drive_tick_per_inch();
    int pass = 0;
    r.sim.before_pass = [&](int) {
      pass++;
      if ((pass / every) % 2 == 0) return;
      for (auto* side : {&r.chassis.left_motors, &r.chassis.right_motors})
        for (auto& m : *side) m.fake().position += count * r.chassis.drive_tick_per_inch();
    };
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    CAPTURE(every);
    CHECK_FALSE(r.chassis.interfered);
    // It reaches the wall by about 1 s; 200 ms later is the team's window, 667 ms later would be the backstop
    double stopped_at = 0;
    for (const auto& s : r.trace)
      if (s.avg > 22.4) {
        stopped_at = s.t_ms - r.trace.front().t_ms;
        break;
      }
    CHECK(elapsed - stopped_at < 500.0);
  }
}

TEST_CASE("a motion shorter than big_error does not return before the robot has moved (start allowance)") {
  // 2 in drive and a 5 degree turn start at rest already inside big_error (3 in, 7 degrees): nothing has moved, so nothing is
  // stopped yet
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 100_ms, 500_ms);
  r.chassis.pid_drive_set(2_in, 110);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK(r.trace.back().avg > 1.0);
  Rig t(sim::archetype_light_fast(), 1);
  t.chassis.pid_turn_exit_condition_set(0_ms, 0_deg, 0_ms, 7_deg, 100_ms, 500_ms);
  t.chassis.pid_turn_set(5_deg, 110);
  REQUIRE(t.wait([&] { t.chassis.pid_wait(); }, 3000));
  CHECK(std::fabs(t.chassis.drive_angle_get()) > 3.0);
}
