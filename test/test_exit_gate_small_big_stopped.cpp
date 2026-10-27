// Small exit and big exit only end a wait once the robot has also stopped: their timers count only while the library's speed (net
// movement over the last 100 ms) is under the stop speed (1.5 in/s, 4 deg/s), so a heavy robot crawling in is not cut off short and a
// light robot crossing the small band at speed is not called done.
//
// The velocity exit is 3.2.2's: it fires once the speed has been at or under 5 units/s for velocity_exit_time. When that time is short
// next to the small and big exit times it can end a wait while the robot still crawls in under 5 in/s, before small can count. It is
// clean when it ends the wait inside big_error, and interfered otherwise.
//
// Every speed here is the sim's own.
#include <cmath>
#include <cstdio>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {
// The velocity exit's threshold, which is the drive stop speed, plus one sensor count over a 100 ms window
double velocity_exit_floor(Rig& r) { return FLOOR_DISTANCE + (1.0 / r.chassis.drive_tick_per_inch()) / 0.1; }
}  // namespace

TEST_CASE("a heavy robot crawling in is not cut off by the big exit: it returns stopped, and closer than before (24 in, shipped exits)") {
  for (int passes : {2, 3}) {
    Rig r(sim::archetype_heavy_slow(), passes);
    r.chassis.pid_drive_set(24_in, 110);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    CAPTURE(passes);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.drive_speed_over(90) < r.drive_floor(90));  // over the shortest exit window: the small exit is what ends it
    // A big exit that only looks at position lets this robot go 1.25 in short, moving 3.8 in/s, at 1470 ms.  The tuned exits end it up to 2 in
    // short, stopped
    double err = std::fabs(24.0 - r.trace.back().avg);
    std::printf("  [heavy/%d] returned at %.0f ms, %.2f in short (position-only big exit: 1470 ms, 1.25 in)\n", passes, elapsed, err);
    CHECK(err <= 2.0);
  }
}

TEST_CASE("a light robot crossing the small band at speed does not return from it (50/1/250/3/500/500)") {
  // P 20, D 10: it overshoots through the 1 in small band at about 30 in/s and the shipped small exit time of 50 ms is shorter than
  // the time it spends in the band
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_constants_set(20.0, 0.0, 10.0);
  r.chassis.pid_drive_exit_condition_set(50_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.drive_speed_over(50) < r.drive_floor(50));
  CHECK(r.drive_speed_now() < FLOOR_DISTANCE);
}

TEST_CASE("a bump inside the band after the robot stopped delays the exit by the bump, not by a second full timer") {
  // light_fast on a 24 in drive, exits 90/1/250/3/500/500. The bump is a 40 N push for 20 ms once it has settled in the band.
  struct Run {
    double returned_at;
    double bump_end;
  };
  auto run = [&](double bump_start_ms) {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_drive_set(24_in, 110);
    r.sim.push(40.0, bump_start_ms, 20.0);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    CHECK_FALSE(r.chassis.interfered);
    return Run{elapsed, bump_start_ms + 20.0};
  };
  Run calm = [&] {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_drive_set(24_in, 110);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    return Run{elapsed, 0.0};
  }();
  // Bump 100 ms before the undisturbed return: inside the band, the robot is stopped, then moved a little
  Run bumped = run(calm.returned_at - 100.0);
  std::printf("  [bump] undisturbed return at %.0f ms, bumped at %.0f ms (bump ended at %.0f)\n", calm.returned_at, bumped.returned_at, bumped.bump_end);
  // One small exit time (90 ms) of stillness after the bump, give or take a few passes: not a second 250 ms big exit timer
  CHECK(bumped.returned_at <= bumped.bump_end + 90.0 + 150.0);
}

// The motions from test_exit_gate_stuck_watch_speed.cpp with every exit on. The 50 ms velocity exit time is shorter than the 90 ms small
// exit time, so the velocity exit ends these while the robot still crawls in under 5 in/s: the library's speed is then over the stop
// speed on both robots (about 5 in/s on classroom, 1.7 in/s on light_fast), inside big_error, so both read clean. light_fast is nearly stopped by
// then; the classroom robot is still crawling under 5 in/s, and the drive keeps running after the wait returns, so it finishes the error
// it was let go at.
TEST_CASE("every exit on, 50 ms velocity window: pid_wait() on a 24 in drive returns under the velocity exit's 8 in/s, clean") {
  for (auto arch : {archetype_classroom(), sim::archetype_light_fast()}) {
    Rig r(arch, 1);
    r.chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 50_ms, 500_ms);
    r.chassis.pid_drive_set(24_in, 110, true);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CAPTURE(std::string(arch.name));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.drive_speed_now() < FLOOR_DISTANCE);
    double left = std::fabs(24.0 - r.trace.back().avg);
    CHECK(left < 1.0);
    if (std::string(arch.name) == "classroom") {
      CHECK(r.drive_speed_over(100) < velocity_exit_floor(r));
      CHECK(r.run_on(500).distance < left + 0.3);
    } else {
      CHECK(r.drive_speed_over(50) < r.drive_floor(50));
      CHECK(r.run_on(500).distance < 0.3);
    }
  }
}

// The turns and swings end on the velocity exit inside big_error, so they are clean. The classroom robot's swing is still turning at
// about 4.7 deg/s by the library's speed when its velocity exit ends it, over the 4 deg/s stop speed, and is clean all the same.
TEST_CASE("every exit on, 50 ms velocity window: a 90 degree turn and a 90 degree left swing are done turning when they return") {
  for (auto arch : {archetype_classroom(), sim::archetype_light_fast()}) {
    {
      Rig r(arch, 1);
      r.chassis.pid_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 50_ms, 500_ms);
      r.chassis.pid_turn_set(90_deg, 110);
      REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
      CAPTURE(std::string(arch.name));
      CHECK_FALSE(r.chassis.interfered);
      CHECK(r.angle_speed_now() < FLOOR_ANGLE);
      CHECK(r.angle_speed_over(50) < r.angle_floor(50));
      CHECK(r.run_on(500).angle < FLOOR_ANGLE * 0.5);
    }
    {
      Rig r(arch, 1);
      r.chassis.pid_swing_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 50_ms, 500_ms);
      r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
      REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
      CAPTURE(std::string(arch.name));
      CHECK_FALSE(r.chassis.interfered);
      CHECK(r.angle_speed_now() < FLOOR_ANGLE);
      CHECK(r.angle_speed_over(50) < r.angle_floor(50));
      CHECK(r.run_on(500).angle < FLOOR_ANGLE * 0.5);
    }
  }
}

// The sticky robot crawls the last inch at 2 to 3 in/s: over the stop speed, so the small and big timers can't count, but under the
// velocity exit's 5 in/s, so the 100 ms velocity exit ends the wait (its time is shorter than the 200 ms big exit time) while the
// library's speed is about 2.5 in/s. That is over the stop speed, and inside big_error, so it is clean.
TEST_CASE("The corpus exits (90/1/200/3/100/100) on a sticky robot: pid_wait() on a 24 in drive returns under the velocity exit's 8 in/s, clean") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.drive_speed_now() < FLOOR_DISTANCE);
  CHECK(r.drive_speed_over(100) < velocity_exit_floor(r));
  double left = std::fabs(24.0 - r.trace.back().avg);
  CHECK(r.run_on(500).distance < left + 0.3);
}
