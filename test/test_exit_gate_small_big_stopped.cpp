// Small exit and big exit only end a wait once the robot has also stopped.
//
// Both exits used to look at position only: the robot has been inside the band for small_exit_time / big_exit_time. A heavy robot
// crawling in at 3 to 5 in/s was cut off 1 to 1.5 in short when the big exit's timer ran out, and a light robot spending
// small_exit_time crossing the small band at speed was called done. Now each exit also needs the robot to have travelled less
// than the stop speed (1.5 in/s, 4 deg/s) times that exit's own time over that time.
//
// Timing: the position timer keeps counting exactly as it did. Once it has met its time the exit fires on the first pass the robot
// is also stopped, provided it has stayed inside the band; it does not start a fresh timer. Leaving the band resets it.
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

TEST_CASE("a heavy robot crawling in is not cut off by the big exit: it returns stopped, and closer than before (24 in, shipped exits)") {
  for (int passes : {2, 3}) {
    Rig r(sim::archetype_heavy_slow(), passes);
    r.chassis.pid_drive_set(24_in, 110);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    CAPTURE(passes);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.drive_speed_over(90) < r.drive_floor(90));  // over the shortest exit window: the small exit is what ends it
    // The start commit returns at 1.25 in short, moving 3.8 in/s, at 1470 ms
    double err = std::fabs(24.0 - r.trace.back().avg);
    std::printf("  [heavy/%d] returned at %.0f ms, %.2f in short (start commit: 1470 ms, 1.25 in)\n", passes, elapsed, err);
    CHECK(err <= 1.25 - 0.5);
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
  CHECK(r.drive_speed_now() < 3.0);
}

TEST_CASE("a robot hunting about its target is not let go by the small or big exit while it oscillates above the floor") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_constants_set(3000.0, 0.0, 0.0);  // 0.43 in peak to peak at about 25 Hz, see test_exit_gate_stuck_watch_speed.cpp
  r.chassis.pid_print_toggle(true);
  r.chassis.pid_drive_set(24_in, 110);
  double elapsed = 0;
  bool returned = false;
  std::string out = test_stub::capture_stdout([&] { returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed); });
  REQUIRE(returned);
  CHECK(out.find("Small Exit") == std::string::npos);
  CHECK(out.find("Big Exit") == std::string::npos);
  CHECK(out.find("counted as settled") != std::string::npos);
  CHECK(elapsed < 3000);
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

// The motions from test_exit_gate_stuck_watch_speed.cpp with every exit on: here the small and big exits are what could end them
// while the robot is still moving.
TEST_CASE("every exit on, 50 ms velocity window: pid_wait() does not return while the robot is still moving (24 in drive)") {
  for (auto arch : {archetype_classroom(), sim::archetype_light_fast()}) {
    Rig r(arch, 1);
    r.chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 50_ms, 500_ms);
    r.chassis.pid_drive_set(24_in, 110, true);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CAPTURE(arch.name);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.drive_speed_now() < 3.0);
    CHECK(r.drive_speed_over(50) < r.drive_floor(50));
    CHECK(std::fabs(24.0 - r.trace.back().avg) < 1.0);
    CHECK(r.run_on(500).distance < 0.3);
  }
}

TEST_CASE("every exit on, 50 ms velocity window: a 90 degree turn and a 90 degree left swing are done turning when they return") {
  for (auto arch : {archetype_classroom(), sim::archetype_light_fast()}) {
    {
      Rig r(arch, 1);
      r.chassis.pid_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 50_ms, 500_ms);
      r.chassis.pid_turn_set(90_deg, 110);
      REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
      CAPTURE(arch.name);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(r.angle_speed_now() < 8.0);
      CHECK(r.angle_speed_over(50) < r.angle_floor(50));
      CHECK(r.run_on(500).angle < 1.0);
    }
    {
      Rig r(arch, 1);
      r.chassis.pid_swing_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 50_ms, 500_ms);
      r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
      REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
      CAPTURE(arch.name);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(r.angle_speed_now() < 8.0);
      CHECK(r.angle_speed_over(50) < r.angle_floor(50));
      CHECK(r.run_on(500).angle < 1.0);
    }
  }
}

TEST_CASE("2550R's exits (90/1/200/3/100/100) on a sticky robot: pid_wait() on a 24 in drive does not return while it is still moving") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(100) < r.drive_floor(100));
  CHECK(r.run_on(500).distance < 0.3);
}
