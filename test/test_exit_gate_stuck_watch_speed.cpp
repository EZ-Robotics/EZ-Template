// The velocity exit on its own: the position exits get a 5 s time here so they never fire, and the velocity exit (3.2.2's, at or under
// 5 units/s for velocity_exit_time) is what ends each wait. A 50 or 100 ms velocity exit time lets the robot go while it still crawls in
// under 5 in/s. It is clean when it ends the wait inside big_error and under the stop speed (1.5 in/s, 4 deg/s), and interfered
// otherwise. The same motions with the position exits on are in test_exit_gate_small_big_stopped.cpp.
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

// The velocity exit's own threshold, 5 in/s, plus one sensor count over the library's 100 ms speed window
double velocity_exit_floor(Rig& r) { return 5.0 + (1.0 / r.chassis.drive_tick_per_inch()) / 0.1; }
}  // namespace

// The velocity exit lets the classroom robot go at about 5 in/s by the library's speed, 0.6 in short: over the stop speed, so interfered.
// The drive keeps running after the wait returns, so the robot finishes the error it was let go at.
TEST_CASE("classroom robot, 50 ms velocity exit only: pid_wait() on a 24 in drive returns under the velocity exit's 5 in/s, interfered") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 50_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(100) < velocity_exit_floor(r));
  double left = std::fabs(24.0 - r.trace.back().avg);
  CHECK(left < 1.0);
  CHECK(r.run_on(500).distance < left + 0.3);
}

// light_fast is nearly stopped when the velocity exit ends it, but the library's 100 ms speed still reads about 1.7 in/s, over the stop
// speed, so it is interfered
TEST_CASE("light_fast, 50 ms velocity exit only: pid_wait() on a 24 in drive returns nearly stopped, interfered") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 50_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(50) < r.drive_floor(50));
  CHECK(r.run_on(500).distance < 0.3);
}

// The turn and light_fast's swing end on the velocity exit inside big_error and under the stop speed, so they are clean. The classroom
// robot's swing is still turning at about 4.7 deg/s by the library's speed when its velocity exit ends it, over the 4 deg/s stop speed,
// so it is interfered.
TEST_CASE("light_fast, 50 ms velocity exit only: a 90 degree turn and a 90 degree left swing are done turning when they return") {
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_turn_exit_condition_set(5000_ms, 1_deg, 5000_ms, 3_deg, 50_ms, 500_ms);
    r.chassis.pid_turn_set(90_deg, 110);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.angle_speed_now() < 8.0);
    CHECK(r.angle_speed_over(50) < r.angle_floor(50));
    CHECK(r.run_on(500).angle < 1.0);
  }
  {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_swing_exit_condition_set(5000_ms, 1_deg, 5000_ms, 3_deg, 50_ms, 500_ms);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK(r.chassis.interfered);
    CHECK(r.angle_speed_now() < 8.0);
    CHECK(r.angle_speed_over(50) < r.angle_floor(50));
    CHECK(r.run_on(500).angle < 1.0);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_swing_exit_condition_set(5000_ms, 1_deg, 5000_ms, 3_deg, 50_ms, 500_ms);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.angle_speed_now() < 8.0);
    CHECK(r.angle_speed_over(50) < r.angle_floor(50));
    CHECK(r.run_on(500).angle < 1.0);
  }
}

// The sticky robot crawls the last inch at 2 to 3 in/s, under the velocity exit's 5 in/s, so the velocity exit lets it go with the library's
// speed at about 2.5 in/s: over the stop speed, so interfered
TEST_CASE("a sticky robot, 100 ms velocity exit only: pid_wait() on a 24 in drive returns under the velocity exit's 5 in/s, interfered") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 5000_ms, 3_in, 100_ms, 500_ms);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK(r.chassis.interfered);
  CHECK(r.drive_speed_now() < 3.0);
  CHECK(r.drive_speed_over(100) < velocity_exit_floor(r));
  double left = std::fabs(24.0 - r.trace.back().avg);
  CHECK(r.run_on(500).distance < left + 0.3);
}

TEST_CASE("a robot hunting about its target returns clean near the target, never hangs, at the shipped exits") {
  Rig r(sim::archetype_light_fast(), 1);
  make_hunter(r.chassis);
  r.chassis.pid_print_toggle(true);
  r.chassis.pid_drive_set(24_in, 110);
  double elapsed = 0;
  bool returned = false;
  std::string out = test_stub::capture_stdout([&] { returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed); });
  REQUIRE(returned);
  CHECK_FALSE(r.chassis.interfered);
  std::printf("  [hunter] returned at %.0f ms at %.2f in, verdict %s: %s", elapsed, r.trace.back().avg, r.chassis.interfered ? "interfered" : "clean",
              out.empty() ? "\n" : out.substr(out.rfind('\n', out.size() - 2) + 1).c_str());
  CHECK(std::fabs(24.0 - r.trace.back().avg) < 1.0);
  CHECK(elapsed < 3000);
}

TEST_CASE("a robot stopped inside big_error with its encoders flickering a count every tick still ends at the team's velocity exit time") {
  for (int every : {1, 2}) {
    Rig r(sim::archetype_light_fast(), 1);
    // Big exit off and the robot held outside small_error: the velocity exit ends this, clean, as it is inside big_error and stopped
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
    // It reaches the wall by about 1 s; the velocity exit time is 200 ms, plus up to a 100 ms speed window to read it stopped
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
