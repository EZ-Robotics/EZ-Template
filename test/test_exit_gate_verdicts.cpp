// One rule for "finished": every wait reaches the same verdict for the same motion at the same instant.
//
// Before, "settled", "reached the checkpoint" and "interfered" were decided in six places by five slightly different rules, so
// one motion could be clean in pid_wait() and interfered in pid_wait_until() or the odom quick waits. Now:
//   1. Nothing but "stopped" (Fix B / Fix C) ends a wait as settled.
//   2. The mA exit inside big_error of the motion's final target is a settle, not an interference. Outside it, interfered.
//   3. A checkpoint counts as reached when the robot crossed it, is within the motion's small_error of it, or settled inside
//      big_error of the final target with the checkpoint between where it rested and the final target, or it can never be
//      reached (printed, not interfered).
//   4. An odom point wait on the path's last point follows the same rules with xy and heading.
#include <cmath>
#include <cstdio>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {
void exits_2550R(Drive& c) {
  c.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
  c.pid_turn_exit_condition_set(90_ms, 1_deg, 200_ms, 3_deg, 100_ms, 100_ms);
  c.pid_swing_exit_condition_set(90_ms, 1_deg, 200_ms, 3_deg, 100_ms, 100_ms);
  c.pid_odom_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
  c.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 200_ms, 3_deg, 100_ms, 100_ms);
}

// When a calm run of the same swing first passes `heading` degrees, so a second run can hold the robot there from that moment
double time_at_heading(const sim::SimArchetype& a, double target, double heading) {
  Rig r(a, 1);
  r.chassis.pid_swing_set(ez::LEFT_SWING, target, 110, 0);
  r.wait([&] { r.chassis.pid_wait(); }, 3000);
  for (const auto& s : r.trace)
    if (std::fabs(s.heading) >= heading) return s.t_ms;
  return 0;
}
}  // namespace

// ---- the odom quick waits on a finished move -------------------------------------------------------------

TEST_CASE("2550R's exits: pid_odom_set + pid_wait_quick() on a finished move is clean, on every robot") {
  for (auto arch : {sim::archetype_light_fast(), sim::archetype_sticky_high_friction(), archetype_classroom()}) {
    Rig r(arch, 1);
    exits_2550R(r.chassis);
    r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 110}, true);
    REQUIRE(r.wait([&] { r.chassis.pid_wait_quick(); }, 3000));
    CAPTURE(std::string(arch.name));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(std::hypot(12.0 - r.chassis.odom_x_get(), 24.0 - r.chassis.odom_y_get()) < 3.0);
  }
}

TEST_CASE("2550R's exits: pid_wait_until_index(last) on a 3 point path is clean, on every robot") {
  for (auto arch : {sim::archetype_light_fast(), sim::archetype_sticky_high_friction(), archetype_classroom()}) {
    Rig r(arch, 1);
    exits_2550R(r.chassis);
    r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in}, fwd, 110}, {{24_in, 24_in}, fwd, 110}}, true);
    REQUIRE(r.wait([&] { r.chassis.pid_wait_until_index(2); }, 3000));
    CAPTURE(std::string(arch.name));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(std::hypot(24.0 - r.chassis.odom_x_get(), 24.0 - r.chassis.odom_y_get()) < 3.0);
  }
}

TEST_CASE("controls: a mid-path wait_until a wall blocks, and a quick wait pinned 6 in short of the target, are still interfered") {
  {
    Rig r(sim::archetype_light_fast(), 1);
    exits_2550R(r.chassis);
    r.chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110}, {{0_in, 48_in}, fwd, 110}}, true);
    r.sim.wall(18.0);  // 6 in before the point the wait is for
    REQUIRE(r.wait([&] { r.chassis.pid_wait_until(pose{0, 24, 0}); }, 3000));
    CHECK(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    exits_2550R(r.chassis);
    r.chassis.pid_odom_set({{0_in, 24_in}, fwd, 110}, true);
    r.sim.wall(18.0);  // 6 in short of the target
    REQUIRE(r.wait([&] { r.chassis.pid_wait_quick(); }, 3000));
    CHECK(r.chassis.interfered);
  }
}

// ---- wait_until's stuck path ignored the within-small_error rule --------------------------------------------

TEST_CASE("a sticky robot's fast-exit chained swing is clean when it settles inside small_error of the checkpoint") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  r.chassis.pid_swing_exit_condition_set(40_ms, 3_deg, 150_ms, 7_deg, 150_ms, 300_ms);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 0);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
}

TEST_CASE("2550R's exits on a sticky robot: pid_wait_until(23.5 in) on a 24 in drive returns near the checkpoint, clean") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  exits_2550R(r.chassis);
  r.chassis.pid_drive_set(24_in, 110, true);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_until(23.5_in); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.trace.back().avg >= 23.0);
}

TEST_CASE("a chained drive with the fast exits is not interfered") {
  for (auto arch : {sim::archetype_light_fast(), sim::archetype_sticky_high_friction()}) {
    Rig r(arch, 1);
    r.chassis.pid_drive_exit_condition_set(40_ms, 3_in, 150_ms, 7_in, 150_ms, 300_ms);
    r.chassis.pid_drive_set(24_in, 110);
    REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000));
    CAPTURE(std::string(arch.name));
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("control: pinned 4 degrees short of a checkpoint with small_error 3 is still interfered") {
  const auto arch = sim::archetype_sticky_high_friction();
  double t41 = time_at_heading(arch, 90.0, 41.0);
  REQUIRE(t41 > 0);
  Rig r(arch, 1);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
  r.sim.pin(t41, 20000);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_until(45_deg); }, 4000));
  CHECK(r.chassis.interfered);
}

TEST_CASE("control: a chained drive, turn, swing and drive on light_fast at the shipped exits is never interfered") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(24_in, 110);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  r.chassis.pid_turn_set(90_deg, 110);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 135_deg, 110, 0);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  r.chassis.pid_drive_set(12_in, 110);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
}

// ---- a near-final checkpoint was interfered while pid_wait() on the same motion was clean --------------------

TEST_CASE("a sticky robot's left swing to 90 at the shipped exits: pid_wait_until(89) is clean, and so is pid_wait_until(89) then pid_wait()") {
  {
    Rig r(sim::archetype_sticky_high_friction(), 1);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    REQUIRE(r.wait([&] { r.chassis.pid_wait_until(89_deg); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_sticky_high_friction(), 1);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    REQUIRE(r.wait([&] { r.chassis.pid_wait_until(89_deg); }, 3000));
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("a sticky robot's 24 in drive: pid_wait_until(23 in) is clean when it parks short of the checkpoint") {
  Rig r(sim::archetype_sticky_high_friction(), 1);
  // The start commit's big exit parks this robot at about 21.5 in. A wall there holds it to that whatever the exits do to where it
  // parks: 2.5 in short of the target (inside big_error), 1.5 in short of the checkpoint (outside small_error)
  r.chassis.pid_drive_set(24_in, 110);
  r.sim.wall(21.5);
  REQUIRE(r.wait([&] { r.chassis.pid_wait_until(23_in); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
}

TEST_CASE("controls: pinned by a wall at 80 degrees on a swing to 90, pid_wait_until(85) is still interfered; an unreachable checkpoint prints and is not") {
  {
    const auto arch = sim::archetype_sticky_high_friction();
    double t80 = time_at_heading(arch, 90.0, 80.0);
    REQUIRE(t80 > 0);
    Rig r(arch, 1);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0);
    r.sim.pin(t80, 20000);
    REQUIRE(r.wait([&] { r.chassis.pid_wait_until(85_deg); }, 4000));
    CHECK(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_print_toggle(false);
    r.chassis.pid_drive_set(24_in, 110);
    std::string out = test_stub::capture_stdout([&] { REQUIRE(r.wait([&] { r.chassis.pid_wait_until(30_in); }, 3000)); });
    CHECK(out.find("can't be reached") != std::string::npos);
    CHECK_FALSE(r.chassis.interfered);
  }
}

// ---- The mA exit inside big_error ------------------------------------------------------------------------------------------

TEST_CASE("2550R's pin2: an mA exit against a wall inside big_error of the target is clean, outside it is interfered") {
  for (double wall : {25.0, 20.0}) {
    Rig r(sim::archetype_light_fast(), 1);
    exits_2550R(r.chassis);
    r.chassis.pid_drive_set(27, 80, true);
    r.sim.wall(wall);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait_quick(); }, 3000, &elapsed));
    CAPTURE(wall);
    // 27 in target: a wall at 25 is 2 in short (inside big_error 3), a wall at 20 is 7 in short (outside it)
    CHECK(r.chassis.interfered == (wall < 24.0));
  }
}

TEST_CASE("a finished 90 degree turn on a heavy robot with a short mA timeout is clean (250/1/250/3/0/100)") {
  for (int passes : {2, 3}) {
    Rig r(sim::archetype_heavy_slow(), passes);
    r.chassis.pid_turn_exit_condition_set(250_ms, 1_deg, 250_ms, 3_deg, 0_ms, 100_ms);
    r.chassis.pid_turn_set(90_deg, 90);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CAPTURE(passes);
    CHECK_FALSE(r.chassis.interfered);
  }
}
