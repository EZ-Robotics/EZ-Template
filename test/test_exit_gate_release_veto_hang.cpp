// pid_wait() must not loop for ever on a robot whose exit the gate releases on one pass and the recheck of the latched exit vetoes
// on the next. The gate hands the wait SMALL_EXIT / BIG_EXIT on a pass where the robot reads stopped; the recheck right after reads the
// window one pass later, finds the robot not stopped, and puts the side back to RUNNING. A side that is released on the same pass the
// stuck watch would be consulted is never fed to it (the watch is only called for a side that is still RUNNING), so a robot that
// alternates between the two readings is ended by nothing. The rule: every pass of a wait that does not return consults the watch for
// each side that was RUNNING when the pass began, and that includes the pass its exit is released.
//
// Every wait is capped in sim time here, so "no return by T" is a failure, not a hung suite.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

constexpr int CAP_TICKS = 1500;  // 15 s of sim time
constexpr double RETURN_BY_MS = 3000.0;

}  // namespace

// Fuzz seed 7126 of the audit. The drive kP is high with no derivative, so the robot buzzes about its target at the sample rate.
TEST_CASE("pid_wait() DRIVE on a buzzing light robot with a short team window returns within 3 s, not interfered") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_drive_constants_set(3000.0, 0.0, 0.0);
  r.chassis.pid_drive_exit_condition_set(50, 0.75, 250, 3.75, 1000, 100);
  double first_ms = 0, ms = 0;
  r.chassis.pid_drive_set(-36_in, 33, true);
  bool first = r.wait([&] { r.chassis.pid_wait_quick(); }, CAP_TICKS, &first_ms);
  REQUIRE(first);
  r.chassis.pid_drive_set(-6_in, 70, true);
  bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait() did not return within 15 s");
  CHECK_MESSAGE(ms < RETURN_BY_MS, "returned after " << ms << " ms");
  CHECK_FALSE(r.chassis.interfered);
}

TEST_CASE("pid_wait() DRIVE with exits (30, 1, 250, 3, 500, 500) on a light robot returns within 3 s at 1, 2 and 3 passes per poll") {
  for (int passes : {1, 2, 3}) {
    Rig r(sim::archetype_light_fast(), passes, false, 1);
    r.chassis.pid_drive_constants_set(3000.0, 0.0, 0.0);
    r.chassis.pid_drive_exit_condition_set(30, 1, 250, 3, 500, 500);
    double ms = 0;
    r.chassis.pid_drive_set(18_in, 90);
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
    CHECK_MESSAGE(ok, passes << " passes per poll: pid_wait() did not return within 15 s");
    CHECK_MESSAGE(ms < RETURN_BY_MS, passes << " passes per poll: returned after " << ms << " ms");
  }
}

TEST_CASE("pid_wait() DRIVE on the classroom robot with a 50 ms small exit time and a high kP returns within 3 s") {
  Rig r(archetype_classroom(), 1, false, 1);
  r.chassis.pid_drive_constants_set(10000.0, 0.0, 0.0);
  PID& p = r.chassis.leftPID;
  r.chassis.pid_drive_exit_condition_set(50, p.exit.small_error, p.exit.big_exit_time, p.exit.big_error, p.exit.velocity_exit_time, p.exit.mA_timeout);
  double ms = 0;
  r.chassis.pid_drive_set(12_in, 90);
  bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait() did not return within 15 s");
  CHECK_MESSAGE(ms < RETURN_BY_MS, "returned after " << ms << " ms");
}

namespace {

// A reading that alternates between "stopped" and "moving" on consecutive passes, which is what makes the gate release an exit
// on one pass and the recheck veto it on the next. The tracker counts the path length over a window of 9 passes (90 ms); a jump up
// on one pass and back down two passes later puts 4 or 5 jumps in that window on alternate passes, and with the jump sized so 4 are
// under the floor (4 deg/s * 90 ms = 0.36 deg) and 5 are over, the window reads stopped, moving, stopped, moving. The robot is pinned
// for the whole run so that only the scripted jumps move it.
constexpr int ALT_WINDOW_MS = 90;
// Each jump counts as 0.08 deg of travel after the tracker's one-count backlash band of 0.01 deg is taken off both ends: 4 * 0.08 is
// under 0.36 and 5 * 0.08 is over it.
constexpr double ALT_JUMP_DEG = 0.10;

void alternate_heading(Rig& r, int from_ms, int to_ms) {
  r.sim.pin(from_ms, to_ms - from_ms);
  for (int t = from_ms; t < to_ms; t += 40) {
    r.sim.carry(0.0, ALT_JUMP_DEG / 0.01, t, 10);
    r.sim.carry(0.0, -ALT_JUMP_DEG / 0.01, t + 20, 10);
  }
}

}  // namespace

TEST_CASE("pid_wait() TURN returns within 4 s on a heading that alternates between stopped and moving on consecutive passes") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_turn_exit_condition_set(ALT_WINDOW_MS, 1, 250, 3, ALT_WINDOW_MS, 500);
  r.chassis.pid_turn_set(0_deg, 110);
  alternate_heading(r, 0, 20000);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait() did not return within 15 s");
  CHECK_MESSAGE(ms < 4000.0, "returned after " << ms << " ms");
}

TEST_CASE("pid_wait() SWING returns within 4 s on a heading that alternates between stopped and moving on consecutive passes") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_swing_exit_condition_set(ALT_WINDOW_MS, 1, 250, 3, ALT_WINDOW_MS, 500);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
  alternate_heading(r, 0, 20000);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait() did not return within 15 s");
  CHECK_MESSAGE(ms < 4000.0, "returned after " << ms << " ms");
}

namespace {

// The same for the odom pose, on the classroom robot, whose sensor count is 0.011 in. The xy axis counts path length over the window
// against 1.5 in/s: over 70 ms that is 0.105 in, and a jump the sensor reads as five counts is worth 0.034 in of travel (the one-count
// backlash band is taken off both ends), so a window of 7 passes holds 3 of them (0.102 in, stopped) or 4 (0.136 in, moving) on
// alternate passes. The heading never moves.
constexpr int ALT_XY_WINDOW_MS = 70;

void alternate_position(Rig& r, int from_ms, int to_ms) {
  double jump_in = 0.04 + 2.0 / r.chassis.drive_tick_per_inch();  // the sensor reads it as five counts
  r.sim.pin(from_ms, to_ms - from_ms);
  for (int t = from_ms; t < to_ms; t += 40) {
    r.sim.carry(jump_in / 0.01, 0.0, t, 10);
    r.sim.carry(-jump_in / 0.01, 0.0, t + 20, 10);
  }
}

}  // namespace

TEST_CASE("pid_wait() odom point returns within 4 s on a pose that alternates between stopped and moving on consecutive passes") {
  Rig r(archetype_classroom(), 1, false, 1);
  r.chassis.pid_odom_drive_exit_condition_set(ALT_XY_WINDOW_MS, 1, 250, 3, ALT_XY_WINDOW_MS, 500);
  r.chassis.pid_odom_turn_exit_condition_set(ALT_XY_WINDOW_MS, 1, 250, 3, ALT_XY_WINDOW_MS, 500);
  r.chassis.pid_odom_set({{0_in, 0.3_in}, fwd, 110});
  alternate_position(r, 0, 20000);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait() did not return within 15 s");
  CHECK_MESSAGE(ms < 4000.0, "returned after " << ms << " ms");
}

// The pid_wait_until() loops have the same shape: the gate releases on one pass, the next pass's recheck of the latched exit vetoes it.
TEST_CASE("pid_wait_until(distance) on a plain drive returns within 4 s on a pose that alternates between stopped and moving") {
  Rig r(archetype_classroom(), 1, false, 1);
  r.chassis.pid_drive_exit_condition_set(ALT_XY_WINDOW_MS, 1, 250, 3, ALT_XY_WINDOW_MS, 500);
  r.chassis.pid_drive_set(0.3_in, 110);
  alternate_position(r, 0, 20000);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until(0.25_in); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait_until() did not return within 15 s");
  CHECK_MESSAGE(ms < 4000.0, "returned after " << ms << " ms");
}

TEST_CASE("pid_wait_until(angle) on a turn returns within 4 s on a heading that alternates between stopped and moving") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_turn_exit_condition_set(ALT_WINDOW_MS, 1, 250, 3, ALT_WINDOW_MS, 500);
  r.chassis.pid_turn_set(0.5_deg, 110);
  alternate_heading(r, 0, 20000);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until(0.4_deg); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait_until() did not return within 15 s");
  CHECK_MESSAGE(ms < 4000.0, "returned after " << ms << " ms");
}

TEST_CASE("pid_wait_until(angle) on a swing returns within 4 s on a heading that alternates between stopped and moving") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_swing_exit_condition_set(ALT_WINDOW_MS, 1, 250, 3, ALT_WINDOW_MS, 500);
  r.chassis.pid_swing_set(ez::LEFT_SWING, 0.5_deg, 110);
  alternate_heading(r, 0, 20000);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until(0.4_deg); }, CAP_TICKS, &ms);
  CHECK_MESSAGE(ok, "pid_wait_until() did not return within 15 s");
  CHECK_MESSAGE(ms < 4000.0, "returned after " << ms << " ms");
}
