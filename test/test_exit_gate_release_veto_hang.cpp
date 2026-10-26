// pid_wait() and pid_wait_until() must return on a robot whose readings say "stopped" on one pass and "moving" on the next: a buzzing
// drive, or a pinned robot whose sensor is made to jump up and back down every few passes. Exits that latch once and are never rechecked
// end each of these.
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

// A pinned heading that jumps up by ALT_JUMP_DEG and back down 20 ms later, every 40 ms: a reading that flickers between moving and
// stopped on consecutive passes. The robot is pinned for the whole run so that only the scripted jumps move it.
constexpr int ALT_WINDOW_MS = 90;
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

// The same for the odom pose and the drive sensors, on the classroom robot: a jump the sensor reads as a few counts, up and back down.
// The heading never moves.
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

// The pid_wait_until() loops on the same flickering readings
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

// A healthy 6 in drive at the corpus exits (90/1/200/3/100/100) ends on the velocity exit (100 ms, at 5 in/s) once the robot is inside big_error, and the robot
// is under the stop speed by then, so it is clean. The velocity exit reads the library's speed over 100 ms windows, so it can only start counting at the end of
// the first window the robot spent under 5 in/s, up to two windows after the robot really got under it. The bound is that, plus the velocity exit time, plus
// two passes.
TEST_CASE("a healthy 6 in drive at the corpus exits ends clean once the robot is under 5 in/s: no later than the velocity exit allows, stopped") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
  double ms = 0;
  r.chassis.pid_drive_set(6_in, 40);
  bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
  REQUIRE(ok);
  CHECK_FALSE(r.chassis.interfered);
  // When the robot's true speed first got under the velocity exit's 5 in/s and stayed there
  double slow_at = -1;
  for (size_t i = 1; i < r.trace.size(); i++) {
    double v = std::fabs(r.trace[i].avg - r.trace[i - 1].avg) / ((r.trace[i].t_ms - r.trace[i - 1].t_ms) / 1000.0);
    if (v > 5.0)
      slow_at = -1;
    else if (slow_at < 0)
      slow_at = r.trace[i].t_ms;
  }
  REQUIRE(slow_at >= 0);
  // The trace ends on the pass the wait returned on, in the sim's own time
  double returned_at = r.trace.back().t_ms;
  double bound = slow_at + 2 * 100 + 100 + 2 * util::DELAY_TIME;
  CHECK_MESSAGE(returned_at <= bound, "returned at " << returned_at << " ms, the robot was under 5 in/s from " << slow_at << " ms");
  CHECK_MESSAGE(r.drive_speed_over(100) < r.drive_floor(100), "still moving at " << r.drive_speed_over(100) << " in/s");
  auto after = r.run_on(500);
  CHECK_MESSAGE(after.distance < 0.3, "moved " << after.distance << " in in the 500 ms after it returned");
}
