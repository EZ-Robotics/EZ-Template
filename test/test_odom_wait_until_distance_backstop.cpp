// pid_wait_until(distance) on an odom move that never reaches the distance ends with the motion, not in the middle of it.
//
// The sides' stuck watches on a wait_until() read the wheel distance still to go to the checkpoint. On a move whose path turns back (forward to
// 24 in, then in reverse to 6 in) a checkpoint at 30 in is never reached, and the distance to it first closes and then grows, which reads as no
// progress to a watch that only accepts new lows: the wait ended as interfered, a second into a motion that was still driving at 8 to 24 in/s.
// The same goes for a checkpoint on the wrong side of a reverse move.
//
// The stuck backstop of an odom wait_until() is the odom progress watch pid_wait() uses (progress along the path, not the wheels' distance to
// the checkpoint). The wait then ends when the robot crosses the checkpoint, when the motion settles (printing that the checkpoint can't be
// reached when it never could be, which is not an interference, exactly as the plain drive's wait does), or, when the robot is really stuck, as
// interfered.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Case {
  const char* name;
  sim::SimArchetype arch;
  int passes;
  bool team_exits;  // 2550R's exits: 90 ms / 1 in, 200 ms / 3 in, 100 ms velocity, 100 ms mA
  bool path;        // forward to (0, 24), then in reverse to (0, 6); otherwise one reverse move to (0, -24)
  double checkpoint;
  double final_y;
};

struct Outcome {
  bool returned;
  double elapsed;
  bool interfered;
  double off;    // true distance to the final target, in
  double speed;  // true speed over the last 100 ms, in/s
  std::string out;
};

Outcome run(const Case& c) {
  Rig r(c.arch, c.passes, false);
  r.chassis.pid_print_toggle(false);
  if (c.team_exits) {
    r.chassis.pid_odom_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
    r.chassis.pid_odom_turn_exit_condition_set(90, 3, 200, 7, 100, 100);
  }
  if (c.path)
    r.chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 6.0, ANGLE_NOT_SET}, rev, 110}});
  else
    r.chassis.pid_odom_ptp_set({{0.0, -24.0, ANGLE_NOT_SET}, rev, 110});
  Outcome o{};
  o.out = test_stub::capture_stdout([&]() { o.returned = r.wait([&]() { r.chassis.pid_wait_until(c.checkpoint); }, 6000, &o.elapsed); });
  o.interfered = r.chassis.interfered;
  o.off = r.distance_to(0.0, c.final_y);
  o.speed = r.drive_speed_over(100);
  return o;
}

}  // namespace

TEST_CASE("pid_wait_until(distance) on an odom move that never reaches it ends with the motion and says the checkpoint can't be reached") {
  const Case cases[] = {
      {"sticky, defaults, forward then reverse path, checkpoint 30", sim::archetype_sticky_high_friction(), 1, false, true, 30.0, 6.0},
      {"classroom, 2550R exits, forward then reverse path, checkpoint 30", archetype_classroom(), 1, true, true, 30.0, 6.0},
      {"heavy_slow, 2 passes, reverse move, checkpoint 12 (wrong sign)", sim::archetype_heavy_slow(), 2, false, false, 12.0, -24.0},
  };
  for (const Case& c : cases) {
    Outcome o = run(c);
    INFO(c.name);
    MESSAGE(c.name, ": returned=", o.returned, " elapsed=", o.elapsed, " interfered=", o.interfered, " true distance to the final target=", o.off,
            " in, speed over 100 ms=", o.speed, " in/s");
    REQUIRE(o.returned);
    // Never mid motion: the robot is at the end of the move and stopped
    CHECK(o.off < 3.0);
    CHECK(o.speed < FLOOR_DISTANCE + 1.0);
    CHECK_FALSE(o.interfered);
    CHECK(o.out.find("can't be reached") != std::string::npos);
  }
}

// A robot that really is stuck still ends the wait as interfered, and in bounded time, whatever the checkpoint.
TEST_CASE("pid_wait_until(distance) on an odom move still ends as interfered when the robot is pinned short of the checkpoint") {
  for (double checkpoint : {30.0, 60.0}) {
    Rig r(archetype_classroom(), 1, false);
    r.chassis.pid_print_toggle(false);
    r.sim.pin(300, 60000);
    r.chassis.pid_odom_ptp_set({{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
    double elapsed = 0;
    bool ok = r.wait([&]() { r.chassis.pid_wait_until(checkpoint); }, 6000, &elapsed);
    CAPTURE(checkpoint);
    MESSAGE("checkpoint ", checkpoint, ": returned=", ok, " elapsed=", elapsed, " interfered=", r.chassis.interfered);
    REQUIRE(ok);
    CHECK(r.chassis.interfered);
    CHECK(elapsed < 15000);
  }
}

TEST_CASE("pid_wait_until(distance) on an odom move still ends on the crossing of a checkpoint it reaches") {
  Rig r(archetype_classroom(), 1, false);
  r.chassis.pid_print_toggle(false);
  r.chassis.pid_odom_ptp_set({{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
  double elapsed = 0;
  bool ok = r.wait([&]() { r.chassis.pid_wait_until(30.0); }, 6000, &elapsed);
  REQUIRE(ok);
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.trace.back().avg >= 29.0);
  CHECK(r.trace.back().avg < 36.0);  // ended on the crossing, not with the move
}
