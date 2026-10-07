// Inside big_error a wait that sees no new low for max(the team's window, step / stop speed) used to call the robot settled. That is the
// ending for a robot hunting back and forth about its target, and it is only that: a robot that is being dragged through its target (or
// away from it) by something else makes no new low either, and was called settled while it was moving at 2 to 3 in/s, up to 2.3 in off,
// or at 8 deg/s and 4.6 deg off. The backstop now calls the robot settled only when its NET displacement over the backstop window (where it
// is now against where it was at the start of the window, a distance and not a path length) is under the stop speed times the window, which
// is a robot that is oscillating in place. A robot that is going somewhere leaves big_error, and the ordinary stuck clock outside it ends the
// wait as interfered.
//
// (The sim reports heading counterclockwise positive, so a library heading of 90 is -90 in the trace.)
// The robot is carried by the sim from the first moment it is inside 0.5 in (1.2 deg) of its target, at a speed that is not
// something its motors can change. Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

constexpr int CAP_TICKS = 2000;  // 20 s of sim time

// The sim time at which a motion first has `field` within `band` of `target`, found by running it undisturbed. The carried run follows the
// same path up to that instant (no noise, same archetype), so a carry that starts then starts from within the band.
template <typename Set>
double first_within(const sim::SimArchetype& a, int passes, Set set, double Sample::*field, double target, double band) {
  Rig r(a, passes, false, 1);
  set(r.chassis);
  r.run_on(4000);
  for (const auto& s : r.trace)
    if (std::fabs(s.*field - target) < band) return s.t_ms;
  return -1.0;
}

struct Result {
  bool returned;
  bool interfered;
  double ms;
  double position;  // where the robot really was, wheel average (inches) or heading (degrees)
};

}  // namespace

TEST_CASE("a robot carried through its drive target is not called settled while it is moving") {
  for (double v : {2.0, 3.0, -3.0}) {
    auto set = [](Drive& c) { c.pid_drive_set(24_in, 110); };
    double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::avg, 24.0, 0.5);
    REQUIRE(t0 > 0);
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.sim.carry(v, 0.0, t0, 30000);
    set(r.chassis);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
    CAPTURE(v);
    REQUIRE(ok);
    CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms with the robot " << r.trace.back().avg - 24.0 << " in from the target");
    CHECK_MESSAGE(ms < 8000.0, "returned after " << ms << " ms");
  }
}

TEST_CASE("a robot carried through its turn target is not called settled while it is turning") {
  for (double w : {8.0, -8.0}) {
    auto set = [](Drive& c) { c.pid_turn_set(90_deg, 110); };
    double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::heading, -90.0, 1.2);
    REQUIRE(t0 > 0);
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.sim.carry(0.0, w, t0, 30000);
    set(r.chassis);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
    CAPTURE(w);
    REQUIRE(ok);
    CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms with the robot " << r.trace.back().heading + 90.0 << " deg from the target");
    CHECK_MESSAGE(ms < 8000.0, "returned after " << ms << " ms");
  }
}

TEST_CASE("a robot carried through its swing target is not called settled while it is swinging") {
  for (double w : {8.0, -8.0}) {
    auto set = [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0); };
    double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::heading, -90.0, 1.2);
    REQUIRE(t0 > 0);
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.sim.carry(0.0, w, t0, 30000);
    set(r.chassis);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
    CAPTURE(w);
    REQUIRE(ok);
    CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms with the robot " << r.trace.back().heading + 90.0 << " deg from the target");
    CHECK_MESSAGE(ms < 8000.0, "returned after " << ms << " ms");
  }
}

TEST_CASE("a robot carried through its odom point is not called settled while it is moving") {
  for (double v : {2.0, -3.0}) {
    auto set = [](Drive& c) { c.pid_odom_set(odom{pose{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}); };
    double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::avg, 24.0, 0.5);
    REQUIRE(t0 > 0);
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.sim.carry(v, 0.0, t0, 30000);
    set(r.chassis);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
    CAPTURE(v);
    REQUIRE(ok);
    CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms with the robot " << r.trace.back().avg - 24.0 << " in from the target");
    CHECK_MESSAGE(ms < 8000.0, "returned after " << ms << " ms");
  }
}

TEST_CASE("a robot carried away from the checkpoint at the end of its drive is not called settled by pid_wait_until()") {
  auto set = [](Drive& c) { c.pid_drive_set(24_in, 110); };
  double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::avg, 24.0, 0.5);
  REQUIRE(t0 > 0);
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.sim.carry(-3.0, 0.0, t0, 30000);
  set(r.chassis);
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until(24_in); }, CAP_TICKS, &ms);
  REQUIRE(ok);
  CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms with the robot " << r.trace.back().avg - 24.0 << " in from the target");
  CHECK_MESSAGE(ms < 8000.0, "returned after " << ms << " ms");
}

TEST_CASE("a robot carried just over the stop speed for a long time still ends, interfered, in bounded time") {
  // 1.8 in/s and 5 deg/s: over the 1.5 in/s and 4 deg/s floors by a hair, slow enough that it stays inside big_error for seconds. It has to leave
  // it (big_error is 3 in / 7 deg) and the stuck clock outside it ends the wait.
  {
    auto set = [](Drive& c) { c.pid_drive_set(24_in, 110); };
    double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::avg, 24.0, 0.5);
    REQUIRE(t0 > 0);
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.sim.carry(1.8, 0.0, t0, 60000);
    set(r.chassis);
    double ms = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms));
    CHECK(r.chassis.interfered);
    CHECK_MESSAGE(ms < 10000.0, "returned after " << ms << " ms");
  }
  {
    auto set = [](Drive& c) { c.pid_turn_set(90_deg, 110); };
    double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::heading, -90.0, 1.2);
    REQUIRE(t0 > 0);
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.sim.carry(0.0, 5.0, t0, 60000);
    set(r.chassis);
    double ms = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms));
    CHECK(r.chassis.interfered);
    CHECK_MESSAGE(ms < 10000.0, "returned after " << ms << " ms");
  }
}

TEST_CASE("a robot hunting about its target still ends clean at a stop speed low enough that the backstop outlasts the tracker's history") {
  // light_fast with a P of 3000 and no D chatters about a 24 in target at 0.43 in peak to peak. At 0.3 in/s the backstop is 1 in / 0.3 in/s =
  // 3.3 s, longer than the 2.5 s the tracker keeps, so the net displacement has to be answered over what it has stored: a robot that cannot
  // be told about is not one that may be held for ever.
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_drive_constants_set(3000.0, 0.0, 0.0);
  r.chassis.pid_drive_exit_stop_speed_set(0.3);
  r.chassis.pid_drive_set(24_in, 110);
  double ms = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(std::fabs(24.0 - r.trace.back().avg) < 1.0);
  CHECK_MESSAGE(ms < 12000.0, "returned after " << ms << " ms");
}

TEST_CASE("a robot carried over a low stop speed for a long time ends in bounded time even when the backstop outlasts the tracker's history") {
  auto set = [](Drive& c) {
    c.pid_drive_exit_stop_speed_set(0.3);
    c.pid_drive_set(24_in, 110);
  };
  double t0 = first_within(sim::archetype_light_fast(), 1, set, &Sample::avg, 24.0, 0.5);
  REQUIRE(t0 > 0);
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.sim.carry(1.0, 0.0, t0, 120000);
  set(r.chassis);
  double ms = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 4000, &ms));
  CHECK(r.chassis.interfered);
  CHECK_MESSAGE(ms < 30000.0, "returned after " << ms << " ms");
}
