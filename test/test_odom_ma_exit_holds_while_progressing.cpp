// An odom wait holds an mA exit while the robot is still getting somewhere, on the path and in its heading.
//
// Over current alone does not end a wait while the robot is moving: the exit is only taken once the robot has stopped, or has stopped
// getting anywhere (see ExitGate::take_mA()). Before the last point of a pure pursuit path "getting anywhere" was read as the distance to the
// point the robot is driving to, which is always about one look ahead away and does not change while the robot follows the path, so a burst of
// over current anywhere mid-path ended the wait at the next mA window with the robot driving at full speed (point 56 to 65 of 131 on a 72 in move,
// at 15.8 in/s). A robot that turns first, before it translates, makes no progress in distance for as long as it turns, so the same burst in the
// first few hundred milliseconds ended a 180 degree turn at 210 ms with the robot turning at 276 deg/s.
//
// Progress is the path left (the distance to the point being driven to plus the length of the path after it) and the heading error: the exit is
// held while either comes down by its stop speed over the window. A robot that gets nowhere on either still ends at mA_timeout.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// 2550R's exits, mA_timeout 100 ms
void team_exits(Drive& c) {
  c.pid_odom_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
  c.pid_odom_turn_exit_condition_set(90, 3, 200, 7, 100, 100);
  c.pid_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
  c.pid_turn_exit_condition_set(90, 3, 200, 7, 100, 100);
}

// Every drive motor reports over current from from_ms to to_ms of sim time, whatever the sim says
// (the sim writes the motors' over current flag at the end of each tick, so the flag is set after the sim's own tick, as the waits read it)
sim::SimRobot* g_sim = nullptr;
Drive* g_drive = nullptr;
void (*g_sim_tick)() = nullptr;
double g_from_ms = 0, g_to_ms = 0;

void burst_hook() {
  g_sim_tick();
  double t = g_sim->now_ms();
  if (t >= g_from_ms && t < g_to_ms) {
    for (auto& m : g_drive->left_motors) m.fake().over_current = true;
    for (auto& m : g_drive->right_motors) m.fake().over_current = true;
  }
}

struct Burst {
  Burst(Rig& r, double from_ms, double to_ms) {
    g_sim = &r.sim;
    g_drive = &r.chassis;
    g_sim_tick = test_stub::g_clock.on_delay;
    g_from_ms = from_ms;
    g_to_ms = to_ms;
    test_stub::g_clock.on_delay = &burst_hook;
  }
  ~Burst() { test_stub::g_clock.on_delay = g_sim_tick; }
};

enum class Move {
  PathForward,    // pid_odom_set({0, 72}): pure pursuit, ~131 points
  PathTurnFirst,  // pid_odom_set({0, -24}, fwd): a 180 degree turn, then the drive
  PointForward,   // pid_odom_ptp_set({0, 72})
  PointTurnFirst  // pid_odom_ptp_set({0, -24}, fwd)
};

struct Wait {
  const char* name;
  std::function<void(Drive&)> run;
};

struct Outcome {
  bool returned = false;
  bool interfered = false;
  double ms = 0;
  double off = 0;    // true distance to the end of the move, in
  double speed = 0;  // true speed over the last 100 ms, in/s
  double turn = 0;   // true turn rate over the last 100 ms, deg/s
  std::string out;
};

double end_y(Move m) { return (m == Move::PathForward || m == Move::PointForward) ? 72.0 : -24.0; }

Outcome run(const sim::SimArchetype& a, int passes, Move m, const Wait& w, double from_ms, double to_ms) {
  Rig r(a, passes);
  r.chassis.pid_print_toggle(true);
  team_exits(r.chassis);
  Burst burst(r, from_ms, to_ms);
  Outcome o;
  o.out = test_stub::capture_stdout([&]() {
    switch (m) {
      case Move::PathForward:
        r.chassis.pid_odom_set({{0_in, 72_in}, fwd, 110});
        break;
      case Move::PathTurnFirst:
        r.chassis.pid_odom_set({{0_in, -24_in}, fwd, 127});
        break;
      case Move::PointForward:
        r.chassis.pid_odom_ptp_set({{0.0, 72.0, ANGLE_NOT_SET}, fwd, 110});
        break;
      case Move::PointTurnFirst:
        r.chassis.pid_odom_ptp_set({{0.0, -24.0, ANGLE_NOT_SET}, fwd, 127});
        break;
    }
    o.returned = r.wait([&] { w.run(r.chassis); }, 3000, &o.ms);
  });
  o.interfered = r.chassis.interfered;
  o.off = r.distance_to(0.0, end_y(m));
  o.speed = r.drive_speed_over(100);
  o.turn = r.angle_speed_over(100);
  return o;
}

const Wait WAITS[] = {
    {"pid_wait", [](Drive& c) { c.pid_wait(); }},
    {"pid_wait_quick", [](Drive& c) { c.pid_wait_quick(); }},
};

}  // namespace

TEST_CASE("a burst of over current mid path does not end a pure pursuit wait while the robot is driving the path") {
  struct Window {
    double from, to;
  };
  for (const Wait& w : WAITS)
    for (Window b : {Window{400, 800}, Window{500, 600}, Window{300, 1000}}) {
      Outcome o = run(archetype_classroom(), 1, Move::PathForward, w, b.from, b.to);
      INFO(w.name << ", burst " << b.from << " to " << b.to << " ms");
      MESSAGE(w.name, " burst ", b.from, "-", b.to, ": returned ", o.returned, " at ", o.ms, " ms interfered ", o.interfered, ", ", o.off, " in from the end, ",
              o.speed, " in/s");
      REQUIRE(o.returned);
      // The path is 72 in: it is driven to its end, clean
      CHECK(o.off < 3.0);
      CHECK_FALSE(o.interfered);
    }
}

TEST_CASE("a burst of over current while the robot turns to face the way it goes does not end the wait while it is turning") {
  for (Move m : {Move::PathTurnFirst, Move::PointTurnFirst})
    for (const Wait& w : WAITS) {
      // Over current for the first 300 ms, the whole of a 180 degree turn at speed 127
      Outcome o = run(archetype_classroom(), 1, m, w, 0, 300);
      INFO((m == Move::PathTurnFirst ? "path" : "point") << ", " << w.name);
      MESSAGE((m == Move::PathTurnFirst ? "path" : "point"), ", ", w.name, ": returned ", o.returned, " at ", o.ms, " ms interfered ", o.interfered, ", ",
              o.off, " in from the end, ", o.speed, " in/s, turning ", o.turn, " deg/s");
      REQUIRE(o.returned);
      CHECK(o.off < 3.0);
      CHECK_FALSE(o.interfered);
    }
}

TEST_CASE("a burst of over current while the robot turns first does not end pid_wait_until(point) or pid_wait_until_index while it is turning") {
  for (bool index : {false, true}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(false);
    team_exits(r.chassis);
    Burst burst(r, 0, 300);
    if (index)
      r.chassis.pid_odom_set({{{0_in, -12_in}, fwd, 127}, {{0_in, -24_in}, fwd, 127}});
    else
      r.chassis.pid_odom_set({{0_in, -24_in}, fwd, 127});
    double ms = 0;
    bool ok = r.wait(
        [&] {
          if (index)
            r.chassis.pid_wait_until_index(0);
          else
            r.chassis.pid_wait_until(pose{0.0, -12.0, 0.0});
        },
        3000, &ms);
    INFO((index ? "pid_wait_until_index" : "pid_wait_until(point)"));
    MESSAGE((index ? "index" : "point"), ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", y ", r.true_position().y);
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.true_position().y <= -11.5);
  }
}

TEST_CASE("pid_wait_until(point) on a pure pursuit path before its last point does not end on a burst of over current while the robot is driving") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_print_toggle(false);
  team_exits(r.chassis);
  Burst burst(r, 400, 800);
  r.chassis.pid_odom_set({{0_in, 72_in}, fwd, 110});
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 60.0, 0.0}); }, 3000, &ms);
  MESSAGE("returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", y ", r.true_position().y);
  REQUIRE(ok);
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.true_position().y >= 59.5);
}

TEST_CASE("pid_wait_until(distance) on a pure pursuit path does not end on a burst of over current while the robot is driving") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_print_toggle(false);
  team_exits(r.chassis);
  Burst burst(r, 400, 800);
  r.chassis.pid_odom_set({{0_in, 72_in}, fwd, 110});
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until(60.0); }, 3000, &ms);
  MESSAGE("returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", y ", r.true_position().y);
  REQUIRE(ok);
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.true_position().y >= 59.0);
}

TEST_CASE("pid_wait_until_index on a pure pursuit path does not end on a burst of over current while the robot is driving") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_print_toggle(false);
  team_exits(r.chassis);
  Burst burst(r, 400, 800);
  r.chassis.pid_odom_set({{{0_in, 36_in}, fwd, 110}, {{0_in, 72_in}, fwd, 110}});
  double ms = 0;
  bool ok = r.wait([&] { r.chassis.pid_wait_until_index(0); }, 3000, &ms);
  MESSAGE("returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", y ", r.true_position().y);
  REQUIRE(ok);
  CHECK_FALSE(r.chassis.interfered);
  CHECK(r.true_position().y >= 35.0);
}

// What the hold must not do: keep a robot that is getting nowhere. Each is capped in sim time, so "no return" is a failure and not a hung suite.
TEST_CASE("a pure pursuit robot that is pinned while over current still ends on mA close to mA_timeout") {
  for (Move m : {Move::PathForward, Move::PointForward})
    for (const Wait& w : WAITS) {
      Rig r(archetype_classroom(), 1);
      r.chassis.pid_print_toggle(false);
      team_exits(r.chassis);
      r.sim.pin(500, 60000);
      Burst burst(r, 500, 60000);
      if (m == Move::PathForward)
        r.chassis.pid_odom_set({{0_in, 72_in}, fwd, 110});
      else
        r.chassis.pid_odom_ptp_set({{0.0, 72.0, ANGLE_NOT_SET}, fwd, 110});
      double ms = 0;
      bool ok = r.wait([&] { w.run(r.chassis); }, 3000, &ms);
      INFO((m == Move::PathForward ? "path" : "point") << ", " << w.name);
      MESSAGE((m == Move::PathForward ? "path" : "point"), ", ", w.name, ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered);
      REQUIRE(ok);
      CHECK(r.chassis.interfered);
      // 500 ms to contact, mA_timeout, and the window it has to read stopped over, which is the same length, and a few polls
      CHECK(ms < 500 + 100 + 100 + 150);
    }
}

TEST_CASE("a pure pursuit robot that is dragged away from the path while over current still ends, as interfered, in bounded time") {
  for (const Wait& w : WAITS) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(false);
    team_exits(r.chassis);
    r.sim.carry(-15.0, 0.0, 500, 60000);
    Burst burst(r, 500, 60000);
    r.chassis.pid_odom_set({{0_in, 72_in}, fwd, 110});
    double ms = 0;
    bool ok = r.wait([&] { w.run(r.chassis); }, 3000, &ms);
    INFO(w.name);
    MESSAGE(w.name, ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered);
    REQUIRE(ok);
    CHECK(r.chassis.interfered);
    CHECK(ms < 4000.0);
  }
}

// The hold is bounded by what there is to make progress on: a robot that is over current for the whole move and drives it ends with it
TEST_CASE("a pure pursuit robot over current for the whole move ends, however long the path") {
  for (double length : {24.0, 72.0}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(false);
    team_exits(r.chassis);
    Burst burst(r, 0, 60000);
    r.chassis.pid_odom_set({{0_in, length * 1_in}, fwd, 110});
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, 6000, &ms);
    MESSAGE("length ", length, ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", ", r.distance_to(0.0, length), " in from the end");
    REQUIRE(ok);
    // It drove the path to its end and stopped there, and the wait ended with the motion (the 72 in move takes 3 s)
    CHECK(r.distance_to(0.0, length) < 3.0);
    CHECK(ms < 5000.0);
  }
}

// Each progress is judged from its own lowest value. A robot that is held in place and turned toward the way it has to face is getting
// somewhere on its heading for as long as it is turned, and is held for that long; turned back it is making no progress on anything (its heading
// is no lower than the lowest it reached), and the exit is taken within a window or two of the turn back starting, not when it has got back to
// where it was. Turned the other way it is getting nowhere from the start.
TEST_CASE("a robot that is held in place and turned toward its heading is held while it is turned and not once it is turned back") {
  struct Turn {
    double rate;  // deg/s, positive turns the robot toward the heading it has to take
    bool toward;
  };
  for (Turn t : {Turn{40.0, true}, Turn{-40.0, false}}) {
    double rate = t.rate;
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(false);
    team_exits(r.chassis);
    Burst burst(r, 0, 60000);
    r.sim.pin(100, 60000);
    // Turned for 800 ms, then back for as long as the move lasts. Which way turns toward the heading depends on the sign, one of the two does.
    r.sim.carry(0.0, rate, 100, 800);
    r.sim.carry(0.0, -rate, 900, 60000);
    r.chassis.pid_odom_set({{0_in, -24_in}, fwd, 127});
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
    INFO("rate " << rate);
    MESSAGE("rate ", rate, ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", heading error ", r.chassis.current_a_odomPID.error);
    REQUIRE(ok);
    CHECK(r.chassis.interfered);
    if (t.toward) {
      CHECK(ms >= 900.0);
      CHECK(ms < 1400.0);
    } else {
      CHECK(ms < 600.0);
    }
  }
}

// pid_wait_until(distance) watches each side's wheel distance to the checkpoint, which grows on a path that turns back. Over current while
// the robot is driving the way back must not end the wait with the robot driving at full speed.
TEST_CASE("pid_wait_until(distance) on a path that turns back does not end on a burst of over current while the robot is driving") {
  for (double from : {600.0, 800.0, 1000.0}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(false);
    team_exits(r.chassis);
    Burst burst(r, from, from + 400);
    r.chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 6.0, ANGLE_NOT_SET}, rev, 110}});
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait_until(30.0); }, 3000, &ms);
    INFO("burst from " << from);
    MESSAGE("burst from ", from, ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered, ", ", r.distance_to(0.0, 6.0), " in from the end, ",
            r.drive_speed_over(100), " in/s");
    REQUIRE(ok);
    CHECK(r.distance_to(0.0, 6.0) < 3.0);
    // Clean, and not with the robot still driving into its last inches
    CHECK(r.drive_speed_over(100) < FLOOR_DISTANCE + 1.0);
    CHECK_FALSE(r.chassis.interfered);
  }
}

// What the hold must not do on this wait either: keep a robot that is getting nowhere. Pinned, or dragged away from the move, while over current
// it ends as interfered in bounded time, close to the mA window of being stopped.
TEST_CASE("pid_wait_until(distance) on an odom path ends in bounded time when the robot is pinned or dragged away while over current") {
  for (bool pinned : {true, false}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(false);
    team_exits(r.chassis);
    Burst burst(r, 500, 60000);
    if (pinned)
      r.sim.pin(500, 60000);
    else
      r.sim.carry(-15.0, 0.0, 500, 60000);
    r.chassis.pid_odom_set({{0_in, 72_in}, fwd, 110});
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait_until(60.0); }, 3000, &ms);
    const char* label = pinned ? "pinned" : "dragged away";
    INFO(label);
    MESSAGE(label, ": returned ", ok, " at ", ms, " ms, interfered ", r.chassis.interfered);
    REQUIRE(ok);
    CHECK(r.chassis.interfered);
    CHECK(ms < (pinned ? 500 + 100 + 100 + 150 : 4000.0));
  }
}
