// pid_wait_until(point) and pid_wait_until_index() on an odom move count a checkpoint the robot has already passed as crossed.
//
// The wait used to read which side of the checkpoint the robot was on when it was called and then waited for that to change. A robot that
// was already past the checkpoint by then (a second wait on a point a third of an inch after the first, two points of a path that close
// together, a wait called a few hundred milliseconds late) never saw it change: the wait ran until the motion's own exits ended it, a second
// or more later, and called a motion that was driving the whole time interfered.
//
// Whether the robot is past is decided from the motion's own geometry: how far the robot is along the way into the checkpoint, from the point
// before it on the motion (where it started, for the first). A checkpoint that is not on the way, or that the robot has not reached, is
// waited for as ever.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

void idle(int ms) {
  for (int t = 0; t < ms; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
}

struct Archs {
  const char* name;
  sim::SimArchetype arch;
  int passes;
};

std::vector<Archs> robots() {
  return {{"classroom", archetype_classroom(), 1}, {"light_fast", sim::archetype_light_fast(), 1}, {"heavy_slow, 2 passes", sim::archetype_heavy_slow(), 2}};
}

}  // namespace

TEST_CASE("a second pid_wait_until on a point just past the first one returns at once, clean, with the motion still going") {
  for (const auto& a : robots())
    for (double gap : {0.3, 0.1, 0.6}) {
      Rig r(a.arch, a.passes);
      r.chassis.pid_print_toggle(true);
      std::string out;
      double first_ms = 0, second_ms = 0, y_second = 0;
      bool first_ok = false, second_ok = false;
      out = test_stub::capture_stdout([&]() {
        r.chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110});
        first_ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 12.0, 0.0}); }, 3000, &first_ms);
        second_ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 12.0 + gap, 0.0}); }, 3000, &second_ms);
        y_second = r.true_position().y;
      });
      INFO(a.name << ", checkpoints 12 and " << 12.0 + gap);
      MESSAGE(a.name, " gap ", gap, ": first ", first_ms, " ms, second ", second_ms, " ms, true y at second return ", y_second);
      REQUIRE(first_ok);
      REQUIRE(second_ok);
      CHECK_FALSE(r.chassis.interfered);
      // Crossed in truth, and not by much: the robot is a few hundredths to a few tenths of an inch beyond the second checkpoint
      CHECK(y_second >= 12.0 + gap - 0.2);
      CHECK(y_second < 12.0 + gap + 2.0);
      // The motion is still driving, and the second wait did not outlive the crossing by more than a poll or two
      CHECK(second_ms <= 120.0);
      // The rest of the motion is unharmed
      bool done = r.wait([&] { r.chassis.pid_wait(); }, 3000);
      REQUIRE(done);
      CHECK_FALSE(r.chassis.interfered);
      CHECK(std::fabs(r.true_position().y - 24.0) < 1.5);
    }
}

TEST_CASE("pid_wait_until_index on points a third of an inch apart returns on the second at once") {
  for (const auto& a : robots()) {
    Rig r(a.arch, a.passes);
    r.chassis.pid_print_toggle(true);
    double second_ms = 0, y_second = 0;
    bool ok1 = false, ok2 = false;
    test_stub::capture_stdout([&]() {
      r.chassis.pid_odom_pp_set({{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 12.3, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110}});
      ok1 = r.wait([&] { r.chassis.pid_wait_until_index(0); }, 3000);
      ok2 = r.wait([&] { r.chassis.pid_wait_until_index(1); }, 3000, &second_ms);
      y_second = r.true_position().y;
    });
    INFO(a.name);
    MESSAGE(a.name, ": second index wait ", second_ms, " ms, true y ", y_second);
    REQUIRE(ok1);
    REQUIRE(ok2);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(y_second >= 12.3 - 0.2);
    CHECK(second_ms <= 150.0);
  }
}

TEST_CASE("pid_wait_until called after the robot has gone past the point returns at once") {
  for (const auto& a : robots())
    for (int late_ms : {400, 800}) {
      Rig r(a.arch, a.passes);
      r.chassis.pid_print_toggle(true);
      double ms = 0, y = 0;
      bool ok = false;
      std::string out = test_stub::capture_stdout([&]() {
        r.chassis.pid_odom_ptp_set({{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110});
        idle(late_ms);
        y = r.true_position().y;
        ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 2.0, 0.0}); }, 3000, &ms);
      });
      INFO(a.name << ", " << late_ms << " ms late");
      MESSAGE(a.name, ", ", late_ms, " ms late: robot at ", y, ", wait ", ms, " ms: ", out);
      REQUIRE(ok);
      REQUIRE(y > 2.5);  // the robot really was past it
      CHECK_FALSE(r.chassis.interfered);
      CHECK(ms <= 60.0);
      CHECK(out.find("Wait Until Exit Success") != std::string::npos);
    }
}

// What the rule must not do: read a checkpoint the robot has not reached as crossed.
TEST_CASE("pid_wait_until on a point the robot has not reached waits for the crossing") {
  for (const auto& a : robots()) {
    Rig r(a.arch, a.passes);
    r.chassis.pid_print_toggle(true);
    double ms = 0;
    bool ok = false;
    test_stub::capture_stdout([&]() {
      r.chassis.pid_odom_ptp_set({{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
      ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 30.0, 0.0}); }, 3000, &ms);
    });
    INFO(a.name);
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(r.true_position().y >= 29.8);
    CHECK(r.true_position().y < 34.0);
  }
}

TEST_CASE("pid_wait_until on a point behind the start or beyond the end of the move is never read as crossed") {
  struct P {
    const char* name;
    double y;
  };
  const P points[] = {{"behind the start", -6.0}, {"beyond the end", 40.0}};
  for (const P& p : points)
    for (int late_ms : {0, 300}) {
      Rig r(archetype_classroom(), 1);
      r.chassis.pid_print_toggle(true);
      double ms = 0;
      bool ok = false;
      std::string out = test_stub::capture_stdout([&]() {
        r.chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110});
        idle(late_ms);
        ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, p.y, 0.0}); }, 6000, &ms);
      });
      INFO(p.name << ", " << late_ms << " ms late");
      MESSAGE(p.name, ", ", late_ms, " ms late: returned ", ok, " after ", ms, " ms at y ", r.true_position().y);
      REQUIRE(ok);
      CHECK(out.find("Wait Until Exit Success") == std::string::npos);
    }
}

// A point to the side of the move, called when the robot is level with it already: the robot is nowhere near it, so it is not crossed.
TEST_CASE("pid_wait_until on a point far to the side of the move is not read as crossed on the first look") {
  for (double x : {30.0, -30.0}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(true);
    double ms = 0, y_call = 0;
    bool ok = false;
    std::string out = test_stub::capture_stdout([&]() {
      r.chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110});
      while (r.true_position().y < 18.0) pros::delay(util::DELAY_TIME);
      y_call = r.true_position().y;
      ok = r.wait([&] { r.chassis.pid_wait_until(pose{x, 12.0, 0.0}); }, 6000, &ms);
    });
    INFO("x " << x);
    MESSAGE("x ", x, ": called at y ", y_call, ", returned ", ok, " after ", ms, " ms: ", out);
    REQUIRE(ok);
    CHECK(ms > 50.0);
  }
}

// The last leg of a path that comes back near its start ends at a point whose way in, from the corner before it, points back toward the
// start: the robot at the start is on the far side of that leg's plane, and has not been anywhere near it.
TEST_CASE("pid_wait_until on the last point of a U shaped path is not read as crossed at the start") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_print_toggle(true);
  double ms = 0;
  bool ok = false;
  std::string out = test_stub::capture_stdout([&]() {
    r.chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{24.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{24.0, 2.0, ANGLE_NOT_SET}, fwd, 110}});
    ok = r.wait([&] { r.chassis.pid_wait_until(pose{24.0, 2.0, 0.0}); }, 6000, &ms);
  });
  MESSAGE("returned ", ok, " after ", ms, " ms at ", r.true_position().x, ", ", r.true_position().y);
  REQUIRE(ok);
  CHECK(ms > 60.0);  // not at the first look, which is one poll after the call
}

// A boomerang that arrives pointing back across the line from where it started: heading 180 into (12, 24) from the origin. The robot swings
// out past the point and comes back around to it, so being on one side of it by the heading, or the line from the start, says nothing.
TEST_CASE("pid_wait_until on a boomerang's point is not read as crossed while the robot is still coming around to it") {
  for (int late_ms : {0, 250}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(true);
    double ms = 0, d_call = 0;
    bool ok = false;
    std::string out = test_stub::capture_stdout([&]() {
      r.chassis.pid_odom_boomerang_set(odom{pose{12.0, 24.0, 180.0}, fwd, 110});
      idle(late_ms);
      d_call = r.distance_to(12.0, 24.0);
      ok = r.wait([&] { r.chassis.pid_wait_until(pose{12.0, 24.0, 0.0}); }, 6000, &ms);
    });
    MESSAGE(late_ms, " ms late: ", d_call, " in from the point at the call, returned ", ok, " after ", ms, " ms, ", r.distance_to(12.0, 24.0), " in from it");
    REQUIRE(ok);
    REQUIRE(d_call > 6.0);
    CHECK(r.distance_to(12.0, 24.0) < 6.0);
  }
}

// A path that turns back passes a point twice. A robot that is past it on the way out has passed it, whether the path is only its corners
// or has points between them.
TEST_CASE("pid_wait_until on a point a path passes twice returns at once when the robot is past it on the way out") {
  for (bool injected : {false, true}) {
    Rig r(archetype_classroom(), 1);
    r.chassis.pid_print_toggle(true);
    bool ok = false;
    double ms = 0, y = 0;
    test_stub::capture_stdout([&]() {
      std::vector<odom> path = {{{0.0, 30.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 0.0, ANGLE_NOT_SET}, rev, 110}};
      if (injected)
        r.chassis.pid_odom_injected_pp_set(path);
      else
        r.chassis.pid_odom_pp_set(path);
      while (r.true_position().y < 18.0) pros::delay(util::DELAY_TIME);
      y = r.true_position().y;
      ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 12.0, 0.0}); }, 6000, &ms);
    });
    INFO("points between the corners: " << injected);
    MESSAGE("called at ", y, ", returned ", ok, " after ", ms, " ms");
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(ms <= 60.0);
  }
}

// A point on top of where the move started has no way in to be on one side of: the wait is as it always was, and ends.
TEST_CASE("pid_wait_until on the point the move started from returns") {
  Rig r(archetype_classroom(), 1);
  r.chassis.pid_print_toggle(true);
  bool ok = false;
  test_stub::capture_stdout([&]() {
    r.chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110});
    ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 0.0, 0.0}); }, 6000);
  });
  CHECK(ok);
}
