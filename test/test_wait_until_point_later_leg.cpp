// pid_wait_until(point) on a pure pursuit path. The point is crossed when the robot has got to it along the path, not when it is level with it:
//  - a checkpoint on a later leg than the one the robot is driving is not crossed when the robot is abeam of it. The sign of the side of the
//    point the robot is on flipped as soon as the look-ahead moved on to the next leg, so a wait for a point on the last leg of a hairpin came
//    back clean within 20 to 400 ms with the robot 8 to 30 in short of it;
//  - a checkpoint on an earlier leg than the one the robot is now driving has been crossed, wherever on the new leg the robot is. A wait called
//    for it late, with the robot on a leg that runs back past it, ended interfered with the robot still driving.
#include <cstdio>
#include <functional>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

odom Q(double x, double y, drive_directions d = fwd) { return odom{pose{x, y, ANGLE_NOT_SET}, d, 110}; }

struct Res {
  bool ret;
  bool interfered;
  double ms;
  double call_x, call_y;  // where the robot really was when the wait was called
  double end_x, end_y;    // ...and when it came back
  double end_speed;
};

// The robot drives `path`; `pre` ms later pid_wait_until(checkpoint) is called
Res run(const sim::SimArchetype& arch, const std::vector<odom>& path, int pre, pose checkpoint) {
  Rig r(arch, 1, false, 1);
  r.chassis.pid_odom_pp_set(path);
  for (int t = 0; t < pre; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
  Res res{};
  auto p = r.true_position();
  res.call_x = p.x;
  res.call_y = p.y;
  res.ret = r.wait([&] { r.chassis.pid_wait_until(checkpoint); }, 2500, &res.ms);
  res.interfered = r.chassis.interfered;
  p = r.true_position();
  res.end_x = p.x;
  res.end_y = p.y;
  res.end_speed = r.drive_speed_over(100);
  return res;
}

}  // namespace

namespace {

bool near(const Res& r, double x, double y, double within) { return std::hypot(r.end_x - x, r.end_y - y) <= within; }

}  // namespace

TEST_CASE("pid_wait_until(point) for a point on a later leg waits for the robot to get there") {
  struct Case {
    const char* name;
    std::vector<odom> path;
    double x, y;
    int pre;
  };
  std::vector<odom> square = {Q(0, 30), Q(30, 30), Q(30, 0)};
  std::vector<odom> hairpin = {Q(0, 40), Q(12, 40), Q(12, 0)};
  std::vector<Case> cases = {
      {"square, last point", square, 30, 0, 0},      {"square, last point", square, 30, 0, 600},      {"square, mid last leg", square, 30, 10, 0},
      {"square, mid last leg", square, 30, 10, 600}, {"hairpin, last point", hairpin, 12, 0, 0},      {"hairpin, last point", hairpin, 12, 0, 600},
      {"hairpin, mid last leg", hairpin, 12, 10, 0}, {"hairpin, mid last leg", hairpin, 12, 10, 600},
  };
  for (const auto& c : cases) {
    Res r = run(archetype_classroom(), c.path, c.pre, pose{c.x, c.y, ANGLE_NOT_SET});
    CAPTURE(c.name);
    CAPTURE(c.pre);
    REQUIRE(r.ret);
    CHECK_FALSE(r.interfered);
    CHECK_MESSAGE(near(r, c.x, c.y, 8.0), "came back at (" << r.end_x << ", " << r.end_y << "), " << std::hypot(r.end_x - c.x, r.end_y - c.y)
                                                           << " in short of (" << c.x << ", " << c.y << ") after " << r.ms << " ms");
  }
}

TEST_CASE("pid_wait_until(point) for a point on an earlier leg, called with the robot on a later leg, has been crossed") {
  struct Case {
    const char* name;
    std::vector<odom> path;
    double x, y;
    int pre;
  };
  std::vector<odom> hairpin = {Q(0, 36), Q(12, 36), Q(12, 6)};
  std::vector<odom> u = {Q(0, 24), Q(24, 24), Q(24, 2)};
  std::vector<Case> cases = {{"hairpin", hairpin, 0, 30, 2300},
                             {"hairpin", hairpin, 0, 30, 2500},
                             {"hairpin", hairpin, 0, 30, 2700},
                             {"hairpin", hairpin, 0, 30, 2900},
                             {"u", u, 0, 20, 2200},
                             {"u", u, 0, 20, 2400},
                             {"u", u, 0, 20, 2600},
                             {"u", u, 0, 12, 2600}};
  for (const auto& c : cases) {
    Res r = run(archetype_classroom(), c.path, c.pre, pose{c.x, c.y, ANGLE_NOT_SET});
    CAPTURE(c.name);
    CAPTURE(c.pre);
    REQUIRE(r.ret);
    CHECK_MESSAGE(!r.interfered,
                  "ended interfered after " << r.ms << " ms with the robot at (" << r.end_x << ", " << r.end_y << ") doing " << r.end_speed << " in/s");
    CHECK_MESSAGE(r.ms <= 100.0, "took " << r.ms << " ms");
  }
}
