// pid_wait_until(point) for a point on a leg of a pure pursuit path that the robot has not reached yet, called while the look-ahead has already
// rounded the corner onto that leg but the robot is still on the leg before. The robot is not past the point just because it is on the far side of
// the plane through the point that is square to the point's leg: that plane lies behind the robot for any turn of more than 90 degrees, and ahead of
// the corner for a lesser one only when the robot is already on the new leg. Such a wait came back clean after 10 ms with the robot 8 to 24 in short.
#include <cmath>
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
  return res;
}

struct Case {
  const char* name;
  std::vector<odom> path;
  double x, y;
  int pre;
};

void check(const sim::SimArchetype& arch, const std::vector<Case>& cases) {
  for (const auto& c : cases) {
    Res r = run(arch, c.path, c.pre, pose{c.x, c.y, ANGLE_NOT_SET});
    CAPTURE(c.name);
    CAPTURE(c.pre);
    REQUIRE(r.ret);
    CHECK_FALSE(r.interfered);
    double short_by = std::hypot(r.end_x - c.x, r.end_y - c.y);
    CHECK_MESSAGE(short_by <= 8.0, "called with the robot at (" << r.call_x << ", " << r.call_y << "), came back after " << r.ms << " ms at (" << r.end_x
                                                                << ", " << r.end_y << "), " << short_by << " in short of (" << c.x << ", " << c.y << ")");
  }
}

// Points `along` inches past a corner at (0, 24) on a last leg that runs (cos, -sin) of 45 degrees
std::vector<Case> sharp_turn_cases() {
  std::vector<odom> path = {Q(0, 24), Q(17, 7)};
  std::vector<Case> cases;
  for (int pre : {0, 300})
    for (double along : {1.0, 4.0, 9.0}) cases.push_back({"135 degree turn", path, along * 0.7071, 24 - along * 0.7071, pre});
  return cases;
}

}  // namespace

TEST_CASE("pid_wait_until(point) on the leg after a turn of more than 90 degrees waits for the robot to get there") {
  check(archetype_classroom(), sharp_turn_cases());
  check(sim::archetype_light_fast(), sharp_turn_cases());
}

TEST_CASE("pid_wait_until(point) on the leg after a hairpin waits for the robot to get there") {
  // The point is on the way back, where the start of the path lies on the far side of its square-on plane
  check(archetype_classroom(), {{"hairpin", {Q(0, 30), Q(3, 0)}, 1.5, 15, 0}, {"hairpin", {Q(0, 30), Q(3, 0)}, 1.5, 15, 300}});
}

TEST_CASE("pid_wait_until(point) just past a right angle corner waits for the robot to get there when called before it has turned") {
  std::vector<odom> u = {Q(0, 24), Q(24, 24), Q(24, 2)};
  std::vector<Case> cases;
  for (int pre : {700, 1000, 1300}) cases.push_back({"right angle", u, 24, 22.5, pre});
  check(archetype_classroom(), cases);
  std::vector<Case> fast;
  for (int pre : {300, 400}) fast.push_back({"right angle", u, 24, 22.5, pre});
  check(sim::archetype_light_fast(), fast);
}

TEST_CASE("pid_wait_until(point) on the first leg of a path with more legs waits for the robot to get there") {
  std::vector<odom> u = {Q(0, 24), Q(24, 24), Q(24, 2)};
  check(archetype_classroom(), {{"first leg", u, 0, 12, 0}, {"first leg", u, 0, 12, 300}});
}
