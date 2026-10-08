// pid_wait_quick_chain() on a pure pursuit path that comes back to where it started: it is meant to hand over to the next motion while the robot is
// still carrying speed through the last point. The way back lies on top of the way out, so the robot never read as past the last point, and the wait fell
// back to waiting for the robot to settle, 0.8 in/s and 2.7 in short of the point, with the speed the chain is for gone.
#include <cmath>
#include <functional>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

odom Q(double x, double y, drive_directions d = fwd) { return odom{pose{x, y, ANGLE_NOT_SET}, d, 110}; }

struct Res {
  bool ret = false;
  bool interfered = false;
  double ms = 0;  // how long the wait took
  double x = 0, y = 0;
  double speed = 0;  // true speed (in/s) over the last 100 ms
};

enum class Setter {
  Coarse,
  Dense
};

Res run(const sim::SimArchetype& arch, int passes, Setter how, const std::vector<odom>& path, int pre, const std::function<void(Drive&)>& wait) {
  Rig r(arch, passes, false, 1);
  if (how == Setter::Coarse)
    r.chassis.pid_odom_pp_set(path);
  else
    r.chassis.pid_odom_set(path);
  for (int t = 0; t < pre; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
  Res res;
  res.ret = r.wait([&] { wait(r.chassis); }, 1500, &res.ms);
  res.interfered = r.chassis.interfered;
  auto p = r.true_position();
  res.x = p.x;
  res.y = p.y;
  res.speed = r.drive_speed_over(100);
  return res;
}

}  // namespace

TEST_CASE("waiting for the end of a path that comes back to its start is answered when the robot crosses it, with the speed it has") {
  // heavy_slow, which carries the most momentum: pid_wait_quick_chain() on the way back down a line the robot came up
  for (Setter how : {Setter::Coarse, Setter::Dense}) {
    CAPTURE((int)how);
    Res r = run(sim::archetype_heavy_slow(), 3, how, {Q(0, 30), Q(0, 10)}, 0, [](Drive& d) { d.pid_wait_quick_chain(); });
    REQUIRE(r.ret);
    CHECK_FALSE(r.interfered);
    CHECK_MESSAGE(r.speed >= 5.0,
                  "came back after " << r.ms << " ms at (" << r.x << ", " << r.y << ") doing " << r.speed << " in/s: the robot had already settled");
  }
}
