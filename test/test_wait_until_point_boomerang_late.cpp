// A wait for a point of a pure pursuit path that the robot has already passed, on a path with a boomerang point in it. The wait is answered at once, as
// it is on a path without one. With a boomerang anywhere in the path the legs the wait reads the robot's place from were not built, so the wait ran to the
// end of the whole motion (and called a stopped robot interfered on the slower drivetrains).
#include <cmath>
#include <functional>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

odom Q(double x, double y, drive_directions d = fwd) { return odom{pose{x, y, ANGLE_NOT_SET}, d, 110}; }
odom QB(double x, double y, double theta) { return odom{pose{x, y, theta}, fwd, 110}; }

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

TEST_CASE("a late wait for a point the robot has passed, on a path that ends in a boomerang, is answered at once") {
  std::vector<odom> path = {Q(12, 24), QB(24, 48, 90)};
  // The robot crosses (12, 24) at about 640 ms
  for (Setter how : {Setter::Coarse, Setter::Dense}) {
    CAPTURE((int)how);
    Res a = run(sim::archetype_light_fast(), 1, how, path, 700, [](Drive& d) { d.pid_wait_until_index(0); });
    REQUIRE(a.ret);
    CHECK_FALSE(a.interfered);
    CHECK_MESSAGE(a.ms <= 150.0, "index wait took " << a.ms << " ms");
    Res b = run(sim::archetype_light_fast(), 1, how, path, 700, [](Drive& d) { d.pid_wait_until(pose{12, 24, ANGLE_NOT_SET}); });
    REQUIRE(b.ret);
    CHECK_FALSE(b.interfered);
    CHECK_MESSAGE(b.ms <= 150.0, "point wait took " << b.ms << " ms");
  }
  Res c = run(archetype_classroom(), 1, Setter::Dense, path, 1500, [](Drive& d) { d.pid_wait_until_index(0); });
  REQUIRE(c.ret);
  CHECK_FALSE(c.interfered);
  CHECK_MESSAGE(c.ms <= 150.0, "classroom index wait took " << c.ms << " ms");
}
