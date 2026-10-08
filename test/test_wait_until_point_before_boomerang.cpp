// A wait for the last straight vertex of a pure pursuit path whose next point is a boomerang, or for a point just short of it. A robot driving such a
// path cuts the corner (the boomerang's carrot sits behind its target, so it curves away early) and never gets level with the vertex along the last leg,
// so the wait was never answered: it ran to the end of the motion and called a healthy move interfered, or returned a second late with the robot a
// robot length or more past the point. It is answered where the robot passes the vertex, like on a path without a boomerang.
#include <cmath>
#include <functional>
#include <string>
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
  double ms = 0;
  double away = 0;  // how far the robot is from the point waited for when the wait returns
};

enum class Setter {
  Coarse,
  Dense
};

Res run(const sim::SimArchetype& arch, int passes, Setter how, const std::vector<odom>& path, const pose& point, const std::function<void(Drive&)>& wait) {
  Rig r(arch, passes, false, 1);
  r.chassis.pid_print_toggle(false);
  if (how == Setter::Coarse)
    r.chassis.pid_odom_pp_set(path);
  else
    r.chassis.pid_odom_set(path);
  Res res;
  res.ret = r.wait([&] { wait(r.chassis); }, 1500, &res.ms);
  res.interfered = r.chassis.interfered;
  auto p = r.true_position();
  res.away = std::hypot(p.x - point.x, p.y - point.y);
  return res;
}

struct Arch {
  const char* name;
  sim::SimArchetype arch;
  int passes;
};

std::vector<Arch> archs() {
  return {{"light_fast", sim::archetype_light_fast(), 1},
          {"classroom", archetype_classroom(), 1},
          {"heavy_slow", sim::archetype_heavy_slow(), 2},
          {"sticky", sim::archetype_sticky_high_friction(), 1}};
}

void check(const char* what, const Arch& a, Setter how, const std::vector<odom>& path, const pose& point, const std::function<void(Drive&)>& wait) {
  Res r = run(a.arch, a.passes, how, path, point, wait);
  INFO(std::string(what), " on ", std::string(a.name), ", setter ", (int)how, ": returned=", r.ret, " at ", r.ms, " ms, interfered=", r.interfered, ", ",
       r.away, " in from the point");
  REQUIRE(r.ret);
  CHECK_FALSE(r.interfered);
  CHECK(r.away <= 10.0);
}

}  // namespace

TEST_CASE("a wait for the vertex before a boomerang is answered where the robot passes it") {
  const std::vector<std::vector<odom>> paths = {{Q(0, 24), QB(24, 24, 90)}, {Q(0, 24), QB(0, 4, 180)}, {Q(0, 24), Q(24, 24), Q(24, 48), QB(0, 48, 270)}};
  const pose vertex[] = {pose{0, 24, ANGLE_NOT_SET}, pose{0, 24, ANGLE_NOT_SET}, pose{24, 48, ANGLE_NOT_SET}};
  const int index[] = {0, 0, 2};
  for (const Arch& a : archs())
    for (Setter how : {Setter::Coarse, Setter::Dense})
      for (int i = 0; i < 3; i++) {
        CAPTURE(i);
        check("index wait", a, how, paths[i], vertex[i], [&](Drive& d) { d.pid_wait_until_index(index[i]); });
      }
}

TEST_CASE("a wait for a point a few inches before the vertex that leads to a boomerang is answered where the robot passes it") {
  const std::vector<odom> path = {Q(0, 24), QB(24, 24, 0)};
  const pose point{0, 20, ANGLE_NOT_SET};
  for (const Arch& a : archs())
    for (Setter how : {Setter::Coarse, Setter::Dense}) check("point wait", a, how, path, point, [&](Drive& d) { d.pid_wait_until(point); });
}
