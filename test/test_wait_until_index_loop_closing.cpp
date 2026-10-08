// Waits for a vertex of a pure pursuit path that closes a loop: the last leg into the vertex comes home to where the path began (or to a point on its
// first leg), and the path then goes on up that first leg again, either with one more straight leg or with a boomerang. The robot on the way into
// that vertex is as near the first leg as the leg it is on, and once it has cut the corner it is on the way up the first leg's line, so neither
// "nearest leg" nor the plane through the vertex ever said it had got there: the wait ran to the end of the motion and called it interfered, or came
// back 20 in late with the robot at the far end of the path.
//
// The way the robot has got along the path tells it apart: once the robot is clearly on a later leg than the vertex's own, the vertex is behind it.
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
odom B(double x, double y, double theta) { return odom{pose{x, y, theta}, fwd, 110}; }

struct Res {
  bool ret = false;
  bool interfered = false;
  double ms = 0;
  double x = 0, y = 0;
};

enum class Setter {
  Coarse,
  Dense
};

Res run(const sim::SimArchetype& arch, int passes, Setter how, const std::vector<odom>& path, const std::function<void(Drive&)>& wait) {
  Rig r(arch, passes, false, 1);
  if (how == Setter::Coarse)
    r.chassis.pid_odom_pp_set(path);
  else
    r.chassis.pid_odom_set(path);
  Res res;
  res.ret = r.wait([&] { wait(r.chassis); }, 1500, &res.ms);
  res.interfered = r.chassis.interfered;
  auto p = r.true_position();
  res.x = p.x;
  res.y = p.y;
  return res;
}

struct Arch {
  sim::SimArchetype a;
  int passes;
};
std::vector<Arch> archs() {
  return {{archetype_classroom(), 1}, {sim::archetype_light_fast(), 1}, {sim::archetype_heavy_slow(), 3}, {sim::archetype_sticky_high_friction(), 3}};
}

struct Shape {
  const char* name;
  std::vector<odom> path;
  double vx, vy;  // the vertex waited for (index 3)
};

std::vector<Shape> shapes() {
  return {
      {"square home, then up the first leg", {Q(0, 24), Q(24, 24), Q(24, 0), Q(0, 0), Q(0, 24)}, 0, 0},
      {"square home along the other way, then out again", {Q(24, 0), Q(24, 24), Q(0, 24), Q(0, 0), Q(24, 0)}, 0, 0},
      {"square home, then a boomerang up the first leg", {Q(0, 24), Q(24, 24), Q(24, 0), Q(0, 0), B(0, 24, 90)}, 0, 0},
      {"square home, then a boomerang facing forward", {Q(0, 24), Q(24, 24), Q(24, 0), Q(0, 0), B(0, 24, 0)}, 0, 0},
      {"loop that stops short of home, then a boomerang", {Q(0, 24), Q(24, 24), Q(24, 4), Q(0, 4), B(0, 28, 90)}, 0, 4},
  };
}

}  // namespace

TEST_CASE("waiting for the vertex where a loop comes home is answered there, not at the end of the path") {
  for (const Shape& shape : shapes())
    for (auto& arch : archs())
      for (Setter how : {Setter::Coarse, Setter::Dense}) {
        Res r = run(arch.a, arch.passes, how, shape.path, [](Drive& d) { d.pid_wait_until_index(3); });
        INFO(std::string(shape.name), ", ", std::string(arch.a.name), ", ", std::string(how == Setter::Coarse ? "coarse" : "dense"),
             " setter: returned=", r.ret, " at ", r.ms, " ms, interfered=", r.interfered, ", robot at (", r.x, ", ", r.y, ")");
        REQUIRE(r.ret);
        CHECK_FALSE(r.interfered);
        // The robot has just got to the vertex (it rounds it a few inches short), not driven on up the first leg
        CHECK(std::hypot(r.x - shape.vx, r.y - shape.vy) <= 10.0);
      }
}

TEST_CASE("waiting for a point on the way into the vertex where a loop comes home is answered there too") {
  for (const Shape& shape : shapes())
    for (auto& arch : archs()) {
      Res r = run(arch.a, arch.passes, Setter::Coarse, shape.path, [&](Drive& d) { d.pid_wait_until(pose{shape.vx + 4.0, shape.vy, ANGLE_NOT_SET}); });
      INFO(std::string(shape.name), ", ", std::string(arch.a.name), ": returned=", r.ret, " at ", r.ms, " ms, interfered=", r.interfered, ", robot at (", r.x,
           ", ", r.y, ")");
      REQUIRE(r.ret);
      CHECK_FALSE(r.interfered);
    }
}
