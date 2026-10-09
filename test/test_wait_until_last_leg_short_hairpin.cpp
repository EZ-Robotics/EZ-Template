// Waiting for the last point of a pure pursuit path whose last leg is shorter than the look-ahead and turns back by more than 90 degrees (drive out to a
// point, then hop back a few inches). The look-ahead is on the last leg as soon as the robot is within a look-ahead of the leg's start, so a robot that is
// still on its way out to that point is "level with the end of the last leg" along the leg's direction: it is behind the corner, and the last leg's end,
// which lies back the way the robot came from, is behind it too. That is not the robot having driven the last leg. The wait returned clean at once,
// with the robot a few inches short of the corner still driving at 18 to 84 in/s, and pid_wait_quick() handed the next motion the robot mid-drive.
//
// The point is crossed where the robot comes to rest by it (pure pursuit rounds the corner well short of it, so that is not at the corner), not 3 to 6 in
// short of it with the robot still at speed.
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Shape {
  const char* name;
  std::vector<pose> points;
  int wait;  // 0 pid_wait_until_index(last), 1 pid_wait_quick()
};

}  // namespace

TEST_CASE("waiting for the last point of a path with a short last leg that turns back is answered only once the robot has got to the corner and come back") {
  const Shape shapes[] = {
      {"out 36 and back 4, index", {{0, 36, ANGLE_NOT_SET}, {0, 32, ANGLE_NOT_SET}}, 0},
      {"out 36 and back 2, quick", {{0, 36, ANGLE_NOT_SET}, {0, 34, ANGLE_NOT_SET}}, 1},
      {"out 36 and back 4 sideways, index", {{0, 36, ANGLE_NOT_SET}, {3, 33, ANGLE_NOT_SET}}, 0},
      {"out 12 and back 2, index", {{0, 12, ANGLE_NOT_SET}, {0, 10, ANGLE_NOT_SET}}, 0},
  };
  const sim::SimArchetype archs[] = {archetype_classroom(), sim::archetype_light_fast(), sim::archetype_heavy_slow(), sim::archetype_sticky_high_friction()};
  for (const Shape& shape : shapes)
    for (const sim::SimArchetype& arch : archs) {
      Rig r(arch, 1, false, 1);
      r.chassis.pid_print_toggle(false);
      std::vector<odom> path;
      for (const pose& p : shape.points) path.push_back(odom{p, fwd, 110});
      r.chassis.pid_odom_set(path);
      const pose target = shape.points[1];
      double elapsed = 0;
      bool ok = r.wait(
          [&] {
            if (shape.wait == 0) r.chassis.pid_wait_until_index(1);
            if (shape.wait == 1) r.chassis.pid_wait_quick();
          },
          3000, &elapsed);
      auto p = r.true_position();
      INFO(shape.name, ", ", std::string(arch.name), ": returned=", ok, " at ", elapsed, " ms, interfered=", r.chassis.interfered, ", robot at (", p.x, ", ",
           p.y, ")");
      REQUIRE(ok);
      CHECK(std::hypot(p.x - target.x, p.y - target.y) < 2.0);
    }
}
