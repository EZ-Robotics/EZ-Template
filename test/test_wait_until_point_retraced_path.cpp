// Waits for a point of a pure pursuit path that the path runs back over, such as an out-and-back along one line. Where two legs lie on top of each
// other, which one the robot is on cannot be read from where it is, only from the way it is going.
//
//   - A wait for the turn-around point of an out-and-back (or a point near it) is answered when the robot turns around there. The robot never gets all
//     the way to the point (the look-ahead rounds it first and the robot turns about a look-ahead short), and with the return leg on top of the way
//     out it never read as nearer the way back either, so the wait ran to the end of the whole motion and called the motion interfered.
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

struct Arch {
  sim::SimArchetype a;
  int passes;
};
std::vector<Arch> archs() {
  return {{archetype_classroom(), 1}, {sim::archetype_light_fast(), 1}, {sim::archetype_heavy_slow(), 3}, {sim::archetype_sticky_high_friction(), 3}};
}

// The wait came back clean while the robot was still at the turn-around: it has not driven back along the way it came
void check_turned_around(const Res& r, double apex_x, double apex_y, const char* what) {
  CAPTURE(what);
  REQUIRE(r.ret);
  CHECK_FALSE(r.interfered);
  double from_apex = std::hypot(r.x - apex_x, r.y - apex_y);
  CHECK_MESSAGE(from_apex <= 10.0, "came back after " << r.ms << " ms at (" << r.x << ", " << r.y << "), " << from_apex << " in from the turn-around");
}

}  // namespace

TEST_CASE("waiting for the turn-around point of an out-and-back is answered where the robot turns around") {
  for (auto& arch : archs()) {
    for (Setter how : {Setter::Coarse, Setter::Dense}) {
      CAPTURE(std::string(arch.a.name));
      CAPTURE((int)how);
      std::vector<odom> path = {Q(0, 30), Q(0, 0)};
      check_turned_around(run(arch.a, arch.passes, how, path, 0, [](Drive& d) { d.pid_wait_until_index(0); }), 0, 30, "index of the turn-around");
      check_turned_around(run(arch.a, arch.passes, how, path, 0, [](Drive& d) { d.pid_wait_until(pose{0, 30, ANGLE_NOT_SET}); }), 0, 30, "the turn-around");
      check_turned_around(run(arch.a, arch.passes, how, path, 0, [](Drive& d) { d.pid_wait_until(pose{0, 27, ANGLE_NOT_SET}); }), 0, 30, "3 in short of it");
    }
  }
}

TEST_CASE("waiting for the turn-around point of a reversing out-and-back is answered where the robot turns around") {
  for (auto& arch : archs()) {
    for (Setter how : {Setter::Coarse, Setter::Dense}) {
      CAPTURE(std::string(arch.a.name));
      CAPTURE((int)how);
      std::vector<odom> path = {Q(0, -30, rev), Q(0, -6, rev)};
      check_turned_around(run(arch.a, arch.passes, how, path, 0, [](Drive& d) { d.pid_wait_until_index(0); }), 0, -30, "index of the turn-around");
    }
  }
}

TEST_CASE("a wait for the turn-around of an out-and-back called after the robot has turned back is answered at once") {
  // classroom has turned around by 1500 ms and is on its way back at 2000 ms
  for (Setter how : {Setter::Coarse, Setter::Dense}) {
    CAPTURE((int)how);
    Res r = run(archetype_classroom(), 1, how, {Q(0, 30), Q(0, 0)}, 2200, [](Drive& d) { d.pid_wait_until_index(0); });
    REQUIRE(r.ret);
    CHECK_FALSE(r.interfered);
    CHECK_MESSAGE(r.ms <= 150.0, "took " << r.ms << " ms");
  }
}
