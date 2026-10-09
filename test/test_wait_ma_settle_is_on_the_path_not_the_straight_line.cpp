// A robot held where it started is not settled because the path it was sent along ends within the big error of that place. The mA exit before the end of a
// pure pursuit path was judged on the straight line to the path's last point, so a path that loops back onto its start, or a hook that comes back to
// within a few inches of it, counted a robot that had not driven away as having arrived. It has to be the distance left along the path.
//
// Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

// The auto task passes on every `every`th tick, and the drive motors read over current from the start
struct Paced {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline int every = 1, count = 0;
  static void tick() {
    rig->sim.passes_per_tick(count++ % every == 0 ? 1 : 0);
    inner();
    for (auto& m : rig->chassis.left_motors) m.fake().over_current = true;
    for (auto& m : rig->chassis.right_motors) m.fake().over_current = true;
  }
  static void install(Rig& r, int ev) {
    rig = &r;
    every = ev;
    count = 0;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Paced::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

struct Row {
  const char* name;
  sim::SimArchetype arch;
  int every;
  std::vector<odom> path;
};

}  // namespace

TEST_CASE("a robot pinned at the start of a path that ends beside its start is ended as interfered by an mA exit") {
  auto at = [](double x, double y) { return odom{pose{x, y, ANGLE_NOT_SET}, fwd, 110}; };
  const std::vector<odom> loop = {at(0, 24), at(24, 24), at(24, 0), at(0, 0)};
  const std::vector<odom> hook = {at(0, 36), at(2, 36), at(2, 2)};
  const Row rows[] = {
      {"classroom loop", archetype_classroom(), 1, loop},
      {"light loop", sim::archetype_light_fast(), 1, loop},
      {"heavy loop, 100 ms pace", sim::archetype_heavy_slow(), 10, loop},
      {"sticky loop, 300 ms pace", sim::archetype_sticky_high_friction(), 30, loop},
      {"classroom hook", archetype_classroom(), 1, hook},
      {"light hook, 50 ms pace", sim::archetype_light_fast(), 5, hook},
  };
  for (const Row& row : rows) {
    Rig r(row.arch, 1, false, 1);
    r.chassis.pid_print_toggle(false);
    // mA exits only
    r.chassis.pid_odom_drive_exit_condition_set(0, 0, 0, 0, 0, 100);
    r.chassis.pid_odom_turn_exit_condition_set(0, 0, 0, 0, 0, 100);
    r.sim.pin(10, 1e9);
    Paced::install(r, row.every);
    r.chassis.pid_odom_set(row.path);
    double ms = 0;
    bool returned = r.wait([&] { r.chassis.pid_wait(); }, 6000, &ms);
    Paced::uninstall();
    auto p = r.true_position();
    double from_start = std::hypot(p.x, p.y);
    INFO(row.name, ": returned=", returned, " at ", ms, " ms, ", from_start, " in from where it started, interfered=", r.chassis.interfered);
    REQUIRE(returned);
    CHECK(from_start < 3.0);
    CHECK(r.chassis.interfered);
  }
}
