// An mA exit counted as settled, or a stuck robot counted as stopped inside the big error windows, has to be inside them of the point the whole motion goes
// to. The wait judged it against the point the robot was driving to at that moment, which a pure pursuit path (every pid_odom_set() to a pose is one)
// has an earlier point of while the motion is on its way. With an auto task that passes every 200 to 600 ms the point it was driving to trails the robot
// to within the big error of it at the first reading after the mA exit, and the wait returned clean with the robot 35 to 44 in from its target, pinned or
// driving at many times the speed of a robot that has stopped.
//
// Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <string>

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
  bool pinned;
};

}  // namespace

TEST_CASE("a robot far from the end of a path is not settled because it is inside the big error of a point on the way, at any task pace") {
  const Row rows[] = {
      {"classroom pinned, 300 ms pace", archetype_classroom(), 30, true},
      {"classroom pinned, 500 ms pace", archetype_classroom(), 50, true},
      {"sticky pinned, 400 ms pace", sim::archetype_sticky_high_friction(), 40, true},
      {"light pinned, 60 ms pace", sim::archetype_light_fast(), 6, true},
      {"heavy driving, 600 ms pace", sim::archetype_heavy_slow(), 60, false},
      {"sticky driving, 250 ms pace", sim::archetype_sticky_high_friction(), 25, false},
  };
  for (const Row& row : rows) {
    Rig r(row.arch, 1, false, 1);
    r.chassis.pid_print_toggle(false);
    // mA exits only
    r.chassis.pid_odom_drive_exit_condition_set(0, 0, 0, 0, 0, 100);
    r.chassis.pid_odom_turn_exit_condition_set(0, 0, 0, 0, 0, 100);
    if (row.pinned) r.sim.pin(300, 1e9);
    Paced::install(r, row.every);
    r.chassis.pid_odom_set(odom{pose{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
    double ms = 0;
    bool returned = r.wait([&] { r.chassis.pid_wait(); }, 6000, &ms);
    Paced::uninstall();
    auto p = r.true_position();
    double away = std::hypot(p.x - 0.0, p.y - 48.0);
    INFO(row.name, ": returned=", returned, " at ", ms, " ms, ", away, " in from the target, interfered=", r.chassis.interfered);
    REQUIRE(returned);
    if (away > 8.0) CHECK(r.chassis.interfered);
  }
}
