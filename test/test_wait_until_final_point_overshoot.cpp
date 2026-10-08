// pid_wait_until(point) for the last point of a pure pursuit path, with the robot ending up beyond it. A robot that overshoots the end of its path
// (the auto task stalled while it drove on) and comes to rest a little past it and to the side is nearer the last leg than the last point of it, but
// not by more than the tolerance that says two legs are as near as each other, so the earlier leg counted as just as near and the robot was taken to be
// before the point for ever: the wait never saw the crossing and ended stuck seconds later, interfered, with the robot hunting about the point.
//
// A robot that is level with the end of the last leg or beyond it, along that leg, has crossed the point whichever earlier leg it is also near.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

// Runs the auto task on every tick except from `from` for `ms` of sim time
struct Stall {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline double from = 0, ms = 0;
  static void tick() {
    double t = rig->sim.now_ms();
    rig->sim.passes_per_tick(t >= from && t < from + ms ? 0 : 1);
    inner();
  }
  static void install(Rig& r, double f, double m) {
    rig = &r;
    from = f;
    ms = m;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Stall::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

}  // namespace

TEST_CASE("waiting for the last point of a path is answered when the robot overshoots it and ends up off to the side") {
  struct Row {
    sim::SimArchetype arch;
    int passes;
  };
  const Row rows[] = {{sim::archetype_sticky_high_friction(), 1}, {archetype_classroom(), 1}, {sim::archetype_light_fast(), 1}};
  for (const Row& row : rows)
    for (double from : {300.0, 500.0, 700.0})
      for (double ms : {400.0, 600.0, 800.0}) {
        Rig r(row.arch, row.passes, false, 1);
        r.chassis.pid_print_toggle(false);
        r.chassis.pid_odom_drive_exit_condition_set(100, 0.5, 300, 2, 300, 1000);
        r.chassis.pid_odom_turn_exit_condition_set(100, 2, 300, 4, 300, 1000);
        Stall::install(r, from, ms);
        r.chassis.pid_odom_set(odom{pose{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110});
        double elapsed = 0;
        bool ok = r.wait([&] { r.chassis.pid_wait_until(pose{0.0, 36.0, ANGLE_NOT_SET}); }, 3000, &elapsed);
        Stall::uninstall();
        auto p = r.true_position();
        INFO(std::string(row.arch.name), ", task stalled ", ms, " ms from ", from, " ms: returned=", ok, " at ", elapsed,
             " ms, interfered=", r.chassis.interfered, ", robot at (", p.x, ", ", p.y, ")");
        REQUIRE(ok);
        // The robot reached the point (or came to rest within the settle error of it), so the wait was answered and not called stuck seconds later
        CHECK_FALSE(r.chassis.interfered);
      }
}
