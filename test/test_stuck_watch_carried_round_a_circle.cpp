// A robot that is carried round a circle through and past its target (spun as it goes, as a robot hooked on a field element and pivoting is) is ended by
// pid_wait() as interfered within a few seconds, not after laps of the circle. The heading error of such a robot rises past the target, wraps and comes
// down again, lap after lap, and every fall from a peak is a run of new lows. The rise at the start of the first lap earned the stuck watch a restart for a
// rise it cannot tell from the robot's own overshoot, on the heading and then again on the distance, and that carried it to the first peak and the fall
// after it. A rise like that is given the one restart per wait, not one per channel.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

// The auto task passes on every `every`th tick
struct Pace {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline int every = 1, count = 0;
  static void tick() {
    rig->sim.passes_per_tick(count++ % every == 0 ? 1 : 0);
    inner();
  }
  static void install(Rig& r, int ev) {
    rig = &r;
    every = ev;
    count = 0;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Pace::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

}  // namespace

TEST_CASE("a robot carried round a circle through its target is ended as interfered within a few seconds, not after laps") {
  const sim::SimArchetype archs[] = {sim::archetype_light_fast(), archetype_classroom()};
  for (const sim::SimArchetype& arch : archs)
    for (int every : {3, 6}) {
      Rig r(arch, 1, false, 1);
      r.chassis.pid_print_toggle(false);
      r.chassis.pid_odom_drive_exit_condition_set(100, 1, 300, 4, 300, 1000);
      r.chassis.pid_odom_turn_exit_condition_set(100, 2, 300, 4, 300, 1000);
      r.sim.carry(5.5, 20.0, 1320.0, 60000.0);
      Pace::install(r, every);
      r.chassis.pid_odom_ptp_set(odom{pose{0.0, -24.0, ANGLE_NOT_SET}, fwd, 110});
      double ms = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait(); }, 5000, &ms);
      Pace::uninstall();
      INFO(std::string(arch.name), ", task pace ", every * 10, " ms: returned=", ok, " at ", ms, " ms, interfered=", r.chassis.interfered);
      REQUIRE(ok);
      CHECK(r.chassis.interfered);
      CHECK(ms < 8000.0);
    }
}
