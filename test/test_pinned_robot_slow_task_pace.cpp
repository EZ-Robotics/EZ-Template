// A robot that is held still short of its target is called stuck after the stuck watch's window, whatever the pace of the auto task. A task that passes
// every 50 to 100 ms leaves its newest sample older than three passes on a fair share of the polls, and the stuck verdict has to be given on one such
// poll as often as on any other: a poll on which the tracker is stale and the sensors cannot yet say anything is not a robot that is on the move. The
// verdict used to be taken back on those polls, the watch started over (up to four times), and the wait ran one to four whole windows longer.
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

TEST_CASE("a robot pinned short of its target is called stuck about as soon at a task pace of 50 to 100 ms as at 30 ms") {
  const sim::SimArchetype archs[] = {sim::archetype_light_fast(), archetype_classroom(), sim::archetype_sticky_high_friction()};
  for (const sim::SimArchetype& arch : archs)
    for (int exits = 0; exits < 2; exits++)
      for (double pin_at : {155.0, 400.0}) {
        // The slowest return over paces 3 to 5 is what the same robot is called stuck at when the tracker is never stale; no pace may add more than a window
        double fast = 0;
        for (int every : {3, 4, 5, 6, 7, 8, 9, 10}) {
          Rig r(arch, 1, false, 1);
          r.chassis.pid_print_toggle(false);
          if (exits == 1) {
            r.chassis.pid_odom_drive_exit_condition_set(250, 0.5, 300, 6, 1000, 0);
            r.chassis.pid_odom_turn_exit_condition_set(100, 1, 200, 3, 100, 1000);
          }
          r.sim.pin(pin_at, 60000);
          Pace::install(r, every);
          r.chassis.pid_odom_ptp_set(odom{pose{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
          double ms = 0;
          bool ok = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
          Pace::uninstall();
          INFO(std::string(arch.name), ", exits ", exits, ", pinned from ", pin_at, " ms, task pace ", every * 10, " ms: returned=", ok, " at ", ms,
               " ms, interfered=", r.chassis.interfered);
          REQUIRE(ok);
          CHECK(r.chassis.interfered);
          if (every <= 5) fast = std::fmax(fast, ms);
          // The same robot, to the ms, at the paces that do not leave a stale poll before the verdict, and never a whole stuck window later than them
          else
            CHECK(ms < fast + 700.0);
        }
      }
}
