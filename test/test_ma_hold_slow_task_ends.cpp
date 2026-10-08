// An mA exit that is held while the auto task has not passed has to end. A task that passes every few hundred milliseconds is stale between its passes,
// and a robot that is moving (here an unstable controller spinning and hunting about a path under permanent over current) has moved since the last one by
// the time the next poll comes, so the exit is held on almost every poll. The progress window it is judged over must not start again each time, or it
// never completes: a version of the hold that did hung this robot for the whole 35 s cap.
//
// Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

// The auto task passes on every `every`th tick, and the drive motors read over current from `oc_from` ms on
struct Slow {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline int every = 1, count = 0;
  static inline double oc_from = 0;
  static void tick() {
    rig->sim.passes_per_tick(count++ % every == 0 ? 1 : 0);
    inner();
    if (rig->sim.now_ms() >= oc_from) {
      for (auto& m : rig->chassis.left_motors) m.fake().over_current = true;
      for (auto& m : rig->chassis.right_motors) m.fake().over_current = true;
    }
  }
  static void install(Rig& r, int ev, double oc) {
    rig = &r;
    every = ev;
    count = 0;
    oc_from = oc;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Slow::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

}  // namespace

TEST_CASE("a held mA exit on a hunting robot under permanent over current ends when the auto task passes every 100 to 300 ms") {
  for (int every : {10, 20, 30}) {
    Rig r(archetype_classroom(), 1, false, 1);
    r.chassis.pid_print_toggle(false);
    r.chassis.pid_odom_drive_exit_condition_set(100, 1, 300, 4, 300, 1000);
    r.chassis.pid_odom_turn_exit_condition_set(100, 2, 300, 4, 300, 1000);
    // An unstable controller: four times the xy P and twice the heading P
    auto kxy = r.chassis.xyPID.constants_get();
    r.chassis.xyPID.constants_set(kxy.kp * 4.0, kxy.ki, kxy.kd);
    auto ka = r.chassis.current_a_odomPID.constants_get();
    r.chassis.current_a_odomPID.constants_set(ka.kp * 2.0, ka.ki, ka.kd);
    Slow::install(r, every, 1900.0);
    r.chassis.pid_odom_set(std::vector<odom>{{{0, 36, ANGLE_NOT_SET}, fwd, 110}, {{12, 36, ANGLE_NOT_SET}, fwd, 110}, {{12, 0, ANGLE_NOT_SET}, fwd, 110}});
    double ms = 0;
    bool returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
    Slow::uninstall();
    INFO("task pace ", every * 10, " ms: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered);
    // Never a hang: the same robot returns by 18 s at a 200 ms pace without the hold (it is a slow, unstable robot), and by 4 s at the others
    CHECK(returned);
    CHECK(ms < 25000.0);
  }
}
