// A slow auto task is not a dead one. The stuck check calls a verdict "starved" (never settled, always interfered) when the task has gone
// quiet for good; a task that only passes every 40 or 50 ms is alive, and a robot that finished its motion and sits hunting about the target
// must read as settled whatever pace the task keeps. These tests run the same hunting robot with the task passing every 10 to 50 ms and
// expect the same clean verdict, and keep the other side of it: a slow task that then dies for good inside the big error still reads
// interfered.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// Runs the auto task on every `every`th tick only, and not at all once `die` says so
struct Pace {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline int every = 1;
  static inline int count = 0;
  static inline bool dead = false;
  static inline std::function<bool(const Rig&)> die;
  static void tick() {
    rig->record();
    if (!dead && die && die(*rig)) dead = true;
    rig->sim.passes_per_tick(!dead && count++ % every == 0 ? 1 : 0);
    inner();
  }
  static void install(Rig& r, int n, std::function<bool(const Rig&)> d = nullptr) {
    rig = &r;
    every = n;
    count = 0;
    dead = false;
    die = std::move(d);
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Pace::tick;
  }
};

double avg_position(const Rig& r) { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }

}  // namespace

TEST_CASE("a robot hunting about its drive target reads settled at any auto task pace from 10 to 50 ms") {
  struct Row {
    const char* name;
    sim::SimArchetype arch;
    int passes;
    double kp;
    int slowest;  // the slowest task pace (in ticks) at which this robot still hunts inside the big error rather than outside it
  };
  const Row rows[] = {{"light_fast", sim::archetype_light_fast(), 1, 1000.0, 4}, {"classroom", archetype_classroom(), 1, 3000.0, 5}};
  for (const Row& row : rows)
    for (int every : {1, 2, 3, 4, 5}) {
      if (every > row.slowest) continue;
      Rig r(row.arch, row.passes, false, 1);
      r.chassis.pid_print_toggle(false);
      Pace::install(r, every);
      r.chassis.pid_drive_constants_set(row.kp, 0, 0);
      r.chassis.pid_drive_set(24_in, 110);
      double ms = 0;
      bool returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
      test_stub::g_clock.on_delay = Pace::inner;
      INFO(row.name, " task pace ", every * 10, " ms: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered, ", true position=", avg_position(r));
      // Never a hang
      CHECK(returned);
      if (!returned) continue;
      // The robot is at its target (the sim's truth), so interfered would be a wrong verdict
      CHECK(std::fabs(avg_position(r) - 24.0) < 5.0);
      CHECK_FALSE(r.chassis.interfered);
    }
}

TEST_CASE("a turn hunting about its target reads settled at any auto task pace from 10 to 50 ms") {
  for (int every : {1, 2, 3, 4, 5}) {
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    r.chassis.pid_print_toggle(false);
    Pace::install(r, every);
    r.chassis.pid_turn_constants_set(3.0, 0, 0);
    r.chassis.pid_turn_set(90_deg, 110);
    double ms = 0;
    bool returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
    test_stub::g_clock.on_delay = Pace::inner;
    INFO("task pace ", every * 10, " ms: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered, ", true heading=", -r.sim.heading_deg());
    CHECK(returned);
    if (!returned) continue;
    CHECK(std::fabs(-r.sim.heading_deg() - 90.0) < 7.0);
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("a slow auto task that dies for good inside the big error never lets a wait end clean") {
  for (int every : {1, 3, 5}) {
    Rig r(archetype_classroom(), 1, false);
    r.chassis.pid_print_toggle(false);
    // The task dies the moment the robot is within the big error, and the robot is carried away from the target from then on
    bool carried = false;
    Pace::install(r, every, [&](const Rig& rg) {
      bool now_dead = avg_position(rg) >= 22;
      if (now_dead && !carried) {
        carried = true;
        const_cast<Rig&>(rg).sim.carry(-8, 0, rg.sim.now_ms(), 1.0e9);
      }
      return now_dead;
    });
    r.chassis.pid_drive_set(24_in, 110);
    double ms = 0;
    bool returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
    test_stub::g_clock.on_delay = Pace::inner;
    INFO("task pace ", every * 10, " ms before it died: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered, ", true position=", avg_position(r));
    CHECK(carried);
    CHECK(returned);
    if (!returned) continue;
    CHECK(avg_position(r) < 21.0);
    CHECK(r.chassis.interfered);
  }
}
