// A robot that is being carried through its target, or away from it, inside the big error is not settled however long it has gone without a new
// low: the check that holds that verdict reads the robot's net displacement off the tracker the auto task feeds. That tracker goes stale 30 ms
// after the task's last pass, and a stale tracker used to read "went nowhere", so a task that missed four passes (a hiccup of 40 ms, or just
// a slow steady pace of 50 ms or more) let the same robot be returned clean, "counted as settled", at the next poll. A task that was gone for
// good did the same 30 ms after it stopped. Staleness now keeps the hold (it is bounded in time, see in_place_hold_ms()), and a task that has
// been quiet for a whole window while it is held ends the wait interfered, never settled.
//
// The robot is carried by the sim from the first moment it is within 0.4 in (1 deg) of its target, at 1.4 times the stop speed, which is not
// something its motors can change. Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

// Drives the auto task's schedule from the poll loop and starts the carry on arrival. `passes(ms_since_arrival, tick)` is how many passes the task
// runs on a tick.
struct Drag {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline std::function<int(double, int)> passes;
  static inline bool angular = false, arrived = false;
  static inline double arrival_ms = 0, speed = 0;
  static inline int tick_no = 0;
  static void tick() {
    rig->record();
    if (!arrived) {
      bool there = angular ? (-rig->sim.heading_deg() >= 89.0) : (rig->sim.left().position_in + rig->sim.right().position_in) / 2.0 >= 23.6;
      if (there) {
        arrived = true;
        arrival_ms = rig->sim.now_ms();
        if (angular)
          rig->sim.carry(0.0, speed, arrival_ms, 1.0e9);
        else
          rig->sim.carry(speed, 0.0, arrival_ms, 1.0e9);
      }
    }
    rig->sim.passes_per_tick(passes(arrived ? rig->sim.now_ms() - arrival_ms : -1.0, tick_no++));
    inner();
  }
  static void install(Rig& r, bool ang, double v, std::function<int(double, int)> p) {
    rig = &r;
    angular = ang;
    speed = v;
    passes = std::move(p);
    arrived = false;
    tick_no = 0;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Drag::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

constexpr int CAP_TICKS = 2000;  // 20 s of sim time

double avg_position(const Rig& r) { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }

}  // namespace

TEST_CASE("a robot carried through its target is not called settled when the auto task misses a few passes") {
  struct Row {
    const char* name;
    bool angular;
  };
  const Row rows[] = {{"drive", false}, {"turn", true}};
  for (const Row& row : rows)
    for (int gap_ms : {40, 60, 100, 300})
      for (int offset_ms : {600, 750, 900}) {
        Rig r(sim::archetype_light_fast(), 1, false, 1);
        r.chassis.pid_print_toggle(false);
        double v = row.angular ? 1.4 * FLOOR_ANGLE : 1.4 * FLOOR_DISTANCE;
        Drag::install(r, row.angular, v, [=](double since_arrival, int) { return since_arrival >= offset_ms && since_arrival < offset_ms + gap_ms ? 0 : 1; });
        if (row.angular)
          r.chassis.pid_turn_set(90_deg, 110);
        else
          r.chassis.pid_drive_set(24_in, 110);
        double ms = 0;
        bool returned = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
        Drag::uninstall();
        INFO(row.name, ", a gap of ", gap_ms, " ms ", offset_ms, " ms after arrival: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered,
             ", true position=", row.angular ? -r.sim.heading_deg() : avg_position(r));
        // Never a hang
        CHECK(returned);
        if (!returned) continue;
        // The robot is still being carried at over the stop speed when the wait returns (the sim's truth), so a clean return is a wrong verdict
        CHECK(r.chassis.interfered);
        CHECK(ms < 8000.0);
      }
}

TEST_CASE("a robot carried through its target is not called settled at an auto task pace of 50 ms or more") {
  struct Row {
    const char* name;
    bool angular;
  };
  const Row rows[] = {{"drive", false}, {"turn", true}};
  for (const Row& row : rows)
    for (int every : {5, 6, 8}) {
      Rig r(sim::archetype_light_fast(), 1, false, 1);
      r.chassis.pid_print_toggle(false);
      double v = row.angular ? 1.4 * FLOOR_ANGLE : 1.4 * FLOOR_DISTANCE;
      // Full pace until the hold is under way, then one pass every `every` ticks
      int first = -1;
      Drag::install(r, row.angular, v, [&](double since_arrival, int tick) {
        if (since_arrival < 600) return 1;
        if (first < 0) first = tick;
        return (tick - first) % every == 0 ? 1 : 0;
      });
      if (row.angular)
        r.chassis.pid_turn_set(90_deg, 110);
      else
        r.chassis.pid_drive_set(24_in, 110);
      double ms = 0;
      bool returned = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
      Drag::uninstall();
      INFO(row.name, ", task pace ", every * 10, " ms: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered,
           ", true position=", row.angular ? -r.sim.heading_deg() : avg_position(r));
      CHECK(returned);
      if (!returned) continue;
      CHECK(r.chassis.interfered);
      CHECK(ms < 8000.0);
    }
}

TEST_CASE("an auto task that stops for good while a robot is being carried through its target never lets the wait end clean") {
  struct Row {
    const char* name;
    bool angular;
  };
  const Row rows[] = {{"drive", false}, {"turn", true}};
  for (const Row& row : rows)
    for (int offset_ms : {300, 600, 750, 900}) {
      Rig r(sim::archetype_light_fast(), 1, false, 1);
      r.chassis.pid_print_toggle(false);
      double v = row.angular ? 1.4 * FLOOR_ANGLE : 1.4 * FLOOR_DISTANCE;
      Drag::install(r, row.angular, v, [=](double since_arrival, int) { return since_arrival >= offset_ms ? 0 : 1; });
      if (row.angular)
        r.chassis.pid_turn_set(90_deg, 110);
      else
        r.chassis.pid_drive_set(24_in, 110);
      double ms = 0;
      bool returned = r.wait([&] { r.chassis.pid_wait(); }, CAP_TICKS, &ms);
      Drag::uninstall();
      INFO(row.name, ", task gone ", offset_ms, " ms after arrival: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered,
           ", true position=", row.angular ? -r.sim.heading_deg() : avg_position(r));
      CHECK(returned);
      if (!returned) continue;
      CHECK(r.chassis.interfered);
      // And not long after: a task that has been quiet for a window is no reason to hold the wait further
      CHECK(ms < 8000.0);
    }
}
