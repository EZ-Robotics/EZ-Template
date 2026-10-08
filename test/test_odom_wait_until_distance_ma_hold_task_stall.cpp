// pid_wait_until(distance) on an odom move holds an mA exit while the robot is getting somewhere, judged on the path left to drive. That comes from
// the odom pose, which only the auto task updates, so while the task is stalled it stands still however the robot is driving. A stall as long as
// the mA timeout, with the motors reading over current (a robot pushing a wall, or just accelerating hard), then read as a robot that was not getting
// anywhere, and the wait ended interfered while the robot was still on its way, at 8 in of a 30 in checkpoint and moving at 29 in/s.
//
// An mA exit is not taken on a pose the task has not refreshed: the hold waits for the task to pass again, and a task that never does is ended
// by the starved check, interfered, in bounded time.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// Runs the auto task on every tick except from `stall_from` for `stall_ms` of sim time (0: never again once it starts), and holds the drive motors over
// current from `oc_from` for `oc_ms`
struct Stall {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline double stall_from = 0, stall_ms = 0, oc_from = 0, oc_ms = 0;
  static inline bool forever = false;
  static void tick() {
    double t = rig->sim.now_ms();
    bool stalled = t >= stall_from && (forever || t < stall_from + stall_ms);
    rig->sim.passes_per_tick(stalled ? 0 : 1);
    inner();
    t = rig->sim.now_ms();
    if (t >= oc_from && (forever || t < oc_from + oc_ms)) {
      for (auto& m : rig->chassis.left_motors) m.fake().over_current = true;
      for (auto& m : rig->chassis.right_motors) m.fake().over_current = true;
    }
  }
  static void install(Rig& r, double from, double ms, bool never_again) {
    rig = &r;
    stall_from = oc_from = from;
    stall_ms = oc_ms = ms;
    forever = never_again;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Stall::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

// 2550R's exits, mA_timeout 100 ms
void team_exits(Drive& c) {
  c.pid_odom_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
  c.pid_odom_turn_exit_condition_set(90, 3, 200, 7, 100, 100);
}

struct Arch {
  sim::SimArchetype a;
  int passes;
};
std::vector<Arch> archs() { return {{archetype_classroom(), 1}, {sim::archetype_light_fast(), 1}}; }

}  // namespace

TEST_CASE("pid_wait_until(distance) on an odom move is not ended by an mA exit while the auto task is stalled and the robot is driving") {
  for (auto& arch : archs())
    for (bool pure_pursuit : {false, true})
      for (double gap : {100.0, 200.0, 300.0, 600.0})
        for (double from : {100.0, 300.0, 500.0}) {
          Rig r(arch.a, arch.passes, false, 1);
          r.chassis.pid_print_toggle(false);
          team_exits(r.chassis);
          Stall::install(r, from, gap, false);
          if (pure_pursuit)
            r.chassis.pid_odom_set(odom{pose{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110});
          else
            r.chassis.pid_odom_ptp_set(odom{pose{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110});
          double ms = 0;
          bool ok = r.wait([&] { r.chassis.pid_wait_until(30.0); }, 3000, &ms);
          Stall::uninstall();
          // The sim's own wheel travel, not the trace (which has no samples over a stall of the auto task). The wait ends when either side has driven 30 in
          double along = std::fmax(r.sim.left().position_in, r.sim.right().position_in);
          INFO(std::string(arch.a.name), pure_pursuit ? ", pure pursuit" : ", point to point", ", stall ", gap, " ms from ", from, " ms: returned=", ok, " at ",
               ms, " ms, interfered=", r.chassis.interfered, ", robot ", along, " in along");
          REQUIRE(ok);
          // The robot was never stuck: it drove the 30 in
          CHECK_FALSE(r.chassis.interfered);
          CHECK(along > 29.0);
        }
}

TEST_CASE("pid_wait_until(distance) on an odom move ends in bounded time when the auto task stops for good while the motors are over current") {
  for (auto& arch : archs())
    for (bool pure_pursuit : {false, true})
      for (double from : {100.0, 400.0}) {
        Rig r(arch.a, arch.passes, false, 1);
        r.chassis.pid_print_toggle(false);
        team_exits(r.chassis);
        Stall::install(r, from, 0.0, true);
        if (pure_pursuit)
          r.chassis.pid_odom_set(odom{pose{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110});
        else
          r.chassis.pid_odom_ptp_set(odom{pose{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110});
        double ms = 0;
        bool ok = r.wait([&] { r.chassis.pid_wait_until(30.0); }, 3000, &ms);
        Stall::uninstall();
        INFO(std::string(arch.a.name), pure_pursuit ? ", pure pursuit" : ", point to point", ", task gone from ", from, " ms: returned=", ok, " at ", ms,
             " ms, interfered=", r.chassis.interfered);
        // Never a hang
        REQUIRE(ok);
        CHECK(ms < 8000.0);
      }
}
