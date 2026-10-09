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

namespace {

enum class WaitKind {
  Wait,
  Index,
  Quick,
  UntilPoint
};

const char* kind_name(WaitKind k) {
  switch (k) {
    case WaitKind::Wait:
      return "pid_wait";
    case WaitKind::Index:
      return "pid_wait_until_index";
    case WaitKind::Quick:
      return "pid_wait_quick";
    case WaitKind::UntilPoint:
      return "pid_wait_until(point)";
  }
  return "";
}

}  // namespace

// The same hold with the mA timeout the shipped-style exits use (100 ms) and a task that passes every 150 to 400 ms, so every stretch between two
// passes is longer than the mA window. The robot is driven round a closed square by a controller that runs at 3 to 7 Hz (it hunts and spins), under
// permanent over current from 100 ms. Every wait that reads the mA exit has to end: a hold that gives up its progress window on every stall of the
// task never completes it, and these waits ran for minutes.
TEST_CASE("a held mA exit of 100 ms under permanent over current ends for every odom wait when the auto task passes every 150 to 400 ms") {
  for (int every : {15, 20, 30, 40}) {
    for (WaitKind kind : {WaitKind::Wait, WaitKind::Index, WaitKind::Quick, WaitKind::UntilPoint}) {
      Rig r(archetype_classroom(), 1, false, 1);
      r.chassis.pid_print_toggle(false);
      r.chassis.pid_odom_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
      r.chassis.pid_odom_turn_exit_condition_set(90, 3, 200, 7, 100, 100);
      Slow::install(r, every, 100.0);
      r.chassis.pid_odom_set(std::vector<odom>{{{0, 24, ANGLE_NOT_SET}, fwd, 110},
                                               {{24, 24, ANGLE_NOT_SET}, fwd, 110},
                                               {{24, 0, ANGLE_NOT_SET}, fwd, 110},
                                               {{0, 0, ANGLE_NOT_SET}, fwd, 110},
                                               {{0, 24, ANGLE_NOT_SET}, fwd, 110}});
      double ms = 0;
      bool returned = r.wait(
          [&] {
            switch (kind) {
              case WaitKind::Wait:
                r.chassis.pid_wait();
                break;
              case WaitKind::Index:
                r.chassis.pid_wait_until_index(3);
                break;
              case WaitKind::Quick:
                r.chassis.pid_wait_quick();
                break;
              case WaitKind::UntilPoint:
                r.chassis.pid_wait_until(pose{0, 24, 0.0});
                break;
            }
          },
          4000, &ms);
      Slow::uninstall();
      INFO(kind_name(kind), ", task pace ", every * 10, " ms: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered);
      // A robot that is not getting anywhere is ended within about ten passes of the task (4 s at the slowest pace), not after tens of seconds: the
      // same wait ended within a second before the exit was held. The hold is bounded by how far the robot still has to go over the stop speed.
      CHECK(returned);
      CHECK(ms < 6000.0);
    }
  }
}
