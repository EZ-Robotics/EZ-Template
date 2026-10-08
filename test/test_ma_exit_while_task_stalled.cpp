// An mA exit is a settle only if the errors it is judged by are the robot's, and those only change when the auto task runs. It used to be ended on the
// spot, with the verdict decided by how long the task had been quiet (350 ms or more: interfered, whatever the robot was doing, less: settled on the
// errors the task last wrote):
//   - a task that missed 350 ms or more with the motors over current ended a robot that was at rest inside its band, interfered, though the errors it
//     was judged by were the right ones;
//   - a task that was gone for good with a short mA timeout (100 ms, which is under that 350) ended a robot that was being carried 12 in past its target
//     clean, "counted as settled".
// The exit is now held while the task has not passed, and decided on the first pass the task makes. A task that comes back is judged on its fresh
// errors; one that never does is ended by the stuck watch, interfered, never settled.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

double avg_position(const Rig& r) { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }

// Runs the auto task on every tick except [gap_from, gap_from + gap_ms) of sim time (forever from gap_from when gap_ms is 0, and the robot is carried at
// `carry` in/s from then on when that is not 0), and holds the drive motors over current from `oc_from` until the end
struct Hook {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline double gap_from = 0, gap_ms = 0, oc_from = 0, carry_v = 0, carry_w = 0;
  static inline int passes = 1;
  static inline std::function<bool(const Rig&)> park_when;
  static inline bool parked = false;
  static void tick() {
    double t = rig->sim.now_ms();
    if (park_when && !parked && park_when(*rig)) {
      parked = true;
      gap_from = t;
      if (carry_v != 0.0 || carry_w != 0.0) rig->sim.carry(carry_v, carry_w, t, 1.0e9);
    }
    bool gap = park_when ? parked : (t >= gap_from && (gap_ms == 0 || t < gap_from + gap_ms));
    rig->sim.passes_per_tick(gap ? 0 : passes);
    inner();
    if (rig->sim.now_ms() >= oc_from) {
      for (auto& m : rig->chassis.left_motors) m.fake().over_current = true;
      for (auto& m : rig->chassis.right_motors) m.fake().over_current = true;
    }
  }
  static void install(Rig& r, double from, double ms, double oc, int ps) {
    rig = &r;
    passes = ps;
    carry_w = 0.0;
    gap_from = from;
    gap_ms = ms;
    oc_from = oc;
    carry_v = 0.0;
    park_when = nullptr;
    parked = false;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Hook::tick;
  }
  static void uninstall() { test_stub::g_clock.on_delay = inner; }
};

}  // namespace

TEST_CASE("an mA exit in a long gap of the auto task, with the robot at rest inside its band, ends the wait clean") {
  // light_fast at 3 passes per poll, default exits, forced over current from 150 ms. The robot is at rest inside the 3 in big error well before the gap.
  for (double gap_from : {450.0, 550.0, 650.0})
    for (double gap_ms : {400.0, 600.0, 800.0, 1000.0}) {
      Rig r(sim::archetype_light_fast(), 3, false, 9);
      r.chassis.pid_print_toggle(false);
      Hook::install(r, gap_from, gap_ms, 150.0, 3);
      r.chassis.pid_drive_set(24_in, 110);
      double ms = 0;
      bool returned = r.wait([&] { r.chassis.pid_wait(); }, 2000, &ms);
      Hook::uninstall();
      double off = std::fabs(24.0 - avg_position(r));
      INFO("gap of ", gap_ms, " ms from ", gap_from, " ms: returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered, ", true error ", off,
           " in");
      REQUIRE(returned);
      // The robot arrived and stopped long before the gap. Judged on the sim's own state at the return: inside the big error the exit is a settle
      // (clean), outside it is not
      if (off < 3.0)
        CHECK_FALSE(r.chassis.interfered);
      else
        CHECK(r.chassis.interfered);
    }
}

TEST_CASE("an auto task that is gone for good never lets an mA exit of 100 ms end a wait clean while the robot is carried away") {
  struct Case {
    const char* name;
    bool angular;
  };
  const Case cases[] = {{"drive", false}, {"turn", true}};
  for (const Case& cs : cases) {
    Rig r(archetype_classroom(), 1, false);
    r.chassis.pid_print_toggle(false);
    r.chassis.pid_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
    r.chassis.pid_turn_exit_condition_set(90, 3, 200, 7, 100, 100);
    Hook::install(r, 0.0, 0.0, 0.0, 1);
    // The task is gone the moment the robot is inside the big error, and from then on the robot is carried away from the target
    Hook::carry_v = cs.angular ? 0.0 : -20.0;
    Hook::carry_w = cs.angular ? -50.0 : 0.0;
    Hook::park_when = [&](const Rig& rg) { return cs.angular ? -rg.sim.heading_deg() >= 84.0 : avg_position(rg) >= 22.0; };
    if (cs.angular)
      r.chassis.pid_turn_set(90_deg, 110);
    else
      r.chassis.pid_drive_set(24_in, 110);
    double ms = 0;
    bool returned = r.wait([&] { r.chassis.pid_wait(); }, 2500, &ms);
    Hook::uninstall();
    double off = cs.angular ? std::fabs(90.0 + r.sim.heading_deg()) : std::fabs(24.0 - avg_position(r));
    INFO(std::string(cs.name), ": parked=", Hook::parked, ", returned=", returned, " at ", ms, " ms, interfered=", r.chassis.interfered, ", ", off, " off");
    CHECK(Hook::parked);
    // Never a hang
    CHECK(returned);
    CHECK(r.chassis.interfered);
  }
}
