// A stuck verdict that only the wall clock produced (the auto task made no pass for several windows) must never read as "settled".
//
// The settled reading of a stuck verdict trusts the PIDs' errors, and those only change when the auto task runs. A task that is gone
// for good leaves the last error it computed in place, so a robot that was inside the big error when the task stopped is still
// "inside the big error" to the wait, however far it has been carried since. These tests park the auto task for good the moment the
// robot is inside the big error and then drag it away, and judge every kind of wait against the sim's own positions: the wait has
// to return (never hang) and has to report interfered, because a verdict the task could not have confirmed is never a clean one.
//
// A task that is only starved for a while must not be flagged: the control cases starve it for less than the wall clock fallback
// needs and expect the finished motion to read clean.
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// Wraps the sim's tick: once `when` says the robot is where the test wants the task to die, the auto task stops running (for good, or
// for hold_ms) and the robot is either carried away at (v, w) or held still.
struct Parker {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline std::function<bool(const Rig&)> when;
  static inline bool parked = false;
  static inline double carry_v = 0, carry_w = 0, hold_ms = 0, parked_at = 0;
  static void tick() {
    rig->record();
    double t = rig->sim.now_ms();
    if (!parked && when(*rig)) {
      parked = true;
      parked_at = t;
      rig->sim.passes_per_tick(0);
      if (hold_ms > 0)
        rig->sim.pin(t, hold_ms);
      else
        rig->sim.carry(carry_v, carry_w, t, 1.0e9);
    }
    if (parked && hold_ms > 0 && t >= parked_at + hold_ms) rig->sim.passes_per_tick(1);
    inner();
  }
  static void install(Rig& r, std::function<bool(const Rig&)> w, double v, double ang, double hold) {
    rig = &r;
    when = std::move(w);
    parked = false;
    carry_v = v;
    carry_w = ang;
    hold_ms = hold;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Parker::tick;
  }
};

double avg_position(const Rig& r) { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }

struct Case {
  const char* name;
  std::function<void(Drive&)> start;
  std::function<void(Drive&)> wait;
  std::function<bool(const Rig&)> park_when;
  double carry_v, carry_w;
  std::function<bool(const Rig&)> far_off;  // ground truth: the robot is outside the big error of its target
};

std::vector<Case> cases() {
  auto drive_to = [](double inches) { return [=](Drive& c) { c.pid_drive_set(inches, 110); }; };
  auto turn_to = [](double deg) { return [=](Drive& c) { c.pid_turn_set(deg, 110); }; };
  auto swing_to = [](double deg) { return [=](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, deg, 110); }; };
  auto ptp = [](Drive& c) { c.pid_odom_set({{0_in, 24_in}, fwd, 110}); };
  auto path = [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{0_in, 24_in}, fwd, 110}}); };
  auto long_path = [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{0_in, 24_in}, fwd, 110}, {{0_in, 36_in}, fwd, 110}}); };
  auto at_least = [](double inches) { return [=](const Rig& r) { return avg_position(r) >= inches; }; };
  auto heading_at_least = [](double deg) { return [=](const Rig& r) { return -r.sim.heading_deg() >= deg; }; };
  auto short_of = [](double inches, double band) { return [=](const Rig& r) { return avg_position(r) < inches - band; }; };
  auto heading_short_of = [](double deg, double band) { return [=](const Rig& r) { return -r.sim.heading_deg() < deg - band; }; };
  return {
      {"drive pid_wait", drive_to(24), [](Drive& c) { c.pid_wait(); }, at_least(22), -8, 0, short_of(24, 3)},
      {"odom point pid_wait", ptp, [](Drive& c) { c.pid_wait(); }, at_least(22), -8, 0, short_of(24, 3)},
      {"odom path pid_wait", path, [](Drive& c) { c.pid_wait(); }, at_least(22), -8, 0, short_of(24, 3)},
      {"turn pid_wait", turn_to(90), [](Drive& c) { c.pid_wait(); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"swing pid_wait", swing_to(90), [](Drive& c) { c.pid_wait(); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"drive pid_wait_until the target", drive_to(48), [](Drive& c) { c.pid_wait_until(48_in); }, at_least(46), -8, 0, short_of(48, 3)},
      {"drive pid_wait_quick_chain", drive_to(48), [](Drive& c) { c.pid_wait_quick_chain(); }, at_least(46), -8, 0, short_of(48, 3)},
      {"turn pid_wait_until the target", turn_to(90), [](Drive& c) { c.pid_wait_until(90_deg); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"turn pid_wait_quick", turn_to(90), [](Drive& c) { c.pid_wait_quick(); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"turn pid_wait_quick_chain", turn_to(90), [](Drive& c) { c.pid_wait_quick_chain(); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"swing pid_wait_until the target", swing_to(90), [](Drive& c) { c.pid_wait_until(90_deg); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"swing pid_wait_quick", swing_to(90), [](Drive& c) { c.pid_wait_quick(); }, heading_at_least(84), 0, -20, heading_short_of(90, 7)},
      {"odom point pid_wait_until_point", ptp, [](Drive& c) { c.pid_wait_until_point({0, 24}); }, at_least(22), -8, 0, short_of(24, 3)},
      {"odom point pid_wait_quick", ptp, [](Drive& c) { c.pid_wait_quick(); }, at_least(22), -8, 0, short_of(24, 3)},
      {"odom point pid_wait_quick_chain", ptp, [](Drive& c) { c.pid_wait_quick_chain(); }, at_least(22), -8, 0, short_of(24, 3)},
      {"odom path pid_wait_until_index", long_path, [](Drive& c) { c.pid_wait_until_index(2); }, at_least(34), -8, 0, short_of(36, 3)},
      {"odom path pid_wait_quick", path, [](Drive& c) { c.pid_wait_quick(); }, at_least(22), -8, 0, short_of(24, 3)},
      {"odom point pid_wait_until(distance)", ptp, [](Drive& c) { c.pid_wait_until(24_in); }, at_least(22), -8, 0, short_of(24, 3)},
  };
}

}  // namespace

TEST_CASE("an auto task that is gone for good never lets a wait end clean, on any wait") {
  for (const Case& cs : cases()) {
    Rig r(archetype_classroom(), 1, false);
    r.chassis.pid_print_toggle(false);
    Parker::install(r, cs.park_when, cs.carry_v, cs.carry_w, 0);
    cs.start(r.chassis);
    double elapsed = 0;
    bool ok = r.wait([&]() { cs.wait(r.chassis); }, 1500, &elapsed);
    INFO(std::string(cs.name), ": parked=", Parker::parked, " at ", Parker::parked_at, " ms, elapsed=", elapsed, " ms, interfered=", r.chassis.interfered,
         ", true avg position=", avg_position(r), ", heading=", -r.sim.heading_deg());
    CHECK(Parker::parked);
    // Never a hang
    CHECK(ok);
    if (!ok || !Parker::parked) continue;
    // The robot really is outside the big error of its target now (the sim's truth, not the library's belief)
    CHECK(cs.far_off(r));
    // So a clean return would be a wrong verdict
    CHECK(r.chassis.interfered);
  }
}

TEST_CASE("an auto task starved only for a while does not flag a finished motion") {
  struct Control {
    const char* name;
    std::function<void(Drive&)> start;
    std::function<void(Drive&)> wait;
    std::function<bool(const Rig&)> park_when;
  };
  const Control controls[] = {
      {"drive pid_wait", [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, [](const Rig& r) { return avg_position(r) >= 22; }},
      {"turn pid_wait", [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, [](const Rig& r) { return -r.sim.heading_deg() >= 84; }},
      {"odom point pid_wait", [](Drive& c) { c.pid_odom_set({{0_in, 24_in}, fwd, 110}); }, [](Drive& c) { c.pid_wait(); },
       [](const Rig& r) { return avg_position(r) >= 22; }},
  };
  for (const Control& ct : controls) {
    Rig r(archetype_classroom(), 1, false);
    r.chassis.pid_print_toggle(false);
    // 1.5 s without a pass, held still, then the task comes back: under the 4 windows (2 s at the default 500 ms window) the fallback needs
    Parker::install(r, ct.park_when, 0, 0, 1500);
    ct.start(r.chassis);
    double elapsed = 0;
    bool ok = r.wait([&]() { ct.wait(r.chassis); }, 1500, &elapsed);
    INFO(std::string(ct.name), ": elapsed=", elapsed, " ms, interfered=", r.chassis.interfered);
    CHECK(Parker::parked);
    CHECK(ok);
    if (ok) CHECK_FALSE(r.chassis.interfered);
  }
}
