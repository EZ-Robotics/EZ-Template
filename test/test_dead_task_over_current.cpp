// An mA exit inside the big error windows is a settle, which is only true if the errors it is judged by are the robot's. They change when the auto
// task runs. A task that is gone for good (or has not run for longer than the stuck watches' shortest window) leaves the last errors it computed in
// place, so a robot that was inside the big error when the task stopped still reads as inside it however far it is carried afterwards, and the motors
// of a robot being pushed read over current. These tests park the task for good the moment the robot is inside the big error, carry the robot away
// and hold the motors over current, and judge every wait against the sim's own positions: it has to return (never hang) and report interfered.
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "EZ-Template/travel.hpp"
#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Parker {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline std::function<bool(const Rig&)> when;
  static inline bool parked = false;
  static inline double carry_v = 0, carry_w = 0;
  static void tick() {
    rig->record();
    if (!parked && when(*rig)) {
      parked = true;
      rig->sim.passes_per_tick(0);
      rig->sim.carry(carry_v, carry_w, rig->sim.now_ms(), 1.0e9);
    }
    inner();
    if (parked)
      for (auto* side : {&rig->chassis.left_motors, &rig->chassis.right_motors})
        for (auto& m : *side) {
          m.fake().over_current = true;
          m.fake().current_draw = 3000.0;
        }
  }
  static void install(Rig& r, std::function<bool(const Rig&)> w, double v, double ang) {
    rig = &r;
    when = std::move(w);
    parked = false;
    carry_v = v;
    carry_w = ang;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Parker::tick;
  }
};

double avg_position(const Rig& r) { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }

struct Case {
  const char* name;
  std::function<void(Drive&)> start;
  std::function<void(Drive&)> wait;
  bool angular;
  double target;
};

}  // namespace

TEST_CASE("a task that is gone for good never lets an mA exit end a wait clean, on any wait") {
  const std::vector<Case> cases = {
      {"drive pid_wait", [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, false, 24},
      {"turn pid_wait", [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, true, 90},
      {"swing pid_wait", [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, true, 90},
      {"odom point pid_wait", [](Drive& c) { c.pid_odom_set({{0_in, 24_in}, fwd, 110}); }, [](Drive& c) { c.pid_wait(); }, false, 24},
      {"odom path pid_wait", [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{0_in, 24_in}, fwd, 110}}); }, [](Drive& c) { c.pid_wait(); }, false,
       24},
      {"drive pid_wait_until the target", [](Drive& c) { c.pid_drive_set(48_in, 110); }, [](Drive& c) { c.pid_wait_until(48_in); }, false, 48},
      {"turn pid_wait_until the target", [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait_until(90_deg); }, true, 90},
      {"odom point pid_wait_until_point", [](Drive& c) { c.pid_odom_set({{0_in, 24_in}, fwd, 110}); }, [](Drive& c) { c.pid_wait_until_point({0, 24}); }, false,
       24},
  };
  for (const Case& cs : cases) {
    Rig r(archetype_classroom(), 1, false);
    r.chassis.pid_print_toggle(false);
    double near = cs.angular ? 84 : cs.target - 2;
    std::function<bool(const Rig&)> when = cs.angular ? std::function<bool(const Rig&)>([=](const Rig& rg) { return -rg.sim.heading_deg() >= near; })
                                                      : std::function<bool(const Rig&)>([=](const Rig& rg) { return avg_position(rg) >= near; });
    Parker::install(r, when, cs.angular ? 0.0 : -20.0, cs.angular ? -50.0 : 0.0);
    cs.start(r.chassis);
    double ms = 0;
    bool ok = r.wait([&] { cs.wait(r.chassis); }, 2500, &ms);
    test_stub::g_clock.on_delay = Parker::inner;
    double off = cs.angular ? std::fabs(cs.target + r.sim.heading_deg()) : std::fabs(cs.target - avg_position(r));
    INFO(std::string(cs.name), ": parked=", Parker::parked, ", returned=", ok, " at ", ms, " ms, interfered=", r.chassis.interfered, ", ", off,
         std::string(cs.angular ? " deg" : " in"), " off");
    CHECK(Parker::parked);
    // Never a hang
    CHECK(ok);
    if (!ok || !Parker::parked) continue;
    // The robot really is outside the big error of its target now (the sim's truth, not the library's belief)
    CHECK(off > (cs.angular ? 7.0 : 3.0));
    // So a clean return would be a wrong verdict
    CHECK(r.chassis.interfered);
  }
}

TEST_CASE("PathTracker: it is quiet only when something was sampled and the newest sample is older than asked") {
  ez::detail::PathTracker t;
  CHECK_FALSE(t.quiet(5000, 350));
  std::uint32_t pass = 0;
  for (int ms = 0; ms <= 200; ms += 10) t.sample(0.0, 0.0, 5000 + ms, ++pass);
  CHECK_FALSE(t.quiet(5200, 350));
  CHECK_FALSE(t.quiet(5550, 350));
  CHECK(t.quiet(5551, 350));
  t.reset();
  CHECK_FALSE(t.quiet(9000, 350));
}
