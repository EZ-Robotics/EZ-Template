// A robot that is shoved back out of big_error the moment it gets there and then held outside it did not arrive, and every odom wait has to
// say so: interfered. The stuck watch used to settle a stopped robot that had once been inside big_error and was now within big_error
// plus one progress step of the point (a band "just outside"), which a robot carried back a little and pinned meets exactly: it read
// clean while held 3.1 to 3.4 in short of the point on every odom wait type, every archetype, noise on and off, where dev says
// interfered. A stuck verdict counts as settled only when the robot is inside its settle error on every axis the motion has.
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

enum class WaitKind {
  Wait,
  Quick,
  Chain,
  UntilPoint
};

const char* wait_name(WaitKind w) {
  switch (w) {
    case WaitKind::Wait:
      return "pid_wait";
    case WaitKind::Quick:
      return "pid_wait_quick";
    case WaitKind::Chain:
      return "pid_wait_quick_chain";
    default:
      return "pid_wait_until(point)";
  }
}

struct Outcome {
  bool returned = false;
  bool interfered = false;
  double ms = 0;
  double held_out = 0;  // how far the odom pose is from the final point when the wait returned
  std::string printed;
};

// The robot is carried back at 10 in/s for `carry_ms` the first pass it is within 3 in of the final point, then held for the rest of
// the run.
Outcome shove_and_pin(const sim::SimArchetype& a, int passes, bool noise, unsigned seed, bool path3, int carry_ms, WaitKind w) {
  Rig r(a, passes, noise, seed);
  r.chassis.pid_print_toggle(true);
  pose target = path3 ? pose{24, 36, 0} : pose{12, 24, 0};
  bool fired = false;
  r.sim.before_pass = [&](int) {
    if (fired) return;
    pose p = r.chassis.odom_pose_get();
    if (std::hypot(p.x - target.x, p.y - target.y) < 3.0) {
      fired = true;
      double t = r.sim.now_ms();
      r.sim.carry(-10.0, 0.0, t, carry_ms);
      r.sim.pin(t + carry_ms, 1e7);
    }
  };
  Outcome o;
  o.printed = test_stub::capture_stdout([&] {
    if (path3)
      r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in}, fwd, 110}, {{24_in, 36_in}, fwd, 110}}, true);
    else
      r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 110}, true);
    o.returned = r.wait(
        [&] {
          switch (w) {
            case WaitKind::Wait:
              r.chassis.pid_wait();
              break;
            case WaitKind::Quick:
              r.chassis.pid_wait_quick();
              break;
            case WaitKind::Chain:
              r.chassis.pid_wait_quick_chain();
              break;
            default:
              r.chassis.pid_wait_until(pose{target.x, target.y, 0});
              break;
          }
        },
        1500, &o.ms);
  });
  o.interfered = r.chassis.interfered;
  pose p = r.chassis.odom_pose_get();
  o.held_out = std::hypot(p.x - target.x, p.y - target.y);
  return o;
}

}  // namespace

// A robot the pin catches inside big_error (3 in at the shipped defaults) has arrived and reads clean, so only a robot held outside it is
// a case; most of the cases have to be one or the test checks nothing.
static void shove_and_pin_light_fast(bool path3) {
  int held = 0, cases = 0;
  for (int carry_ms : {90, 60})
    for (WaitKind w : {WaitKind::Wait, WaitKind::Quick, WaitKind::Chain, WaitKind::UntilPoint}) {
      Outcome o = shove_and_pin(sim::archetype_light_fast(), 1, false, 1, path3, carry_ms, w);
      CAPTURE(carry_ms);
      CAPTURE(std::string(wait_name(w)));
      CAPTURE(o.held_out);
      REQUIRE(o.returned);
      cases++;
      if (o.held_out > 3.0) {
        held++;
        CHECK_MESSAGE(o.interfered, "returned clean after " << o.ms << " ms held " << o.held_out << " in out\n" << o.printed);
      }
    }
  CHECK_MESSAGE(held >= 6, "only " << held << " of " << cases << " runs were held outside big_error");
}

TEST_CASE("an odom point carried back out of big_error and pinned reads interfered on every wait, light_fast, shipped defaults") {
  shove_and_pin_light_fast(false);
}

TEST_CASE("the same on a three point path ending at (24, 36)") { shove_and_pin_light_fast(true); }

TEST_CASE("the same on sticky, classroom and heavy_slow at 2 and 3 passes per poll, noise on and off") {
  struct Setup {
    const char* name;
    sim::SimArchetype arch;
    int passes;
  };
  std::vector<Setup> setups = {{"sticky", sim::archetype_sticky_high_friction(), 1},
                               {"classroom", archetype_classroom(), 1},
                               {"heavy_slow/2", sim::archetype_heavy_slow(), 2},
                               {"heavy_slow/3", sim::archetype_heavy_slow(), 3}};
  for (auto& s : setups)
    for (bool noise : {false, true})
      for (bool path3 : {false, true})
        for (WaitKind w : {WaitKind::Wait, WaitKind::Quick, WaitKind::Chain, WaitKind::UntilPoint}) {
          Outcome o = shove_and_pin(s.arch, s.passes, noise, 7, path3, 90, w);
          CAPTURE(std::string(s.name));
          CAPTURE(noise);
          CAPTURE(path3);
          CAPTURE(std::string(wait_name(w)));
          CAPTURE(o.held_out);
          REQUIRE(o.returned);
          // Only a robot that really is held outside big_error is a case: a robot the pin catches inside it has arrived
          if (o.held_out > 3.0) CHECK_MESSAGE(o.interfered, "returned clean after " << o.ms << " ms held " << o.held_out << " in out");
        }
}

// The same for the heading: the robot is inside big_error in xy, rotated away at 20 deg/s for 250 ms the pass its heading error first
// reads inside big_error (3 deg), and held with about 3.9 deg of heading error. xy is inside its band and the heading is outside it,
// so the robot did not arrive.
TEST_CASE("an odom point where the robot is rotated out of the heading big_error and pinned reads interfered") {
  Rig r(sim::archetype_light_fast(), 1, false, 1);
  r.chassis.pid_print_toggle(true);
  r.chassis.pid_odom_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
  r.chassis.pid_odom_turn_exit_condition_set(90, 1, 200, 3, 100, 100);
  bool fired = false;
  r.sim.before_pass = [&](int) {
    if (fired) return;
    pose p = r.chassis.odom_pose_get();
    if (std::hypot(p.x - 12, p.y - 24) < 3.0 && std::fabs(r.chassis.current_a_odomPID.error) < 3.0) {
      fired = true;
      double t = r.sim.now_ms();
      r.sim.carry(0.0, -20.0, t, 250);
      r.sim.pin(t + 250, 1e7);
    }
  };
  bool returned = false;
  std::string printed = test_stub::capture_stdout([&] {
    r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 110}, true);
    returned = r.wait([&] { r.chassis.pid_wait(); }, 1500);
  });
  REQUIRE(returned);
  double heading_error = std::fabs(r.chassis.current_a_odomPID.error);
  CHECK_MESSAGE(heading_error > 3.0, "the scenario: the heading is held outside its big_error (" << heading_error << " deg)");
  CHECK_MESSAGE(r.chassis.interfered, "returned clean with " << heading_error << " deg of heading error\n" << printed);
}
