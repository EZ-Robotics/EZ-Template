// An axis whose small error and big error are both 0 has its window exits turned off. That is not a request to never settle: the stuck
// check still has to be able to call a robot that has come to rest at its target settled, and it does that against the big error the
// library ships for that axis (3 in for distance, 7 degrees for angle) when the team has set none of its own.
//
// Before, a motion on such an axis finished at its target and then reported "Stuck before settling" (interfered) about half a second
// late, on every robot, because "inside the big error" was never true. These tests run the motions on four robots, judge them against
// the sim's own positions, and also hold a robot far outside the fallback band to make sure a robot that is not there is still
// interfered.
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

struct Robot {
  const char* name;
  sim::SimArchetype arch;
  int passes;
};

std::vector<Robot> robots() {
  return {{"light_fast", sim::archetype_light_fast(), 1},
          {"heavy_slow", sim::archetype_heavy_slow(), 2},
          {"sticky", sim::archetype_sticky_high_friction(), 1},
          {"classroom", archetype_classroom(), 1}};
}

double avg_position(const Rig& r) { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }

// The exits with both errors zeroed and the team's own windows kept
void zero_xy(Drive& c) { c.pid_odom_drive_exit_condition_set(90, 0.0, 250, 0.0, 500, 750); }
void zero_odom_angle(Drive& c) { c.pid_odom_turn_exit_condition_set(90, 0.0, 250, 0.0, 500, 750); }

// Once the robot is within 2 in of its point, turns it 40 degrees (carried for 1 s) and holds it there for good
struct Dragger {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline bool triggered = false;
  static void tick() {
    rig->record();
    if (!triggered && avg_position(*rig) >= 22.0) {
      triggered = true;
      double t = rig->sim.now_ms();
      rig->sim.carry(0.0, -40.0, t, 1000.0);
      rig->sim.pin(t + 1000.0, 1.0e9);
    }
    inner();
  }
  static void install(Rig& r) {
    rig = &r;
    triggered = false;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Dragger::tick;
  }
};

struct Zeroed {
  const char* name;
  std::function<void(Drive&)> apply;
};

}  // namespace

TEST_CASE("odom motions end clean at their target when an axis has both errors zeroed") {
  const Zeroed zeroed[] = {
      {"xy zeroed", [](Drive& c) { zero_xy(c); }},
      {"angle zeroed", [](Drive& c) { zero_odom_angle(c); }},
      {"both zeroed",
       [](Drive& c) {
         zero_xy(c);
         zero_odom_angle(c);
       }},
  };
  for (const Robot& robot : robots())
    for (const Zeroed& z : zeroed)
      for (int path = 0; path < 2; path++) {
        Rig r(robot.arch, robot.passes, false);
        r.chassis.pid_print_toggle(false);
        z.apply(r.chassis);
        if (path == 0)
          r.chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
        else
          r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{0_in, 24_in}, fwd, 110}});
        double elapsed = 0;
        bool ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed);
        INFO(std::string(robot.name), " / ", std::string(z.name), " / ", std::string(path == 0 ? "point" : "path"), ": elapsed=", elapsed,
             " ms, interfered=", r.chassis.interfered, ", true position=", avg_position(r), ", true heading=", -r.sim.heading_deg());
        REQUIRE(ok);
        CHECK_FALSE(r.chassis.interfered);
        // At rest inside the fallback band, by the sim's own state
        CHECK(std::fabs(24.0 - avg_position(r)) < 3.0);
        CHECK(std::fabs(r.sim.heading_deg()) < 7.0);
        CHECK(r.drive_speed_over(250) < r.drive_floor(250));
      }
}

TEST_CASE("drive, turn and swing end clean at their target when both errors are zeroed") {
  for (const Robot& robot : robots())
    for (int kind = 0; kind < 3; kind++) {
      Rig r(robot.arch, robot.passes, false);
      r.chassis.pid_print_toggle(false);
      if (kind == 0) {
        r.chassis.pid_drive_exit_condition_set(90, 0.0, 250, 0.0, 500, 500);
        r.chassis.pid_drive_set(24, 110);
      } else if (kind == 1) {
        r.chassis.pid_turn_exit_condition_set(90, 0.0, 250, 0.0, 500, 500);
        r.chassis.pid_turn_set(90, 110);
      } else {
        r.chassis.pid_swing_exit_condition_set(90, 0.0, 250, 0.0, 500, 500);
        r.chassis.pid_swing_set(ez::LEFT_SWING, 90, 110);
      }
      double elapsed = 0;
      bool ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed);
      std::string what = kind == 0 ? "drive" : kind == 1 ? "turn" : "swing";
      INFO(std::string(robot.name), " / ", what, ": elapsed=", elapsed, " ms, interfered=", r.chassis.interfered, ", true position=", avg_position(r),
           ", true heading=", -r.sim.heading_deg());
      REQUIRE(ok);
      CHECK_FALSE(r.chassis.interfered);
      if (kind == 0)
        CHECK(std::fabs(24.0 - avg_position(r)) < 3.0);
      else
        CHECK(std::fabs(90.0 + r.sim.heading_deg()) < 7.0);
    }
}

TEST_CASE("a robot held far outside the fallback band is still interfered when an axis has both errors zeroed") {
  for (const Robot& robot : robots()) {
    // Distance: held 8 in short of an odom point with the xy errors zeroed (the fallback band is 3 in)
    {
      Rig r(robot.arch, robot.passes, false);
      r.chassis.pid_print_toggle(false);
      zero_xy(r.chassis);
      r.chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
      r.sim.wall(8.0);
      double elapsed = 0;
      bool ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed);
      INFO(std::string(robot.name), " / xy held: elapsed=", elapsed, " ms, interfered=", r.chassis.interfered, ", true position=", avg_position(r));
      REQUIRE(ok);
      CHECK(avg_position(r) < 24.0 - 3.0);
      CHECK(r.chassis.interfered);
    }
    // Angle: the robot arrives, then is turned 40 degrees off and held there for good, with the angle errors zeroed (the fallback band is 7
    // degrees)
    {
      Rig r(robot.arch, robot.passes, false);
      r.chassis.pid_print_toggle(false);
      zero_odom_angle(r.chassis);
      r.chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
      Dragger::install(r);
      double elapsed = 0;
      bool ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed);
      INFO(std::string(robot.name), " / angle held: elapsed=", elapsed, " ms, interfered=", r.chassis.interfered, ", true heading=", -r.sim.heading_deg());
      REQUIRE(Dragger::triggered);
      REQUIRE(ok);
      CHECK(std::fabs(r.sim.heading_deg()) > 7.0);
      CHECK(r.chassis.interfered);
    }
  }
}
