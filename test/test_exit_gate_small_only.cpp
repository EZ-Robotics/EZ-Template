// A team that configures only the small exit (big_error 0, big_exit_time 0) still has a robot that comes to rest inside its small
// band arrive, not get stuck. The speed gate holds the small exit until the robot has stopped over the small exit's own time; a
// heavy or sticky robot coasting in can use up the stuck watch's no-progress window first, and with no big_error the watch had no
// settled case to read it as, so it reported the finished motion as interfered. The start commit returned clean for the same
// robot, because it left on the first pass inside the band. Something stopped outside the small band is still interfered.
#include <cmath>
#include <cstdio>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Arch {
  sim::SimArchetype a;
  int passes;
  const char* name;
};

std::vector<Arch> coasters() {
  return {{sim::archetype_sticky_high_friction(), 1, "sticky"},
          {sim::archetype_heavy_slow(), 2, "heavy_slow 2 passes"},
          {sim::archetype_heavy_slow(), 3, "heavy_slow 3 passes"}};
}

}  // namespace

TEST_CASE("small exit only: an undisturbed odom point move that rests inside the small band is not interfered") {
  for (auto& ar : coasters()) {
    Rig r(ar.a, ar.passes, false, 1);
    r.chassis.pid_odom_drive_exit_condition_set(90, 1, 0, 0, 500, 750);
    r.chassis.pid_odom_set(odom{pose{12, 24, ANGLE_NOT_SET}, fwd, 110});
    CAPTURE(ar.name);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(std::fabs(std::hypot(12.0, 24.0) - r.trace.back().avg) < 1.0);
  }
}

TEST_CASE("small exit only: an undisturbed turn that rests inside the small band is not interfered") {
  for (auto& ar : coasters()) {
    Rig r(ar.a, ar.passes, false, 1);
    r.chassis.pid_turn_exit_condition_set(90, 1.5, 0, 0, 500, 750);
    r.chassis.pid_turn_set(90, 110);
    CAPTURE(ar.name);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
    CHECK(std::fabs(90.0 - std::fabs(r.trace.back().heading)) < 1.5);
  }
}

TEST_CASE("small exit only: an undisturbed turn to a point and a swing that rest inside the small band are not interfered") {
  {
    Rig r(sim::archetype_sticky_high_friction(), 1, false, 1);
    r.chassis.pid_turn_exit_condition_set(90, 1.5, 0, 0, 500, 750);
    r.chassis.pid_turn_set(pose{12, 24}, fwd, 110);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_heavy_slow(), 2, false, 1);
    r.chassis.pid_swing_exit_condition_set(90, 1.5, 0, 0, 500, 750);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90, 110);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("controls: with the big exit on, or big_error set and its time 0, the same moves are not interfered") {
  for (int big_time : {0, 250}) {
    for (auto& ar : coasters()) {
      {
        Rig r(ar.a, ar.passes, false, 1);
        r.chassis.pid_odom_drive_exit_condition_set(90, 1, big_time, 3, 500, 750);
        r.chassis.pid_odom_set(odom{pose{12, 24, ANGLE_NOT_SET}, fwd, 110});
        CAPTURE(ar.name);
        CAPTURE(big_time);
        REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
        CHECK_FALSE(r.chassis.interfered);
      }
      {
        Rig r(ar.a, ar.passes, false, 1);
        r.chassis.pid_turn_exit_condition_set(90, 1.5, big_time, 3, 500, 750);
        r.chassis.pid_turn_set(90, 110);
        CAPTURE(ar.name);
        CAPTURE(big_time);
        REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
        CHECK_FALSE(r.chassis.interfered);
      }
    }
  }
}

TEST_CASE("controls: with only the small exit configured, a robot pinned or walled outside the small band is still interfered") {
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_odom_drive_exit_condition_set(90, 1, 0, 0, 500, 750);
    r.chassis.pid_odom_set(odom{pose{12, 24, ANGLE_NOT_SET}, fwd, 110});
    r.sim.pin(300, 600000);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_odom_drive_exit_condition_set(90, 1, 0, 0, 500, 750);
    r.chassis.pid_odom_set(odom{pose{12, 24, ANGLE_NOT_SET}, fwd, 110});
    r.sim.wall(8);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_drive_exit_condition_set(90, 1, 0, 0, 500, 750);
    r.chassis.pid_drive_set(48_in, 110);
    r.sim.pin(400, 20000);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_turn_exit_condition_set(90, 1.5, 0, 0, 500, 750);
    r.chassis.pid_turn_set(90_deg, 90);
    r.sim.pin(0, 20000);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK(r.chassis.interfered);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_swing_exit_condition_set(90, 1.5, 0, 0, 500, 750);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90, 0);
    r.sim.pin(0, 20000);
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
    CHECK(r.chassis.interfered);
  }
}
