// pid_wait_until(distance) on an odom move waits for xy AND heading to be stopped before it takes xy's window exit (a robot
// turning at the end of its point has not finished). Its only backstop is the left / right wheel watch, which sees the wheels, not
// the heading: a robot held in place and spun has one wheel making real progress toward the distance, so neither side reads stuck
// until that wheel gets there, 36 to 94 s at 6 deg/s. The start commit took 1.3 s. The gate has to wait on what a backstop
// watches: xy alone, since the heading has none in this loop (pid_wait and pid_wait_until_point watch the heading channel and
// gate on both).
//
// Every wait here is bounded in simulated time: "no return by the cap" is reported as a failure, never waited out.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

constexpr int CAP_TICKS = 20000;
constexpr double BOUND_MS = 4000;

}  // namespace

TEST_CASE("spun and pinned on a settled odom point: pid_wait_until(distance) comes back within a few seconds") {
  for (double deg_s : {6.0, 8.0, -6.0}) {
    for (double scale : {0.5, 1.5}) {
      Rig r(sim::archetype_sticky_high_friction(), 1, false, 1);
      r.chassis.pid_odom_drive_exit_condition_set(0, 0, 250, 3, 0, 750);
      r.chassis.pid_odom_turn_exit_condition_set(0, 0, 250, 5, 0, 0);
      r.sim.carry(0.0, deg_s, 1200, 600000);
      pose p{-12, -24, ANGLE_NOT_SET};
      double dd = std::hypot(p.x, p.y);
      r.chassis.pid_odom_set(odom{p, rev, 110});
      double e = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait_until(dd * scale); }, CAP_TICKS, &e);
      CAPTURE(deg_s);
      CAPTURE(scale);
      CAPTURE(e);
      REQUIRE(ok);
      CHECK(e < BOUND_MS);
      if (std::getenv("EZ_SPIN_PRINT")) std::printf("[spin] %.0f deg/s x%.1f elapsed %.0f interfered=%d\n", deg_s, scale, e, (int)r.chassis.interfered);
    }
  }
}

TEST_CASE("controls: a robot pinned outside big_error on pid_wait_until(distance) is still interfered, and at the same time") {
  for (double pin_at : {150.0, 300.0}) {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_odom_drive_exit_condition_set(0, 0, 250, 3, 0, 750);
    r.chassis.pid_odom_turn_exit_condition_set(0, 0, 250, 5, 0, 0);
    pose p{12, 24, ANGLE_NOT_SET};
    double dd = std::hypot(p.x, p.y);
    r.chassis.pid_odom_set(odom{p, fwd, 110});
    r.sim.pin(pin_at, 600000);
    double e = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait_until(dd * 0.9); }, CAP_TICKS, &e);
    CAPTURE(pin_at);
    REQUIRE(ok);
    CHECK(r.chassis.interfered);
    CHECK(e < BOUND_MS);
    if (std::getenv("EZ_SPIN_PRINT")) std::printf("[spin-pin] pin=%.0f interfered=%d elapsed %.0f\n", pin_at, (int)r.chassis.interfered, e);
  }
}
