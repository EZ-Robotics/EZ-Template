// A repeated pid_wait_until_point() + pid_wait_quick_chain() on an odom point that has already settled must come back in a few
// seconds, whatever the exit set. With the small exit off (small_error 0) the stuck watch's progress step was the velocity exit's
// per-pass noise floor, a few hundredths of an inch, and inside big_error every such step restarted the watch's settle clock. A
// sticky robot creeping home at about step / window (0.075 in/s) made a new low just inside the 750 ms window each time, and the
// ~3 in of big_error holds tens of those steps, so the chain wait ran about 30 s. The start commit returned in a fraction of a
// second. Inside big_error the watch now credits the same step the backstop is sized from (1 in, 3 deg), as it already does when
// small_error is set.
//
// Every wait here is also bounded in simulated time: "no return by the cap" is reported as a failure, never waited out.
#include <cmath>
#include <cstdio>
#include <functional>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

constexpr int CAP_TICKS = 20000;
constexpr double BOUND_MS = 4000;

struct Cfg {
  const char* name;
  int small_time;
  double small_error;
  int big_time;
  double big_error;
};

void configure(Rig& r, const Cfg& c) {
  r.chassis.pid_odom_drive_exit_condition_set(c.small_time, c.small_error, c.big_time, c.big_error, 0, 750);
  r.chassis.pid_odom_turn_exit_condition_set(c.small_time, c.small_error * 1.5, c.big_time, c.big_error == 0 ? 0 : 5, 0, 0);
}

}  // namespace

TEST_CASE("settled sticky odom point: repeated until_point + quick_chain waits come back within a few seconds") {
  const Cfg cfgs[] = {{"small exit off, big on", 0, 0, 250, 3}, {"small exit on, big on", 90, 1, 250, 3}};
  for (const Cfg& c : cfgs) {
    for (int reverse = 0; reverse < 2; reverse++) {
      Rig r(sim::archetype_sticky_high_friction(), 1, false, 1);
      configure(r, c);
      pose p = reverse ? pose{-12, -24, ANGLE_NOT_SET} : pose{12, 24, ANGLE_NOT_SET};
      r.chassis.pid_odom_set(odom{p, reverse ? rev : fwd, 110});
      for (int round = 0; round < 3; round++) {
        double e_point = 0, e_chain = 0;
        bool ok_point = r.wait([&] { r.chassis.pid_wait_until_point(p); }, CAP_TICKS, &e_point);
        bool ok_chain = ok_point && r.wait([&] { r.chassis.pid_wait_quick_chain(); }, CAP_TICKS, &e_chain);
        CAPTURE(c.name);
        CAPTURE(reverse);
        CAPTURE(round);
        CAPTURE(e_point);
        CAPTURE(e_chain);
        REQUIRE(ok_point);
        REQUIRE(ok_chain);
        CHECK(e_point < BOUND_MS);
        CHECK(e_chain < BOUND_MS);
        if (std::getenv("EZ_CREEP_PRINT")) std::printf("[creep] %s rev=%d round=%d point %.0f chain %.0f\n", c.name, reverse, round, e_point, e_chain);
      }
    }
  }
}

TEST_CASE("controls: a robot pinned outside big_error on a chain wait is still interfered, and at the same time") {
  const Cfg cfgs[] = {{"small exit off, big on", 0, 0, 250, 3}, {"small exit on, big on", 90, 1, 250, 3}};
  for (const Cfg& c : cfgs) {
    for (double pin_at : {150.0, 300.0}) {
      Rig r(sim::archetype_light_fast(), 1);
      configure(r, c);
      pose p{12, 24, ANGLE_NOT_SET};
      r.chassis.pid_odom_set(odom{p, fwd, 110});
      r.sim.pin(pin_at, 600000);
      double e = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, CAP_TICKS, &e);
      CAPTURE(c.name);
      CAPTURE(pin_at);
      REQUIRE(ok);
      CHECK(r.chassis.interfered);
      CHECK(e < BOUND_MS);
      // The same instant the start of this change returned at, so the settle clock cannot have moved a pin's verdict
      double expected = c.small_error > 0 ? (pin_at == 150.0 ? 920.0 : 1080.0) : (pin_at == 150.0 ? 930.0 : 1080.0);
      CHECK(std::fabs(e - expected) < 1.0);
      if (std::getenv("EZ_CREEP_PRINT")) std::printf("[creep-pin] %s pin=%.0f interfered=%d elapsed %.0f\n", c.name, pin_at, (int)r.chassis.interfered, e);
    }
  }
}
