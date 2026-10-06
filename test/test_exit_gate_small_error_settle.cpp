// The settle clock (what decides a robot inside big_error is "settled while still moving") has to credit the same coarse step
// whatever the team's small_error is. It used the team's own small_error when one was set, so a tight one (0.03 to 0.2 in on a
// drive, 0.2 deg on the heading) made big_error / step tens to hundreds of steps and the clock restarted on every one: a sticky
// robot creeping home in a heading limit cycle held the second pid_wait_quick_chain() on a settled odom point for about 31 s,
// where the start commit took 270 ms. Small errors of 0.25 in and up stayed inside 3.4 s, which is why only a tight one showed.
//
// Every wait here is also bounded in simulated time: "no return by the cap" is reported as a failure, never waited out.
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

struct Cfg {
  const char* name;
  double small_error;          // odom drive
  double heading_small_error;  // odom turn
};

void configure(Rig& r, const Cfg& c) {
  r.chassis.pid_odom_drive_exit_condition_set(90, c.small_error, 250, 3, 0, 750);
  r.chassis.pid_odom_turn_exit_condition_set(90, c.heading_small_error, 250, 5, 0, 0);
}

}  // namespace

TEST_CASE("tight small_error: repeated until_point + quick_chain waits on a settled sticky odom point come back within a few seconds") {
  const Cfg cfgs[] = {{"se 0.05 / 0.2 deg", 0.05, 0.2}, {"se 0.03 / 0.2 deg", 0.03, 0.2}, {"se 0.1 / 0.2 deg", 0.1, 0.2}, {"se 0.2 / 0.2 deg", 0.2, 0.2}};
  for (const Cfg& c : cfgs) {
    for (int reverse = 0; reverse < 2; reverse++) {
      Rig r(sim::archetype_sticky_high_friction(), 1, false, 1);
      configure(r, c);
      pose p = reverse ? pose{-12, -24, ANGLE_NOT_SET} : pose{12, 24, ANGLE_NOT_SET};
      r.chassis.pid_odom_set(odom{p, reverse ? rev : fwd, 110});
      for (int round = 0; round < 3; round++) {
        // A shove before the third round, as the repro had
        if (round == 2) {
          for (int t = 0; t < 500; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
          r.sim.displace(1.0);
        }
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
        if (std::getenv("EZ_SETTLE_PRINT")) std::printf("[settle-se] %s rev=%d round=%d point %.0f chain %.0f\n", c.name, reverse, round, e_point, e_chain);
      }
    }
  }
}

TEST_CASE("controls: a robot pinned outside big_error is interfered at the same instant whatever small_error is") {
  const Cfg cfgs[] = {{"se 0.05 / 0.2 deg", 0.05, 0.2}, {"se 1 / 1.5 deg", 1.0, 1.5}, {"se 0 / 0", 0.0, 0.0}};
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
      if (std::getenv("EZ_SETTLE_PRINT")) std::printf("[settle-pin] %s pin=%.0f interfered=%d elapsed %.0f\n", c.name, pin_at, (int)r.chassis.interfered, e);
    }
  }
}
