// The no-new-low backstop inside big_error calls a robot settled only when its net displacement over the backstop window is under the stop
// speed times that window. A window that long (up to a couple of seconds) is mostly made of the time the robot sat still when a shove or a
// drag begins late in it, so the displacement is averaged down and a robot that is moving several times faster than the stop speed reads as
// going nowhere. Movement over the last stretch of the window has to count too.
//
// (The sim reports heading counterclockwise positive.)
#include <cmath>

#include "EZ-Template/travel.hpp"
#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;
using ez::detail::PathTracker;

namespace {

constexpr double FLOOR_IN_S = 1.5;
constexpr double COUNT_450_325 = 3.25 * M_PI / 400.0;
constexpr std::uint32_t T0 = 5000;

struct Feed {
  PathTracker t;
  std::uint32_t pass = 0;
  explicit Feed(double band) { t.band_set(band); }
  void at(int ms, double x) { t.sample(x, 0.0, T0 + ms, ++pass); }
  std::uint32_t now(int ms) const { return T0 + ms; }
};

}  // namespace

TEST_CASE("PathTracker: a thing that sat still and was then carried at several times the floor for the last quarter second is not in place") {
  for (double v : {6.0, 15.0, -15.0}) {
    for (int w : {600, 1000, 2000}) {
      Feed f(COUNT_450_325);
      int still_until = 2500, end = 2750;
      for (int ms = 0; ms <= end; ms += 10) f.at(ms, ms <= still_until ? 0.0 : v * (ms - still_until) / 1000.0);
      CAPTURE(v);
      CAPTURE(w);
      CHECK_FALSE(f.t.in_place(w, FLOOR_IN_S, f.now(end)));
    }
  }
}

TEST_CASE("PathTracker: a thing that was carried a little over the floor for the last half of the window is not in place") {
  for (double v : {2.0, -2.0}) {
    Feed f(COUNT_450_325);
    int still_until = 1000, end = 2000;
    for (int ms = 0; ms <= end; ms += 10) f.at(ms, ms <= still_until ? 0.0 : v * (ms - still_until) / 1000.0);
    CAPTURE(v);
    CHECK_FALSE(f.t.in_place(1000, FLOOR_IN_S, f.now(end)));
  }
}

TEST_CASE("a shove that begins late in the backstop window is not read as a robot going nowhere") {
  // Window exits off and a long velocity exit, so inside big_error only the no-new-low backstop can end the wait while the robot is shoved
  // about. A shove that lands late in the backstop window, on a robot that has sat at its target for most of it, must not be read as a
  // robot going nowhere: either the wait goes on, or the robot really is going slowly when it returns.
  for (std::uint32_t seed = 1; seed <= 8; seed++) {
    Rig r(sim::archetype_light_fast(), 1, true, seed);
    r.chassis.pid_swing_exit_condition_set(90, 0.0, 0, 0.0, 500, 500);
    r.sim.push(-29.5584, 995.511, 117.021);
    r.sim.push(-70.8485, 1522.84, 280.275);
    r.sim.push(100.463, 1546.89, 288.928);
    r.chassis.pid_swing_set(ez::LEFT_SWING, -45_deg, 110, 0);
    double ms = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 2000, &ms));
    CAPTURE(seed);
    double speed = r.angle_speed_over(100);
    if (!r.chassis.interfered) CHECK_MESSAGE(speed < 2.0 * 4.0, "returned clean after " << ms << " ms while turning at " << speed << " deg/s");
  }
}

TEST_CASE("a robot carried at several times the stop speed from before the wait settles still ends, interfered, in bounded time") {
  for (double w : {8.0, 16.0, -16.0}) {
    for (int start : {500, 700, 900}) {
      Rig r(sim::archetype_light_fast(), 1, false, 1);
      r.chassis.pid_turn_exit_condition_set(90, 0.0, 0, 0.0, 500, 500);
      r.sim.carry(0.0, w, start, 60000);
      r.chassis.pid_turn_set(90_deg, 110);
      double ms = 0;
      REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 2000, &ms));
      CAPTURE(w);
      CAPTURE(start);
      CHECK(r.chassis.interfered);
      CHECK_MESSAGE(ms < 10000.0, "returned after " << ms << " ms");
    }
  }
}

TEST_CASE("PathTracker: one noisy sample on a thing creeping under the floor does not make it go somewhere") {
  // 2.3 deg/s against a 4 deg/s floor, still for the first second. The newest reading is 0.2 deg high (a sensor's noise on one pass): by the two
  // end samples of the last 100 ms that is 4.3 deg/s and a position beyond everything before it, but the averages over the two halves of it,
  // which one sample moves a fifth of a half, read 3.1 deg/s.
  Feed f(0.01);
  int end = 2000;
  for (int ms = 0; ms <= end; ms += 10) f.at(ms, ms <= 1000 ? 0.0 : 2.3 * (ms - 1000) / 1000.0);
  f.at(end + 10, 2.3 * (end + 10 - 1000) / 1000.0 + 0.2);
  for (int w : {600, 1000, 1500}) {
    CAPTURE(w);
    CHECK(f.t.in_place(w, 4.0, f.now(end + 10)));
  }
}
