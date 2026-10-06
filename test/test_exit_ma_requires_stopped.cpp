// An mA exit only ends a wait when the robot is also stopped: zero velocity (the library's floor, measured over the mA window) AND over
// current. A heavy robot accelerating hard draws over current for as long as it accelerates, and the exit used to end the wait at the team's
// mA_timeout wherever the robot happened to be: a 150 degree turn at speed 127 returned at 83 degrees, still turning. A robot that is pinned
// or held reads stopped, so it still ends on mA at mA_timeout (test_exit_gate_disturbance_controls.cpp, test_pto_ma_exit.cpp).
//
// Every wait is capped in sim time, so "no return" is a failure and not a hung suite. Noise off, heavy_slow at 2 and 3 passes per poll (it is
// only stable there).
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Exits {
  const char* name;
  bool defaults;
  int small_t;
  double small_e;
  int big_t;
  double big_e;
  int vel_t;
  int mA_t;
};

const Exits DEFAULT_EXITS = {"defaults", true, 0, 0, 0, 0, 0, 0};
const Exits R2550 = {"2550R", false, 90, 1, 200, 3, 100, 100};

void apply(Drive& c, const Exits& e) {
  if (e.defaults) return;
  c.pid_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_turn_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_swing_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_odom_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_odom_turn_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
}

enum class Kind {
  Turn,
  Point,
  Boomerang,
  Path
};

struct Result {
  bool returned = false;
  bool interfered = false;
  double ms = 0;
  double speed = 0;      // true speed over the last 100 ms, deg/s for a turn, in/s for the rest
  double error = 0;      // true distance (in) or heading error (deg) to where it was sent
  bool hunting = false;  // the sim left the robot chattering about its target (heavy_slow does at the sample rate), which is never stopped
};

Result run(const Exits& e, int passes, Kind kind, double amount, int speed) {
  Rig r(sim::archetype_heavy_slow(), passes, false, 1);
  apply(r.chassis, e);
  Result res;
  std::function<void()> set;
  pose target{0, 0, 0};
  switch (kind) {
    case Kind::Turn:
      r.chassis.pid_turn_set(amount, speed);
      break;
    case Kind::Point:
      target = {12, 24, 0};
      r.chassis.pid_odom_set({{12_in, 24_in}, fwd, speed});
      break;
    case Kind::Boomerang:
      target = {12, 24, 0};
      r.chassis.pid_odom_set({{12_in, 24_in, 45_deg}, fwd, speed});
      break;
    case Kind::Path:
      target = {24, 36, 0};
      r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, speed}, {{12_in, 24_in}, fwd, speed}, {{24_in, 36_in}, fwd, speed}});
      break;
  }
  res.returned = r.wait([&] { r.chassis.pid_wait(); }, 1500, &res.ms);
  res.interfered = r.chassis.interfered;
  res.hunting = r.hunting();
  if (kind == Kind::Turn) {
    res.speed = r.angle_speed_over(100);
    res.error = std::fabs(amount - r.chassis.drive_angle_get());
  } else {
    res.speed = r.drive_speed_over(100);
    res.error = std::hypot(target.x - r.chassis.odom_x_get(), target.y - r.chassis.odom_y_get());
  }
  return res;
}

}  // namespace

TEST_CASE("heavy_slow at speed 127 turning 150 and 180 degrees at the default exits does not return on an mA exit while still turning") {
  for (int passes : {2, 3})
    for (double deg : {150.0, 180.0}) {
      Result r = run(DEFAULT_EXITS, passes, Kind::Turn, deg, 127);
      CAPTURE(passes);
      CAPTURE(deg);
      REQUIRE(r.returned);
      CHECK_MESSAGE(r.error < 3.0, "returned after " << r.ms << " ms " << r.error << " deg short, turning at " << r.speed << " deg/s");
      // A robot the sim leaves chattering about its target is never stopped and is ended by the stuck watch, not by mA
      CHECK_MESSAGE((r.hunting || r.speed < FLOOR_ANGLE + 1.0), "returned after " << r.ms << " ms still turning at " << r.speed << " deg/s");
    }
}

TEST_CASE("heavy_slow at speed 127 at the 2550R exits (90, 1, 200, 3, 100, 100) does not return on an mA exit while still moving") {
  struct Case {
    const char* name;
    Kind kind;
    double amount;
  };
  for (int passes : {2, 3})
    for (Case c : {Case{"turn 90", Kind::Turn, 90}, Case{"turn 180", Kind::Turn, 180}, Case{"point (12, 24)", Kind::Point, 0},
                   Case{"boomerang (12, 24, 45)", Kind::Boomerang, 0}, Case{"path (0,12) (12,24) (24,36)", Kind::Path, 0}}) {
      Result r = run(R2550, passes, c.kind, c.amount, 127);
      CAPTURE(passes);
      CAPTURE(std::string(c.name));
      REQUIRE(r.returned);
      double floor = c.kind == Kind::Turn ? FLOOR_ANGLE : FLOOR_DISTANCE;
      CHECK_MESSAGE(r.error < 3.0, "returned after " << r.ms << " ms " << r.error << " short, moving at " << r.speed);
      CHECK_MESSAGE((r.hunting || r.speed < floor + 1.0), "returned after " << r.ms << " ms still moving at " << r.speed << ", " << r.error << " short");
    }
}
