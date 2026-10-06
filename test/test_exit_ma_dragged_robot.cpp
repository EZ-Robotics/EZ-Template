// An mA exit now waits for the robot to be stopped, so a robot that is moving while its motors are over current (dragged away from its
// target by something else, its drive stalled) is no longer ended by mA. The stuck watch has to end it, and does: it makes no progress
// toward the target. A robot that is held still while it draws over current is stopped and still ends on mA at mA_timeout.
//
// Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

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

void apply(Drive& c, const Exits& e) {
  if (e.defaults) return;
  c.pid_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_turn_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_odom_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_odom_turn_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
}

const Exits EXITS[] = {{"defaults", true, 0, 0, 0, 0, 0, 0}, {"2550R", false, 90, 1, 200, 3, 100, 100}, {"mA only", false, 250, 1, 250, 3, 0, 300}};

}  // namespace

TEST_CASE("a robot dragged away from its drive target while its motors stall returns interfered, not stuck for ever") {
  for (auto& e : EXITS)
    for (bool heavy : {false, true}) {
      Rig r(heavy ? sim::archetype_heavy_slow() : sim::archetype_light_fast(), heavy ? 2 : 1, false, 1);
      apply(r.chassis, e);
      r.sim.carry(-15.0, 0.0, 300, 20000);
      r.chassis.pid_drive_set(24_in, 127);
      double ms = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait(); }, 1500, &ms);
      CAPTURE(std::string(e.name));
      CAPTURE(heavy);
      REQUIRE(ok);
      CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms");
      CHECK_MESSAGE(ms < 5000.0, "returned after " << ms << " ms");
    }
}

TEST_CASE("a robot dragged away from its odom point while its motors stall returns interfered, not stuck for ever") {
  for (auto& e : EXITS)
    for (bool heavy : {false, true}) {
      Rig r(heavy ? sim::archetype_heavy_slow() : sim::archetype_light_fast(), heavy ? 2 : 1, false, 1);
      apply(r.chassis, e);
      r.sim.carry(-15.0, 0.0, 300, 20000);
      r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 127});
      double ms = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait(); }, 1500, &ms);
      CAPTURE(std::string(e.name));
      CAPTURE(heavy);
      REQUIRE(ok);
      CHECK_MESSAGE(r.chassis.interfered, "returned clean after " << ms << " ms");
      CHECK_MESSAGE(ms < 5000.0, "returned after " << ms << " ms");
    }
}

TEST_CASE("a robot held still while its motors stall still ends on mA at mA_timeout") {
  for (bool heavy : {false, true}) {
    Rig r(heavy ? sim::archetype_heavy_slow() : sim::archetype_light_fast(), heavy ? 2 : 1, false, 1);
    r.chassis.pid_drive_exit_condition_set(90, 1, 200, 3, 0, 100);
    r.sim.pin(150, 20000);
    r.chassis.pid_drive_set(24_in, 127);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, 1500, &ms);
    CAPTURE(heavy);
    REQUIRE(ok);
    CHECK(r.chassis.interfered);
    // 150 ms to contact, the mA timeout, and the window it has to read stopped over, which is the same length
    CHECK_MESSAGE(ms < 150 + 100 + 100 + 150, "returned after " << ms << " ms");
  }
}
