// What must not move when the exit speed gate lands: a robot something is really stopping still reads interfered, on time, and the
// waits a team already relies on (the checks from issue 513) still behave. Shove recovery is test_shove_recovery.cpp, 2550R's pin2
// is in test_exit_gate_verdicts.cpp. Every case here passes on the start commit as well.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

TEST_CASE("controls: a drive pinned outside big_error is interfered within one floored window of the contact") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(48_in, 110);
  r.sim.pin(400, 20000);
  double elapsed = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
  CHECK(r.chassis.interfered);
  // pinned at 400 ms; the velocity exit's 500 ms window (above the 350 ms floor), plus a little for the pass it is noticed on
  CHECK(elapsed <= 400 + 500 + 150);
}

TEST_CASE("controls: a turn or swing pinned from the start is interfered within 1600 ms") {
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_turn_set(90_deg, 90);
    r.sim.pin(0, 20000);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    CHECK(r.chassis.interfered);
    CHECK(elapsed <= 1600);
  }
  {
    Rig r(sim::archetype_light_fast(), 1);
    r.chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 90, 0);
    r.sim.pin(0, 20000);
    double elapsed = 0;
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &elapsed));
    CHECK(r.chassis.interfered);
    CHECK(elapsed <= 1600);
  }
}

TEST_CASE("controls: a drive held back by a steady force is interfered within 5 s") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(48_in, 110);
  r.sim.push(-70.0, 0, 20000);
  double elapsed = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 1000, &elapsed));
  CHECK(r.chassis.interfered);
  CHECK(elapsed <= 5000);
}

TEST_CASE("controls: a 3 s continuous push is interfered no later than the push plus one window") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(48_in, 110);
  r.sim.push(-90.0, 300, 3000);
  double elapsed = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 1000, &elapsed));
  CHECK(r.chassis.interfered);
  CHECK(elapsed <= 300 + 3000 + 500 + 150);
}

TEST_CASE("controls (issue 513): a drive at speed 20 with the shipped exits completes, uninterfered, on light_fast") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(24_in, 20);
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(std::fabs(24.0 - r.trace.back().avg) < 3.0);
}

TEST_CASE("controls (issue 513): pid_wait_until(24) then pid_wait() on a 24 in drive finishes the second wait within 500 ms of the first") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(24_in, 110);
  double first = 0, second = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait_until(24_in); }, 3000, &first));
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 3000, &second));
  CHECK_FALSE(r.chassis.interfered);
  CHECK(second <= 500);
}

TEST_CASE("controls (issue 513): a competition disable mid-wait ends the wait on the next pass with interfered true") {
  Rig r(sim::archetype_light_fast(), 1);
  r.sim.use_real_auto_task(true);
  test_stub::g_competition.autonomous = true;
  r.chassis.pid_drive_set(48_in, 110);
  r.sim.before_pass = [&](int pass) {
    if (pass == 30) test_stub::g_competition.disabled = true;
  };
  double elapsed = 0;
  REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 1000, &elapsed));
  CHECK(r.chassis.interfered);
  CHECK(elapsed <= 30 * 10 + 50);
}
