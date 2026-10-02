// The drive samples what "stopped" is measured on (travel.hpp) from the auto task, once per fresh pass. These hold the drive's
// side of that: how often it samples, what starts a fresh history, and what is and is not counted as the robot moving.
#include <cmath>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {
constexpr int LEFT = 0, HEADING = 2, ODOM_XY = 4;  // Drive::Travel
}

TEST_CASE("travel: sampled once per auto task pass, so passes caught up in one tick are one sample and history is by time") {
  for (int passes : {1, 2, 3}) {
    Rig r(sim::archetype_light_fast(), passes);
    r.chassis.pid_drive_set(24_in, 110);
    for (int i = 0; i < 300; i++) pros::delay(10);
    double travel = 0, span = 0;
    CAPTURE(passes);
    // 3 s in: the full 2 s window is there whatever the number of passes per tick (3 passes would keep only 850 ms of history
    // if every pass were a sample)
    REQUIRE(DriveTestAccess::travel(r.chassis, LEFT).travel_over(2000, pros::millis(), travel, span));
    CHECK(span == doctest::Approx(2000.0));
  }
}

TEST_CASE("travel: the tracker agrees with the sim's true path over a window, to within a count at each end") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(24_in, 110);
  for (int i = 0; i < 40; i++) pros::delay(10);  // in the middle of the drive, 0.4 s in; the trace's last sample is the pass the tracker's is
  double travel = 0, span = 0;
  REQUIRE(DriveTestAccess::travel(r.chassis, LEFT).travel_over(250, pros::millis(), travel, span));
  double truth = r.speed_over(&Sample::left, (int)span) * span / 1000.0;
  double count = 1.0 / r.chassis.drive_tick_per_inch();
  CHECK(span == doctest::Approx(250.0));
  CHECK(travel == doctest::Approx(truth).epsilon(0.02));
  CHECK(std::fabs(travel - truth) <= 2.0 * count + 1e-9);
}

TEST_CASE("travel: every new motion starts a fresh history, so its first window never holds the last motion's travel") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_drive_set(24_in, 110);
  for (int i = 0; i < 40; i++) pros::delay(10);  // moving fast
  double travel = 0, span = 0;
  REQUIRE(DriveTestAccess::travel(r.chassis, LEFT).travel_over(100, pros::millis(), travel, span));
  REQUIRE(travel > 1.0);

  // A chained motion: the next setter, with the robot still moving
  r.chassis.pid_drive_set(24_in, 110);
  // Not one pass has run for it: nothing is sampled for it yet, and nothing of the old motion answers for it
  CHECK(DriveTestAccess::travel_stopped(r.chassis, LEFT, 50));  // nothing to veto with: ungated, not "moving"
  pros::delay(10);
  CHECK(DriveTestAccess::travel(r.chassis, LEFT).active());
  CHECK_FALSE(DriveTestAccess::travel(r.chassis, LEFT).travel_over(100, pros::millis(), travel, span));  // not 100 ms of it yet
  for (int i = 0; i < 10; i++) pros::delay(10);
  REQUIRE(DriveTestAccess::travel(r.chassis, LEFT).travel_over(100, pros::millis(), travel, span));
  // Only what the robot did since the new motion began: it is still the same fast robot, so this is not zero, but the history
  // starts after the setter and so cannot hold the old motion's 40 passes
  CHECK(span == doctest::Approx(100.0));
  CHECK(DriveTestAccess::travel(r.chassis, LEFT).travel_over(2000, pros::millis(), travel, span) == false);
}

TEST_CASE("travel: odom xy counts what odom moved, and a pose set is not travel") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
  for (int i = 0; i < 20; i++) pros::delay(10);
  double travel = 0, span = 0;
  REQUIRE(DriveTestAccess::travel(r.chassis, ODOM_XY).travel_over(100, pros::millis(), travel, span));
  CHECK(travel > 0.5);  // the robot is driving

  Rig s(sim::archetype_light_fast(), 1);
  s.chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  for (int i = 0; i < 50; i++) pros::delay(10);
  s.chassis.odom_xyt_set(60_in, -40_in, 0_deg);  // relocalizing a robot that is sitting still
  for (int i = 0; i < 50; i++) pros::delay(10);
  REQUIRE(DriveTestAccess::travel(s.chassis, ODOM_XY).travel_over(300, pros::millis(), travel, span));
  CHECK(travel == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("travel: the drive sides and heading are sampled even with odometry off") {
  Rig r(sim::archetype_light_fast(), 1);
  r.chassis.odom_enable(false);
  r.chassis.pid_drive_set(24_in, 110);
  for (int i = 0; i < 30; i++) pros::delay(10);
  double travel = 0, span = 0;
  REQUIRE(DriveTestAccess::travel(r.chassis, LEFT).travel_over(100, pros::millis(), travel, span));
  CHECK(travel > 0.5);
  CHECK(DriveTestAccess::travel(r.chassis, HEADING).active());
}
