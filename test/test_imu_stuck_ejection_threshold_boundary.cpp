// Coverage gap in maintenance.cpp's check_imu_task(): the ejection check is
// `imu_stuck_passes[port] >= IMU_STUCK_PASSES_THRESHOLD` inside the `is_bad` lambda.
// IMU_STUCK_PASSES_THRESHOLD is 50, and imu_stuck_passes[port] increments by exactly 1 per
// consecutive rotating-but-unchanged pass (confirmed by reading the pass-update loop just
// above `is_bad`), so the 50th consecutive stuck pass is the exact tick where an unhealthy
// IMU is first considered bad and ejected.
//
// No existing test pinned that exact tick. test_imu_straight_line_false_eject.cpp's
// passes_until_ejected() tests assert `passes <= 51`, which is deliberately loose ("a couple
// of passes of slack is kept for the assertion, not the loop" per that file's own comment)
// and does not distinguish ejection landing on pass 50 from landing on pass 51. That slack is
// exactly wide enough to hide `>=` becoming `>`: with `>`, the 50th pass leaves
// imu_stuck_passes[port] == 50, and 50 > 50 is false, so ejection doesn't happen until pass
// 51 -- still inside the existing tests' `<= 51` bound, so they keep passing either way.
//
// This file pins the boundary directly: not yet ejected after 49 consecutive stuck passes,
// ejected on the 50th. That distinguishes `>=` from `>` at the exact tick, independent of any
// slack margin.
#include <algorithm>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
void reset_before_construction() { test_stub::reset_all(); }
void mark_calibrated(Drive& chassis) { DriveTestAccess::imu_calibration_complete(chassis) = true; }

// Same shape as test_imu_straight_line_false_eject.cpp's turn_one_pass(): a genuine in-place
// turn (encoders diverge) with only the "healthy" IMU's rotation advancing, so the other IMU
// reads as rotating-but-unchanged every pass. Calls check_imu_task() directly, not through
// ez_auto_task(), since this is about the ejection tick itself, not the auton/opcontrol gate.
void turn_one_pass(Drive& c, pros::Imu* healthy) {
  c.left_motors.front().fake().position += 50;
  c.right_motors.front().fake().position -= 50;
  healthy->fake_rotation += 2.0;
  DriveTestAccess::check_imu_task(c);
}
}  // namespace

TEST_CASE("check_imu_task(): a frozen IMU during a turn is not yet ejected after 49 consecutive stuck passes, but is ejected on the 50th") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360, 1.0);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);

  pros::Imu* frozen = chassis.good_imus[0];   // port 5, never updated below -- the broken sensor
  pros::Imu* healthy = chassis.good_imus[1];  // port 6, updated every pass -- turning for real

  REQUIRE(chassis.good_imus.size() == 2);

  for (int pass = 0; pass < 49; pass++) turn_one_pass(chassis, healthy);
  CHECK(chassis.good_imus.size() == 2);  // 49 consecutive stuck passes: threshold (50) not yet reached

  turn_one_pass(chassis, healthy);  // the 50th consecutive stuck pass
  CHECK(chassis.good_imus.size() == 1);  // ejected on exactly this pass, not one later
  CHECK(std::find(chassis.good_imus.begin(), chassis.good_imus.end(), frozen) == chassis.good_imus.end());
  CHECK(chassis.good_imus.front() == healthy);
}
