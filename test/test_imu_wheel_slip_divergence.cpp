// check_imu_task()'s rotation gate (maintenance.cpp) reads the two drive sides' encoder deltas
// diverging as "the robot is rotating". A genuine in-place turn is exactly that shape, and so is a
// straight leg where one side's wheel slips against the floor (or a defended wheel) while the robot
// itself never actually turns: the encoders disagree either way, and this gate cannot tell them
// apart. If a healthy, genuinely flat IMU sits through a slip like this, it reads as "rotating but
// unchanged" every pass, the same signal a real frozen sensor produces during a real turn -- this
// test checks whether that combination survives IMU_STUCK_PASSES_THRESHOLD consecutive passes
// without a healthy backup IMU being falsely ejected. Deliberately not changing the gate itself:
// which signal to trust here is a genuine judgment call for the maintainer, not a mechanical fix.
//
// It does not survive: this reproduces a real, confirmed defect (a healthy redundant IMU gets
// ejected after a sustained one-sided wheel slip with no real rotation). Marked should_fail() so it
// documents the gap without failing the suite -- the same treatment this codebase already gives
// another accepted-but-undecided finding elsewhere (see test_n5_stuck_floor.cpp). Not fixed here:
// whether/how to distinguish "wheel slip" from "genuine rotation" using only the two drive
// encoders is a real design question, not a mechanical fix, and belongs to the maintainer.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
void reset_before_construction() { test_stub::reset_all(); }
void mark_calibrated(Drive& chassis) { DriveTestAccess::imu_calibration_complete(chassis) = true; }

// Only the left side's encoder advances -- a genuine wheel slip or a defended wheel spinning in
// place -- while the right side and both IMUs stay exactly where they are: the robot has not
// actually rotated at all, only one encoder disagrees with the other.
void one_side_slips(Drive& c) {
  c.left_motors.front().fake().position += 50;
  // right side untouched, both IMUs untouched: no real rotation happened
  DriveTestAccess::check_imu_task(c);
}
}  // namespace

TEST_CASE("check_imu_task(): one side's wheel slipping (encoders diverge, both IMUs genuinely flat) does not eject a healthy redundant IMU" *
          doctest::should_fail()) {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(500, 100);  // an active motion, matching how this is really invoked

  REQUIRE(chassis.good_imus.size() == 2);

  // Comfortably past IMU_STUCK_PASSES_THRESHOLD (50) -- if the slip is going to cause a false
  // ejection, it happens well inside this many passes.
  for (int pass = 0; pass < 100; pass++) one_side_slips(chassis);

  CHECK(chassis.good_imus.size() == 2);
}
