// check_imu_task() judges a redundant IMU "stuck" by watching for translation (the drive
// physically moving) while that IMU's heading reading stays frozen. But an ordinary straight
// leg IS translation with genuinely no rotation -- a healthy IMU correctly reports a flat
// heading the whole time, indistinguishable by that check from a real frozen sensor. The fix
// must gate suspicion on the drive actually ROTATING (encoders disagreeing), using the same
// drive_sensor_left()/right() the function already reads, not just moving.
//
// The bug is mode-agnostic: check_imu_task() runs unconditionally from ez_auto_task() every
// pass, with no look at pros::competition::is_autonomous(). Tests (a) and (b) below drive it
// through the real ez_auto_task() path (see test_competition_transition_stop.cpp for the same
// pattern) with the fake competition status actually set to autonomous / enabled-not-autonomous,
// rather than only varying `mode`, so the AUTON-vs-DRIVER-CONTROL distinction the finding calls
// out is exercised for real, not just implied by the mechanism.
#include <algorithm>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// Drive has no usable copy/move constructor (it owns a mutex), so it can't be built by a
// factory that returns it by value -- construct it directly in each TEST_CASE and use this
// only to reset the fake-hardware registries beforehand and flip on calibration afterward.
void reset_before_construction() { test_stub::reset_all(); }
void mark_calibrated(Drive& chassis) { DriveTestAccess::imu_calibration_complete(chassis) = true; }

void set_field(bool disabled, bool autonomous) {
  test_stub::g_competition.disabled = disabled;
  test_stub::g_competition.autonomous = autonomous;
}

// One real pass of ez_auto_task() -- same pattern as test_competition_transition_stop.cpp --
// so check_imu_task() is exercised through its actual caller, under real fake competition state,
// not called directly in isolation.
void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}

// Advances both sides' fake encoders forward together by the same amount -- a straight leg --
// then runs one real ez_auto_task() pass. Neither IMU's fake_rotation is touched: on a real
// straight leg a healthy IMU's heading genuinely does not change, so leaving both flat is the
// honest simulation, not a shortcut.
void drive_straight_one_pass(Drive& c) {
  c.left_motors.front().fake().position += 50;   // ~1.02in at this chassis's tick/inch
  c.right_motors.front().fake().position += 50;
  run_one_auto_task_pass(c);
}

// Advances the fake encoders apart -- a genuine in-place turn -- and advances the "healthy"
// IMU's rotation to match, the way a real IMU would while the robot actually turns. Calls
// check_imu_task() directly (not through ez_auto_task()): these turn-ejection tests aren't
// about the auton/opcontrol distinction, just the rotation gate itself.
void turn_one_pass(Drive& c, pros::Imu* healthy) {
  c.left_motors.front().fake().position += 50;
  c.right_motors.front().fake().position -= 50;
  healthy->fake_rotation += 2.0;
  DriveTestAccess::check_imu_task(c);
}

// Same, but the ASYMMETRIC way -- only the left side moves, the right sits dead still -- the
// shape of a swing turn, not a pivot where both sides move oppositely. The fix reads the
// *difference* between the two side-deltas; an implementation that instead required BOTH sides
// to move (or move above threshold individually) would wrongly treat this as stationary.
void swing_turn_one_pass(Drive& c, pros::Imu* healthy) {
  c.left_motors.front().fake().position += 50;
  // right side untouched: a swing turn's non-driving side doesn't move at all
  healthy->fake_rotation += 2.0;
  DriveTestAccess::check_imu_task(c);
}

// Runs turn_one_pass() until good_imus shrinks (the frozen IMU gets ejected) or the pass cap is
// hit, returning how many calls it took. IMU_STUCK_PASSES_THRESHOLD (maintenance.cpp) is 50, and
// the very first rotating pass already counts (prev_imu_values starts at the frozen IMU's own
// resting value of 0.0, matching its first "unchanged" reading), so ejection is expected on
// exactly the 50th call -- a couple of passes of slack is kept for the assertion, not the loop.
int passes_until_ejected(Drive& c, pros::Imu* healthy, void (*one_pass)(Drive&, pros::Imu*), int cap = 500) {
  int passes = 0;
  while (passes < cap && c.good_imus.size() == 2) {
    one_pass(c, healthy);
    passes++;
  }
  return passes;
}
}  // namespace

TEST_CASE("check_imu_task(): a long straight-line drive in AUTON mode does not eject a healthy redundant IMU") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);

  set_field(/*disabled=*/false, /*autonomous=*/true);
  run_one_auto_task_pass(chassis);  // syncs last_was_autonomous, matching real startup
  chassis.pid_drive_set(500, 100);  // an active auton drive motion, matching how this is really invoked

  REQUIRE(chassis.good_imus.size() == 2);
  for (int pass = 0; pass < 500; pass++) drive_straight_one_pass(chassis);

  CHECK(chassis.good_imus.size() == 2);
}

TEST_CASE("check_imu_task(): the same straight-line scenario in DRIVER CONTROL (opcontrol) mode does not eject a healthy redundant IMU") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);

  // Enabled, never autonomous -- real driver control field state -- and no pid_*_set call at
  // all, so `mode` stays DISABLE too: a driver steering the robot straight by joystick with no
  // auton motion active. check_imu_task() runs regardless, through the real ez_auto_task() pass.
  set_field(/*disabled=*/false, /*autonomous=*/false);
  REQUIRE(chassis.mode == ez::DISABLE);
  REQUIRE(chassis.good_imus.size() == 2);

  for (int pass = 0; pass < 500; pass++) drive_straight_one_pass(chassis);

  CHECK(chassis.good_imus.size() == 2);
  CHECK(chassis.mode == ez::DISABLE);  // still never entered an auton motion
}

TEST_CASE("check_imu_task(): a genuinely frozen IMU during an actual turn is still ejected, within a reasonable bound") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);

  pros::Imu* frozen = chassis.good_imus[0];   // port 5, never updated below -- the broken sensor
  pros::Imu* healthy = chassis.good_imus[1];  // port 6, updated every pass -- turning for real

  REQUIRE(chassis.good_imus.size() == 2);
  int passes = passes_until_ejected(chassis, healthy, turn_one_pass);

  CHECK(chassis.good_imus.size() == 1);
  CHECK(std::find(chassis.good_imus.begin(), chassis.good_imus.end(), frozen) == chassis.good_imus.end());
  CHECK(chassis.good_imus.front() == healthy);
  // IMU_STUCK_PASSES_THRESHOLD is 50; ejection should land on essentially that pass, not
  // somewhere unbounded later in the 500-pass cap.
  CHECK(passes <= 51);
}

// Regression guard for the fix itself: gating on the two sides *diverging* (not on both moving)
// must still catch a frozen IMU during an asymmetric swing turn, where only one side's encoder
// ever moves.
TEST_CASE("check_imu_task(): a genuinely frozen IMU during a swing turn (only one side moving) is still ejected, within a reasonable bound") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);

  pros::Imu* frozen = chassis.good_imus[0];   // port 5, never updated below -- the broken sensor
  pros::Imu* healthy = chassis.good_imus[1];  // port 6, updated every pass -- turning for real

  REQUIRE(chassis.good_imus.size() == 2);
  int passes = passes_until_ejected(chassis, healthy, swing_turn_one_pass);

  CHECK(chassis.good_imus.size() == 1);
  CHECK(std::find(chassis.good_imus.begin(), chassis.good_imus.end(), frozen) == chassis.good_imus.end());
  CHECK(chassis.good_imus.front() == healthy);
  CHECK(passes <= 51);
}

// What could go wrong with this fix: narrowing detection from "moved" to "rotating" means a
// frozen IMU now survives a straight leg of ANY length (that's the point of the fix), so it can
// be carried, still marked good, into whatever motion follows. This is the concrete version of
// that tradeoff: a frozen primary IMU is not caught during a long straight leg, but once a real
// turn actually starts afterward, it is still caught within the same bound as a turn that was
// frozen from the start -- the straight leg costs no extra passes once rotation begins.
TEST_CASE("check_imu_task(): a frozen primary IMU that survives a straight leg is still caught once a real turn starts") {
  reset_before_construction();
  Drive chassis({1, -2}, {-3, 4}, {5, 6}, 3.25, 360);
  mark_calibrated(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(500, 100);

  pros::Imu* frozen = chassis.good_imus[0];   // port 5, never updated -- the broken sensor
  pros::Imu* healthy = chassis.good_imus[1];  // port 6, updated only once turning starts below

  REQUIRE(chassis.good_imus.size() == 2);
  for (int pass = 0; pass < 200; pass++) drive_straight_one_pass(chassis);
  CHECK(chassis.good_imus.size() == 2);  // survived the whole straight leg, frozen the entire time

  chassis.pid_turn_set(90, 100);
  int turn_passes = passes_until_ejected(chassis, healthy, turn_one_pass);

  CHECK(chassis.good_imus.size() == 1);
  CHECK(std::find(chassis.good_imus.begin(), chassis.good_imus.end(), frozen) == chassis.good_imus.end());
  CHECK(chassis.good_imus.front() == healthy);
  CHECK(turn_passes <= 51);  // no slower than a turn that had been frozen from the start
}
