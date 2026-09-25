// pid_wait_until_point() calls its own leading pros::delay(10) BEFORE taking any mode/target
// snapshot to compare against -- unlike wait_until_turn_swing_internal() and
// pid_wait_until_index_started(), which both snapshot first, specifically to guard against this (see
// their own comments). A concurrent motion setter call from another task landing during that first
// ~10ms is invisible to the retarget guard a few lines later: by the time mode_snapshot/
// retarget_target are read, they already reflect whatever the concurrent call just changed, not the
// motion this wait was actually started for.
//
// See test_wait_retarget_before_first_delay.cpp (audit/wait-exit-round5-timing) for the identical bug
// in pid_wait()/wait_until_drive() -- not fixed here, only pid_wait_until_point() is (see that file's
// own header for why the other two are a separate fix).
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_retarget_at = 1;
void (*g_retarget)(Drive&) = nullptr;

// Every pass but the retargeting one holds the ORIGINAL motion's errors comfortably away from
// exiting, so the only way a pre-fix run can end is by having silently adopted whatever the
// concurrent call changed things to.
void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  if (g_pass == g_retarget_at) {
    g_retarget(c);
    return;
  }
  c.xyPID.error = 10.0;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}

// A real, concurrent retarget from a DIFFERENT mode -- doesn't touch xyPID/current_a_odomPID's
// targets at all, only `mode`.
void retarget_cross_mode(Drive& c) { c.pid_turn_set(30, 100); }

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  e_mode mode;
};

Outcome run_hijack(Drive& chassis, pose target, int retarget_at, void (*retarget)(Drive&)) {
  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = retarget_at;
  g_retarget = retarget;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 200;
  Outcome o{true, 0, false, chassis.drive_mode_get()};
  try {
    chassis.pid_wait_until_point(target);
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.mode = chassis.drive_mode_get();
  return o;
}

// Same-family case: the retarget lands on another point-to-point motion, so `mode` itself never
// changes -- only odom_target_start does. Baited exactly like
// test_wait_quick_headingpid_no_clobber.cpp's pass-1 case (small_exit_time=0 plus zeroed errors on
// the NEW motion, so its own exit conditions read as already satisfied the instant they're first
// checked) so a pre-fix run's outcome is deterministic, not timing- or geometry-dependent.
void retarget_same_family(Drive& c) {
  c.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: fires the instant it's checked
  c.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  c.pid_odom_ptp_set({{0.0, 90.0, 0.0}, fwd, 100});  // a real, different point-to-point motion
  c.xyPID.error = 0.0;
  c.xyPID.derivative = 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}
}  // namespace

TEST_CASE("pid_wait_until_point(): a cross-mode retarget landing during the leading settle delay is caught, not silently absorbed") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  REQUIRE(chassis.mode == POINT_TO_POINT);

  Outcome o = run_hijack(chassis, {0.0, 24.0, 0.0}, /*retarget_at=*/1, retarget_cross_mode);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " mode=", (int)o.mode);

  REQUIRE(o.returned);
  // Bug: pid_wait_until_point() takes its mode/target snapshot AFTER its own leading pros::delay(10),
  // so a concurrent pid_turn_set() landing during that delay is read as though it had always been this
  // call's own starting mode -- except it isn't: by the time the "Mode needs to be an odom mode" check
  // runs, mode is already TURN, so this returns silently, with interfered left false and the original
  // POINT_TO_POINT motion abandoned, untracked.
  CHECK(o.interfered);
  CHECK(o.mode == ez::TURN);
}

TEST_CASE("pid_wait_until_point(): control -- the identical cross-mode retarget landing one pass later, after the snapshot, IS caught") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});

  Outcome o = run_hijack(chassis, {0.0, 24.0, 0.0}, /*retarget_at=*/3, retarget_cross_mode);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " mode=", (int)o.mode);

  REQUIRE(o.returned);
  // The existing mid-loop guard already catches this one -- isolates timing as the only variable
  // between this file's two cross-mode outcomes.
  CHECK(o.interfered);
}

TEST_CASE("pid_wait_until_point(): a same-family retarget landing during the leading settle delay is caught, not silently absorbed") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  REQUIRE(chassis.mode == POINT_TO_POINT);

  Outcome o = run_hijack(chassis, {0.0, 24.0, 0.0}, /*retarget_at=*/1, retarget_same_family);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  REQUIRE(o.returned);
  // Bug: pid_wait_until_point()'s mode/target snapshot is taken AFTER the concurrent pid_odom_ptp_set()
  // already landed, so retarget_target already equals the NEW odom_target_start -- the guard inside the
  // loop compares the new target against itself for the rest of the call and never fires. Meanwhile the
  // NEW motion's own (zeroed, small_exit_time=0) exit conditions read as satisfied the instant they're
  // first checked, so this returns a clean, uninterfered "success" for a motion this call was never
  // actually waiting for.
  CHECK(o.interfered);
}

// What could go wrong with moving the snapshot earlier: it could misread an ordinary, un-retargeted
// call as though something had retargeted it, if anything internal happens to touch `mode`/
// odom_target_start during the call's own first iteration. Covers all three ways this function is
// reached.
TEST_CASE("pid_wait_until_point(): an ordinary, un-retargeted call still returns cleanly, not falsely interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: fires on the first in-tolerance pass
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  chassis.xyPID.error = 0.0;
  chassis.xyPID.derivative = 0.0;
  chassis.current_a_odomPID.error = 0.0;
  chassis.current_a_odomPID.derivative = 0.0;

  test_stub::g_clock.on_delay = nullptr;
  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait_until_point({0.0, 24.0, 0.0});
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_quick() on point-to-point: an ordinary, un-retargeted call still returns cleanly, not falsely interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 45.0}, fwd, 100});
  chassis.xyPID.error = 0.0;
  chassis.xyPID.derivative = 0.0;
  chassis.current_a_odomPID.error = 0.0;
  chassis.current_a_odomPID.derivative = 0.0;

  test_stub::g_clock.on_delay = nullptr;
  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until_index(): an ordinary, un-retargeted call still returns cleanly, not falsely interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  std::vector<odom> path;
  for (int i = 1; i <= 10; i++) path.push_back({{0.0, 7.0 + i, ANGLE_NOT_SET}, fwd, 100});
  chassis.pid_odom_pp_set(path);

  test_stub::g_clock.on_delay = nullptr;
  test_stub::g_clock.delay_calls_until_stop = 400;
  bool returned = true;
  try {
    chassis.pid_wait_until_index(3);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}
