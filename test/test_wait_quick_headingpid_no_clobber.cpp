// pid_wait_quick() (unlike pid_wait(), see test_headingpid_no_clobber.cpp) had no check that its
// own motion was still the current one before writing headingPID once its inner wait returned.
// The inner wait (pid_wait_until_point() / pid_wait_until_index()) already ends early with
// interfered=true when it notices a concurrent retarget, but that alone does not stop
// pid_wait_quick()'s own write below it from running on whatever odom_target_start now holds --
// a stale, already-abandoned pid_wait_quick() call could clobber headingPID with the hijacking
// motion's own in-flight heading, corrupting shared state that motion is already relying on.
// headingPID drives heading-hold, relative turns/swings, and odom vector targeting, so this is
// load-bearing.
//
// raw_pid_odom_ptp_set() sets headingPID to the angle that FACES the target point, not the
// target's own final theta -- a value pid_wait_quick()'s write (new_turn_target_compute of
// odom_target_start.theta) cannot coincidentally reproduce, so any clobber is caught reliably.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

std::vector<odom> straight_path(int points, double start_y) {
  std::vector<odom> path;
  for (int i = 1; i <= points; i++) path.push_back({{0.0, start_y + i, ANGLE_NOT_SET}, fwd, 110});
  return path;
}

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_retarget_at = -1;
bool g_retargeted = false;
double g_heading_after_retarget = 0.0;

// Retargets to a point-to-point motion whose final theta (45) disagrees with the angle that
// faces the point from the origin (~0) -- pid_odom_ptp_set() itself sets headingPID to face the
// point as part of legitimately starting this new motion; that is the hijacking task's own
// in-flight state the stale pid_wait_quick() call must not then stomp.
void retarget_to_ptp() {
  g_chassis->pid_odom_ptp_set({{0.0, 90.0, 45.0}, fwd, 100});
  g_heading_after_retarget = g_chassis->headingPID.target_get();
  g_retargeted = true;
}

void on_delay_ptp_original() {
  ++g_pass;
  if (g_pass == g_retarget_at) retarget_to_ptp();
  // Pin the ORIGINAL motion's errors so it can never finish on its own before the retarget lands.
  g_chassis->xyPID.error = 10.0;
  g_chassis->xyPID.derivative = 0.0;
  g_chassis->current_a_odomPID.error = 10.0;
  g_chassis->current_a_odomPID.derivative = 0.0;
}

void on_delay_pp_original() {
  ++g_pass;
  if (g_pass == g_retarget_at) retarget_to_ptp();
  // The original path never advances pp_index -- "stuck" from this wait's point of view.
}

bool g_pass1_retargeted = false;
double g_pass1_heading_after_retarget = 0.0;

// Lands during pid_wait_until_point()'s OWN first pros::delay(10) -- before it has taken any baseline
// of its own. The new motion's errors are zeroed here too, so its exit conditions read as already
// satisfied the moment the inner wait's post-delay snapshot and first check run.
// A real compute_error() call, not a direct `.error =` write -- small_exit_time=0 on the new motion
// only needs ONE real compute to fire the instant it's checked (see PID.cpp's freshness gate).
void on_delay_pass1_retarget() {
  if (g_pass1_retargeted) return;
  g_chassis->pid_odom_ptp_set({{0.0, 90.0, 45.0}, fwd, 100});
  g_chassis->xyPID.compute_error(0.0, 0.0);
  g_chassis->current_a_odomPID.compute_error(0.0, 0.0);
  g_pass1_heading_after_retarget = g_chassis->headingPID.target_get();
  g_pass1_retargeted = true;
}
}  // namespace

TEST_CASE("pid_wait_quick() on point-to-point does not clobber headingPID when retargeted mid-wait") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = 3;
  g_retargeted = false;
  g_heading_after_retarget = 0.0;
  test_stub::g_clock.on_delay = on_delay_ptp_original;
  test_stub::g_clock.delay_calls_until_stop = 60;

  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered,
          " final_heading=", chassis.headingPID.target_get(), " heading_after_retarget=", g_heading_after_retarget);

  REQUIRE(returned);
  REQUIRE(g_retargeted);
  CHECK(chassis.interfered);
  CHECK(chassis.headingPID.target_get() == g_heading_after_retarget);
}

TEST_CASE("pid_wait_quick() on pure pursuit does not clobber headingPID when retargeted mid-wait") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_pp_set(straight_path(40, 7.0));

  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = 3;
  g_retargeted = false;
  g_heading_after_retarget = 0.0;
  test_stub::g_clock.on_delay = on_delay_pp_original;
  test_stub::g_clock.delay_calls_until_stop = 60;

  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  REQUIRE(returned);
  REQUIRE(g_retargeted);
  CHECK(chassis.interfered);
  CHECK(chassis.headingPID.target_get() == g_heading_after_retarget);
}

// pid_wait_until_point() (the inner wait for the POINT_TO_POINT branch) now also takes its own
// retarget baseline before its own first settle delay (see test_wait_until_point_retarget_before_
// first_delay.cpp), so this specific retarget is caught twice over -- by the inner wait itself, and by
// pid_wait_quick()'s own outer guard below (snapshotted before the inner call, before even its delay).
// This test is kept as a regression check on the outer guard specifically: baited so the new motion's
// own exit conditions are already satisfied the instant the inner wait's baseline is taken, so even an
// inner wait blind to the retarget would return "cleanly" almost immediately instead of hanging --
// exactly the shape that would read as an innocuous success if the outer guard alone didn't catch it.
TEST_CASE("pid_wait_quick() on point-to-point catches a retarget landing in the inner wait's own first settle delay") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0 on the NEW motion: fires the instant it's checked
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 100});
  chassis.xyPID.error = 10.0;  // the ORIGINAL motion is nowhere near done when the retarget lands
  chassis.current_a_odomPID.error = 10.0;

  g_chassis = &chassis;
  g_pass1_retargeted = false;
  g_pass1_heading_after_retarget = 0.0;
  test_stub::g_clock.on_delay = on_delay_pass1_retarget;
  test_stub::g_clock.delay_calls_until_stop = 30;

  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " retargeted=", g_pass1_retargeted, " interfered=", chassis.interfered,
          " final_heading=", chassis.headingPID.target_get(), " heading_after_retarget=", g_pass1_heading_after_retarget);

  REQUIRE(returned);
  REQUIRE(g_pass1_retargeted);
  CHECK(chassis.interfered);
  CHECK(chassis.headingPID.target_get() == g_pass1_heading_after_retarget);
}

// What could go wrong with the fix: it could skip the legitimate heading write on a healthy,
// un-retargeted pid_wait_quick() call, leaving headingPID stale instead of facing the requested
// final heading once the motion genuinely settles.
TEST_CASE("pid_wait_quick() on point-to-point still sets headingPID to the final theta when not retargeted") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: fires on the first in-tolerance pass
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 45.0}, fwd, 100});
  // A real compute_error() call, not a direct `.error =` write -- small_exit_time=0 only needs ONE
  // real compute to fire on the wait's first check (see PID.cpp's freshness gate).
  chassis.xyPID.compute_error(0.0, 0.0);
  chassis.current_a_odomPID.compute_error(0.0, 0.0);

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
  double expected = DriveTestAccess::new_turn_target_compute(chassis, 45.0, chassis.drive_angle_get(), shortest);
  CHECK(chassis.headingPID.target_get() == expected);
}
