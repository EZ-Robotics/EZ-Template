// pid_wait_quick() on an odom motion writes headingPID once its inner wait returns: the motion's final theta, resolved to the equivalent
// angle nearest the IMU.  A wait follows a new motion set from another task mid-wait, so after a retarget the inner wait returns on the
// new motion's exits and the write is the new motion's own final heading, exactly what waiting on the new motion alone leaves.  What
// must not happen is the replaced motion's final heading landing in headingPID.  headingPID drives heading-hold, relative turns/swings,
// and odom vector targeting, so this is load-bearing.
//
// The original point-to-point motions here end at theta 0, which resolves to 0; the new point-to-point motion ends at theta 45, which resolves to 45
// with the IMU at 0.  raw_pid_odom_ptp_set() sets headingPID to the angle that faces the target point (0 here), not the final theta.
//
// pid_wait_quick() hands its inner wait the replaced motion's point, so the restarted wait checks the new motion against that old point and
// can return partway through the new motion (3.2.2 behavior).  The robot never moves here, so no point is ever passed and the waits end on
// the new motion's exits.  A path retargeted to a point-to-point is a mode pid_wait_until_index_started() doesn't accept: it prints the
// misuse message and returns, and pid_wait_until_index() goes on to wait for the old path's point on the new motion.
#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

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

// Retargets to a point-to-point motion whose final theta (45) disagrees with the original motion's (0).
void retarget_to_ptp() {
  g_chassis->pid_odom_ptp_set({{0.0, 90.0, 45.0}, fwd, 100});
  g_heading_after_retarget = g_chassis->headingPID.target_get();
  g_retargeted = true;
}

void on_delay_ptp_original() {
  ++g_pass;
  if (g_pass == g_retarget_at) retarget_to_ptp();
  // Pin the errors of whichever motion is live, so neither finishes on its own: only their velocity exits can end them.
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
int g_pass1_passes = 0;

// Lands during pid_wait_until_point()'s OWN first pros::delay(10), before it has polled anything.  The new motion's errors are zeroed
// here, so with small_exit_time=0 its exits fire on the first poll of it.
void on_delay_pass1_retarget() {
  ++g_pass1_passes;
  if (g_pass1_retargeted) return;
  g_chassis->pid_odom_ptp_set({{0.0, 90.0, 45.0}, fwd, 100});
  g_chassis->xyPID.compute_error(0.0, 0.0);
  g_chassis->current_a_odomPID.compute_error(0.0, 0.0);
  g_pass1_heading_after_retarget = g_chassis->headingPID.target_get();
  g_pass1_retargeted = true;
}
}  // namespace

TEST_CASE("pid_wait_quick() on point-to-point follows a point-to-point set mid-wait and leaves the new motion's final heading") {
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
  // Room for the new motion's velocity exits, counted from the retarget (see below)
  test_stub::g_clock.delay_calls_until_stop = 300;

  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered, " final_heading=", chassis.headingPID.target_get(),
          " heading_after_retarget=", g_heading_after_retarget);

  REQUIRE(returned);
  REQUIRE(g_retargeted);
  // The inner wait starts over on the new motion: one leading delay (pass 4), then its pinned xy and angle end on their velocity exits,
  // armed by the 1000 ms fallback (speed reads 0) and then 500 ms more, 151 polls.  Without the restart it would end a pass sooner.  Both
  // errors are outside big_error, so the verdict is interfered.
  CHECK(chassis.interfered);
  CHECK(g_pass == g_retarget_at + 1 + 150);
  // headingPID holds the new motion's own final heading, not the replaced motion's
  double new_final = DriveTestAccess::new_turn_target_compute(chassis, 45.0, chassis.drive_angle_get(), shortest);
  double old_final = DriveTestAccess::new_turn_target_compute(chassis, 0.0, chassis.drive_angle_get(), shortest);
  REQUIRE(new_final != old_final);
  CHECK(chassis.headingPID.target_get() == new_final);
}

TEST_CASE("pid_wait_quick() on pure pursuit returns when a point-to-point is set mid-wait, leaving the new motion's final heading") {
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
  bool returned = true;
  std::string printed = test_stub::capture_stdout([&] {
    test_stub::g_clock.on_delay = on_delay_pp_original;
    test_stub::g_clock.delay_calls_until_stop = 60;
    try {
      chassis.pid_wait_quick();
    } catch (test_stub::StopLoop&) {
      returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = nullptr;
  });
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  REQUIRE(returned);
  REQUIRE(g_retargeted);
  // 3.2.2 behavior: pid_wait_until_index_started() starts over after the retarget (one leading delay, pass 4), finds a point-to-point
  // where it only accepts pure pursuit, prints the misuse message and returns.  pid_wait_until_index() then waits on the old path's point
  // with pid_wait_until_point(), which accepts the point-to-point: one more leading delay (pass 5), then the new motion's xy and angle
  // errors (0, nothing computes them here) small-exit after 10 polls, the first on pass 5.  Small exits are clean.
  CHECK(printed.find("Mode needs to be pure pursuit!") != std::string::npos);
  CHECK(g_pass == g_retarget_at + 1 + 1 + 10 - 1);
  CHECK_FALSE(chassis.interfered);
  CHECK(chassis.mode == ez::POINT_TO_POINT);
  // pid_wait_quick()'s closing write resolves odom_target_start's theta, which is the new motion's 45, not the replaced path's
  double new_final = DriveTestAccess::new_turn_target_compute(chassis, 45.0, chassis.drive_angle_get(), shortest);
  CHECK(chassis.headingPID.target_get() == new_final);
}

// pid_wait_until_point() (the inner wait for the POINT_TO_POINT branch) remembers the motion it was called for before its own first
// delay, so a retarget landing in that delay is followed like any other.
TEST_CASE("pid_wait_quick() on point-to-point follows a retarget landing in the inner wait's own first delay") {
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
  g_pass1_passes = 0;
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
  MESSAGE("returned=", returned, " retargeted=", g_pass1_retargeted, " interfered=", chassis.interfered, " final_heading=", chassis.headingPID.target_get(),
          " heading_after_retarget=", g_pass1_heading_after_retarget);

  REQUIRE(returned);
  REQUIRE(g_pass1_retargeted);
  // The inner wait sees the new motion after its first delay, starts over on it (a second leading delay, pass 2), and the new motion's
  // zeroed errors with small_exit_time=0 end it on that first poll, clean.  Without the restart it would return on pass 1.
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass1_passes == 2);
  // headingPID holds the new motion's own final heading, not the replaced motion's
  double new_final = DriveTestAccess::new_turn_target_compute(chassis, 45.0, chassis.drive_angle_get(), shortest);
  double old_final = DriveTestAccess::new_turn_target_compute(chassis, 0.0, chassis.drive_angle_get(), shortest);
  REQUIRE(new_final != old_final);
  CHECK(chassis.headingPID.target_get() == new_final);
}

// Control: a healthy, un-retargeted pid_wait_quick() call sets headingPID to the requested final heading once the motion settles.
TEST_CASE("pid_wait_quick() on point-to-point still sets headingPID to the final theta when not retargeted") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(0, 1.0, 250, 3.0, 500, 750);  // small_exit_time=0: fires on the first in-tolerance pass
  chassis.pid_odom_turn_exit_condition_set(0, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 45.0}, fwd, 100});
  // Both errors at 0, inside small_error: with small_exit_time=0 the exits fire on the first poll
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
