// Every wait follows a new motion set from another task mid-wait: a second pid_turn_set()/pid_swing_set()/pid_drive_set()/pid_odom_*_set()
// starts the wait over on the new motion, and the wait returns when that motion's own exits fire, with the verdict it earns.  These tests
// script a real second setter call partway through an otherwise healthy, steadily-closing wait (the on_delay-hook idiom) and check the
// wait doesn't end at the retarget, doesn't hang, and returns on the new motion's exit.
//
// The scripts write each PID's error directly, so speed reads 0 (stopped) and the small exit counts as soon as the error is inside
// small_error.  They keep closing the same error ramp after the retarget, so for most cases here the end pass is the same whether the wait
// restarted or not: those cases pin that the wait carries on through the retarget and returns on the small exit, clean.  The pure pursuit
// case and the frozen turn case do tell the two apart, and say how.
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
void (*g_script)(Drive&, int) = nullptr;

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_script(*g_chassis, g_pass);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run(Drive& chassis, void (*script)(Drive&, int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_pass = 0;
  g_script = script;
  script(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}

// The pass a concurrent retarget lands on: early enough that the healthy, steadily-closing error (100+ passes to reach 0) is nowhere
// near exiting yet.
constexpr int RETARGET_AT = 10;
// Polls for a small exit once the error is inside small_error: 30 ms, a strict > at 10 ms per poll.
constexpr int SMALL_POLLS = 4;
// Polls for a big exit once the error is inside big_error: 100 ms, a strict > at 10 ms per poll.
constexpr int BIG_POLLS = 11;
// Polls for a small exit when the test sets its own 90 ms small exit time, a strict > at 10 ms per poll.
constexpr int EXPLICIT_SMALL_POLLS = 10;

// ---- TURN / TURN_TO_POINT (pid_wait(), shared branch) ----
// Inside the turn's small_error (3) from pass 115 on: 60 - 0.5 * 115 = 2.5.
void turn_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
  if (n == RETARGET_AT) c.pid_turn_set(150, 100);  // a real second turn, different target, mid-wait
}
constexpr int TURN_IN_SMALL_AT = 115;

// ---- SWING (pid_wait()) ----
// Inside the swing's small_error (3) from pass 106 on: 45 - 0.4 * 106 = 2.6.
void swing_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.error = e;
  c.swingPID.derivative = e > 0.0 ? -0.4 : 0.0;
  if (n == RETARGET_AT) c.pid_swing_set(ez::RIGHT_SWING, 120, 100);
}
constexpr int SWING_IN_SMALL_AT = 106;

// ---- wait_until_drive(), DRIVE mode ----
// Inside the drive's small_error (1) from pass 127 on: 20 - 0.15 * 127 = 0.95.  The drive sensors never move, so the 12 in checkpoint is
// never passed and the wait ends on the drive's exit (its failsafe).
void drive_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  c.leftPID.error = e;
  c.leftPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.rightPID.error = e;
  c.rightPID.derivative = e > 0.0 ? -0.15 : 0.0;
  if (n == RETARGET_AT) c.pid_drive_set(200, 100);  // a real second drive, different target, mid-wait
}
constexpr int DRIVE_IN_SMALL_AT = 127;
// The drive's big_error is 4 in, so the error is inside it first, from pass 107: 20 - 0.15 * 107 = 3.95.  The big exit gets there before the small one.
constexpr int DRIVE_IN_BIG_AT = 107;

// ---- wait_until_drive(), odom (POINT_TO_POINT) mode ----
void odom_wud_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  c.leftPID.error = e;
  c.leftPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.rightPID.error = e;
  c.rightPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.xyPID.error = e;
  c.xyPID.derivative = e > 0.0 ? -0.15 : 0.0;
  if (n == RETARGET_AT) c.pid_odom_ptp_set({{0.0, 90.0, ANGLE_NOT_SET}, fwd, 100});  // a real second odom motion, mid-wait
}

// ---- pid_wait(), odom POINT_TO_POINT branch (the "final point" loop) ----
void odom_ptp_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  c.xyPID.error = e;
  c.xyPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
  if (n == RETARGET_AT) c.pid_odom_ptp_set({{0.0, 90.0, ANGLE_NOT_SET}, fwd, 100});
}

// ---- pid_wait(), odom PURE_PURSUIT branch (the pre-last-point loop) ----
void odom_pp_healthy_then_retargeted(Drive& c, int n) {
  double e = 7.3 - 0.15 * (n % 3);  // never inside small_error or big_error, never exits on its own
  c.xyPID.error = e;
  c.xyPID.derivative = 0.3;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
  if (n == RETARGET_AT) {
    std::vector<odom> new_path;
    for (int i = 1; i <= 5; i++) new_path.push_back({{5.0, (double)i, ANGLE_NOT_SET}, fwd, 100});
    c.pid_odom_pp_set(new_path);  // a real second pure pursuit path, mid-wait, still not on its last point
  }
}

// ---- TURN / TURN_TO_POINT (wait_until_turn_swing(), via pid_wait_until(angle)) ----
void turn_wait_until_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
  if (n == RETARGET_AT) c.pid_turn_set(150, 100);  // a real second turn, different target, mid-wait
}

// ---- SWING (wait_until_turn_swing(), via pid_wait_until(angle)) ----
void swing_wait_until_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.error = e;
  c.swingPID.derivative = e > 0.0 ? -0.4 : 0.0;
  if (n == RETARGET_AT) c.pid_swing_set(ez::RIGHT_SWING, 120, 100);  // a real second swing, different target, mid-wait
}

// ---- pid_wait_until_point() ----
void point_wait_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  c.xyPID.error = e;
  c.xyPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
  if (n == RETARGET_AT) c.pid_odom_ptp_set({{0.0, 90.0, ANGLE_NOT_SET}, fwd, 100});  // a real second odom motion, mid-wait
}

// ---- A concurrent setter from a DIFFERENT mode: turnPID's error freezes where it was (inside small_error), and the wait moves onto
// the swing.
void turn_frozen_in_tolerance_then_mode_changed(Drive& c, int n) {
  if (n < RETARGET_AT) {
    c.turnPID.error = 60.0 - 0.5 * n;
    c.turnPID.derivative = -0.5;
  } else if (n == RETARGET_AT) {
    c.turnPID.error = 0.5;  // now inside small_error(2.0) -- frozen here, nothing updates it again
    c.turnPID.derivative = 0.0;
    c.pid_swing_set(ez::RIGHT_SWING, 45, 100);  // changes mode away from TURN, but never touches turnPID's own target
  }
  // n > RETARGET_AT deliberately left untouched -- mirrors turn_pid_task() no longer running once mode left TURN.
}
}  // namespace

// Each expected end pass is the first pass inside small_error plus the rest of the small exit's polls.  pid_wait(), wait_until_drive()
// and wait_until_turn_swing() take one more delay after the poll that latches the exit; pid_wait_until_point() and wait_until_drive()'s
// odom xy check return from that poll.

TEST_CASE("pid_wait() TURN: a concurrent pid_turn_set() mid-wait is followed, and the wait returns on the new turn's exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == TURN_IN_SMALL_AT + SMALL_POLLS - 1 + 1);
}

TEST_CASE("pid_wait() SWING: a concurrent pid_swing_set() mid-wait is followed, and the wait returns on the new swing's exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  Outcome o = run(chassis, swing_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == SWING_IN_SMALL_AT + SMALL_POLLS - 1 + 1);
}

TEST_CASE("wait_until_drive() DRIVE: a concurrent pid_drive_set() mid-wait is followed, and the wait returns on the new drive's exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, drive_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == DRIVE_IN_BIG_AT + BIG_POLLS - 1 + 1);
}

TEST_CASE("wait_until_drive() odom: a concurrent pid_odom_ptp_set() mid-wait is followed, and the wait returns on the new motion's xy exit") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
  Outcome o = run(chassis, odom_wud_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == DRIVE_IN_SMALL_AT + EXPLICIT_SMALL_POLLS - 1);
}

TEST_CASE("pid_wait() odom POINT_TO_POINT: a concurrent pid_odom_ptp_set() mid-wait is followed, and the wait returns on the new motion's exits") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
  Outcome o = run(chassis, odom_ptp_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == DRIVE_IN_SMALL_AT + EXPLICIT_SMALL_POLLS - 1 + 1);
}

TEST_CASE("pid_wait() odom PURE_PURSUIT: a concurrent pid_odom_pp_set() mid-wait (before the last point) is followed onto the new path") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  std::vector<odom> path;
  for (int i = 1; i <= 40; i++) path.push_back({{0.0, 7.0 + i, ANGLE_NOT_SET}, fwd, 100});
  chassis.pid_odom_pp_set(path);
  Outcome o = run(chassis, odom_pp_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  // Nothing advances pp_index, so the new path is pinned before its last point too and ends on the mid-path stall check: speed reads 0,
  // the check counts after 1000 ms with no movement and ends the wait once it passes the 500 ms velocity exit time, 151 polls.  It starts
  // over with the wait on the new path, whose first poll comes after the restart's leading delay on pass RETARGET_AT + 1.  Without the
  // restart it would have ended on pass 151.
  CHECK(o.interfered);
  CHECK(o.passes == RETARGET_AT + 151);
}

TEST_CASE("wait_until_turn_swing() TURN: a concurrent pid_turn_set() mid-wait is followed, and the wait returns on the new turn's exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_wait_until_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(45.0); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  // The IMU never moves, so the 45 degree checkpoint is never passed and the wait ends on the turn's exit (its failsafe)
  CHECK(o.passes == TURN_IN_SMALL_AT + SMALL_POLLS - 1 + 1);
}

TEST_CASE("wait_until_turn_swing() SWING: a concurrent pid_swing_set() mid-wait is followed, and the wait returns on the new swing's exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 60, 100);
  Outcome o = run(chassis, swing_wait_until_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(30.0); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == SWING_IN_SMALL_AT + SMALL_POLLS - 1 + 1);
}

// ---- A retarget landing in the wait's first delay.  wait_until_turn_swing() takes no delay before its first poll, so pass 1 is the
// delay after that poll, the earliest a second task can land. ----
constexpr int FIRST_DELAY_RETARGET_AT = 1;

void turn_wait_until_retargeted_in_first_delay(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
  if (n == FIRST_DELAY_RETARGET_AT) c.pid_turn_set(150, 100);
}

void swing_wait_until_retargeted_in_first_delay(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.error = e;
  c.swingPID.derivative = e > 0.0 ? -0.4 : 0.0;
  if (n == FIRST_DELAY_RETARGET_AT) c.pid_swing_set(ez::RIGHT_SWING, 120, 100);
}

// Control: TURN_TO_POINT works out its aim point every pass without touching turnPID's stored target, so an ordinary, un-retargeted
// turn-to-point wait must not be read as retargeted.
void turn_to_point_never_retargeted(Drive&, int) {}

TEST_CASE("wait_until_turn_swing() TURN: a concurrent pid_turn_set() landing in the wait's first delay is followed onto the new turn") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_wait_until_retargeted_in_first_delay, 300, [&] { chassis.pid_wait_until(45.0); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == TURN_IN_SMALL_AT + SMALL_POLLS - 1 + 1);
}

TEST_CASE("wait_until_turn_swing() SWING: a concurrent pid_swing_set() landing in the wait's first delay is followed onto the new swing") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 60, 100);
  Outcome o = run(chassis, swing_wait_until_retargeted_in_first_delay, 300, [&] { chassis.pid_wait_until(30.0); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == SWING_IN_SMALL_AT + SMALL_POLLS - 1 + 1);
}

TEST_CASE("wait_until_turn_swing() TURN_TO_POINT: an ordinary, un-retargeted wait never false-fires interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 110);
  REQUIRE(chassis.mode == TURN_TO_POINT);
  Outcome o = run(chassis, turn_to_point_never_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait_until_point(): a concurrent pid_odom_ptp_set() mid-wait is followed, and the wait returns on the new motion's exits") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
  Outcome o = run(chassis, point_wait_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until_point({0.0, 24.0, ANGLE_NOT_SET}); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == DRIVE_IN_SMALL_AT + EXPLICIT_SMALL_POLLS - 1);
}

// ---- A concurrent setter from a DIFFERENT mode changes `mode` without touching turnPID's target.  The wait follows it onto the swing
// instead of finishing on turnPID's frozen error. ----
TEST_CASE("pid_wait() TURN: a concurrent pid_swing_set() mid-wait is followed onto the swing even though it never retargets turnPID") {
  Drive chassis = make_chassis();
  chassis.pid_turn_exit_condition_set(100, 2.0, 0, 0.0, 0, 0);  // small exit only, ~10 passes to fire once in tolerance
  chassis.pid_turn_set(90, 100);
  chassis.pid_print_toggle(true);  // the exit print names the PID the wait ended on
  Outcome o{};
  std::string printed = test_stub::capture_stdout([&] { o = run(chassis, turn_frozen_in_tolerance_then_mode_changed, 300, [&] { chassis.pid_wait(); }); });
  CHECK(o.returned);
  // swingPID is never computed here, so its error is 0 and the swing small-exits cleanly (90 ms) after the restart's leading delay on pass
  // RETARGET_AT + 1, with one more delay after.  The frozen turn would have small-exited on the same pass (100 ms from pass RETARGET_AT),
  // so the exit print is what shows the wait ended on the swing.
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == RETARGET_AT + 1 + SMALL_POLLS - 1 + 1);
  CHECK(chassis.mode == SWING);
  CHECK(printed.find("  Swing: Small Exit") != std::string::npos);
  CHECK(printed.find("  Turn:") == std::string::npos);
}
