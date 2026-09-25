// pid_wait()'s DRIVE branch already snapshots leftPID/rightPID's target at wait start and returns
// interfered=true the instant it notices a second task retargeting them mid-wait (a concurrent
// pid_drive_set() from another task) instead of silently finishing on whatever motion happens to be
// live when its exit condition next fires. Every sibling wait shares the exact same hazard -- none of
// them lock leftPID/rightPID/turnPID/swingPID/xyPID across their own pros::delay() -- but only the
// DRIVE branch had the guard. These tests script a real second pid_turn_set()/pid_swing_set()/
// pid_drive_set()/pid_odom_*_set() call partway through an otherwise healthy, steadily-closing wait
// (the same on_delay-hook idiom test_jc1_non_odom_stuck.cpp uses) and check the wait ends early with
// interfered=true instead of continuing to poll -- and eventually cleanly succeed on -- the new motion.
#include <functional>

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

// The pass a concurrent retarget lands on -- early enough that a healthy, steadily-closing error
// (chosen so it takes 100+ passes to reach 0 on its own) is nowhere near exiting yet.
constexpr int RETARGET_AT = 10;
// How many extra passes the guard is allowed to notice the retarget and return -- generous relative
// to a single pros::delay() cycle, tight relative to the 100+ passes a silent, uninterrupted finish
// would take.
constexpr int GUARD_SLACK = 5;

// ---- TURN / TURN_TO_POINT (pid_wait(), shared branch) ----
void turn_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
  if (n == RETARGET_AT) c.pid_turn_set(150, 100);  // a real second turn, different target, mid-wait
}

// ---- SWING (pid_wait()) ----
void swing_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.error = e;
  c.swingPID.derivative = e > 0.0 ? -0.4 : 0.0;
  if (n == RETARGET_AT) c.pid_swing_set(ez::RIGHT_SWING, 120, 100);
}

// ---- wait_until_drive(), DRIVE mode ----
void drive_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  c.leftPID.error = e;
  c.leftPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.rightPID.error = e;
  c.rightPID.derivative = e > 0.0 ? -0.15 : 0.0;
  if (n == RETARGET_AT) c.pid_drive_set(200, 100);  // a real second drive, different target, mid-wait
}

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
  double e = 7.3 - 0.15 * (n % 3);  // same shape as test_pp_wait_stuck.cpp's cruising(), never exits on its own
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

// ---- TURN / TURN_TO_POINT (wait_until_turn_swing_internal(), via pid_wait_until(angle)) ----
void turn_wait_until_healthy_then_retargeted(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
  if (n == RETARGET_AT) c.pid_turn_set(150, 100);  // a real second turn, different target, mid-wait
}

// ---- SWING (wait_until_turn_swing_internal(), via pid_wait_until(angle)) ----
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

// ---- Fix #2 regression: a concurrent retarget from a DIFFERENT mode doesn't touch this wait's own
// PID target at all, so a target-only guard misses it entirely -- turnPID.error freezes at whatever
// it was, and if that frozen value happens to land inside small_error, the wait would eventually
// report a clean SMALL_EXIT (interfered=false) for a motion nothing is running anymore.
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

TEST_CASE("pid_wait() TURN: a concurrent pid_turn_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("pid_wait() SWING: a concurrent pid_swing_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  Outcome o = run(chassis, swing_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("wait_until_drive() DRIVE: a concurrent pid_drive_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, drive_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("wait_until_drive() odom: a concurrent pid_odom_ptp_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
  Outcome o = run(chassis, odom_wud_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("pid_wait() odom POINT_TO_POINT: a concurrent pid_odom_ptp_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
  Outcome o = run(chassis, odom_ptp_healthy_then_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("pid_wait() odom PURE_PURSUIT: a concurrent pid_odom_pp_set() mid-wait (before the last point) ends the wait instead of continuing on the new path") {
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
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

// ---- wait_until_turn_swing_internal() had ZERO guard prior to this fix -- unlike every other wait,
// including its own sibling pid_wait() TURN/SWING branches above. ----
TEST_CASE("wait_until_turn_swing_internal() TURN: a concurrent pid_turn_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_wait_until_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(45.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("wait_until_turn_swing_internal() SWING: a concurrent pid_swing_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 60, 100);
  Outcome o = run(chassis, swing_wait_until_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until(30.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

// ---- Round-4 regression: wait_until_turn_swing_internal() now runs one pros::delay() (to seed
// SingleStuckWatch from a real first error instead of a leftover/zero one -- see that delay's own
// comment) BEFORE mode_snapshot/turn_target/swing_target are captured -- this function used to take
// its snapshot immediately, with no delay ahead of it at all. A retarget landing in this new first
// delay (pass 1, not the RETARGET_AT=10 the tests above use) would have been captured as this call's
// own baseline instead of being noticed as a retarget, the same "snapshot after the first settle
// delay" shape pid_wait() and wait_until_drive() also had and are now fixed for (see
// test_retarget_caught_before_first_settle_delay.cpp); pid_wait_until_point() still has it and is
// tracked separately, not fixed here.
constexpr int FIRST_DELAY_RETARGET_AT = 1;

void turn_wait_until_retargeted_in_first_delay(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
  if (n == FIRST_DELAY_RETARGET_AT) c.pid_turn_set(150, 100);  // lands in the delay before the snapshot
}

void swing_wait_until_retargeted_in_first_delay(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.error = e;
  c.swingPID.derivative = e > 0.0 ? -0.4 : 0.0;
  if (n == FIRST_DELAY_RETARGET_AT) c.pid_swing_set(ez::RIGHT_SWING, 120, 100);  // lands in the delay before the snapshot
}

// Control: TURN_TO_POINT recomputes its own aim point every pass without ever touching turnPID's
// stored target (see the matching comment on pid_wait()'s TURN/TURN_TO_POINT branch above), so
// moving the snapshot earlier must not make an ordinary, un-retargeted turn-to-point wait
// false-fire interfered just because nothing ever touched turnPID.target_get() to begin with.
void turn_to_point_never_retargeted(Drive&, int) {}

TEST_CASE("wait_until_turn_swing_internal() TURN: a concurrent pid_turn_set() landing in the function's own first settle delay ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_wait_until_retargeted_in_first_delay, 300, [&] { chassis.pid_wait_until(45.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= FIRST_DELAY_RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("wait_until_turn_swing_internal() SWING: a concurrent pid_swing_set() landing in the function's own first settle delay ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 60, 100);
  Outcome o = run(chassis, swing_wait_until_retargeted_in_first_delay, 300, [&] { chassis.pid_wait_until(30.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= FIRST_DELAY_RETARGET_AT + GUARD_SLACK);
}

TEST_CASE("wait_until_turn_swing_internal() TURN_TO_POINT: an ordinary, un-retargeted wait never false-fires interfered") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 110);
  REQUIRE(chassis.mode == TURN_TO_POINT);
  Outcome o = run(chassis, turn_to_point_never_retargeted, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

// ---- pid_wait_until_point() had the identical total gap. ----
TEST_CASE("pid_wait_until_point(): a concurrent pid_odom_ptp_set() mid-wait ends the wait instead of finishing on the new target") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
  Outcome o = run(chassis, point_wait_healthy_then_retargeted, 300, [&] { chassis.pid_wait_until_point({0.0, 24.0, ANGLE_NOT_SET}); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}

// ---- Every guard above only watched its OWN family's target -- a concurrent setter from a DIFFERENT
// mode changes `mode` unnoticed, freezing the old PID's error.  If that frozen value happens to land
// inside its own exit window, the wait reports a clean, uninterfered "success" for an abandoned motion.
TEST_CASE("pid_wait() TURN: a concurrent pid_swing_set() mid-wait is caught via the mode change even though it never retargets turnPID itself") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(100, 2.0, 0, 0.0, 0, 0);  // small exit only, ~10 passes to fire once in tolerance
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, turn_frozen_in_tolerance_then_mode_changed, 300, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes <= RETARGET_AT + GUARD_SLACK);
}
