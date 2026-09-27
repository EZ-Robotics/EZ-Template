// PID::exit_condition()'s VELOCITY_EXIT channel is guarded against a stale (unrefreshed) reading:
// PID.cpp's k_prev_checked/k_unchanged_time machinery (in exit_condition()'s velocity block) exists
// specifically because a raw value that hasn't actually advanced since the last poll -- because
// ez_auto_task didn't run that pass, not because the robot is really stopped -- must not, on its own,
// count as evidence of a stall (see test_pid.cpp's staleness tests). Until this fix, SMALL_EXIT and
// BIG_EXIT (the `j`/`i` timers, just above the velocity block in the same function) had no equivalent
// guard: `j += util::DELAY_TIME` ran every single call where `fabs(error) < exit.small_error`, with no
// check that `error` was a fresh sample rather than the same value this function already saw last
// time. They're now gated by compute_count/last_checked_compute -- see their comments in PID.hpp and
// exit_condition()'s own use of them in PID.cpp.
//
// ez_auto_task computes leftPID/rightPID/turnPID's `error` every pass; the calling wait task's own loop
// (pid_wait(), wait_until_*()) independently calls pros::delay(util::DELAY_TIME) and polls
// exit_condition() every pass of its own, regardless of whether ez_auto_task's pass actually ran in
// between (WAIT_BEHAVIOR_SPEC.md Sec.5: "every wait's own loop polls its PID's error/derivative every
// DELAY_TIME regardless of whether ez_auto_task's per-mode function is still updating them"; see also
// StuckWatch's own class comment in exit_conditions.cpp for the identical framing). test_pp_wait_stuck.cpp's
// "ez_auto_task is starved of time" tests and test_stuck_watch_dead_task_fallback.cpp already accept a
// stalled/dead ez_auto_task as a realistic, in-scope shape -- no mode change or competition transition
// required -- and test it for the direction where the frozen error sits OUTSIDE tolerance (proving
// StuckWatch/the wall-clock fallback don't misfire there). test_stuck_watch_dead_task_fallback.cpp's own
// setup comment zeroes small_error/big_error specifically to isolate the velocity/StuckWatch path --
// which is exactly why it never exercises this one. This file
// is the direction nothing exercises: a frozen error that is already INSIDE tolerance the moment
// ez_auto_task stops ticking.
//
// The exit_conditions.cpp authors already knew "leftover/zero error at motion start" is a real state a
// fresh PID can be in -- wait_until_turn_swing_internal()'s own comment (exit_conditions.cpp, around its
// SingleStuckWatch construction) says so directly: "a fresh Drive's very first turn or swing seeds
// SingleStuckWatch from a leftover/zero error instead of a real, computed one" -- and fixes it there by
// delaying before constructing the watch. That fix only moves when the watch's OWN seed is read; it does
// nothing for exit_condition()'s small/big timers, which start accumulating against whatever `error`
// already holds the instant they're first called, real or not, delay or no delay -- because the delay
// only guarantees the *wait's own* loop has run once, never that ez_auto_task has.
//
// No artificial setup is even required: PID::variables_reset() (PID.cpp, run by every PID's own
// constructor) sets `error = 0`, and `0` is inside every default small_error band (1in DRIVE, 3deg
// TURN/SWING) on its own. So a completely fresh chassis's very first motion, if ez_auto_task hasn't
// run yet, already starts inside tolerance -- see the "fresh chassis, no seeding" cases below. The
// "leftover from an earlier motion" cases make the same point with an explicit, deliberately nonzero
// value, matching the more common real-world shape (chained motions, not just a robot's first ever
// motion). All exit-condition constants below are the constructor's own shipped defaults
// (drive_defaults_set(), drive.cpp) -- nothing here is a non-default setter or a pasted constant.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a fresh ez_auto_task pass overwrites a stale in-tolerance leftover error before it can matter") {
  // Control: ez_auto_task DOES get to run (one real pass, matching what a healthy background task
  // would do almost immediately), computing a real error from the actual, unmoved sensor position
  // against the new 24in target -- overwriting the stale 0.3in leftover with ~24in before anything can
  // accumulate against it. This is the sane, expected result the buggy cases below fail to reach.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.leftPID.error = 0.3;
  chassis.rightPID.error = 0.3;
  chassis.pid_drive_set(24, 100);

  // One real ez_auto_task-equivalent pass: recompute error from the real (unmoved) sensor vs the real
  // target, the same as Drive::drive_pid_task() -> leftPID.compute(drive_sensor_left()) would.
  chassis.leftPID.compute(chassis.drive_sensor_left());
  chassis.rightPID.compute(chassis.drive_sensor_right());
  CHECK(std::fabs(chassis.leftPID.error) > 20.0);  // real error is ~24in, nowhere near small_error (1in)

  test_stub::g_clock.delay_calls_until_stop = 50;  // 500ms -- comfortably more than small/big exit need
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;

  // Must still be running -- a real 24in leg can't have settled in 500ms of a never-driven robot.
  CHECK_FALSE(returned);
}

TEST_CASE("pid_wait() DRIVE: a stale in-tolerance leftover error must not fire a false SMALL_EXIT when ez_auto_task never runs") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);

  // Simulate the tail end of a PREVIOUS motion that settled here (default small_error is 1in).
  chassis.leftPID.error = 0.3;
  chassis.rightPID.error = 0.3;

  // Start a brand-new, unrelated 24in motion. pid_drive_set()'s own call into PID::motion_reset()
  // (PID.cpp) deliberately leaves `error` alone -- it stays 0.3 until the next real
  // leftPID.compute()/compute_error() call, which only Drive::drive_pid_task() (run from
  // ez_auto_task) makes.
  chassis.pid_drive_set(24, 100);

  // ez_auto_task never gets a pass in from here on -- killed, deadlocked, or simply not yet scheduled;
  // no mode change, no competition transition, nothing sensor- or physically-related. Nothing updates
  // leftPID/rightPID's `error`. The calling task's own delay loop is NOT stalled -- pros::millis() and
  // this loop's own call cadence advance completely normally, exactly like test_pp_wait_stuck.cpp's and
  // test_stuck_watch_dead_task_fallback.cpp's own "busy/dead higher-priority task" premise.
  //
  // Budget is well short of SingleStuckWatch's own dead-task wall-clock fallback (STUCK_START_ALLOWANCE_MS
  // 1000ms + STUCK_STARVED_WINDOWS*window_ 2000ms at the shipped 500ms velocity_exit_time -- see
  // test_stuck_watch_dead_task_fallback.cpp), on purpose: this test isolates exit_condition()'s own
  // small/big timers from that separate backstop. A correctly-guarded exit_condition() simply never
  // returns non-RUNNING here -- it does not need to "catch" the dead task itself, only not report a
  // false clean settle before StuckWatch gets its own chance to.
  test_stub::g_clock.delay_calls_until_stop = 50;  // 500ms of real time
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  int passes_used = 50 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " passes_used=", passes_used, " interfered=", chassis.interfered,
          " drive_sensor_left=", chassis.drive_sensor_left());

  // Fixed: without a fresh compute() to back it up, `error` staying inside small_error forever must
  // not be credited toward SMALL_EXIT at all -- the wait is still RUNNING at the end of the budget,
  // the same as the real-compute control case above, not a false clean exit at ~9 passes.
  CHECK_FALSE(returned);
  // The wheels never moved a single tick toward the 24in target either way.
  CHECK(chassis.drive_sensor_left() == doctest::Approx(0.0));
  CHECK(chassis.drive_sensor_right() == doctest::Approx(0.0));
}

TEST_CASE("pid_wait() DRIVE: a completely fresh chassis's first-ever motion must not settle on its own unseeded 0 error") {
  // No leftover value is set at all -- PID::variables_reset() (PID.cpp, run by leftPID/rightPID's own
  // constructors) already leaves error at 0, which is inside the default 1in small_error band on its
  // own. This is the plainest possible version of the bug: the very first motion a brand-new Drive
  // ever runs, before ez_auto_task has had a single chance to compute anything.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  int passes_used = 50 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " passes_used=", passes_used, " interfered=", chassis.interfered);

  // Fixed: a compute_count of 0 compared against itself is not fresh, so an unseeded 0 error can't be
  // credited either -- still RUNNING at the end of the budget.
  CHECK_FALSE(returned);
  CHECK(chassis.drive_sensor_left() == doctest::Approx(0.0));
}

TEST_CASE("pid_wait() TURN: the same stale in-tolerance leftover error must not fire a false settle at default 3deg/7deg constants") {
  // Same unguarded PID.cpp mechanism, reached through TURN's call site (pid_wait()'s TURN branch, via
  // SingleStuckWatch/turnPID) instead of DRIVE's leftPID/rightPID -- confirms this isn't specific to
  // one wait or one mode. 2.5deg is inside the shipped default small_error (3deg, drive_defaults_set(),
  // drive.cpp), the same way a real turn would settle.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);

  chassis.turnPID.error = 2.5;  // leftover from wherever a previous turn settled
  chassis.pid_turn_set(90, 100);
  // turn_set_internal()'s own motion_reset() (set_turn_pid.cpp) leaves turnPID.error untouched, same as
  // the DRIVE setter above.

  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  int passes_used = 50 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " passes_used=", passes_used, " interfered=", chassis.interfered,
          " drive_angle_get=", chassis.drive_angle_get());

  // Fixed: still RUNNING at the end of the budget, not a false settle at ~9 passes.
  CHECK_FALSE(returned);
  // The robot never actually turned toward the new 90 degree target either way.
  CHECK(chassis.drive_angle_get() == doctest::Approx(0.0));
}

TEST_CASE("pid_wait() SWING: the same stale in-tolerance leftover error must not fire a false settle") {
  // Same mechanism again, through SWING's call site (pid_wait()'s SWING branch, via
  // SingleStuckWatch/swingPID) -- swing_set_internal()'s own motion_reset() (set_swing_pid.cpp) leaves
  // swingPID.error untouched, same as every mode above.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);

  chassis.swingPID.error = 2.5;  // leftover from wherever a previous swing settled
  chassis.pid_swing_set(LEFT_SWING, 90, 100);

  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  int passes_used = 50 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " passes_used=", passes_used, " interfered=", chassis.interfered,
          " drive_angle_get=", chassis.drive_angle_get());

  CHECK_FALSE(returned);
  CHECK(chassis.drive_angle_get() == doctest::Approx(0.0));
}

TEST_CASE("pid_wait() odom: stale in-tolerance leftover xy/angle error must not fire a false settle") {
  // Same mechanism once more, through the odom branch (pid_wait()'s POINT_TO_POINT/PURE_PURSUIT
  // branch, via xyPID/current_a_odomPID) -- these share PID::exit_condition() the same as every other
  // mode above, so the fix has to cover them too, not just the non-odom PIDs the pulled cases exercise.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // Leftover from wherever a previous odom motion settled (shipped defaults: xy small_error 1in,
  // angle small_error 3deg -- drive_defaults_set(), drive.cpp).
  chassis.xyPID.error = 0.5;
  chassis.current_a_odomPID.error = 1.0;

  chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100}});
  // Skip straight to the "last point" logic, same as test_odom_pp_setter_resets_xy_timer.cpp -- that's
  // where xy_exit/a_exit's own small/big timers actually run. raw_pid_odom_ptp_set()'s own
  // motion_reset() calls (set_odom_pid.cpp) leave xyPID.error/current_a_odomPID.error untouched, same
  // as every setter above -- neither gets refreshed until Drive::odom_pid_task() (ez_auto_task) makes
  // a real compute_error() pass.
  DriveTestAccess::pp_index(chassis) = (int)DriveTestAccess::pp_movements(chassis).size() - 1;

  test_stub::g_clock.delay_calls_until_stop = 50;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  int passes_used = 50 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " passes_used=", passes_used, " interfered=", chassis.interfered);

  CHECK_FALSE(returned);
}

// ---- End-to-end: does a dead ez_auto_task actually get released, once the false settle above can no
// longer end the wait by itself? -----------------------------------------------------------------------
// Suppressing the false settle only removes exit_condition() as a way for these waits to return. Something
// else still has to end them, or a dead task now hangs pid_wait() forever instead of returning early-but-
// wrong. That something is StuckWatch/SingleStuckWatch's own dead-task wall-clock fallback (window_ ==
// shipped default velocity_exit_time 500ms; STUCK_START_ALLOWANCE_MS 1000ms since this robot never moves +
// STUCK_STARVED_WINDOWS(4)*window_ 2000ms, exit_conditions.cpp) -- ~3000ms worst case. 6000ms of simulated
// time, with no fresh compute anywhere in the budget (the same dead task shape as every case above, just
// watched long enough to reach that fallback), is comfortably past it.
//
// Each case below only asserts that the wait actually returns -- CHECK(returned) -- which is this fix's
// own responsibility: exit_condition() must not stall it, whichever way it eventually resolves. Odom is
// additionally asserted uninterfered=false (a true stuck result), since its own settled check
// (target_distance(), the odom branch's `settled` local in exit_conditions.cpp) is a LIVE measurement, not
// this stale `error`. DRIVE/TURN/SWING are not: pid_wait()'s own stuck-but-settled carve-out for these
// modes (leftPID.error/rightPID.error, turnPID.error, swingPID.error against big_error --
// exit_conditions.cpp's DRIVE/TURN/SWING branches) reads the exact same stale, unrefreshed `error` this
// fix's own timers were changed to stop trusting -- so it independently reaches the same "inside
// big_error, therefore settled" conclusion this fix exists to prevent, just ~3s later instead of ~110ms
// later. `interfered` is logged for these three, deliberately not asserted either way: this fix's job is
// only to keep the wait from hanging, not to correct that separate carve-out (a different root cause, in a
// different function, outside this fix's scope) -- asserting interfered==false here would lock in a known
// gap as if it were guaranteed correct behavior.

TEST_CASE("pid_wait() DRIVE: with the false settle suppressed, a dead ez_auto_task still releases the wait instead of hanging") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.leftPID.error = 0.3;
  chassis.rightPID.error = 0.3;
  chassis.pid_drive_set(24, 100);

  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  test_stub::g_clock.delay_calls_until_stop = 600;  // 6s -- past the ~3s dead-task fallback bound
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " elapsed_ms=", elapsed, " interfered=", chassis.interfered);

  CHECK(returned);
}

TEST_CASE("pid_wait() TURN: with the false settle suppressed, a dead ez_auto_task still releases the wait instead of hanging") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.turnPID.error = 2.5;
  chassis.pid_turn_set(90, 100);

  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  test_stub::g_clock.delay_calls_until_stop = 600;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " elapsed_ms=", elapsed, " interfered=", chassis.interfered);

  CHECK(returned);
}

TEST_CASE("pid_wait() SWING: with the false settle suppressed, a dead ez_auto_task still releases the wait instead of hanging") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.swingPID.error = 2.5;
  chassis.pid_swing_set(LEFT_SWING, 90, 100);

  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  test_stub::g_clock.delay_calls_until_stop = 600;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " elapsed_ms=", elapsed, " interfered=", chassis.interfered);

  CHECK(returned);
}

TEST_CASE("pid_wait() odom: with the false settle suppressed, a dead ez_auto_task still releases the wait, correctly flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  chassis.xyPID.error = 0.5;
  chassis.current_a_odomPID.error = 1.0;

  chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100}});
  DriveTestAccess::pp_index(chassis) = (int)DriveTestAccess::pp_movements(chassis).size() - 1;

  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  test_stub::g_clock.delay_calls_until_stop = 600;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " elapsed_ms=", elapsed, " interfered=", chassis.interfered);

  // Odom's own settled check is a live measurement (target_distance()), not this stale error -- so unlike
  // DRIVE/TURN/SWING above, this one IS asserted: a robot that never moved an inch must read as stuck.
  CHECK(returned);
  CHECK(chassis.interfered);
}
