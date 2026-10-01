// GitHub issue #527: PID::exit_condition() calls PID::timers_reset() -- which zeroes every
// channel's timer together, including the mA (over-current) timer -- whenever ANY channel latches.
// Drive::pid_wait()'s PURE_PURSUIT intermediate-point loop calls xyPID.exit_condition(...) every
// pass and only ever inspects the result for mA_EXIT: xyPID's target there is a moving look-ahead
// point, not the real path, so SMALL_EXIT/BIG_EXIT/VELOCITY_EXIT are discarded on purpose. But a
// SMALL_EXIT firing on that (discarded, irrelevant) look-ahead-point error still wipes the SAME
// call's mA timer before it can ever reach mA_timeout -- so a motor that is genuinely, continuously
// over-current the whole leg never actually exits through mA_EXIT; the loop only ends once
// StuckWatch's own, much later fallback window runs out instead.
//
// xy's own velocity_exit_time is deliberately set larger than its mA_timeout here -- the issue's
// own "becomes a real problem" case -- so a wrongly-delayed exit is a large, unambiguous timing
// difference from a correct one, not just a differently-labeled exit at the same tick (which is
// what happens when velocity_exit_time is left at 0, per the issue).
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// xy: small_error/small_exit_time recur on every single fresh pass (small_exit_time 0 means
// SMALL_EXIT latches the very first fresh compute inside small_error -- the worst case the issue
// describes, "small_exit_time recurs faster than mA_timeout"). mA_timeout (200ms, ~21 passes) is
// what should actually end this leg once the fix is in place. velocity_exit_time (500ms) is left
// well above mA_timeout so a fallback exit through StuckWatch (which also gets a further ~1000ms
// start allowance since nothing in this test ever "moves") lands far later than a correct mA_EXIT
// would, not at the same tick.
void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(0, 5.0, 0, 0.0, 500, 200);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}, {{0.0, 36.0, 0.0}, fwd, 110}});
  // Starting a move doesn't touch xyPID/current_a_odomPID's exit conditions (only
  // pid_odom_..._exit_condition_set does), but re-asserting them here keeps this test's intent
  // explicit, matching test_purepursuit_midpath_stall.cpp's own pattern.
  chassis.xyPID.exit_condition_set(0, 5.0, 0, 0.0, 500, 200);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

Drive* g_chassis = nullptr;
int g_passes = 0;

// Every simulated pass: a real compute_error() lands (so the small exit timer's freshness gate --
// PID.cpp's error_fresh -- actually credits it, see DriveTestAccess::refresh()'s own comment) with
// xy's error held inside small_error the whole time (the moving look-ahead point being tracked
// well, the ordinary case this loop is written for) while both drive motors keep reporting over
// current the whole time too (a genuinely, continuously stalled mechanism).
void scripted_small_exit_with_continuous_overcurrent() {
  ++g_passes;
  DriveTestAccess::refresh(g_chassis->xyPID);
  DriveTestAccess::refresh(g_chassis->current_a_odomPID);
}

// Runs `wait` with the fake pros::delay() set to throw after `max_delays` calls, so a wait that
// never returns fails the test instead of hanging it. Mirrors the helper in
// test_purepursuit_midpath_stall.cpp.
template <typename F>
bool returns(int max_delays, F&& wait) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool done = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    done = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return done;
}
}  // namespace

TEST_CASE(
    "pid_wait ends a pure pursuit leg through mA_EXIT, not just StuckWatch's much later fallback, when a discarded SMALL_EXIT keeps re-firing on xy's look-ahead point while a motor stays continuously over current") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  chassis.xyPID.error = 1.0;  // held inside small_error (5.0) the entire test

  chassis.left_motors[0].fake().over_current = true;
  chassis.right_motors[0].fake().over_current = true;

  g_chassis = &chassis;
  g_passes = 0;
  test_stub::g_clock.on_delay = scripted_small_exit_with_continuous_overcurrent;

  bool done = returns(400, [&] { chassis.pid_wait(); });
  test_stub::g_clock.on_delay = nullptr;

  // Nothing in this test drives pp_task, so the path never advances past its first point --
  // whatever ended the wait, it did so from inside the intermediate-point loop, not the final one.
  REQUIRE(DriveTestAccess::pp_index(chassis) != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  REQUIRE(done);
  CHECK(chassis.interfered);

  // xy's own mA_timeout is 200ms (~21 passes at the 10ms DELAY_TIME) -- the fix makes this the
  // fallback that ends the wait. Without it, StuckWatch is what ends the wait instead, and much
  // later: this test never runs ez_auto_task, so StuckWatch's own pass-counter confirmation
  // (stuck_passes(), driven only by that task) never advances, leaving only its wall-clock fallback
  // able to fire -- STUCK_START_ALLOWANCE_MS (1000ms, nothing here ever "moves") plus
  // STUCK_STARVED_WINDOWS (4) times xy's own velocity_exit_time (500ms), ~3000ms, ~pass 300 (this
  // was observed at pass 302 with the bug present, confirming the arithmetic). 30 passes is
  // comfortably past a genuine mA_EXIT and comfortably short of that fallback -- this is the
  // assertion the discarded-SMALL_EXIT bug fails (it never reaches mA_EXIT at all, so the wait only
  // ends via the much later fallback).
  CHECK(g_passes <= 30);
}
