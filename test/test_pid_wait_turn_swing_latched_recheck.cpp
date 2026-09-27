// pid_wait()'s TURN and SWING branches latch turn_exit/swing_exit the first time it goes
// non-RUNNING:
//
//   turn_exit = turn_exit != RUNNING ? turn_exit : without_velocity(turnPID.exit_condition(...));
//
// and then fall out of their `while (turn_exit == RUNNING) { ... }` loop on the next condition
// check -- but not the SAME pass: the loop's own trailing pros::delay(DELAY_TIME) still runs once,
// unconditionally, on the very pass that latched, before the loop condition is re-checked. That one
// extra pass is a real window: a disturbance landing during it (a defender shove, a collision) is
// never looked at again -- turnPID.error/swingPID.error is never re-read once turn_exit/swing_exit is
// non-RUNNING -- and the wait falls out reporting a clean, uninterfered finish regardless of where
// the PID actually ended up.
//
// This is the identical shape pid_wait()'s DRIVE branch was fixed for (see
// test_pid_wait_drive_latched_side_recheck.cpp) and pid_wait()'s odom branch before that (see
// test_odom_latched_axis_drift_after_exit.cpp): right before trusting a clean latched exit, recheck
// it against the live error, using the same window it exited through, and un-latch (back to RUNNING)
// if it has drifted back outside since. TURN and SWING get the identical treatment here, including
// the settled-via-stuck carve-out DRIVE's own fix needed (see exit_conditions.cpp's comment on why a
// stuck-but-settled break must not fall into the recheck at all -- there's nothing on that path to
// un-latch, since a side that's still RUNNING was never latched).
//
// The latch happens on whichever exact pass PID.cpp's own per-call small_exit_time counter first
// crosses its threshold (see PID.cpp's exit_condition(), which counts DELAY_TIME per fresh in-window
// CALL, not wall-clock time) -- and the one-pass trust window this bug is about only exists on that
// specific pass and the one right after it. Landing the shove on the wrong pass either lands well
// before the latch (so the exit timer resets every pass and never actually latches at all -- a
// different, unrelated shape entirely) or well after the wait has already returned on a genuinely
// clean latch (nothing to catch). Rather than hardcode the one pass this depends on, this sweeps the
// shove's landing pass across a range that brackets it -- pre-fix, whichever value lands in the
// latch-to-return window fails; post-fix, all of them pass.
#include <cmath>

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
int g_shove_at = 0;

// Held at 2deg (inside small_error=3) through pass g_shove_at, then shoved to 10deg (well outside
// big_error=7) from g_shove_at+1 onward, forever. A DriveTestAccess::refresh() call every pass, not a
// bare `.error =` write -- PID.cpp's small/big exit timers only credit `error` when a real compute
// has landed since they last checked, so without this the exit could never actually latch and this
// test would only ever exercise the StuckWatch fallback.
void turn_script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = (g_pass <= g_shove_at) ? 2.0 : 10.0;
  g_chassis->turnPID.error = e;
  g_chassis->turnPID.derivative = 0.0;
  DriveTestAccess::refresh(g_chassis->turnPID);
}
void swing_script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = (g_pass <= g_shove_at) ? 2.0 : 10.0;
  g_chassis->swingPID.error = e;
  g_chassis->swingPID.derivative = 0.0;
  DriveTestAccess::refresh(g_chassis->swingPID);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  double error_at_return;
};

Outcome run_wait(Drive& chassis, void (*script)(), int shove_at, int max_passes, double* error_field) {
  g_chassis = &chassis;
  g_pass = 0;
  g_shove_at = shove_at;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.error_at_return = *error_field;
  return o;
}
}  // namespace

TEST_CASE("pid_wait() TURN: a latched exit is rechecked, so a shove landing right at the latch is still reported as interfered") {
  for (int shove_at = 6; shove_at <= 16; shove_at++) {
    SUBCASE(("shove_at=" + std::to_string(shove_at)).c_str()) {
      Drive chassis = make_chassis();
      chassis.pid_print_toggle(false);
      // velocity_exit_time set nonzero (500ms) purely to give SingleStuckWatch a real window --
      // without_velocity() strips any raw VELOCITY_EXIT exit_condition() itself might latch back to
      // RUNNING inside pid_wait()'s TURN loop, so this can't introduce a second latch path, only let
      // the stuck fallback actually resolve a shove that lands too early to ever latch at all (a
      // window_==0 SingleStuckWatch always returns not-stuck, see its own comment).
      chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 0);
      chassis.pid_turn_set(90, 100);
      REQUIRE(chassis.mode == TURN);

      Outcome o = run_wait(chassis, turn_script, shove_at, 400, &chassis.turnPID.error);
      CAPTURE(shove_at);
      MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " error=", o.error_at_return);

      REQUIRE(o.returned);
      // Never acceptable: a clean (interfered==false) return while the live error sits outside the
      // window it supposedly exited through. Either the wait is genuinely settled (error inside
      // big_error) or it must say so via interfered.
      CHECK((o.interfered || std::fabs(o.error_at_return) < chassis.turnPID.exit.big_error));
    }
  }
}

TEST_CASE("pid_wait() SWING: a latched exit is rechecked, so a shove landing right at the latch is still reported as interfered") {
  for (int shove_at = 6; shove_at <= 16; shove_at++) {
    SUBCASE(("shove_at=" + std::to_string(shove_at)).c_str()) {
      Drive chassis = make_chassis();
      chassis.pid_print_toggle(false);
      chassis.pid_swing_exit_condition_set(90, 3.0, 250, 7.0, 500, 0);
      chassis.pid_swing_set(ez::LEFT_SWING, 90, 100);
      REQUIRE(chassis.mode == SWING);

      Outcome o = run_wait(chassis, swing_script, shove_at, 400, &chassis.swingPID.error);
      CAPTURE(shove_at);
      MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " error=", o.error_at_return);

      REQUIRE(o.returned);
      CHECK((o.interfered || std::fabs(o.error_at_return) < chassis.swingPID.exit.big_error));
    }
  }
}

TEST_CASE("pid_wait() TURN control: a latched exit that never drifts back out stays a clean, uninterfered finish") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_turn_set(90, 100);

  g_shove_at = 1000000;  // never shoved within this run's budget
  Outcome o = run_wait(chassis, turn_script, 1000000, 300, &chassis.turnPID.error);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() SWING control: a latched exit that never drifts back out stays a clean, uninterfered finish") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_swing_set(ez::LEFT_SWING, 90, 100);

  Outcome o = run_wait(chassis, swing_script, 1000000, 300, &chassis.swingPID.error);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered);

  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}
