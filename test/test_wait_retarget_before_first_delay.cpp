// pid_wait() remembers the motion it was called for before its own leading pros::delay(), so a concurrent setter call from another task
// landing during that first ~10 ms is still seen as a new motion: the wait starts over on it, dispatching on the new motion's mode, and
// returns when that motion's exits fire.  This file supplies the repro for a different-mode retarget (a pid_turn_set() while waiting on
// a drive) landing in that delay, plus a same-harness control with the identical retarget landing a couple of passes later, inside the
// DRIVE branch's polling loop.  Both follow the turn.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
int g_retarget_at = 1;  // 1 = during the leading delay; 3 = inside the DRIVE branch's polling loop

// Every pass after the retargeting one drives turnPID.error down toward 0 at a steady rate, standing in for the turn this concurrent task
// started being driven to completion.
void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  if (g_pass == g_retarget_at) {
    c.pid_turn_set(90, 100);  // a real, concurrent retarget -- different mode, different PID entirely
    return;
  }
  int since = g_pass - g_retarget_at;
  double e = std::fmax(0.0, 90.0 - 3.0 * (double)(since > 0 ? since : 0));
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -3.0 : 0.0;
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  e_mode mode;
};

Outcome run_hijack(Drive& chassis, int retarget_at) {
  g_chassis = &chassis;
  g_pass = 0;
  g_retarget_at = retarget_at;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 200;
  Outcome o{true, 0, false, chassis.drive_mode_get()};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  o.passes = 200 - test_stub::g_clock.delay_calls_until_stop;
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.interfered = chassis.interfered;
  o.mode = chassis.drive_mode_get();
  return o;
}

// The turn's error is inside its small_error (3) 30 passes after the retarget (90 - 3 * 30 = 0), and the small exit then takes 4 polls
// to pass 30 ms, the first of them on that pass.  pid_wait() delays once more after the poll that latches the exit.
constexpr int TURN_END_AFTER_RETARGET = 30 + 4 - 1 + 1;
}  // namespace

TEST_CASE("pid_wait(): a retarget landing during the leading delay is followed onto the new motion") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);

  // The caller's actual intent: wait for this 24in drive.
  chassis.pid_drive_set(24, 100);
  double orphaned_left_target = chassis.leftPID.target_get();
  double orphaned_right_target = chassis.rightPID.target_get();
  REQUIRE(chassis.drive_mode_get() == ez::DRIVE);

  // Landing on pass 1 -- pid_wait()'s own leading pros::delay(), before it has read `mode` at all.
  Outcome o = run_hijack(chassis, 1);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " mode=", (int)o.mode);

  REQUIRE(o.returned);
  // The wait follows the turn and returns on the turn's small exit, on the turn's schedule counted from the retarget, clean.
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == 1 + TURN_END_AFTER_RETARGET);
  CHECK(o.mode == ez::TURN);
  // The replaced drive's targets are left as they were: following the turn doesn't touch them.
  CHECK(chassis.leftPID.target_get() == doctest::Approx(orphaned_left_target));
  CHECK(chassis.rightPID.target_get() == doctest::Approx(orphaned_right_target));
}

TEST_CASE("pid_wait(): control -- the identical retarget landing a couple of passes later, inside the polling loop, is followed the same way") {
  // Same setup, same concurrent pid_turn_set() call, only the timing changes: it lands on pass 3, inside the DRIVE branch's own polling
  // loop.  This isolates timing as the only variable between this file's two outcomes.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  chassis.leftPID.error = 20.0;  // outside small_error, so the DRIVE branch is still RUNNING at pass 3
  chassis.leftPID.derivative = -0.1;
  chassis.rightPID.error = 20.0;
  chassis.rightPID.derivative = -0.1;

  Outcome o = run_hijack(chassis, 3);
  MESSAGE("returned=", o.returned, " passes=", o.passes, " interfered=", o.interfered, " mode=", (int)o.mode);

  REQUIRE(o.returned);
  CHECK_FALSE(o.interfered);
  CHECK(o.passes == 3 + TURN_END_AFTER_RETARGET);
  CHECK(o.mode == ez::TURN);
}
