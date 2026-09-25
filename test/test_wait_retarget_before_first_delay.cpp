// Every public wait's concurrent-retarget guard (test_wait_retarget_guard.cpp,
// test_drive_retarget_guard.cpp, etc.) snapshots `mode` plus the relevant PID target/odom_target_start
// BEFORE its own polling loop starts, so a retarget landing mid-loop is caught on the very next pass.
// Two of these waits' comments explicitly call out a narrower, earlier hazard and were fixed for it:
//
//   wait_until_turn_swing_internal() (exit_conditions.cpp, around its own settle pros::delay()):
//     "Snapshotted here, BEFORE the settle delay below (not after it): a concurrent retarget landing
//      during that delay would otherwise be captured as this call's own baseline instead of being
//      noticed."
//
//   pid_wait_until_index_started() (exit_conditions.cpp, around its own settle pros::delay()):
//     "Snapshotted before the settle delay below, not after: a concurrent motion setter can retarget
//      the drive during this call's own first pros::delay(), before anything else has taken a baseline
//      to compare against. Checking against a pre-delay snapshot catches that as a retarget instead of
//      either silently treating the new motion as this call's own, or ... misreporting it."
//
// pid_wait() and wait_until_drive() did not have this same fix: each of them still called its own
// leading pros::delay() FIRST, and only took its `mode`/target snapshot afterward -- pid_wait() did
// not even snapshot `mode` before deciding which branch (DRIVE/odom/TURN/SWING) to run at all; it
// re-read `mode` fresh, after the delay, for that very decision. A concurrent setter call from
// another task landing during that first ~10ms was invisible to both: not merely "silently finishing
// on the wrong target" (the already-guarded mid-loop case), but entering an entirely different
// branch/mode than the one the call was actually started for, with nothing to tell the caller their
// original motion was abandoned. Both now snapshot before that leading delay, matching
// wait_until_turn_swing_internal()/pid_wait_until_index_started() above.
//
// The standalone pid_wait_until_point() has the identical shape and is not fixed here -- see its own
// tracked gap -- so this file's coverage is deliberately limited to pid_wait() and, by way of
// pid_wait_quick_chain(), wait_until_drive(). This file supplies the runnable repro those two were
// missing, plus a same-harness control proving the timing is the only variable: the exact same
// concurrent pid_turn_set() call, landing one pass later (after the snapshot instead of during the
// leading delay), IS caught by the existing guard.
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
int g_retarget_at = 1;  // 1 = during the leading settle delay (before the snapshot); 3 = after it

// Every pass but the retargeting one drives turnPID.error down toward 0 at a realistic, steady rate --
// standing in for the turn this concurrent task started actually being driven to completion (by
// ez_auto_task, or by whatever else is now live), not the staleness bug
// test_small_exit_stale_during_starvation.cpp already covers separately.
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
}  // namespace

TEST_CASE("pid_wait(): a retarget landing during the leading settle delay is caught, not silently absorbed") {
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
  // Correct: a clear signal that this call never actually waited on the DRIVE motion it was
  // started for.
  CHECK(o.interfered);
  // The concurrent task's own pid_turn_set() call already moved mode to TURN before this call's
  // pre-dispatch check ran -- this call ends there instead of running that TURN motion's own wait
  // loop, but it doesn't (and can't, without new locking) revert mode back to DRIVE for the
  // now-abandoned motion either.
  CHECK(o.mode == ez::TURN);
  // The original DRIVE motion is still sitting there, targeted but never waited for by this call, and
  // this call's own return gives the caller nothing indicating that -- interfered is false, so a
  // caller chaining `pid_drive_set(24); pid_wait();` sees a clean, ordinary-looking return.
  CHECK(chassis.leftPID.target_get() == doctest::Approx(orphaned_left_target));
  CHECK(chassis.rightPID.target_get() == doctest::Approx(orphaned_right_target));
}

TEST_CASE("pid_wait(): control -- the identical retarget landing one pass later, after the snapshot, IS caught") {
  // Same setup, same concurrent pid_turn_set() call, only the timing changes: it now lands after
  // pid_wait() has already taken its mode_snapshot (pass 1 is a normal, un-retargeted pass; the
  // retarget lands on pass 3, inside the DRIVE branch's own polling loop). This isolates timing as the
  // only variable between this file's two outcomes.
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
  // The existing mid-loop guard catches this one, exactly as test_wait_retarget_guard.cpp already
  // proves for other retarget-after-the-snapshot cases.
  CHECK(o.interfered);
}
