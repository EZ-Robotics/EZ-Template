// pid_wait()'s DRIVE branch used to take its retarget-detection snapshot (mode_snapshot, plus the
// relevant PID target or odom_target_start) AFTER its own first pros::delay() -- not before it. The
// same shape also reached wait_until_drive(), by way of pid_wait_quick_chain()'s DRIVE branch (its
// own fallthrough to pid_wait_until(), with no outer snapshot of its own at all -- unlike its
// PURE_PURSUIT/POINT_TO_POINT branches, which DO take an outer snapshot before calling their inner
// wait for exactly this reason). test_drive_retarget_guard.cpp and
// test_all_public_waits_retarget_table.cpp already prove the guard reliably catches a concurrent
// pid_*_set() from a second task landing on any LATER pass. This file exercises the one window those
// tests deliberately don't: a retarget landing during that exact first delay, which a snapshot taken
// right after it would otherwise miss (since that snapshot would already reflect the new motion).
//
// Both functions now snapshot mode (plus, for pid_wait()'s DRIVE branch, leftPID/rightPID's target)
// before their own leading delay instead, matching wait_until_turn_swing_internal()/
// pid_wait_until_index_started(), which already did -- closing the cross-mode case this file covers
// for both functions, and the same-mode case for DRIVE specifically.
//
// Covered here: pid_wait() DRIVE, and wait_until_drive() by way of pid_wait_quick_chain() DRIVE.
// NOT covered, and still open: pid_wait()'s odom/TURN/SWING branches still take their own
// odom_target_start/turn_target/swing_target snapshot AFTER the shared leading delay (only `mode`
// itself was hoisted ahead of it, which is what closes the cross-mode case for every branch here) --
// a SAME-mode retarget landing in that window (e.g. a second pid_turn_set() while already turning)
// is still invisible to those three branches specifically. pid_wait_until_point() has the identical
// delay-then-snapshot shape in full and is tracked and fixed elsewhere, not here.
//
// The exposure window is small -- at most one DELAY_TIME (10ms default) per wait call, the gap
// between the wait starting and it taking its own baseline -- so this needs no unusual configuration,
// only ordinary two-task timing: a second task's pid_*_set() call landing in that first ~10ms window,
// which is exactly the kind of race a real two-task auton (a watchdog task, a sensor-triggered abort
// task, or any task sharing the chassis) can hit. TEAM_CORPUS.md records repos calling
// pid_wait_quick_chain() 90-120 times in a single auton file, which is why that DRIVE-mode path is
// exercised here as the primary case alongside the plain pid_wait() one.
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

// A second task's pid_drive_set() call, timed to land during the wait's own FIRST settle delay --
// before it has taken any snapshot of the motion it's supposed to be waiting for.
void retarget_on_first_pass_drive() {
  ++g_pass;
  Drive& c = *g_chassis;
  if (g_pass == 1) {
    c.pid_drive_set(6, 100);  // second task retargets to a much shorter drive
  }
  // Held inside small_error from the very first pass of whichever motion is live -- this is what
  // would let the new (post-retarget) motion small-exit almost immediately if the retarget went
  // undetected, reading as an innocuous, uninterfered success.
  c.leftPID.error = 0.5;
  c.rightPID.error = 0.5;
}

struct Outcome {
  bool returned;
  bool interfered;
  std::string printed;
  double left_target_after;
};

Outcome run_scripted(Drive& chassis, void (*fn)()) {
  g_chassis = &chassis;
  g_pass = 0;
  Outcome o{true, false, "", 0.0};
  o.printed = test_stub::capture_stdout([&] {
    test_stub::g_clock.on_delay = fn;
    test_stub::g_clock.delay_calls_until_stop = 30;
    try {
      chassis.pid_wait_quick_chain();
    } catch (test_stub::StopLoop&) {
      o.returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = nullptr;
  });
  o.interfered = chassis.interfered;
  o.left_target_after = chassis.leftPID.target_get();
  return o;
}
}  // namespace

TEST_CASE("pid_wait_quick_chain() DRIVE: a retarget landing during the inner wait's own first settle delay is caught") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);                               // the guard message is gated on this; captured and checked below
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);  // small exit only: 50 ms/1 in
  chassis.pid_drive_set(48, 100);
  double original_left_target = chassis.leftPID.target_get();

  Outcome o = run_scripted(chassis, retarget_on_first_pass_drive);
  MESSAGE("returned=" << o.returned << " interfered=" << o.interfered << " leftPID.target=" << o.left_target_after
                      << " original target=" << original_left_target);

  REQUIRE(o.returned);
  // Correct behavior: this is the same shape of concurrent retarget test_drive_retarget_guard.cpp
  // proves is always caught (guard message printed, interfered set true) when it lands on any pass
  // after the first. It must be caught here too -- pid_wait_quick_chain() is meant to end the wait
  // it was called for, not silently adopt whatever motion is live by the time it gets around to
  // taking its first look.
  CHECK(o.printed.find("retargeted by a concurrent motion") != std::string::npos);
  CHECK(o.interfered);
}

// Same shape, in the simpler pid_wait() DRIVE branch directly (no chaining involved) -- confirms this
// isn't an artifact of pid_wait_quick_chain()'s own extra bookkeeping.
TEST_CASE("pid_wait() DRIVE: a retarget landing during the wait's own first settle delay is caught") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);  // the guard message is gated on this; captured and checked below
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);
  chassis.pid_drive_set(48, 100);
  double original_left_target = chassis.leftPID.target_get();

  g_chassis = &chassis;
  g_pass = 0;
  bool returned = true;
  std::string printed = test_stub::capture_stdout([&] {
    test_stub::g_clock.on_delay = retarget_on_first_pass_drive;
    test_stub::g_clock.delay_calls_until_stop = 30;
    try {
      chassis.pid_wait();
    } catch (test_stub::StopLoop&) {
      returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = nullptr;
  });
  MESSAGE("returned=" << returned << " interfered=" << chassis.interfered << " leftPID.target=" << chassis.leftPID.target_get()
                      << " original target=" << original_left_target);

  REQUIRE(returned);
  CHECK(printed.find("retargeted by a concurrent motion") != std::string::npos);
  CHECK(chassis.interfered);
}
