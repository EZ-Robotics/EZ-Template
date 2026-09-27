// pid_wait()'s DRIVE branch shares the same leading-delay-then-seed-the-backstop shape as the turn
// and swing waits (see test_turn_wait_stuck_watch_seed_survives_skipped_first_tick.cpp for the full
// explanation of the underlying gap): a single, fixed pros::delay(DELAY_TIME) runs once before each
// side's SingleStuckWatch is constructed from that side's own current error, on the assumption that
// the delay always contains at least one real background PID pass. It doesn't have to -- the wait
// loop and the background PID task are two independently scheduled periodic loops that are never
// lock-stepped, so an entirely ordinary single missed tick during that one delay (no starvation,
// nothing wrong afterward) seeds the backstop from a stale error instead of a real one.
//
// This matters more here than for turn/swing: DRIVE is the library's most-used wait family (see
// WAIT_BEHAVIOR_SPEC.md section 3's own framing), this reproduces at the real shipped-default exit
// constants (90ms/1in/250ms/3in/500ms/500ms, confirmed from drive.cpp's own constructor -- NOT the
// turn-style 3in/7in numbers), and it is not limited to a brand-new Drive's very first motion ever:
// motion_reset() never touches `error` (see PID.cpp), so an ordinary SECOND leg of the same auton,
// started after the first one already converged cleanly, seeds its own backstop from whatever
// `error` the first motion's last real compute left behind -- not a fresh-Drive-specific artifact,
// but the same stale-seed shape on every motion whose own first tick happens to be missed. The third
// test below checks that directly.
//
// Every script here drives the PIDs through real compute_error() calls, not direct `.error =`
// writes, so this reproduces (or would be fixed) under a freshness-aware seed just as it would on
// real hardware -- matching the idiom in test_stuckwatch_starvation_cadence.cpp. Derivative is left
// wherever compute_error() puts it (0, since `cur` is never moved): harmless here because
// pid_wait()'s DRIVE branch already filters VELOCITY_EXIT through without_velocity()
// (WAIT_BEHAVIOR_SPEC.md section 8.6), so a static derivative can't end this wait on its own.
//
// All three scripts share the same close-then-bump-then-slow-recover shape: a real bump survives
// cleanly when every tick lands (control), but false-stops the wait when only the very first tick
// of the wait being tested is skipped, because the stale-to-real jump on the next tick reads as a
// fake overshoot and spends the underlying Channel's one-shot recovery allowance before the real
// bump ever happens.
#include <algorithm>
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_fresh_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Closes normally from 60 down to 16, then a real bump back up to 40, then a slow recovery -- slow
// enough that beating the PRE-BUMP low (16, so needs to reach under 15 at small_error/big_error's
// shared 1in step) takes far longer than one progress-backstop window if the rebound was already
// spent, but comfortably fits inside one window if it wasn't (only needs to beat the bump's own low
// of 40, i.e. reach under 39).
double disturbance_error(int n) {
  if (n <= 2) return 60.0;
  if (n <= 13) return 60.0 - 4.0 * (n - 2);  // n=3:56 ... n=13:16
  if (n == 14) return 40.0;                  // a single real bump
  return std::fmax(40.0 - 0.3 * (n - 14), 0.0);
}

// The first tick this wait's own on_delay() ever sees is skipped entirely -- no compute at all --
// simulating the background drive PID task simply not getting a pass in during the wait's fixed
// leading delay this one time.
// n==1 is a genuinely missed tick: no compute, and no auto_task_passes credit either -- the
// background task did not run at all this pass, not just skip writing a value.
void skip_first_tick(Drive& c, int n) {
  if (n == 1) return;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = disturbance_error(n);
  c.leftPID.compute_error(e, c.leftPID.cur);
  c.rightPID.compute_error(e, c.rightPID.cur);
}

// Control: the identical shape, but every tick lands and is credited.
void every_tick_lands(Drive& c, int n) {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = n == 1 ? 60.0 : disturbance_error(n);
  c.leftPID.compute_error(e, c.leftPID.cur);
  c.rightPID.compute_error(e, c.rightPID.cur);
}

// A fast, ordinary, undisturbed close from 60 to 0 -- used only to let a first motion finish
// cleanly before a second one starts, for the "later leg of the same auton" variant.
void converge_cleanly(Drive& c, int n) {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = std::fmax(60.0 - 2.0 * n, 0.0);
  c.leftPID.compute_error(e, c.leftPID.cur);
  c.rightPID.compute_error(e, c.rightPID.cur);
}

Drive* g_chassis = nullptr;
void (*g_script)(Drive&, int) = nullptr;
int g_pass = 0;

void on_delay() {
  ++g_pass;
  g_script(*g_chassis, g_pass);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run_wait(Drive& chassis, void (*script)(Drive&, int), int max_passes) {
  g_chassis = &chassis;
  g_script = script;
  g_pass = 0;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a real bump-and-slow-recovery survives when every tick lands") {
  Drive chassis = make_fresh_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  // Shipped defaults, confirmed from drive.cpp's own constructor call.
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(90, 100);

  Outcome o = run_wait(chassis, every_tick_lands, 300);
  MESSAGE("every tick lands: returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() DRIVE: an ordinary missed first tick makes the same survivable bump false-stuck the wait") {
  Drive chassis = make_fresh_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(90, 100);

  Outcome o = run_wait(chassis, skip_first_tick, 300);
  MESSAGE("first tick skipped: returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() DRIVE: a later leg of the same auton is exposed the same way, not just a fresh Drive's first motion") {
  Drive chassis = make_fresh_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 500);

  // First motion: converges normally and finishes with error near 0, the ordinary way.
  chassis.pid_drive_set(60, 100);
  Outcome first = run_wait(chassis, converge_cleanly, 100);
  MESSAGE("first motion: returned=" << first.returned << " passes=" << first.passes << " interfered=" << first.interfered);
  REQUIRE(first.returned);
  REQUIRE_FALSE(first.interfered);

  // Second motion: skip its own first tick, then the same bump-and-slow-recovery shape.
  chassis.pid_drive_set(90, 100);
  Outcome second = run_wait(chassis, skip_first_tick, 300);
  MESSAGE("second motion, first tick skipped: returned=" << second.returned << " passes=" << second.passes << " interfered=" << second.interfered);
  CHECK(second.returned);
  CHECK_FALSE(second.interfered);
}
