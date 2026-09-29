// wait_until_turn_swing_internal() lets the PID run at least one iteration (a single, fixed
// pros::delay(DELAY_TIME)) before constructing its no-progress backstop (SingleStuckWatch), so the
// watch seeds from a real, computed error instead of a fresh Drive's leftover/zero one -- see the
// comment on that delay in exit_conditions.cpp and the existing test covering it,
// test_wait_until_turn_swing_first_motion_watch_seed.cpp.
//
// That fix assumes the wait's own fixed-length leading delay always contains at least one real
// ez_auto_task pass. It doesn't have to: the wait loop and ez_auto_task are two independently
// scheduled periodic loops that both nominally run every DELAY_TIME but are never lock-stepped
// (PID.cpp's own exit_condition() comment says this explicitly, about the same two loops, for the
// same reason). An entirely ordinary single missed tick -- ez_auto_task simply not getting
// scheduled during that specific 10ms window, with no starvation and no gap afterward -- is enough
// to make the watch seed from a stale value the original fix was written to avoid.
//
// The result is the exact bug the fix closed, reappearing through a path the fix's own test never
// exercises (that test always feeds a real value on the very first tick). The stale-to-real jump
// reads as an "overshoot" the moment the real first error arrives, spending the underlying
// Channel's one-shot rebound allowance (see exit_conditions.cpp's Channel comment) on nothing -- so
// this turn's first GENUINE disturbance (a single ordinary bump, not a second one) then gets zero
// grace and must fully recover past its pre-disturbance low within one progress window, or the wait
// ends early with interfered=true, on a robot that was never actually stuck.
//
// This is not specific to a brand-new Drive's very first turn either: turn_set_internal() (like
// every other motion setter) never resets `error` (see PID.cpp's motion_reset()), so an ordinary
// second turn later in the same auton, run only after the first one already finished cleanly, seeds
// its own backstop from whatever `error` the first turn's last real compute left behind -- the third
// test below checks that directly.
//
// Every script drives turnPID through real compute_error() calls, not direct `.error =` writes, so
// this reproduces (or would be fixed) under a freshness-aware seed just as it would on real
// hardware -- matching the idiom in test_stuckwatch_starvation_cadence.cpp. Derivative is left
// wherever compute_error() puts it (0, since `cur` is never moved): harmless here because
// pid_wait()'s TURN branch and wait_until_turn_swing_internal() both filter VELOCITY_EXIT through
// without_velocity() (WAIT_BEHAVIOR_SPEC.md section 8.6), so a static derivative can't end either
// wait on its own.
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

// Closes normally from 60 down to 16 degrees, then a real bump back up to 40, then a slow recovery
// -- slow enough that beating the PRE-BUMP low (16, so needs to reach under 13 at turn's 3-degree
// step) takes far longer than one progress-backstop window if the rebound was already spent, but
// comfortably fits inside one window if it wasn't (only needs to beat the bump's own low of 40, i.e.
// reach under 37).
double turn_disturbance_error(int n) {
  if (n <= 2) return 60.0;
  if (n <= 13) return 60.0 - 4.0 * (n - 2);  // n=3:56 ... n=13:16
  if (n == 14) return 40.0;                  // a single real bump
  return std::fmax(40.0 - 0.3 * (n - 14), 0.0);
}

// The first tick this wait's own on_delay() ever sees is skipped entirely -- no compute at all --
// simulating ez_auto_task simply not getting a pass in during the wait's fixed leading delay this
// one time.
// n==1 is a genuinely missed tick: no compute, and no auto_task_passes credit either -- ez_auto_task
// did not run at all this pass, not just skip writing a value.
void skip_first_tick(Drive& c, int n) {
  if (n == 1) return;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  c.turnPID.compute_error(turn_disturbance_error(n), c.turnPID.cur);
}

// Control: the identical shape, but every tick lands and is credited.
void every_tick_lands(Drive& c, int n) {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  double e = n == 1 ? 60.0 : turn_disturbance_error(n);
  c.turnPID.compute_error(e, c.turnPID.cur);
}

// A fast, ordinary, undisturbed close from 60 to 0 degrees -- used only to let a first turn finish
// cleanly before a second one starts.
void converge_cleanly(Drive& c, int n) {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  c.turnPID.compute_error(std::fmax(60.0 - 2.0 * n, 0.0), c.turnPID.cur);
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

Outcome run_wait(Drive& chassis, void (*script)(Drive&, int), int max_passes, double wait_target) {
  g_chassis = &chassis;
  g_script = script;
  g_pass = 0;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait_until(wait_target);
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

TEST_CASE("wait_until_turn_swing_internal(): a real bump-and-slow-recovery survives when every tick lands") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(90, 100);

  Outcome o = run_wait(chassis, every_tick_lands, 300, 90.0);
  MESSAGE("every tick lands: returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("wait_until_turn_swing_internal(): an ordinary missed first tick makes the same survivable bump false-stuck the wait") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(90, 100);

  Outcome o = run_wait(chassis, skip_first_tick, 300, 90.0);
  MESSAGE("first tick skipped: returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("wait_until_turn_swing_internal(): a later turn in the same auton is exposed the same way, not just a fresh Drive's first turn") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 500);

  // First turn: converges normally and finishes with error near 0, the ordinary way.
  chassis.pid_turn_set(60, 100);
  Outcome first = run_wait(chassis, converge_cleanly, 100, 60.0);
  MESSAGE("first turn: returned=" << first.returned << " passes=" << first.passes << " interfered=" << first.interfered);
  REQUIRE(first.returned);
  REQUIRE_FALSE(first.interfered);

  // Second turn: skip its own first tick, then the same bump-and-slow-recovery shape.
  chassis.pid_turn_set(150, 100);
  Outcome second = run_wait(chassis, skip_first_tick, 300, 150.0);
  MESSAGE("second turn, first tick skipped: returned=" << second.returned << " passes=" << second.passes << " interfered=" << second.interfered);
  CHECK(second.returned);
  CHECK_FALSE(second.interfered);
}
