// wait_until_turn_swing_internal() constructed its SingleStuckWatch (the no-progress backstop)
// immediately on entry, with no delay first -- unlike pid_wait() and wait_until_drive(), both of
// which let the PID run at least 1 iteration before building their own watch, specifically so it
// seeds from a real, computed error instead of a leftover/zero one.
//
// On a brand-new Drive object, turnPID/swingPID's error is still whatever it was default-
// constructed to (0.0 -- PID::motion_reset(), which turn_set_internal()/swing_set_internal() do
// call, never touches `error`) until the auto task actually runs a pass and computes a real one.
// Seeding the watch from that 0 and then jumping to the real first-computed error on the very
// next pass consumes Channel's one-shot rebound allowance (see exit_conditions.cpp's Channel
// comment) on that artifact instead of a real disturbance -- so this wait's first GENUINE
// disturbance then gets zero grace and must fully recover past the pre-disturbance low within one
// progress-backstop window, or false-stucks.
//
// The tests below script a fresh Drive's very first turn/swing through exactly that shape: a
// normal close, then one real shove-and-slow-recovery disturbance that takes LONGER than one
// window to beat the pre-shove low by a full step. Before this fix, the seed-from-zero artifact
// already spent the rebound, so the real disturbance gets no grace and the wait false-stucks.
// After this fix, the watch seeds from the real first reading, the rebound is still available for
// the real disturbance, and the wait survives it and finishes normally.
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

// n counts on_delay invocations, starting at 1 for the first one (which -- on the fixed code --
// is the call inside wait_until_turn_swing_internal()'s own new seeding delay, i.e. the first
// time this PID's error is ever computed for real; on the base/unfixed code, that same first call
// instead lands after the watch has already been seeded from the leftover 0).
//
// n<=1: the real first reading once the PID starts computing (60 deg error) -- on fixed code this
//       becomes the watch's actual seed; on base code it's compared against the watch's stale 0
//       seed, an artificial ~60 degree "overshoot" that spends the one-shot rebound on nothing.
// n<=12: closes normally, 4 degrees per pass (more than turn's small_error/step of 3, so each
//        pass credits a new low) down to 16 degrees -- an ordinary, undisturbed turn.
// n==13: a REAL shove back up to 40 degrees (e.g. a bump) -- more than a step worse than the
//        16-degree low, a genuine disturbance this Channel's one-shot rebound exists to tolerate.
// n>13: a SLOW recovery down from 40, 0.3 degrees/pass -- slow enough that beating the PRE-SHOVE
//       low (16, so needs to reach under 16-3=13) takes far longer than one progress-backstop
//       window (500ms/50 passes default) if the rebound wasn't granted, but is comfortably inside
//       one window if it was (only needs to beat the SHOVE's own low of 40, i.e. reach under 37,
//       which takes ~10 passes).
double turn_disturbance_error(int n) {
  if (n <= 1) return 60.0;
  if (n <= 12) return 60.0 - 4.0 * (n - 1);  // n=2:56 ... n=12:16
  if (n == 13) return 40.0;
  return std::fmax(40.0 - 0.3 * (n - 13), 0.0);
}

void turn_disturbance_script(Drive& c, int n) {
  c.turnPID.error = turn_disturbance_error(n);
}
void swing_disturbance_script(Drive& c, int n) {
  c.swingPID.error = turn_disturbance_error(n);
}

Drive* g_chassis = nullptr;
void (*g_script)(Drive&, int) = nullptr;
int g_pass = 0;

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

// Does NOT pre-seed error before starting the wait -- on a genuinely fresh Drive, turnPID/
// swingPID.error is whatever it was left at (0.0), and the first time the script sets it for real
// is via on_delay, exactly matching what a real Drive experiences.
Outcome run_first_motion(Drive& chassis, void (*script)(Drive&, int), int max_passes, double wait_target) {
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

TEST_CASE("wait_until_turn_swing_internal(): a fresh Drive's very first turn survives a real shove-and-slow-recovery disturbance") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(90, 100);

  Outcome o = run_first_motion(chassis, turn_disturbance_script, 300, 90.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("wait_until_turn_swing_internal(): a fresh Drive's very first swing survives a real shove-and-slow-recovery disturbance") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(90, 3.0, 250, 7.0, 500, 500);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);

  Outcome o = run_first_motion(chassis, swing_disturbance_script, 300, 45.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

// Regression guard: a genuinely stuck fresh-Drive first turn must still be caught -- this fix
// must not turn the watch into a no-op.
TEST_CASE("wait_until_turn_swing_internal(): a fresh Drive's very first turn is still caught when genuinely pinned") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);

  auto pinned = [](Drive& c, int n) {
    c.turnPID.error = 90.0;
    c.turnPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
  };
  Outcome o = run_first_motion(chassis, pinned, 3000, 90.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

// What could go wrong with this fix: putting the new delay in the wrong place (after, instead of
// before, g_error/g_sgn are computed) would let a short wait_until() target already be behind the
// robot by the time the crossed-check's starting sign is read, latching the wrong sign and making
// the wait require a second flip that may never come instead of ending on the clean first
// crossing. This scripts exactly that shape on a short (2 degree) waypoint: the fake IMU jumps
// past the target during the new delay, on the very first on_delay call.
TEST_CASE("wait_until_turn_swing_internal(): a short waypoint already passed by the time the seeding delay ends still ends via a clean crossing") {
  Drive chassis = make_fresh_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_set(2.0, 100);  // a short turn: 0 -> 2 degrees

  auto jump_past_target_once = [](Drive& c, int n) {
    if (n == 1) DriveTestAccess::all_imus(c)[0]->fake_rotation = 5.0;  // already past the 2-degree target
  };
  Outcome o = run_first_motion(chassis, jump_past_target_once, 50, 2.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
  // A clean first-pass crossing, not a fall-through into exit-condition/stuck-watch polling.
  CHECK(o.passes <= 2);
}
