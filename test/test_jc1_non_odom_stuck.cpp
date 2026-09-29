// DRIVE/TURN/SWING pid_wait() and wait_until_drive()/wait_until_turn_swing_internal() had no progress backstop
// at all (JC-1 in WAIT_BEHAVIOR_SPEC.md, confirmed by Step 3 finding #1/#15): only velocity and mA exits, both
// defeated by a sustained disturbance that never reads as fully "stopped" (so velocity never fires) and never
// draws over current (so mA never fires) -- a continuous spin, a defender holding the robot, or ordinary sensor
// jitter under contact.  These tests script that shape directly on each PID's own error/derivative, the same way
// test_pp_wait_stuck.cpp scripts xyPID/current_a_odomPID, and check the new SingleStuckWatch backstop catches it
// without disturbing a healthy, steadily-closing motion.
#include <functional>

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

// Every script below drives its PID(s) through compute_error(), not a direct `.error =`/`.derivative =`
// write: PID::exit_condition()'s small/big exit timers only credit `error` when a real compute() call
// has landed since the last check (see PID.cpp), the same real signal ez_auto_task provides on real
// hardware. compute_error(err, current)'s own derivative = current - prev_current, so each script below
// picks a `current` sequence whose deltas reproduce the exact derivative sequence the old direct writes
// used, keeping every pass count and assertion in this file unchanged. This file's on_delay() already
// runs once per simulated DELAY_TIME tick (bumping auto_task_passes), exactly standing in for a real
// ez_auto_task pass -- the right place for the real compute this now performs.

// Closes steadily to 0 -- a normal, healthy settle.  Small enough steps that it does NOT clear a full step
// (1 in default small_error) every single pass, so a real regression test for "does this false-flag a slow but
// genuinely progressing motion" too, not just a trivially-fast one.
void healthy_close(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  // Feeding `current` the same closing value as `error` reproduces the old derivative exactly: their
  // deltas are identical (both -0.15/pass while e > 0, then both flat), without needing to track
  // separate running state.
  c.leftPID.compute_error(e, e);
  c.rightPID.compute_error(e, e);
}

// Pinned from the start: error frozen well outside small/big windows, derivative jitters just above
// velocity_zero_main (0.05) every other pass so the velocity exit's own accumulator can never build up --
// exactly the noisy-contact shape Step 3 finding #15 describes, not a literal silent stall.
void pinned_jitter(Drive& c, int n) {
  double cur = (n % 2 == 0) ? 0.1 : -0.1;  // alternating +-0.1 -> a +-0.2 swing in derivative each tick
  c.leftPID.compute_error(24.0, cur);
  c.rightPID.compute_error(24.0, cur);
}

void healthy_turn(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.compute_error(e, e);
}

void pinned_turn_jitter(Drive& c, int n) {
  double cur = (n % 2 == 0) ? 0.15 : -0.15;  // +-0.3 swing, same reasoning as pinned_jitter above
  c.turnPID.compute_error(60.0, cur);
}

void healthy_swing(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.compute_error(e, e);
}

void pinned_swing_jitter(Drive& c, int n) {
  double cur = (n % 2 == 0) ? 0.15 : -0.15;
  c.swingPID.compute_error(45.0, cur);
}

// A raw sensor reading that's bit-identical to itself every single poll, because the robot is
// genuinely, fully stalled -- not a refresh artifact. PID::exit_condition()'s own velocity exit
// can no longer tell this apart from a stale re-read of an unrefreshed sensor (see test_pid.cpp),
// so it never fires from k alone here. This backstop -- a wall-clock fallback that doesn't read
// the sensor at all -- is what has to catch it instead. `current` held fixed at 0 every tick gives
// derivative 0, the same bit-identical-reading shape as before, while still being a real compute()
// call each tick (a genuinely stalled sensor is still polled every pass -- it just never changes).
void pinned_frozen(Drive& c, int n) {
  c.leftPID.compute_error(24.0, 0.0);
  c.rightPID.compute_error(24.0, 0.0);
}

// The realistic shape of the frozen-sensor case above: a robot that drives normally for a while
// (real, fresh, closing readings), then hits a wall and pins there, at which point its sensor
// starts reading bit-identical every poll. What could go wrong with the fix: does the now-frozen
// stretch cost this wait anything beyond the ordinary progress-backstop window, i.e. does the fix
// turn a bounded stop into a materially slower one? It shouldn't -- SingleStuckWatch's own
// window doesn't care whether PID's own velocity exit could also have fired.
void drives_then_pins(Drive& c, int n) {
  bool pinned = n > 20;
  double e = pinned ? 14.0 : std::fmax(14.0, 24.0 - 0.5 * n);
  // cur(n) = -0.5*min(n, 20): the exact running sum the old `cur += rate` accumulation produced,
  // closed-formed so each pass' delta (this call's cur minus the previous call's) reproduces the old
  // rate (-0.5 while n<=20, 0 once pinned) without needing separate persistent state.
  double cur = -0.5 * std::fmin((double)n, 20.0);
  c.leftPID.compute_error(e, cur);
  c.rightPID.compute_error(e, cur);
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a healthy, steadily-closing motion is not falsely flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, healthy_close, 500, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() DRIVE: a sustained disturbance that jitters past velocity's threshold is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  // 3000 passes = 30 s.  Before this fix, this scenario hung the full cap (confirmed in Step 3 finding #15's
  // own sim repro); the fix should end it in well under a second past the odom StuckWatch-equivalent window.
  Outcome o = run(chassis, pinned_jitter, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("wait_until_drive(): the same sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, pinned_jitter, 3000, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("pid_wait() DRIVE: a raw sensor that never changes at all is still caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, pinned_frozen, 3000, [&] { chassis.pid_wait(); });
  MESSAGE("passes=", o.passes);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 200);
}

TEST_CASE("pid_wait() DRIVE: drives normally, then pins at a wall with a frozen sensor -- still caught promptly") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, drives_then_pins, 3000, [&] { chassis.pid_wait(); });
  MESSAGE("passes=", o.passes);
  CHECK(o.returned);
  CHECK(o.interfered);
  // Pins at pass 20; back within one progress-backstop window (500ms/50 passes) plus slack --
  // the fix must not add materially more latency than the backstop's own window already allows.
  CHECK(o.passes <= 20 + 50 + 10);
}

TEST_CASE("pid_wait() TURN: a healthy turn is not falsely flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, healthy_turn, 500, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() TURN: a sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, pinned_turn_jitter, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("wait_until_turn_swing_internal() TURN: a sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, pinned_turn_jitter, 3000, [&] { chassis.pid_wait_until(45_deg); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("pid_wait() SWING: a healthy swing is not falsely flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  Outcome o = run(chassis, healthy_swing, 500, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() SWING: a sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  Outcome o = run(chassis, pinned_swing_jitter, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}
