// A real, corpus-confirmed pattern: a team calls pid_wait_until() for an early checkpoint on a drive,
// then calls pid_wait() right after on that same still-running motion ("until then wait"). Both calls
// share the same leftPID/rightPID objects.
//
// exit_conditions.cpp documents SingleStuckWatch (the DRIVE/TURN/SWING progress backstop) as "the same
// progress backstop as StuckWatch, but for a single PID with no odometry and no path index" (its own
// class comment), and StuckWatch's own comment spells out the intended shape of the startup grace
// period it shares: "a second wait on the same motion doesn't get the allowance again once the robot
// has moved." StuckWatch keeps that promise because it seeds "have we moved" from real odometry
// (distance/heading travelled since the motion's true start, tracked independently of which wait
// function is currently polling it). SingleStuckWatch has no such motion-wide memory: its own "moved_"
// flag lives entirely inside the one instance a wait function constructs on entry, seeded to false
// every time regardless of how far the robot has actually already travelled earlier in the very same
// motion. So a DRIVE/TURN/SWING motion that drove normally for a while and then got pinned -- a
// robot that has demonstrably already moved, by SingleStuckWatch's own class comment's own logic
// should not need the startup allowance again -- still gets the full fresh 1000ms allowance on every
// later wait call chained onto that motion, unlike the odom-backed StuckWatch it claims parity with.
//
// This matters at shipped defaults, with no non-default setter: a robot that drives normally for the
// first part of a motion and is then genuinely pinned (a wall, a defender) part-way through pays the
// backstop's full startup allowance again on every wait call a team chains onto that same motion after
// the pin -- "until then wait" being the ordinary, corpus-confirmed shape of exactly that chaining.
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

// Drives normally toward the 24in target for the first 20 passes (closing steadily, well past a
// single "step" of real progress -- SingleStuckWatch's own moved_ threshold), then pins hard at 14in
// for the rest of the test: error frozen well outside small/big windows, derivative jittering just
// above velocity_zero_main (0.05) every other pass so the velocity exit's own accumulator can never
// build up. Same shape as an existing regression test's "drives, then pins at a wall" script, just
// long enough before the pin that SingleStuckWatch has clearly already registered real movement.
//
// Also advances the fake drive encoders in step with the scripted error, the same as a real robot's
// would: on real hardware, compute_error() is always fed FROM drive_sensor_left()/right(), so the two
// never drift apart the way they would if only PID.error were written here. This matters for a fix
// that judges "has this motion already moved" from real sensor travel since the motion's own start
// (l_start/r_start) rather than from this one wait call's own view of PID.error.
void drives_then_pins(Drive& c, int n) {
  bool pinned = n > 20;
  double e = pinned ? 14.0 : std::fmax(14.0, 24.0 - 0.5 * n);
  double jitter = (n % 2 == 0) ? 0.1 : -0.1;
  double cur = pinned ? jitter : -0.5 * std::fmin((double)n, 20.0);
  c.leftPID.compute_error(e, cur);
  c.rightPID.compute_error(e, cur);
  std::int32_t ticks = (std::int32_t)((24.0 - e) * c.drive_tick_per_inch());
  c.left_motors[0].fake().position = ticks;
  c.right_motors[0].fake().position = ticks;
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  drives_then_pins(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("chained wait calls on a drive that already moved before pinning each pay a fresh startup allowance, instead of remembering it already moved") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  drives_then_pins(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;

  // First call: an early checkpoint well short of the real 24in target, so the settled-exemption
  // does not apply and a genuine stall here must report interfered=true. The robot drives normally
  // through pass 20, then pins -- this call's own SingleStuckWatch is constructed at motion start
  // (pass 0), before the pin, so it correctly sees the real early movement and should not need its
  // startup allowance at all by the time the pin happens.
  test_stub::g_clock.delay_calls_until_stop = 3000;
  bool returned_1 = true;
  try {
    chassis.pid_wait_until(12.0);
  } catch (test_stub::StopLoop&) {
    returned_1 = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  int passes_after_first = g_pass;
  bool interfered_after_first = chassis.interfered;

  // Second call, immediately after, on the exact same still-running motion (no new pid_*_set() in
  // between) -- the real "until then wait" pattern TEAM_CORPUS.md describes. The robot already
  // demonstrably moved (passes 0-20, before either wait call even started) and has been pinned,
  // unresolved, ever since: a backstop that genuinely remembered the motion had already moved would
  // need only its own window (~500ms / ~50 passes) to reconfirm the same, still-unresolved stall --
  // not another full startup allowance on top of it.
  test_stub::g_clock.delay_calls_until_stop = 3000;
  bool returned_2 = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned_2 = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  int passes_in_second_call = g_pass - passes_after_first;

  MESSAGE("passes_after_first=" << passes_after_first << " passes_in_second_call=" << passes_in_second_call
                                 << " interfered_after_first=" << interfered_after_first
                                 << " interfered_after_second=" << chassis.interfered);

  CHECK(returned_1);
  CHECK(returned_2);
  CHECK(interfered_after_first);
  CHECK(chassis.interfered);

  // The defect: the second call's SingleStuckWatch is a fresh instance with moved_ seeded false, so
  // it re-grants the full ~1000ms startup allowance even though the robot demonstrably already moved
  // well before either wait call started -- the exact case StuckWatch's own documented contract says
  // should skip the allowance on a later wait. A backstop honoring that contract would need at most
  // roughly one window's worth of passes (~50, the default 500ms velocity_exit_time) to reconfirm the
  // same still-unresolved pin, not the ~150-pass (1000ms allowance + 500ms window) bound a truly
  // first-ever wait on a motion needs. This fails against the current code, which re-grants the full
  // allowance every time regardless of the motion's real history.
  CHECK(passes_in_second_call < 80);
}
