// Issue #530: Channel::made()'s shove/overshoot leniency (see the class comment on `Channel` in
// exit_conditions.cpp) is granted through a one-shot `rebounded` latch that fires once per Channel's
// entire lifetime, never resets, and permanently blocks any later shove from raising `low` to track
// its own peak. A single shove/overshoot recovers fine (test_stuck_watch_credits_recovery_after_a_shove.cpp
// covers that), but a SECOND, later, independent shove in the same wait gets none of that treatment:
// `low` stays pinned wherever the first shove's recovery left it, so recovering from the second shove
// requires crossing its entire distance back down to `old_low - step`, not just one step past the new
// peak -- and if that full-distance crossing takes longer than the stuck-detection window, StuckWatch
// (by way of SingleStuckWatch, DRIVE's flavor of it, exercised here) reports "stuck" even though the
// robot has been recovering continuously and fast the whole time.
//
// Two cases:
//  1. Two genuinely separate shove episodes in one wait -- a shove, a full recovery that goes on to
//     set a brand new best low well past where the shove started, and then a second, later shove --
//     must not false-flag the second shove's fast recovery as stuck. This is the issue's own repro.
//  2. A single ongoing disturbance oscillating at a fixed amplitude right at the edge -- never
//     achieving real headway past where it started -- must still eventually be caught as stuck. This
//     is the original one-shot design's whole purpose (per Channel's class comment: "a robot being
//     spun or shoved back and forth crosses ... its target over and over, and only the first excursion
//     gets credited"), and a fix for (1) that just re-arms the latch unconditionally on every recovery
//     would silently break it, letting a robot being continuously jittered escape detection forever.
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
std::function<void(Drive&, int)> g_script;

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_script(*g_chassis, g_pass);
}

void set_error(Drive& c, double error, double derivative) {
  c.leftPID.error = error;
  c.leftPID.derivative = derivative;
  c.rightPID.error = error;
  c.rightPID.derivative = derivative;
}

// Steady close (30 -> 10 over 100 passes), a first shove (10 -> 20 over 10 passes), a full recovery
// that keeps going all the way down to a brand new best low well past where the shove started
// (20 -> 2.75 over 115 passes), a second, later shove of similar magnitude (2.75 -> 12.75 over 10
// passes), and then a FAST recovery (12.75 -> 0, at 0.2/pass, i.e. 20in/s in the units the rest of
// this codebase's PIDs use -- inside the issue's own "recovering at up to 25 in/s" range for a
// second shove that must not false-flag).
void two_separate_shoves(Drive& c, int n) {
  double error, derivative;
  if (n <= 100) {
    error = 30.0 - 0.2 * n;
    derivative = -0.2;
  } else if (n <= 110) {
    error = 10.0 + 1.0 * (n - 100);
    derivative = 1.0;
  } else if (n <= 225) {
    error = 20.0 - 0.15 * (n - 110);
    derivative = -0.15;
  } else if (n <= 235) {
    error = 2.75 + 1.0 * (n - 225);
    derivative = 1.0;
  } else {
    error = std::fmax(0.0, 12.75 - 0.2 * (n - 235));
    derivative = error > 0.0 ? -0.2 : 0.0;
  }
  set_error(c, error, derivative);
}

// Steady close (30 -> 10 over 100 passes), then a fixed-amplitude triangle wave forever after,
// oscillating the error between 9.5 and 11.5 (a shove/overshoot-sized swing relative to `step` = 1,
// the DRIVE small_error default) with a 20-pass period -- a mechanism being continuously jittered
// right at the edge, never making any real headway past where the oscillation started.
void continuous_oscillation(Drive& c, int n) {
  double error, derivative;
  if (n <= 100) {
    error = 30.0 - 0.2 * n;
    derivative = -0.2;
  } else {
    int m = n - 100;
    int cyc = m % 20;
    double t, slope;
    if (cyc <= 10) {
      t = -1.0 + 0.2 * cyc;
      slope = 0.2;
    } else {
      t = 1.0 - 0.2 * (cyc - 10);
      slope = -0.2;
    }
    error = 10.5 + t;
    derivative = slope;
  }
  set_error(c, error, derivative);
}
}  // namespace

TEST_CASE("pid_wait credits a second, separate shove's fast recovery as progress, not just the first") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  g_script = two_separate_shoves;
  two_separate_shoves(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 450;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  // Two separate shoves, each recovered from continuously and fast, must not be reported as stuck --
  // even though only the FIRST one gets the one-shot latch's original, unfixed leniency.
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(chassis.leftPID.error) < 1.0);
}

TEST_CASE("pid_wait still reports a continuously oscillating mechanism as stuck, not re-armed every poll") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  g_script = continuous_oscillation;
  continuous_oscillation(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 300;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  // A mechanism oscillating at a fixed amplitude, never getting anywhere, must still be caught as
  // stuck -- proving fresh leniency for a later, separate shove didn't come at the cost of letting the
  // SAME ongoing disturbance re-arm itself on every poll.
  CHECK(chassis.interfered);
  CHECK(std::fabs(chassis.leftPID.error) > 5.0);
}
