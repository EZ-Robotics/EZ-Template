// Step 3 finding #22 (medium, mutation-testing): no test anywhere exercised
// PID::exit_condition()'s internal timers_reset() side effect inside its VELOCITY_EXIT
// branches (this is spec finding F5). Deleting those two timers_reset() calls survived the
// full host suite -- confirmed a real, complete coverage gap.
//
// NOT fixed as a behavior change: this reset is not just wasted side effect to clean up. It is
// what lets a caller that keeps polling the same PID after a VELOCITY_EXIT -- with no setter
// call in between -- get a full fresh velocity_exit_time countdown instead of re-firing
// VELOCITY_EXIT immediately on the very next call. That's a real, reachable path through the
// shipped API: Drive::pid_wait_until() ending on VELOCITY_EXIT (a documented use -- e.g. start
// an intake partway through a drive), immediately followed by Drive::pid_wait() to finish the
// same motion. Removing the reset to "fix" F5's odom-discarded-exit side effect would break
// this legitimate DRIVE/TURN/SWING re-arm behavior instead. That tension is a real design
// question (does an odom caller's discarded velocity exit need different treatment than a
// raw DRIVE/TURN/SWING one that reaches the caller directly?) that belongs to whoever owns
// exit_conditions.cpp's without_velocity() wrapper, not something to resolve unilaterally at
// the PID.cpp level. This test locks in and documents the current, relied-upon behavior instead.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

namespace {
// See test_pid.cpp's dither_tick: a raw reading that's always different from the tick before --
// real sensor dither at a resting position, not a raw value repeating because the sensor hasn't
// refreshed -- so it stays a genuinely fresh sample every tick while remaining inside the stalled
// band.
void dither_tick(PID& pid, double error, bool& toggle, double base, double amplitude = 0.01) {
  toggle = !toggle;
  pid.compute_error(error, base + (toggle ? amplitude : -amplitude));
}
}  // namespace

TEST_CASE("PID exit_condition automatically re-arms after a VELOCITY_EXIT with no explicit timers_reset() call") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arm
  CHECK(pid.exit_condition() == RUNNING);
  bool toggle = false;
  for (int pass = 1; pass < 6; pass++) {
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  dither_tick(pid, 10.0, toggle, 1.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);  // internally calls timers_reset() -- this is the F5 side effect

  // No explicit pid.timers_reset() call here -- this is the real reachable path (a caller
  // that keeps polling exit_condition() on the same PID after a VELOCITY_EXIT, e.g.
  // pid_wait_until() -> pid_wait() on the same motion). The very next call must NOT
  // immediately re-fire VELOCITY_EXIT; it needs a fresh countdown, same as after an explicit
  // timers_reset().
  CHECK(pid.exit_condition() == RUNNING);

  // And the fresh countdown behaves identically to a real motion start: 1 moving tick to arm,
  // then 5 more stationary passes before it can fire again.
  pid.compute_error(10.0, 3.0);
  CHECK(pid.exit_condition() == RUNNING);
  for (int pass = 1; pass < 6; pass++) {
    dither_tick(pid, 10.0, toggle, 3.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  dither_tick(pid, 10.0, toggle, 3.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID secondary velocity channel's VELOCITY_EXIT also re-arms automatically, same as the main channel") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.error = 10.0;
  pid.derivative = 1.0;  // keep the main channel from firing so only the secondary channel is exercised
  CHECK(pid.exit_condition() == RUNNING);

  pid.velocity_sensor_secondary_set(0.0);
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 50);
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);

  CHECK(pid.exit_condition() == RUNNING);  // re-armed with no explicit reset, same as the main channel
}
