// A genuine, above-threshold movement that lands on the exact poll where a velocity_exit_hold_set(true)
// hold is released must still count as real movement, not get discarded by the hold's own release
// resync. velocity_exit_hold_set()'s documented contract only promises that time spent HELD counts
// neither for nor against the exit; a movement on the poll where the hold is already OFF
// (velocity_exit_hold_set(false) has already been called, and this same exit_condition() call itself
// computes held=false) is not "during the hold" by that contract, so it must count as an ordinary
// movement and reset the STOPPED timer, the same as any other poll with the same derivative jump.
//
// See also test_pid_velocity_hold_truly_freezes.cpp, which only exercises a disturbance strictly
// BEFORE the release poll, with the release poll itself reading no derivative change -- this covers
// the adjacent case where the disturbance and the release land on the same poll.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("a genuine movement landing on the same poll a velocity hold releases still resets the STOPPED timer") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 30, 0);  // velocity_exit_time=30ms only; small/big/mA off
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  pid.compute_error(10.0, 1.0);  // real movement (derivative 1.0-0.0): arms, and correctly resets k=0
  CHECK(pid.exit_condition() == RUNNING);

  test_stub::g_clock.now_ms = 10;
  pid.velocity_exit_hold_set(true);
  pid.compute_error(10.0, 1.0);  // no movement (derivative 0) -- held throughout
  CHECK(pid.exit_condition() == RUNNING);

  // The release poll ALSO carries a genuine, above-threshold disturbance (derivative 5.0-1.0=4.0),
  // landing on the exact same exit_condition() call that first sees velocity_exit_hold_set(false).
  test_stub::g_clock.now_ms = 20;
  pid.velocity_exit_hold_set(false);
  pid.compute_error(10.0, 5.0);
  CHECK(pid.exit_condition() == RUNNING);  // can't have fired yet either way

  // From here on the mechanism is genuinely, continuously stopped (derivative 0 every poll). A
  // mechanism whose last real movement was at t=20 needs a full, uninterrupted velocity_exit_time
  // (30ms) of settled dwell measured from t=20 -- recognizing the t=20 disturbance as real movement
  // and resetting k there means it must not cross velocity_exit_time until t=60: t=30 credits the
  // nominal first-call DELAY_TIME (k=10), t=40 (k=20), t=50 (k=30, not yet > 30), t=60 (k=40, fires).
  exit_output result = RUNNING;
  int t = 30;
  for (; t <= 200; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10.0, 5.0);  // cur unchanged -> derivative 0, genuinely stopped
    result = pid.exit_condition();
    if (result != RUNNING) break;
  }
  CHECK(result == VELOCITY_EXIT);
  MESSAGE("fired at t=", t, " (must be >= 60: the t=20 disturbance has to count as real movement)");
  CHECK(t >= 60);
}
