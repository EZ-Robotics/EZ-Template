// PID::exit_condition() calls timers_reset() before returning VELOCITY_EXIT, exactly like its SMALL_EXIT and BIG_EXIT siblings.  That reset is what lets a
// caller that keeps polling the same PID after a VELOCITY_EXIT -- with no setter call in between -- get a full fresh velocity_exit_time countdown instead
// of re-firing VELOCITY_EXIT immediately on the very next call.  That's a real, reachable path through the shipped API: Drive::pid_wait_until() ending on
// VELOCITY_EXIT (a documented use -- e.g. start an intake partway through a drive), immediately followed by Drive::pid_wait() to finish the same motion.
// No other test pinned the reset inside the VELOCITY_EXIT branches, so deleting it survived the host suite.  This test locks in the relied-upon behavior.
//
// The velocity exit only runs once the robot has moved (a 100 ms window read more than 5 units/s) or 1000 ms have passed, and timers_reset() clears that
// arming, so the fresh countdown also needs the new motion to start moving before it can fire.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {
// One 10 ms pass of a bare PID loop: the clock moves on, compute_error() sees the new reading, then exit_condition() polls.
exit_output run_pass(PID& pid, double current) {
  test_stub::g_clock.now_ms += util::DELAY_TIME;
  pid.compute_error(10.0, current);
  return pid.exit_condition();
}
}  // namespace

TEST_CASE("PID exit_condition automatically re-arms after a VELOCITY_EXIT with no explicit timers_reset() call") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  // 10 passes at 100 units/s: the speed window closes on pass 10 and arms
  double cur = 0.0;
  int pass = 0;
  for (; pass < 10; pass++) {
    cur += 1.0;
    CHECK(run_pass(pid, cur) == RUNNING);
  }

  // Then still.  Pass 20 closes the first still window (timer 10) and the timer passes 50 on pass 25.
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 40);
    result = run_pass(pid, cur);
  }
  CHECK(pass == 25);
  CHECK(result == VELOCITY_EXIT);  // internally calls timers_reset()

  // No explicit pid.timers_reset() call here -- this is the real reachable path (a caller
  // that keeps polling exit_condition() on the same PID after a VELOCITY_EXIT, e.g.
  // pid_wait_until() -> pid_wait() on the same motion). The very next call must NOT
  // immediately re-fire VELOCITY_EXIT; it needs a fresh countdown, same as after an explicit
  // timers_reset().
  pass++;
  CHECK(run_pass(pid, cur) == RUNNING);

  // And the fresh countdown behaves like a real motion start: the new motion moves for 10 passes (arming on the pass-30 window), stops after pass 36,
  // and the first window that reads it as still closes on pass 50, so the timer passes 50 on pass 55.
  for (int moving = 0; moving < 10; moving++) {
    pass++;
    cur += 1.0;
    CHECK(run_pass(pid, cur) == RUNNING);
  }
  result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 70);
    result = run_pass(pid, cur);
  }
  CHECK(pass == 55);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID secondary velocity channel's VELOCITY_EXIT also re-arms automatically, same as the main channel") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);

  // keep the main channel from firing so only the secondary channel is exercised: the main sensor moves 100 units/s throughout
  double cur = 0.0;
  for (int pass = 0; pass < 10; pass++) {
    cur += 1.0;
    CHECK(run_pass(pid, cur) == RUNNING);  // arms on pass 10; the secondary reading was never taken, so it never counts
  }

  pid.velocity_sensor_secondary_set(0.0);
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 50);
    cur += 1.0;
    result = run_pass(pid, cur);
  }
  CHECK(pass == 6);  // the secondary timer holds 10 on the first pass and passes 50 on the 6th
  CHECK(result == VELOCITY_EXIT);

  cur += 1.0;
  CHECK(run_pass(pid, cur) == RUNNING);  // re-armed with no explicit reset, same as the main channel
}
