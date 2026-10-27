// The secondary (IMU) velocity channel clears its timer on any single above-threshold reading, the same as the main channel: there is no debounce.  A
// debounce would let the timer accumulate across the zero readings of a mechanism polled faster than its sensor refreshes and fire a false velocity exit
// on something that was actually still moving.  A bare ez::PID user of the secondary channel gets the same single-reading reset as the main channel:
// any above-threshold sample clears m immediately, and only a genuinely sustained stop accumulates it.
//
// The secondary channel is gated by the same arming as the main one, so every case moves the main sensor 1 unit per 10 ms pass (100 units/s): the first
// 100 ms speed window closes on pass 10 and arms the velocity exit, and the main channel never reads as stopped.  m counts 10 ms per pass while the
// secondary reading is at or under its threshold.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {
// One 10 ms pass of a bare PID loop: the clock moves on, the main sensor advances one unit, then exit_condition() polls.
exit_output run_pass(PID& pid, double& current) {
  test_stub::g_clock.now_ms += util::DELAY_TIME;
  current += 1.0;
  pid.compute_error(10.0, current);
  return pid.exit_condition();
}

// Runs the passes up to and including the one that arms (pass 10) with the secondary sensor stationary, so m holds 10 afterwards.
void arm(PID& pid, double& current) {
  for (int pass = 1; pass <= 10; pass++) CHECK(run_pass(pid, current) == RUNNING);
}
}  // namespace

TEST_CASE("PID secondary velocity timer clears on a single above-threshold reading, no debounce") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // stationary
  double cur = 0.0;
  arm(pid, cur);  // arms on pass 10 -- this call also does m += 10 once

  // 4 more stationary passes: m = 50 (10 from the arming pass + 40 here), 1 pass short of exiting.
  for (int pass = 1; pass <= 4; pass++) CHECK(run_pass(pid, cur) == RUNNING);

  pid.velocity_sensor_secondary_set(1.0);                                      // a single above-threshold reading
  CHECK(run_pass(pid, cur) == RUNNING);                                        // clears m at once -- no debounce to survive
  pid.velocity_sensor_secondary_set(0.0);                                      // resume stationary
  for (int pass = 1; pass <= 5; pass++) CHECK(run_pass(pid, cur) == RUNNING);  // fresh 5-pass countdown, m up to 50
  CHECK(run_pass(pid, cur) == VELOCITY_EXIT);                                  // m=60, exceeds 50
}

TEST_CASE("PID secondary velocity timer: a sensor jittering every other pass never accumulates a stall") {
  // A secondary reading that alternates above/below threshold every single pass clears m on every above-threshold tick, so it can never reach
  // exit.velocity_exit_time. This channel is off by default and, per its own doc, cannot tell a real steady cruise from a stall anyway -- a caller
  // relying on it to catch this shape needs a different backstop.
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);
  double cur = 0.0;
  arm(pid, cur);

  bool jitter = false;
  for (int pass = 0; pass < 200; pass++) {
    jitter = !jitter;
    pid.velocity_sensor_secondary_set(jitter ? 1.0 : 0.0);
    CHECK(run_pass(pid, cur) == RUNNING);
  }
}

TEST_CASE("PID secondary velocity timer still clears on sustained above-threshold readings") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);
  double cur = 0.0;
  arm(pid, cur);  // arms, m = 10

  for (int pass = 1; pass <= 3; pass++) CHECK(run_pass(pid, cur) == RUNNING);  // m = 40

  pid.velocity_sensor_secondary_set(1.0);
  CHECK(run_pass(pid, cur) == RUNNING);  // cleared at once

  pid.velocity_sensor_secondary_set(0.0);
  for (int pass = 1; pass <= 5; pass++) CHECK(run_pass(pid, cur) == RUNNING);  // fresh 5-pass countdown
  CHECK(run_pass(pid, cur) == VELOCITY_EXIT);
}
