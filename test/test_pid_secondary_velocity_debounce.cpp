// The secondary (IMU) velocity channel's own noisy-tick debounce, m_miss, was added alongside the
// main channel's k_miss with the stated intent of giving both channels the same behavior, but
// m_miss itself was never actually wired into the secondary channel's own accumulator: a single
// above-threshold secondary reading has always reset m straight to 0, where the main channel needs
// two consecutive ones. This file pins the intended, symmetric behavior. derivative=1.0 arms the
// main channel (unaffected by its own freshness check), but cur is never touched by these tests,
// so after the first check the main channel is stale-suppressed -- it can't fire here regardless,
// which is what isolates these tests to the secondary channel.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID secondary velocity timer needs two consecutive moving passes to clear, not one") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // stationary
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms -- this call also does m += DELAY_TIME once

  // 4 more stationary passes: m = 50 (1 from the arm call + 4 here), 1 pass short of exiting.
  for (int pass = 1; pass <= 4; pass++) CHECK(pid.exit_condition() == RUNNING);

  pid.velocity_sensor_secondary_set(1.0);  // a single above-threshold reading
  CHECK(pid.exit_condition() == RUNNING);  // must not clear m on its own

  pid.velocity_sensor_secondary_set(0.0);  // resume stationary
  CHECK(pid.exit_condition() == VELOCITY_EXIT);  // m was preserved at 50; one more stalled pass fires it
}

TEST_CASE("PID secondary velocity timer is not defeated by an isolated single-tick blip") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms

  // Genuinely stuck (secondary reading), but with an isolated noisy reading every other pass --
  // the same shape the main channel's own jitter test uses.
  bool jitter = false;
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 200);  // don't hang the suite if this regresses
    pid.velocity_sensor_secondary_set(jitter ? 1.0 : 0.0);
    jitter = !jitter;
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID secondary velocity timer still clears on two consecutive above-threshold readings") {
  // The other half of the go-wrong risk for this fix: real, sustained acceleration (not an
  // isolated blip) must still clear m normally, or a caller relying on the secondary channel to
  // resume after a real disturbance would see it wrongly stay primed toward re-firing.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms, m = 10

  for (int pass = 1; pass <= 3; pass++) CHECK(pid.exit_condition() == RUNNING);  // m = 40

  pid.velocity_sensor_secondary_set(1.0);
  CHECK(pid.exit_condition() == RUNNING);  // 1st above-threshold: m not cleared yet
  pid.velocity_sensor_secondary_set(1.0);
  CHECK(pid.exit_condition() == RUNNING);  // 2nd consecutive: NOW it clears

  pid.velocity_sensor_secondary_set(0.0);
  for (int pass = 1; pass <= 5; pass++) CHECK(pid.exit_condition() == RUNNING);  // fresh 5-pass countdown
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}
