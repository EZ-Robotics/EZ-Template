// The secondary (IMU) velocity channel used to require two consecutive above-threshold readings
// to clear its timer (m_miss), mirroring a debounce the main channel also had. Both debounces are
// gone now: Drive no longer routes any of its own waits through the velocity channel at all
// (without_velocity() on every Drive wait), so the debounce's original purpose -- protecting
// Drive's waits from an isolated noisy tick -- no longer applies to anything reachable from the
// shipped library, and left in, it let m accumulate across the zero ticks of a mechanism polled
// faster than its sensor refreshes and fire a false velocity exit on something that was actually
// still moving. A bare ez::PID user of the secondary channel now gets the same single-tick-reset
// behavior as the main channel: any above-threshold sample clears m immediately, and only a
// genuinely sustained stop accumulates it.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID secondary velocity timer clears on a single above-threshold reading, no debounce") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // stationary
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms -- this call also does m += DELAY_TIME once

  // 4 more stationary passes: m = 50 (1 from the arm call + 4 here), 1 pass short of exiting.
  for (int pass = 1; pass <= 4; pass++) CHECK(pid.exit_condition() == RUNNING);

  pid.velocity_sensor_secondary_set(1.0);        // a single above-threshold reading
  CHECK(pid.exit_condition() == RUNNING);        // clears m at once -- no debounce to survive
  pid.velocity_sensor_secondary_set(0.0);         // resume stationary
  for (int pass = 1; pass <= 5; pass++) CHECK(pid.exit_condition() == RUNNING);  // fresh 5-pass countdown, m up to 50
  CHECK(pid.exit_condition() == VELOCITY_EXIT);   // m=60, exceeds 50
}

TEST_CASE("PID secondary velocity timer: a sensor jittering every other pass never accumulates a stall") {
  // Direct consequence of removing the debounce: a secondary reading that alternates above/below
  // threshold every single pass now clears m on every above-threshold tick, so it can never reach
  // exit.velocity_exit_time. This channel is off by default and, per its own doc, cannot tell a
  // real steady cruise from a stall anyway -- a caller relying on it to catch this shape needs a
  // different backstop (Drive's own StuckWatch/SingleStuckWatch, for example, which do not use
  // this channel at all).
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms

  bool jitter = false;
  for (int pass = 0; pass < 200; pass++) {
    jitter = !jitter;
    pid.velocity_sensor_secondary_set(jitter ? 1.0 : 0.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID secondary velocity timer still clears on sustained above-threshold readings") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms, m = 10

  for (int pass = 1; pass <= 3; pass++) CHECK(pid.exit_condition() == RUNNING);  // m = 40

  pid.velocity_sensor_secondary_set(1.0);
  CHECK(pid.exit_condition() == RUNNING);  // cleared at once

  pid.velocity_sensor_secondary_set(0.0);
  for (int pass = 1; pass <= 5; pass++) CHECK(pid.exit_condition() == RUNNING);  // fresh 5-pass countdown
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}
