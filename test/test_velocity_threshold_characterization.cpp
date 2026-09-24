// Step 3 finding #12 (high, verifier-narrowed to a specific low-speed/low-noise band):
// velocity_zero_main's fixed default threshold (0.05 units/10ms, i.e. a literal 5in/s floor
// for a drive/xy PID) is not scaled to commanded speed or gearing, so a real, steady cruise
// slower than that floor eventually reads as "stopped" once the arm fallback engages, and
// false-VELOCITY_EXITs a motion that never actually stopped.
//
// NOT fixed here: this is a PID-level characterization only. Scaling the threshold
// automatically would require PID::exit_condition() to know the caller's commanded speed,
// which lives in Drive, not PID -- a materially bigger change than "smallest fix", and this
// finding duplicates the project's own already-open issue #290 ("velocity-exit false
// positives", hypothesis #1), which predates this audit. A drive-by fix here risks conflicting
// with whatever plan already exists for #290 -- flagging for Jess rather than picking silently.
// This test documents/locks in the current characterization so it's understood and visible.
// (See also: the verifier's own sim repro found the false exit only manifests in a fairly
// narrow low-speed band, and disappears under realistic sensor noise -- the mechanism below is
// real, but its real-world frequency is narrower than "high severity" might suggest on its own.)
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID: a steady sub-threshold cruise eventually false-VELOCITY_EXITs once the arm fallback engages") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  // A real, steady cruise just under the default 0.05 threshold -- not noise, not a stall, and a
  // genuinely fresh raw reading every tick (the raw sensor position keeps advancing by 0.04 each
  // pass, so it's never bit-identical to the tick before).
  double position = 0.0;
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 150);  // 100 passes (1000ms) to arm via the fallback, then ~6 more to exit
    position += 0.04;
    pid.compute_error(100.0, position);
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);  // false exit: the PID never actually stopped moving
}

TEST_CASE("PID: raising velocity_sensor_main_exit below the real cruise speed avoids the false exit (the team's own mitigation, already exposed)") {
  // Confirms the existing public setter is a real, working workaround today, for whoever picks
  // up issue #290 -- this isn't a case where a team has no lever to pull at all.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_main_exit_set(0.02);  // team explicitly lowers the floor below their real cruise speed
  pid.error = 100.0;
  pid.derivative = 0.04;

  for (int pass = 1; pass <= 150; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);  // 0.04 now correctly reads as "still moving"
  }
}
