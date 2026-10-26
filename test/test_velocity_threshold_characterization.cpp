// velocity_zero_main's fixed default threshold (0.05 units per 10 ms tick, i.e. a literal 5 in/s floor for a drive/xy PID) is not scaled to commanded
// speed or gearing, so a real, steady cruise slower than that floor eventually reads as "stopped" once the arm fallback engages, and
// false-VELOCITY_EXITs a motion that never actually stopped.
//
// This is a PID-level characterization only: scaling the threshold automatically would require PID::exit_condition() to know the caller's commanded
// speed, which lives in Drive, not PID, and it overlaps the project's already-open issue #290 ("velocity-exit false positives", hypothesis #1).  This
// test documents/locks in the current characterization so it's understood and visible.
//
// The speed is the net movement over the last 100 ms, so a cruise under 5 units/s never arms the velocity exit by moving; it arms once 1000 ms have passed
// without it.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("PID: a steady sub-threshold cruise eventually false-VELOCITY_EXITs once the arm fallback engages") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  // A real, steady cruise just under the default 5 units/s threshold -- not noise, not a stall: the raw sensor position keeps advancing by 0.04 each
  // 10 ms pass, which is 4 units/s.
  double position = 0.0;
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 150);  // the fallback arms on pass 101 (the arm timer first passes 1000 ms), then ~5 more to exit
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    position += 0.04;
    pid.compute_error(100.0, position);
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);  // false exit: the PID never actually stopped moving
  CHECK(pass == 106);              // arms on pass 101 with the timer at 10, and the timer passes 50 on pass 106
}

TEST_CASE("PID: raising velocity_sensor_main_exit below the real cruise speed avoids the false exit (the team's own mitigation, already exposed)") {
  // Confirms the existing public setter is a real, working workaround today, for whoever picks
  // up issue #290 -- this isn't a case where a team has no lever to pull at all.
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_main_exit_set(0.02);  // team explicitly lowers the floor below their real cruise speed (2 units/s)

  double position = 0.0;
  for (int pass = 1; pass <= 150; pass++) {
    INFO("pass ", pass);
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    position += 0.04;  // 4 units/s
    pid.compute_error(100.0, position);
    CHECK(pid.exit_condition() == RUNNING);  // 4 units/s now correctly reads as "still moving"
  }
}
