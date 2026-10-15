// The secondary-velocity-sensor timer (m) counts real elapsed milliseconds, not one flat credit per call, so a bare PID polled at a fast and at a slow
// cadence gives up after about the same real time.  The main sensor moves at 100 units per second in these runs, so the main channel never stops and the
// velocity exit arms once the first 100 ms speed window has seen it move.
#include <cstdlib>

#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {

// Holds the secondary sensor stationary and polls at a fixed real millisecond cadence, and returns the real elapsed ms at which VELOCITY_EXIT fires.
int secondary_exit_ms(int poll_period_ms, int velocity_exit_time) {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;

  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, velocity_exit_time, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // stationary throughout
  pid.compute_error(10.0, 0.0);
  CHECK(pid.exit_condition() == RUNNING);

  for (int t = poll_period_ms; t <= 6000; t += poll_period_ms) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10.0, 0.1 * t);  // moving at 100 units per second, so only the secondary sensor reads as stopped
    if (pid.exit_condition() == VELOCITY_EXIT) return t;
  }
  return -1;
}

}  // namespace

TEST_CASE("bare PID: the secondary-velocity-sensor timer credits real elapsed wall-clock time, matching across poll cadences") {
  // A flat per-call credit makes a fast cadence exit far sooner than a slow cadence for the same
  // configured velocity_exit_time; a real wall-clock credit converges on (roughly) the same real
  // elapsed time regardless of cadence.
  int fast = secondary_exit_ms(1, 500);
  int slow = secondary_exit_ms(50, 500);
  REQUIRE(fast > 0);
  REQUIRE(slow > 0);
  CHECK(fast >= 480);
  CHECK(fast <= 720);  // the 100 ms speed window has to see the robot move before the velocity exit arms
  CHECK(slow >= 480);
  CHECK(slow <= 720);
  CHECK(std::abs(fast - slow) <= 60);
}
