// Velocity arming looks at the sensor's net movement over a 100 ms window, not at the instantaneous change on whichever compute happens to run right
// before a poll.  A sensor that only refreshes every 3rd compute produces repeated raw values on the computes in between even while the mechanism is
// moving continuously, so arming has to see the movement across the window instead.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("velocity arming is not fooled by a compute cadence that aliases the sensor's own refresh cadence") {
  // A sensor that only refreshes every 3rd compute repeats its raw value on the computes in between, even while the mechanism is moving continuously
  // from the very first real compute. If arming missed this and fell back to the flat 1000 ms VELOCITY_ARM_FALLBACK instead, a genuine stop right
  // after this continuous-movement stretch would exit far later than the configured 300ms.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 300, 0);  // velocity_exit_time=300ms only
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  double sensor = 0.0;
  for (int t = 0; t <= 200; t += 4) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    if (t % 12 == 0) sensor += 24.0;  // sensor refreshes only every 3rd compute -- continuous real movement
    pid.compute_error(10000.0 - sensor, sensor);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // Now genuinely stop: every subsequent compute reads the same, unrefreshing value.
  bool fired = false;
  int t_exit = -1;
  for (int t = 204; t <= 700; t += 4) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10000.0 - sensor, sensor);
    if (pid.exit_condition() == VELOCITY_EXIT) {
      fired = true;
      t_exit = t;
      break;
    }
  }
  CHECK(fired);
  // The last refresh is on t=192.  The window that closes on t=200 still reads it as moving, the first fully still window closes on t=300 and the timer
  // holds 4 ms there (the loop polls every 4 ms); it passes the configured 300 ms on t=600.  If arming had missed the movement and fallen back to
  // the flat 1000 ms fallback instead, this would land past t=1000 -- comfortably distinguishable.
  CHECK(t_exit <= 604);
}
