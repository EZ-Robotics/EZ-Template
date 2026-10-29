// Promise 1: nothing from one motion carries into the next. A new motion's speed is measured from that motion's own readings.
#include "../doctest.h"
#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

TEST_CASE("Promise 1: a lift that stopped reads stopped as soon as its next motion starts, not the speed of the last one") {
  test_stub::reset_all();
  ez::PID lift{1, 0, 4};
  double position = 0;

  // The lift rises at 100 units per second, then stops where it is
  lift.motion_reset(position);
  for (int pass = 0; pass < 25; pass++) {
    test_stub::g_clock.now_ms += 10;
    position += 1;
    lift.compute(position);
  }

  // A new motion starts with the lift still. Every speed it reads must be stopped
  lift.target_set(position + 50);
  lift.motion_reset(position);
  for (int pass = 0; pass < 5; pass++) {
    test_stub::g_clock.now_ms += 10;
    lift.compute(position);
    CHECK(lift.speed_get() == 0);
  }
}
