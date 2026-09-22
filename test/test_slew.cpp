// Forward slew ramp, 3 in / min 70 / max 127: outputs at sensor 0, 1, 2
// are 70, 89, 108; at -5 the output is still >= 70 (clamped); initialize
// with target == current leaves the slew disabled and output at max.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "drive_test_access.hpp"

using namespace ez;

TEST_CASE("slew ramps 70, 89, 108 over sensor 0, 1, 2") {
  slew s(3.0, 70);
  s.initialize(true, 127, 100, 0);  // enabled, max 127, target 100, current 0

  CHECK(s.iterate(0) == doctest::Approx(70));
  CHECK(s.iterate(1) == doctest::Approx(89));
  CHECK(s.iterate(2) == doctest::Approx(108));
}

TEST_CASE("slew output stays clamped to min_speed behind the start") {
  slew s(3.0, 70);
  s.initialize(true, 127, 100, 0);

  CHECK(s.iterate(-5) == doctest::Approx(70));
}

TEST_CASE("slew initialize with target == current disables slew and outputs max") {
  slew s(3.0, 70);
  s.initialize(true, 127, 50, 50);  // target == current

  CHECK_FALSE(s.enabled());
  CHECK(s.output() == doctest::Approx(127));
}

TEST_CASE("slew constants are stored as magnitudes") {
  slew s(-3.0, -70);

  CHECK(s.constants_get().min_speed == 70);
  CHECK(s.constants_get().distance_to_travel == doctest::Approx(3.0));

  s.constants_set(-7.5, -80);
  CHECK(s.constants_get().min_speed == 80);
  CHECK(s.constants_get().distance_to_travel == doctest::Approx(7.5));
}

TEST_CASE("slew with a negative min_speed ramps the same as a positive one") {
  slew negative(3.0, -70);
  slew positive(3.0, 70);
  negative.initialize(true, 127, 100, 0);
  positive.initialize(true, 127, 100, 0);

  for (double sensor : {0.0, 1.0, 2.0, -5.0}) {
    CHECK(negative.iterate(sensor) == doctest::Approx(positive.iterate(sensor)));
    CHECK(negative.iterate(sensor) > 0);
  }
}

TEST_CASE("slew with a negative distance ramps the same as a positive one") {
  slew negative(-3.0, 70);
  slew positive(3.0, 70);
  negative.initialize(true, 127, 100, 0);
  positive.initialize(true, 127, 100, 0);

  CHECK(negative.enabled());
  for (double sensor : {0.0, 1.0, 2.0}) {
    CHECK(negative.iterate(sensor) == doctest::Approx(positive.iterate(sensor)));
  }
}

TEST_CASE("a turn with a negative slew min speed starts toward its target") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
  chassis.pid_turn_constants_set(5.0, 0.0, 0.0, 0.0);
  chassis.slew_turn_set(true);
  chassis.slew_turn_constants_set(3_deg, -70);  // the sign is a mistake, the turn should still go the right way

  chassis.imu->fake_rotation = 0.0;
  chassis.pid_turn_set(90.0, 110);
  DriveTestAccess::turn_pid_task(chassis);

  // A clockwise turn drives the left side forward and starts at the slew min speed
  double left_out = chassis.left_motors[0].fake().voltage / (12000.0 / 127.0);
  CHECK(left_out == doctest::Approx(70).epsilon(0.02));
}

TEST_CASE("slew returns max speed on the pass it finishes") {
  slew s(3.0, 70);
  s.initialize(true, 127, 100, 0);

  CHECK(s.iterate(2) == doctest::Approx(108));  // still ramping
  CHECK(s.enabled());

  // The sensor reaches the end of the ramp. This is the pass that turns slew off, and it has to return max speed
  // rather than the 108 from the pass before.
  CHECK(s.iterate(3) == doctest::Approx(127));
  CHECK_FALSE(s.enabled());
  CHECK(s.output() == doctest::Approx(127));

  // Passes after that keep returning max speed
  CHECK(s.iterate(4) == doctest::Approx(127));
}

TEST_CASE("slew returns max speed on the pass it finishes when the sensor jumps past the end of the ramp") {
  slew s(3.0, 70);
  s.initialize(true, 100, 100, 0);  // max 100, below full speed

  CHECK(s.iterate(1) == doctest::Approx(89));
  CHECK(s.iterate(10) == doctest::Approx(100));
  CHECK_FALSE(s.enabled());
}

TEST_CASE("slew going backwards returns max speed on the pass it finishes") {
  slew s(3.0, 70);
  s.initialize(true, 127, -100, 0);  // target is behind the start

  CHECK(s.iterate(-2) == doctest::Approx(108));
  CHECK(s.iterate(-3) == doctest::Approx(127));
  CHECK_FALSE(s.enabled());
}

TEST_CASE("slew with a distance of 0 returns max speed on its first iterate") {
  slew s(0.0, 80);
  s.initialize(true, 127, 100, 0);  // the ramp is over before it starts

  CHECK(s.iterate(0) == doctest::Approx(127));
  CHECK_FALSE(s.enabled());
  CHECK(s.output() == doctest::Approx(127));
}

TEST_CASE("slew that finishes on its first iterate does not return the previous motion's output") {
  slew s(3.0, 70);
  s.initialize(true, 127, 100, 0);
  CHECK(s.iterate(1) == doctest::Approx(89));  // leaves 89 as the last output

  s.constants_set(0.0, 70);  // the next motion has no ramp
  s.initialize(true, 127, 100, 0);
  CHECK(s.iterate(0) == doctest::Approx(127));
}
