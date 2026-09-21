// Forward slew ramp, 3 in / min 70 / max 127: outputs at sensor 0, 1, 2
// are 70, 89, 108; at -5 the output is still >= 70 (clamped); initialize
// with target == current leaves the slew disabled and output at max.
#include "doctest.h"

#include "EZ-Template/api.hpp"

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
