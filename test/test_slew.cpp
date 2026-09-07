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
