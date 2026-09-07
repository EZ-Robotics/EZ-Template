// util: turn_shortest/turn_longest/wrap_angle on a table of (target, current)
// pairs including the 180 and -180 edges; clamp both overloads;
// absolute_angle_to_point in all four quadrants; united_pose_to_pose
// preserving ANGLE_NOT_SET.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("turn_shortest wraps to the nearer-than-180 target") {
  // |target - current| < 180: returned unchanged.
  CHECK(util::turn_shortest(10, 0) == doctest::Approx(10));
  CHECK(util::turn_shortest(-170, 0) == doctest::Approx(-170));

  // Exactly at the 180 / -180 edge: the wrap loops use a strict `>`/`<`, so
  // these pass through unchanged rather than wrapping.
  CHECK(util::turn_shortest(180, 0) == doctest::Approx(180));
  CHECK(util::turn_shortest(-180, 0) == doctest::Approx(-180));

  // Past the edge: wraps by 360 to the equivalent angle under 180 away.
  CHECK(util::turn_shortest(190, 0) == doctest::Approx(-170));
  CHECK(util::turn_shortest(-190, 0) == doctest::Approx(170));
}

TEST_CASE("turn_longest takes the far way around") {
  CHECK(util::turn_longest(10, 0) == doctest::Approx(-350));
  CHECK(util::turn_longest(190, 0) == doctest::Approx(190));
  // target == current: sgn(error) is 0, so "longest" degenerates to the
  // same value rather than a full 360 loop.
  CHECK(util::turn_longest(0, 0) == doctest::Approx(0));
}

TEST_CASE("wrap_angle constrains to (-180, 180]") {
  CHECK(util::wrap_angle(190) == doctest::Approx(-170));
  CHECK(util::wrap_angle(-190) == doctest::Approx(170));
  CHECK(util::wrap_angle(540) == doctest::Approx(180));
  // Edge values are left alone (strict comparisons, same as turn_shortest).
  CHECK(util::wrap_angle(180) == doctest::Approx(180));
  CHECK(util::wrap_angle(-180) == doctest::Approx(-180));
}

TEST_CASE("clamp: three-argument overload") {
  CHECK(util::clamp(5, 10, -10) == doctest::Approx(5));
  CHECK(util::clamp(15, 10, -10) == doctest::Approx(10));
  CHECK(util::clamp(-15, 10, -10) == doctest::Approx(-10));
}

TEST_CASE("clamp: two-argument overload clamps to [-|max|, |max|]") {
  CHECK(util::clamp(15, 10) == doctest::Approx(10));
  CHECK(util::clamp(-15, 10) == doctest::Approx(-10));
  // A negative max is treated the same as its absolute value.
  CHECK(util::clamp(5, -10) == doctest::Approx(5));
}

TEST_CASE("absolute_angle_to_point: all four quadrants") {
  pose current{0, 0, 0};
  CHECK(util::absolute_angle_to_point({0, 10}, current) == doctest::Approx(0));    // north
  CHECK(util::absolute_angle_to_point({10, 0}, current) == doctest::Approx(90));   // east
  CHECK(util::absolute_angle_to_point({0, -10}, current) == doctest::Approx(180)); // south
  CHECK(util::absolute_angle_to_point({-10, 0}, current) == doctest::Approx(-90)); // west
}

TEST_CASE("united_pose_to_pose preserves ANGLE_NOT_SET") {
  united_pose input{12_in, 6_in, p_ANGLE_NOT_SET};
  pose output = util::united_pose_to_pose(input);
  CHECK(output.x == doctest::Approx(12));
  CHECK(output.y == doctest::Approx(6));
  CHECK(output.theta == ANGLE_NOT_SET);
}

TEST_CASE("united_pose_to_pose converts a set theta to degrees") {
  united_pose input{0_in, 0_in, 45_deg};
  pose output = util::united_pose_to_pose(input);
  CHECK(output.theta == doctest::Approx(45));
}
