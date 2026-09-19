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

TEST_CASE("curvature_mix: gain 1 is arcade, gain 0 is pure curvature") {
  auto arcade = util::curvature_mix(50, 30, 1.0);
  CHECK(arcade.first == doctest::Approx(80));
  CHECK(arcade.second == doctest::Approx(20));

  // Pure curvature scales the turn by fwd / 127.
  auto curve = util::curvature_mix(50, 50, 0.0);
  double turn = 50.0 * 50.0 / 127.0;
  CHECK(curve.first == doctest::Approx(50 + turn));
  CHECK(curve.second == doctest::Approx(50 - turn));

  // Pure curvature has no turn on a point.
  auto none = util::curvature_mix(0, 100, 0.0);
  CHECK(none.first == doctest::Approx(0));
  CHECK(none.second == doctest::Approx(0));
}

TEST_CASE("curvature_mix: point turn gain scales turning with no fwd") {
  auto out = util::curvature_mix(0, 100, 0.5);
  CHECK(out.first == doctest::Approx(50));
  CHECK(out.second == doctest::Approx(-50));

  // Above the gain, fwd speed takes over: fwd 100 is a gain of 100/127, not 0.5.
  auto fast = util::curvature_mix(100, 20, 0.5);
  double turn = 20.0 * 100.0 / 127.0;
  CHECK(fast.first == doctest::Approx(100 + turn));
  CHECK(fast.second == doctest::Approx(100 - turn));
}

TEST_CASE("curvature_mix: out of range gain is clamped") {
  auto high = util::curvature_mix(0, 100, 2.0);
  CHECK(high.first == doctest::Approx(100));
  auto low = util::curvature_mix(0, 100, -1.0);
  CHECK(low.first == doctest::Approx(0));
}

TEST_CASE("curvature_mix: output is continuous in fwd, no jump at the point turn gain") {
  // Left output can change by at most 1 + turn/127 per unit of fwd, so a step
  // of 1 fwd can never move an output by more than 2.  A hard arcade/curvature
  // switch would jump by ~turn here.
  for (double gain : {0.0, 0.3, 0.5, 1.0}) {
    auto prev = util::curvature_mix(0, 100, gain);
    for (int fwd = 1; fwd <= 127; fwd++) {
      auto now = util::curvature_mix(fwd, 100, gain);
      CHECK(fabs(now.first - prev.first) <= 2.0);
      CHECK(fabs(now.second - prev.second) <= 2.0);
      prev = now;
    }
  }
}

TEST_CASE("curvature_mix: saturation keeps the ratio between sides") {
  // fwd 100 + turn 100 at gain 0 wants ~179 on the left, past 127.
  auto out = util::curvature_mix(100, 100, 0.0);
  double turn = 100.0 * 100.0 / 127.0;
  CHECK(out.first == doctest::Approx(127));
  CHECK(out.second / out.first == doctest::Approx((100 - turn) / (100 + turn)));

  // Full fwd and full turn pivots on one wheel.
  auto full = util::curvature_mix(127, 127, 0.0);
  CHECK(full.first == doctest::Approx(127));
  CHECK(full.second == doctest::Approx(0).epsilon(0.001));

  // Never outside +-127 anywhere on the stick square.
  for (int fwd = -127; fwd <= 127; fwd += 9) {
    for (int turn_stick = -127; turn_stick <= 127; turn_stick += 9) {
      auto o = util::curvature_mix(fwd, turn_stick, 0.5);
      CHECK(fabs(o.first) <= 127.0 + 1e-9);
      CHECK(fabs(o.second) <= 127.0 + 1e-9);
    }
  }
}

TEST_CASE("curvature_mix: turn stick yaws the same way driving fwd or reversing") {
  auto fwd = util::curvature_mix(100, 50, 0.5);
  auto rev = util::curvature_mix(-100, 50, 0.5);
  // Positive turn puts more power on the left in both, so right < left.
  CHECK(fwd.second < fwd.first);
  CHECK(rev.second < rev.first);

  // Flipping the turn stick mirrors the sides.
  auto mirrored = util::curvature_mix(100, -50, 0.5);
  CHECK(mirrored.first == doctest::Approx(fwd.second));
  CHECK(mirrored.second == doctest::Approx(fwd.first));
}
