// Promise 8: the small helpers teams call, and the ones every motion goes through, do what their docs say.
#include <cmath>
#include <cstdint>
#include <cstring>

#include "../doctest.h"
#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {
std::uint64_t bits(double v) {
  std::uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}
}  // namespace

// Promise 9: EZ-Units gives the same doubles OkapiLib did, so a target written in units means what it meant on 3.x.
// The bit patterns are OkapiLib 5.1.0's.
TEST_CASE("Promise 9: units convert bit for bit the way OkapiLib did") {
  CHECK(bits(degree.getValue()) == 0x3f91df46a2529d39ULL);
  CHECK(bits(inch.getValue()) == 0x3f9a027525460aa7ULL);
  CHECK(bits(millisecond.getValue()) == 0x3f50624dd2f1a9fcULL);
  CHECK(bits((24.0_in).getValue()) == 0x3fe381d7dbf487fdULL);
  CHECK(bits((90.0_deg).getValue()) == 0x3ff921fb54442d18ULL);
  CHECK(bits((100.0_ms).getValue()) == 0x3fb999999999999aULL);
  CHECK(bits((24_in).getValue()) == bits((24.0_in).getValue()));
  CHECK(bits((2.5_ft).convert(inch)) == 0x403dffffffffffffULL);
  CHECK(bits((1.0_rad).convert(degree)) == 0x404ca5dc1a63c1f8ULL);
  CHECK(bits((30.0_cm).convert(inch)) == 0x40279f3e7cf9f3e7ULL);
}

TEST_CASE("Promise 8: angle helpers turn the short way, the long way, and wrap") {
  CHECK(util::turn_shortest(190, 0) == doctest::Approx(-170));
  CHECK(util::turn_shortest(-190, 0) == doctest::Approx(170));
  CHECK(util::turn_shortest(10, 0) == doctest::Approx(10));
  CHECK(util::turn_longest(10, 0) == doctest::Approx(-350));
  CHECK(util::wrap_angle(190) == doctest::Approx(-170));
  CHECK(util::wrap_angle(540) == doctest::Approx(180));

  // Clockwise from north: north 0, east 90, south 180, west -90
  pose here{0, 0, 0};
  CHECK(util::absolute_angle_to_point({0, 10}, here) == doctest::Approx(0));
  CHECK(util::absolute_angle_to_point({10, 0}, here) == doctest::Approx(90));
  CHECK(util::absolute_angle_to_point({0, -10}, here) == doctest::Approx(180));
  CHECK(util::absolute_angle_to_point({-10, 0}, here) == doctest::Approx(-90));
}

TEST_CASE("Promise 8: clamp keeps a value inside its limits") {
  CHECK(util::clamp(15, 10, -10) == doctest::Approx(10));
  CHECK(util::clamp(-15, 10, -10) == doctest::Approx(-10));
  CHECK(util::clamp(5, 10, -10) == doctest::Approx(5));
  CHECK(util::clamp(-15, 10) == doctest::Approx(-10));
}

TEST_CASE("Promise 8: a point without a heading keeps having no heading when its units are dropped") {
  pose no_heading = util::united_pose_to_pose({12_in, 6_in, p_ANGLE_NOT_SET});
  CHECK(no_heading.x == doctest::Approx(12));
  CHECK(no_heading.y == doctest::Approx(6));
  CHECK(no_heading.theta == ANGLE_NOT_SET);
  CHECK(util::united_pose_to_pose({0_in, 0_in, 45_deg}).theta == doctest::Approx(45));
}

// Promise 2: a held curve button or a bad SD card file can't push the joystick curve to where the stick stops working
TEST_CASE("Promise 2: the joystick curve scale stays between 0 and its max") {
  CHECK(util::curve_scale_clamp(5.5) == doctest::Approx(5.5));
  CHECK(util::curve_scale_clamp(50) == doctest::Approx(util::MAX_CURVE_SCALE));
  CHECK(util::curve_scale_clamp(-50) == doctest::Approx(0));
  CHECK(util::curve_scale_clamp(INFINITY) == doctest::Approx(util::MAX_CURVE_SCALE));
  CHECK(util::curve_scale_clamp(NAN) == doctest::Approx(0));
}

TEST_CASE("Promise 8: curvature drive is arcade at gain 1, stays in range, and mirrors") {
  auto arcade = util::curvature_mix(50, 30, 1.0);
  CHECK(arcade.first == doctest::Approx(80));
  CHECK(arcade.second == doctest::Approx(20));

  // Gain 0 scales the turn by forward speed, so there is no turning in place
  auto curve = util::curvature_mix(50, 50, 0.0);
  CHECK(curve.first == doctest::Approx(50 + 50.0 * 50.0 / 127.0));
  CHECK(util::curvature_mix(0, 100, 0.0).first == doctest::Approx(0));

  for (int fwd = -127; fwd <= 127; fwd += 9) {
    for (int turn = -127; turn <= 127; turn += 9) {
      auto out = util::curvature_mix(fwd, turn, 0.5);
      CHECK(std::fabs(out.first) <= 127.0 + 1e-9);
      CHECK(std::fabs(out.second) <= 127.0 + 1e-9);
    }
  }

  // Past 127 both sides shrink together, so the arc is kept
  auto saturated = util::curvature_mix(100, 100, 0.0);
  double turn = 100.0 * 100.0 / 127.0;
  CHECK(saturated.first == doctest::Approx(127));
  CHECK(saturated.second / saturated.first == doctest::Approx((100 - turn) / (100 + turn)));

  auto mirrored = util::curvature_mix(100, -50, 0.5);
  auto original = util::curvature_mix(100, 50, 0.5);
  CHECK(mirrored.first == doctest::Approx(original.second));
  CHECK(mirrored.second == doctest::Approx(original.first));
}

// Slew starts at its min speed and ramps in a straight line to max over its distance, the way a lift written from the docs uses it
TEST_CASE("Promise 8: slew ramps from its min speed to max over its distance, then stays at max") {
  slew s(3.0, 70);
  s.initialize(true, 127, 100, 0);
  CHECK(s.iterate(0) == doctest::Approx(70));
  CHECK(s.iterate(1.5) == doctest::Approx(70 + (127 - 70) * 0.5));
  CHECK(s.iterate(-5) == doctest::Approx(70));  // behind the start it holds the min
  CHECK(s.iterate(3) == doctest::Approx(127));  // the pass that finishes returns max
  CHECK_FALSE(s.enabled());
  CHECK(s.iterate(4) == doctest::Approx(127));

  // Backward, toward a target behind the start
  slew back(3.0, 70);
  back.initialize(true, 127, -100, 0);
  CHECK(back.iterate(-1.5) == doctest::Approx(70 + (127 - 70) * 0.5));
  CHECK(back.iterate(-3) == doctest::Approx(127));
}

TEST_CASE("Promise 8: slew constants given with the wrong sign still ramp toward the target") {
  slew negative(-3.0, -70);
  slew positive(3.0, 70);
  negative.initialize(true, 127, 100, 0);
  positive.initialize(true, 127, 100, 0);
  for (double sensor : {0.0, 1.0, 2.0}) CHECK(negative.iterate(sensor) == doctest::Approx(positive.iterate(sensor)));
}

TEST_CASE("Promise 8: a slew that starts at its target is off and gives max") {
  slew s(3.0, 70);
  s.initialize(true, 127, 50, 50);
  CHECK_FALSE(s.enabled());
  CHECK(s.output() == doctest::Approx(127));
}

// Promise 8: the wheel rpm given to the constructor or drive_rpm_set() is what distance is measured with. A V5 motor
// counts 50 per turn of its motor shaft, which turns 3600 / rpm times per wheel turn.
TEST_CASE("Promise 8: wheel rpm sets how many encoder counts make an inch") {
  test_stub::reset_all();
  auto counts_per_inch = [](double rpm, double wheel) { return 50.0 * 3600.0 / rpm / (wheel * M_PI); };

  Drive geared({1, -2}, {-3, 4}, 5, 3.25, 450);  // a 600 cartridge on 36:48 gears
  CHECK(geared.drive_tick_per_inch() == doctest::Approx(counts_per_inch(450, 3.25)));
  CHECK(geared.drive_rpm_get() == doctest::Approx(450));

  Drive shifted({1, -2}, {-3, 4}, 5, 3.25, 600);
  shifted.drive_rpm_set(450);
  CHECK(shifted.drive_tick_per_inch() == doctest::Approx(geared.drive_tick_per_inch()));
}
