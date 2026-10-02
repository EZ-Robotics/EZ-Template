// What `ticks`, drive_rpm_set() and drive_ratio_set() mean, in numbers. The header comments on those say:
//   - `ticks` / drive_rpm_set() is the wheel's own RPM: cartridge RPM * (motor gear / wheel gear)
//   - drive_ratio_set() is motor turns per wheel turn (wheel gear / motor gear) and multiplies ticks per wheel turn
// so a 600 cartridge geared 36:48 (motor gear 36T, wheel gear 48T) is 450 wheel RPM, and all three ways of saying
// so have to land on the same ticks per inch. These pass before and after the documentation fix: they lock the
// meaning the docs now state, they are not a reproduction of a bug.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "fake_hardware.hpp"

#include "EZ-Template/api.hpp"

using namespace ez;

namespace {
constexpr double kWheel = 3.25;
// 50 counts per cartridge rev * 3600 / wheel rpm = counts per wheel turn, over the wheel's circumference
double expected_tpi(double wheel_rpm) { return (50.0 * 3600.0 / wheel_rpm) / (kWheel * M_PI); }

Drive make(double ticks) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, kWheel, ticks);
}
}  // namespace

TEST_CASE("ticks is the wheel's rpm: a 600 cartridge geared 36:48 is 450 and reads 39.177 ticks per inch") {
  Drive d = make(450);
  CHECK(d.drive_tick_per_inch() == doctest::Approx(39.177).epsilon(0.0001));
  CHECK(d.drive_tick_per_inch() == doctest::Approx(expected_tpi(450)));
  CHECK(d.drive_rpm_get() == doctest::Approx(450));
  CHECK(d.drive_ratio_get() == doctest::Approx(1.0));
}

TEST_CASE("drive_ratio_set is motor turns per wheel turn: ticks 600 with ratio 48/36 is the same 39.177") {
  Drive d = make(600);
  CHECK(d.drive_tick_per_inch() == doctest::Approx(expected_tpi(600)));  // 29.382, the geared wheel is not accounted for yet
  d.drive_ratio_set(48.0 / 36.0);
  CHECK(d.drive_tick_per_inch() == doctest::Approx(39.177).epsilon(0.0001));
  CHECK(d.drive_tick_per_inch() == doctest::Approx(expected_tpi(450)));
  CHECK(d.drive_ratio_get() == doctest::Approx(48.0 / 36.0));
}

TEST_CASE("the ratio goes the other way from the ticks formula: 36/48 on top of ticks 600 is wrong, not right") {
  Drive d = make(600);
  d.drive_ratio_set(36.0 / 48.0);
  CHECK(d.drive_tick_per_inch() == doctest::Approx(22.037).epsilon(0.0001));
  CHECK(d.drive_tick_per_inch() != doctest::Approx(expected_tpi(450)).epsilon(0.01));
}

TEST_CASE("drive_rpm_set takes the wheel's rpm, the same number the constructor's ticks does") {
  Drive d = make(600);
  d.drive_rpm_set(450);
  CHECK(d.drive_tick_per_inch() == doctest::Approx(39.177).epsilon(0.0001));
  CHECK(d.drive_tick_per_inch() == doctest::Approx(make(450).drive_tick_per_inch()));
  CHECK(d.drive_rpm_get() == doctest::Approx(450));
}

TEST_CASE("a 24 inch reading is 24 inches once ticks is the wheel rpm, and 32 inches off if it is the cartridge rpm") {
  // The encoder counts 24 inches' worth of ticks for a 450 wheel rpm drive (936 ticks / 39.177 = 24 in)
  double ticks_for_24_in = 24.0 * expected_tpi(450);
  CHECK(ticks_for_24_in / make(450).drive_tick_per_inch() == doctest::Approx(24.0));
  CHECK(ticks_for_24_in / make(600).drive_tick_per_inch() == doctest::Approx(32.0).epsilon(0.001));
}
