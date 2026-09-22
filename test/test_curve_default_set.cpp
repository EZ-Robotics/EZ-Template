// opcontrol_curve_default_set holds each curve to 0 - MAX_CURVE_SCALE and then saves what it kept to the SD card,
// over whatever the team had stored there. A value outside that range, or NaN, used to be replaced without a word,
// so a team that typed 25 got 20 (and 20 on the card) and had no way to tell. It now prints what it was given and
// what it used. The clamping and the saved value are unchanged.
#include <algorithm>
#include <cmath>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

template <typename F>
std::string printed_by(F&& action) {
  return test_stub::capture_stdout(std::forward<F>(action));
}
}  // namespace

TEST_CASE("opcontrol_curve_default_set says so when the left curve is above the range") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(25, 5); }) ==
        "EZ-Template: opcontrol_curve_default_set was given a left curve of 25, using 20 (the range is 0 to 20)\n");
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(util::MAX_CURVE_SCALE));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(5));
}

TEST_CASE("opcontrol_curve_default_set says so when the left curve is negative") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(-2.5, 5); }) ==
        "EZ-Template: opcontrol_curve_default_set was given a left curve of -2.5, using 0 (the range is 0 to 20)\n");
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(0));
}

TEST_CASE("opcontrol_curve_default_set says so when the right curve is out of range") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(5, 30.5); }) ==
        "EZ-Template: opcontrol_curve_default_set was given a right curve of 30.5, using 20 (the range is 0 to 20)\n");
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(5));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(util::MAX_CURVE_SCALE));
}

TEST_CASE("opcontrol_curve_default_set prints one line for each curve that was out of range") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(25, -1); }) ==
        "EZ-Template: opcontrol_curve_default_set was given a left curve of 25, using 20 (the range is 0 to 20)\n"
        "EZ-Template: opcontrol_curve_default_set was given a right curve of -1, using 0 (the range is 0 to 20)\n");
}

TEST_CASE("opcontrol_curve_default_set says so when a curve is NaN") {
  Drive chassis = make_chassis();

  // The value prints however the platform spells NaN, so match the parts around it.
  std::string left = printed_by([&] { chassis.opcontrol_curve_default_set(NAN, 5); });
  CHECK(left.find("EZ-Template: opcontrol_curve_default_set was given a left curve of ") == 0);
  CHECK(left.find(", using 0 (the range is 0 to 20)\n") != std::string::npos);
  CHECK(std::count(left.begin(), left.end(), '\n') == 1);
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(0));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(5));

  std::string right = printed_by([&] { chassis.opcontrol_curve_default_set(5, NAN); });
  CHECK(right.find("EZ-Template: opcontrol_curve_default_set was given a right curve of ") == 0);
  CHECK(right.find(", using 0 (the range is 0 to 20)\n") != std::string::npos);
  CHECK(std::count(right.begin(), right.end(), '\n') == 1);
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(5));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(0));
}

TEST_CASE("opcontrol_curve_default_set prints nothing when both curves are in range") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(5.5, 7.25); }) == "");
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(5.5));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(7.25));

  // Both ends of the range are in range, and so is leaving the right curve off, which tank drive does.
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(0, util::MAX_CURVE_SCALE); }) == "");
  CHECK(printed_by([&] { chassis.opcontrol_curve_default_set(util::MAX_CURVE_SCALE); }) == "");
  CHECK(DriveTestAccess::left_curve_scale(chassis) == doctest::Approx(util::MAX_CURVE_SCALE));
  CHECK(DriveTestAccess::right_curve_scale(chassis) == doctest::Approx(0));
}
