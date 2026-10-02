// The 3.x / beta.1-3 constructors took a sixth argument, `ratio` (wheel gear / motor gear), and 4.0 removed it. The
// deprecated six-argument overloads exist so those projects still compile, and they mean what they always meant: the
// distances come out the same as a project that was migrated by hand to `ticks = cartridge_rpm / ratio`.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "fake_hardware.hpp"

#include "EZ-Template/api.hpp"

// These tests call the deprecated overloads on purpose
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

using namespace ez;

namespace {
constexpr double kWheel = 3.25;
constexpr double kCart = 600.0;
constexpr double kRatio = 1.667;  // the example from the upgrade note: 600, 1.667 becomes 360
}  // namespace

TEST_CASE("the six-argument constructor with a ratio reads the same ticks per inch as ticks = cartridge / ratio") {
  test_stub::reset_all();
  Drive old_style({1, -2}, {-3, 4}, 5, kWheel, kCart, kRatio);
  Drive migrated({1, -2}, {-3, 4}, 5, kWheel, kCart / kRatio);
  CHECK(old_style.drive_tick_per_inch() == doctest::Approx(migrated.drive_tick_per_inch()).epsilon(1e-9));
  CHECK(old_style.drive_rpm_get() == doctest::Approx(kCart / kRatio));
}

TEST_CASE("the six-argument redundant-imu constructor does the same") {
  test_stub::reset_all();
  Drive old_style({1, -2}, {-3, 4}, std::vector<int>{5, 6}, kWheel, kCart, kRatio);
  Drive migrated({1, -2}, {-3, 4}, std::vector<int>{5, 6}, kWheel, kCart / kRatio);
  CHECK(old_style.drive_tick_per_inch() == doctest::Approx(migrated.drive_tick_per_inch()).epsilon(1e-9));
  CHECK(DriveTestAccess::all_imus(old_style).size() == 2);
}

TEST_CASE("dropping the ratio argument without folding it in is the 1.667x mistake the overload prevents") {
  test_stub::reset_all();
  Drive old_style({1, -2}, {-3, 4}, 5, kWheel, 600, 1.667);
  Drive dropped({1, -2}, {-3, 4}, 5, kWheel, 600);
  // a 24 in drive on the "dropped" project goes 1.667x too far
  CHECK(old_style.drive_tick_per_inch() / dropped.drive_tick_per_inch() == doctest::Approx(kRatio).epsilon(1e-9));
}

TEST_CASE("a ratio of 1 changes nothing from the five-argument constructor") {
  test_stub::reset_all();
  Drive six({1, -2}, {-3, 4}, 5, kWheel, 450, 1.0);
  Drive five({1, -2}, {-3, 4}, 5, kWheel, 450);
  CHECK(six.drive_tick_per_inch() == doctest::Approx(five.drive_tick_per_inch()).epsilon(1e-12));
}

TEST_CASE("a project that passed a ratio and shifts with drive_rpm_set gets the same distances as the migrated one") {
  test_stub::reset_all();
  Drive old_style({1, -2}, {-3, 4}, 5, kWheel, kCart, kRatio);
  Drive migrated({1, -2}, {-3, 4}, 5, kWheel, kCart / kRatio);
  for (double wheel_rpm : {200.0, 300.0, 450.0}) {
    old_style.drive_rpm_set(wheel_rpm);
    migrated.drive_rpm_set(wheel_rpm);
    CAPTURE(wheel_rpm);
    CHECK(old_style.drive_tick_per_inch() == doctest::Approx(migrated.drive_tick_per_inch()).epsilon(1e-12));
  }
}

TEST_CASE("every five-argument and driver-only call shape still picks its own constructor and reads what it did") {
  test_stub::reset_all();
  Drive a({1, -2}, {-3, 4}, 5, 3.25, 600);                         // int literal ticks
  Drive b({1, -2}, {-3, 4}, std::vector<int>{5, 6}, 3.25, 450.0);  // redundant imus, double ticks
  Drive c({1, -2}, {-3, 4});                                       // driver control only
  CHECK(a.drive_rpm_get() == doctest::Approx(600));
  CHECK(b.drive_rpm_get() == doctest::Approx(450));
  CHECK(c.drive_rpm_get() == doctest::Approx(200));
}

TEST_CASE("after a six-argument constructor drive_rpm_set takes the wheel rpm directly, the folded ratio is not applied again") {
  test_stub::reset_all();
  Drive old_style({1, -2}, {-3, 4}, 5, kWheel, kCart, kRatio);
  old_style.drive_rpm_set(450);
  Drive direct({1, -2}, {-3, 4}, 5, kWheel, 450);
  CHECK(old_style.drive_tick_per_inch() == doctest::Approx(direct.drive_tick_per_inch()).epsilon(1e-9));
  CHECK(old_style.drive_rpm_get() == doctest::Approx(450));
}
