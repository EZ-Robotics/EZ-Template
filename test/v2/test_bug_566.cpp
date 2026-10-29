// #566: a 3.x project that passes a ratio to the Drive constructor keeps working, with ticks / ratio as the wheel rpm.
#include "../doctest.h"
#include "robot.hpp"

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

TEST_CASE("#566: Drive(left, right, imu, 3.25, 600, 1.667) drives like Drive(left, right, imu, 3.25, 360)") {
  test_stub::reset_all();
  ez::Drive with_ratio({1, 2, 3}, {-4, -5, -6}, 7, 3.25, 600, 1.667);
  ez::Drive wheel_rpm({1, 2, 3}, {-4, -5, -6}, 7, 3.25, 600 / 1.667);
  CHECK(with_ratio.drive_rpm_get() == doctest::Approx(wheel_rpm.drive_rpm_get()));
  CHECK(with_ratio.drive_tick_per_inch() == doctest::Approx(wheel_rpm.drive_tick_per_inch()));
}
