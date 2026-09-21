// odom_pose_set() with no heading: a pose built without a theta carries ANGLE_NOT_SET, which
// means "leave the heading alone" everywhere else a pose is consumed (pid_odom_set uses it to
// pick boomerang vs not). Setting only the position must leave the heading where it was, in
// both overloads. A pose that does give a heading, including 0, still sets it, and
// odom_reset() still sets everything to 0.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("odom_pose_set without a heading sets the position and leaves the heading alone") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  REQUIRE(chassis.drive_angle_get() == doctest::Approx(90.0));

  chassis.odom_pose_set(pose{24.0, 6.0});

  CHECK(chassis.odom_x_get() == doctest::Approx(24.0));
  CHECK(chassis.odom_y_get() == doctest::Approx(6.0));
  CHECK(chassis.drive_angle_get() == doctest::Approx(90.0));
}

TEST_CASE("odom_pose_set without a heading leaves the heading alone, united overload") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, -135.0);
  REQUIRE(chassis.drive_angle_get() == doctest::Approx(-135.0));

  chassis.odom_pose_set(united_pose{24_in, 6_in});

  CHECK(chassis.odom_x_get() == doctest::Approx(24.0));
  CHECK(chassis.odom_y_get() == doctest::Approx(6.0));
  CHECK(chassis.drive_angle_get() == doctest::Approx(-135.0));
}

TEST_CASE("odom_pose_set with a heading sets it, including a heading of 0") {
  Drive chassis = make_chassis();

  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  chassis.odom_pose_set(pose{24.0, 6.0, 45.0});
  CHECK(chassis.drive_angle_get() == doctest::Approx(45.0));

  chassis.odom_pose_set(pose{24.0, 6.0, 0.0});
  CHECK(chassis.drive_angle_get() == doctest::Approx(0.0));

  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  chassis.odom_pose_set(united_pose{24_in, 6_in, 30_deg});
  CHECK(chassis.drive_angle_get() == doctest::Approx(30.0));
}

TEST_CASE("odom_reset still sets the position and the heading to 0") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(12.0, 12.0, 90.0);

  chassis.odom_reset();

  CHECK(chassis.odom_x_get() == doctest::Approx(0.0));
  CHECK(chassis.odom_y_get() == doctest::Approx(0.0));
  CHECK(chassis.drive_angle_get() == doctest::Approx(0.0));
}
