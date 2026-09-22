// Whether a swing ramps up (slew) has to be decided from the direction the swing really goes, which is the
// direction of the target after the angle behavior has resolved it. A swing to -170 from a heading of 170 is
// asked for "backwards" (-170 is below 170) but the shortest way there is +20 degrees, so it goes forward.
// The slew constants come from the resolved direction, so the on/off decision has to as well.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Slew on for forward swings and off for backward ones, so the two directions can be told apart
void slew_forward_only(Drive& chassis) {
  chassis.slew_swing_forward_set(true);
  chassis.slew_swing_backward_set(false);
}
}  // namespace

TEST_CASE("swing slew is decided from the resolved target, not the requested one") {
  Drive chassis = make_chassis();
  slew_forward_only(chassis);

  // -170 from 170: the request looks backward, the shortest way there is +20, a forward swing
  CHECK(DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, -170.0, 170.0, shortest));
  // The same request the long way round is -340, a backward swing
  CHECK_FALSE(DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, -170.0, 170.0, longest));

  // 90 from 0: the request looks forward, the long way round is -270, a backward swing
  CHECK_FALSE(DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, 90.0, 0.0, longest));
  CHECK(DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, 90.0, 0.0, shortest));
}

TEST_CASE("swing slew still follows the requested target when the behavior does not change it") {
  Drive chassis = make_chassis();
  slew_forward_only(chassis);

  CHECK(DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, 45.0, 0.0, raw));
  CHECK_FALSE(DriveTestAccess::is_swing_slew_enabled(chassis, LEFT_SWING, -45.0, 0.0, raw));
  // A right swing to a positive heading drives the right side backward
  CHECK_FALSE(DriveTestAccess::is_swing_slew_enabled(chassis, RIGHT_SWING, 45.0, 0.0, raw));
  CHECK(DriveTestAccess::is_swing_slew_enabled(chassis, RIGHT_SWING, -45.0, 0.0, raw));
}

TEST_CASE("the public swing setters use the resolved direction for slew") {
  Drive chassis = make_chassis();
  slew_forward_only(chassis);
  chassis.drive_imu_reset(170.0);
  REQUIRE(chassis.drive_angle_get() == doctest::Approx(170.0));

  chassis.pid_swing_set(LEFT_SWING, -170.0, 110, shortest);
  CHECK(DriveTestAccess::current_slew_on(chassis));

  chassis.pid_swing_set(LEFT_SWING, -170.0, 110, longest);
  CHECK_FALSE(DriveTestAccess::current_slew_on(chassis));

  chassis.pid_swing_set(LEFT_SWING, -170.0, 110, 0, shortest);
  CHECK(DriveTestAccess::current_slew_on(chassis));

  chassis.pid_swing_set(LEFT_SWING, -170.0, 110, 0, longest);
  CHECK_FALSE(DriveTestAccess::current_slew_on(chassis));
}

TEST_CASE("an explicit slew_on is never overridden") {
  Drive chassis = make_chassis();
  slew_forward_only(chassis);
  chassis.drive_imu_reset(170.0);

  // The global setting says slew forward, this call says no
  chassis.pid_swing_set(LEFT_SWING, -170.0, 110, false);
  CHECK_FALSE(DriveTestAccess::current_slew_on(chassis));

  // The global setting says no slew backward, this call says yes
  chassis.pid_swing_set(LEFT_SWING, -170.0, 110, longest, true);
  CHECK(DriveTestAccess::current_slew_on(chassis));
}
