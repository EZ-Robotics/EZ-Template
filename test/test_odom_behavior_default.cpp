// pid_odom_behavior_set()/pid_odom_behavior_get() stored default_odom_type, but nothing read it:
// the odom and united_odom structs defaulted turn_behavior to shortest on their own, so an odom
// movement that didn't name a behavior always turned shortest no matter what the default was set
// to, and pid_angle_behavior_set() never reached odom either.
//
// Fix: a movement that doesn't name a behavior carries ODOM_BEHAVIOR_NOT_SET, and
// raw_pid_odom_ptp_set() resolves it against default_odom_type when the movement starts.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void at_origin(Drive& chassis) {
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.imu->fake_rotation = 0.0;
}
}  // namespace

TEST_CASE("an odom movement with no behavior defaults to shortest") {
  Drive chassis = make_chassis();
  at_origin(chassis);
  CHECK(chassis.pid_odom_behavior_get() == shortest);
  chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == shortest);
}

TEST_CASE("pid_odom_behavior_set() is what an unnamed odom movement turns with") {
  for (e_angle_behavior b : {cw, ccw, longest, shortest}) {
    Drive chassis = make_chassis();
    at_origin(chassis);
    at_origin(chassis);
    chassis.pid_odom_behavior_set(b);
    chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70});
    CHECK(DriveTestAccess::current_angle_behavior(chassis) == b);
  }
}

TEST_CASE("a behavior named on the movement beats the default") {
  Drive chassis = make_chassis();
  at_origin(chassis);
  chassis.pid_odom_behavior_set(ccw);
  chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70, cw});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == cw);

  // Naming shortest explicitly is a choice too, not "unset"
  chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70, shortest});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == shortest);
}

TEST_CASE("the default applies to united_odom and to paths") {
  Drive chassis = make_chassis();
  at_origin(chassis);
  chassis.pid_odom_behavior_set(ccw);

  chassis.pid_odom_ptp_set(united_odom{{24_in, 0_in}, fwd, 70});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == ccw);

  chassis.pid_odom_behavior_set(cw);
  chassis.pid_odom_set(std::vector<odom>{{{12.0, 12.0}, fwd, 70}, {{24.0, 0.0}, fwd, 70}});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == cw);
}

TEST_CASE("pid_angle_behavior_set() reaches odom") {
  Drive chassis = make_chassis();
  at_origin(chassis);
  chassis.pid_angle_behavior_set(longest);
  chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == longest);
}

TEST_CASE("a default of raw keeps the previous behavior, like an explicit raw movement") {
  Drive chassis = make_chassis();
  at_origin(chassis);
  chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70, cw});
  REQUIRE(DriveTestAccess::current_angle_behavior(chassis) == cw);

  chassis.pid_odom_behavior_set(raw);
  chassis.pid_odom_ptp_set(odom{{24.0, 0.0}, fwd, 70});
  CHECK(DriveTestAccess::current_angle_behavior(chassis) == cw);
}
