// After an odom motion finishes, pid_wait() and pid_wait_quick() leave
// headingPID's target at the heading the robot is actually facing, expressed
// as the equivalent angle closest to the IMU. With the IMU wound up to 270 and
// a boomerang to -90 (the same physical heading), the target must be 270, not
// -90: otherwise the next pid_drive_set() sees a 360 degree heading error and
// the robot spins.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

struct HeadingCase {
  double imu;       // Where the IMU reads when the motion finishes
  double theta;     // Heading the user asked for at the end of the motion
  double expected;  // Same physical heading, as the angle nearest the IMU
};
}  // namespace

TEST_CASE("odom motion leaves the heading target at the equivalent angle nearest the IMU") {
  const HeadingCase cases[] = {
      {270.0, -90.0, 270.0},    // Wound up past 180, user wrote the heading as negative
      {-170.0, 190.0, -170.0},  // Wound up negative, user wrote the heading past 180
      {88.0, 90.0, 90.0},       // Already nearest, must be left alone
      {270.0, 270.0, 270.0},    // Same spelling as the IMU
  };

  // Every way an odom motion can end: a boomerang runs as pure pursuit, pid_odom_ptp_set
  // runs point to point, and each can be waited on fully or with pid_wait_quick().
  for (bool point_to_point : {false, true}) {
    for (bool quick : {false, true}) {
      for (const HeadingCase& c : cases) {
        CAPTURE(point_to_point);
        CAPTURE(quick);
        CAPTURE(c.imu);
        CAPTURE(c.theta);

        Drive chassis = make_chassis();
        // No drive task runs on the host, so the position and angle errors never update. Thresholds
        // this wide let both small exits fire on the first passes so the waits return.
        chassis.pid_odom_drive_exit_condition_set(90, 1.0e6, 250, 1.0e6, 0, 0);
        chassis.pid_odom_turn_exit_condition_set(90, 1.0e6, 250, 1.0e6, 0, 0);

        chassis.odom_xyt_set(0.0, 0.0, 0.0);
        chassis.imu->fake_rotation = c.imu;
        REQUIRE(chassis.drive_angle_get() == doctest::Approx(c.imu));

        odom movement{{-12.0, 0.0, c.theta}, fwd, 70};
        if (point_to_point)
          chassis.pid_odom_ptp_set(movement);
        else
          chassis.pid_odom_set(movement);

        // A pure pursuit motion's waits first look for the drive task to reach the last path point.
        // Nothing advances it on the host, so put it there directly.
        DriveTestAccess::pp_index(chassis) = static_cast<int>(DriveTestAccess::pp_movements(chassis).size()) - 1;
        if (quick)
          chassis.pid_wait_quick();
        else
          chassis.pid_wait();

        CHECK(chassis.headingPID.target_get() == doctest::Approx(c.expected));
      }
    }
  }
}
