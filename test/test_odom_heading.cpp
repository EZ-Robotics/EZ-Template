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
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

struct HeadingCase {
  double imu;       // Where the IMU reads when the motion finishes
  double theta;     // Heading the user asked for at the end of the motion
  double expected;  // Same physical heading, as the angle nearest the IMU
};

Drive* g_chassis = nullptr;
// A real compute_error() call every simulated pass, standing in for ez_auto_task (nothing else runs
// it on the host) -- exit_condition()'s small/big timers only credit `error` when a real compute has
// landed since they last checked (see PID.cpp), so even at these deliberately huge thresholds, a
// wait still needs a fresh compute each pass to ever return. Computes every PID a wait in this test
// might poll (xy/angle for the odom path itself, left/right for pid_wait_until_point()'s own
// DRIVE-shaped internals) -- the exact value doesn't matter at these thresholds.
void on_delay() {
  Drive& c = *g_chassis;
  c.xyPID.compute_error(0.0, 0.0);
  c.current_a_odomPID.compute_error(0.0, 0.0);
  c.leftPID.compute_error(0.0, 0.0);
  c.rightPID.compute_error(0.0, 0.0);
}
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
        // No drive task runs on the host, so the position and angle errors never move. Thresholds
        // this wide mean any value at all is inside both small exits, so they fire on the first
        // passes once a real compute lands (see on_delay() above).
        chassis.pid_odom_drive_exit_condition_set(90, 1.0e6, 250, 1.0e6, 0, 0);
        chassis.pid_odom_turn_exit_condition_set(90, 1.0e6, 250, 1.0e6, 0, 0);
        g_chassis = &chassis;
        test_stub::g_clock.on_delay = on_delay;
        test_stub::g_clock.delay_calls_until_stop = 50;  // generous; real fire is a handful of passes

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
        bool returned = true;
        try {
          if (quick)
            chassis.pid_wait_quick();
          else
            chassis.pid_wait();
        } catch (test_stub::StopLoop&) {
          returned = false;
        }
        test_stub::g_clock.delay_calls_until_stop = -1;
        test_stub::g_clock.on_delay = nullptr;

        REQUIRE(returned);
        CHECK(chassis.headingPID.target_get() == doctest::Approx(c.expected));
      }
    }
  }
}
