// odom_pose_set() with no heading: a pose built without a theta carries ANGLE_NOT_SET, which
// means "leave the heading alone" everywhere else a pose is consumed (pid_odom_set uses it to
// pick boomerang vs not). Setting only the position must leave the heading where it was, in
// both overloads. A pose that does give a heading, including 0, still sets it, and
// odom_reset() still sets everything to 0.
#include <cmath>

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

namespace {
// One pass of ez_auto_task(); see test_tracking.cpp.
void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}

// Steps the drive task for `passes` passes (10 ms each), like simulate() in test_odom_target_at_pose.cpp: the
// left/right voltage difference turns the robot at up to about 300 deg/s and nothing else moves it. The
// difference is that the robot's real heading is kept apart from what the IMU reads, because resetting the IMU
// changes what the robot believes without turning it.
void simulate(Drive& chassis, int passes, double& physical_heading) {
  for (int i = 0; i < passes; i++) {
    run_one_auto_task_pass(chassis);
    double left = chassis.left_motors[0].fake().voltage;
    double right = chassis.right_motors[0].fake().voltage;
    double turned = ((left - right) / 2.0 / 12000.0) * 300.0 * 0.01;
    chassis.imu->fake_rotation += turned;
    physical_heading += turned;
  }
}
}  // namespace

// The consequence that matters: a position correction in the middle of an auton, followed by a move. The robot
// faces 90 (+X) and is told it is at (0, 0), then to go to (0, 24), which is straight toward +Y. It has to turn
// to face +Y. If the correction had reset the heading to 0, the robot would believe it already faces +Y, would
// not turn, and would drive off toward +X.
TEST_CASE("a position correction without a heading is followed by a move that turns the robot toward the target") {
  for (bool united : {false, true}) {
    CAPTURE(united);
    Drive chassis = make_chassis();
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.odom_xyt_set(0.0, 0.0, 90.0);
    chassis.drive_sensor_reset();
    double physical_heading = 90.0;

    if (united)
      chassis.odom_pose_set(united_pose{0_in, 0_in});
    else
      chassis.odom_pose_set(pose{0.0, 0.0});

    chassis.pid_odom_set({{0.0, 24.0}, fwd, 60});
    simulate(chassis, 300, physical_heading);  // 3 s, more than enough for a 90 deg turn to finish

    CHECK(std::fabs(std::remainder(physical_heading - 0.0, 360.0)) < 5.0);
  }
}
