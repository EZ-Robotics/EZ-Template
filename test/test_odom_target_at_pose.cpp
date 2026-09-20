// Odom target equal to the robot's own pose: pid_odom_set to the spot the robot
// is already standing on must leave it where it is. Run against a toy drivetrain
// (the left/right voltage difference spins the IMU, nothing else moves) that steps
// the real drive task; the robot must not turn away from its starting heading,
// whether it faces 0 or 90 degrees and whether the move is forward or reverse.
// A target straight ahead is the control: the same sim leaves the heading alone.
//
// When the target is exactly the current position, the two candidate points the
// robot could face are the same distance away, and the tie picks the point
// behind the robot, so without special handling it turns around to face 180 deg.
// The sim never translates the robot, so this covers the turn-around only, not what
// happens if the robot is pushed forward afterwards.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// One pass of ez_auto_task(). pros::Task never runs its callable in the host
// build, so the fake pros::delay() throws right after the first pass to unwind
// the otherwise-infinite loop (see test_tracking.cpp).
void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}

// Steps the drive task for `passes` passes (10 ms each). Whatever the drive
// commands, the left/right voltage difference turns the IMU: 12000 mV on one side
// and -12000 mV on the other is about 300 deg/s, a typical drivetrain's top turn
// rate. Nothing else moves the robot.
void simulate(Drive& chassis, int passes) {
  for (int i = 0; i < passes; i++) {
    run_one_auto_task_pass(chassis);
    double left = chassis.left_motors[0].fake().voltage;
    double right = chassis.right_motors[0].fake().voltage;
    chassis.imu->fake_rotation += ((left - right) / 2.0 / 12000.0) * 300.0 * 0.01;
  }
}

// pid_wait() with the fake pros::delay() set to throw after `max_delays` calls, so a
// motion that never exits fails the test instead of hanging it. Nothing steps the
// drive task while this waits, so it settles on whatever the last pass computed.
bool pid_wait_returns(Drive& chassis, int max_delays) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

// Signed difference from `from` to `to`, wrapped to [-180, 180].
double angle_diff(double to, double from) {
  return std::remainder(to - from, 360.0);
}
}  // namespace

TEST_CASE("pid_odom_set to the robot's own pose does not turn the robot around") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();

  chassis.pid_odom_set({{0.0, 0.0}, fwd, 60});
  simulate(chassis, 300);  // 3 s, more than enough for a 180 deg turn to finish

  CHECK(std::fabs(angle_diff(chassis.odom_theta_get(), 0.0)) < 5.0);
  CHECK(pid_wait_returns(chassis, 500));
}

TEST_CASE("pid_odom_set to the robot's own pose does not turn a robot that starts facing 90 degrees") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.odom_xyt_set(12.0, -6.0, 90.0);
  chassis.drive_sensor_reset();

  chassis.pid_odom_set({{12.0, -6.0}, fwd, 60});
  simulate(chassis, 300);

  CHECK(std::fabs(angle_diff(chassis.odom_theta_get(), 90.0)) < 5.0);
}

TEST_CASE("pid_odom_set to the robot's own pose in reverse does not turn the robot around") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();

  chassis.pid_odom_set({{0.0, 0.0}, rev, 60});
  simulate(chassis, 300);

  CHECK(std::fabs(angle_diff(chassis.odom_theta_get(), 0.0)) < 5.0);
}

TEST_CASE("pid_odom_set to a point straight ahead leaves the heading alone") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();

  chassis.pid_odom_set({{0.0, 24.0}, fwd, 60});
  simulate(chassis, 300);

  CHECK(std::fabs(angle_diff(chassis.odom_theta_get(), 0.0)) < 5.0);
}
