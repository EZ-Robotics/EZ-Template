// pid_wait_until(distance) on an odom move. The wait has to come back once the
// move has ended, even when the robot never travels the distance it was asked to
// wait for. Waiting for 30 inches on a 24 inch move used to leave the wait spinning
// forever unless the velocity or mA exit was on: the exits it watched are measured
// against a point one look ahead from where the move started, so once the robot
// drives past that point they can never fire.
//
// Also covers the look ahead targets those exits use: each side's target starts
// from that side's own sensor reading, not the left side's.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// A Drive owns a lock and a background task that points back at it, so it cannot be copied. It is built where the
// caller needs it (returning a prvalue copies nothing) and then set up in place by configure_chassis().
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

void configure_chassis(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // The velocity and mA exits are off, so only the position exits can end a wait.
  chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.odom_look_ahead_set(7.0);
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

// Sets both drive sensors to `inches`.
void set_sensors(Drive& chassis, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * chassis.drive_tick_per_inch());
  chassis.left_motors[0].fake().position = ticks;
  chassis.right_motors[0].fake().position = ticks;
}

// Drives the robot up the y axis to `inches` and leaves it there at rest, stepping the
// drive task as it goes. The tracking task reads each sensor change as distance
// driven, so the pose follows the sensors. A pure pursuit move only steps onto the
// next point of its path when the robot gets close to the current one, so the robot
// has to move along the path rather than appear at the end of it.
void robot_is_at(Drive& chassis, double inches) {
  double step = inches < 0 ? -1.0 : 1.0;
  for (double at = step; std::fabs(at) < std::fabs(inches); at += step) {
    set_sensors(chassis, at);
    run_one_auto_task_pass(chassis);
  }
  set_sensors(chassis, inches);
  for (int i = 0; i < 60; i++)  // The path index moves up by at most one point per pass
    run_one_auto_task_pass(chassis);
}

// Starts a move to (0, 24) that is either point to point or pure pursuit.
void start_move(Drive& chassis, bool point_to_point, drive_directions direction = fwd) {
  odom movement{{0.0, direction == fwd ? 24.0 : -24.0}, direction, 60};
  if (point_to_point)
    chassis.pid_odom_ptp_set(movement);
  else
    chassis.pid_odom_set(movement);
}

// pid_wait_until(inches) with the fake pros::delay() set to throw after `max_delays`
// calls, so a wait that never returns fails the test instead of hanging it. Nothing
// steps the drive task while this waits, so the robot stays where it was placed.
bool wait_until_returns(Drive& chassis, double inches, int max_delays) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool returned = true;
  try {
    chassis.pid_wait_until(inches);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

// pid_wait() bounded the same way.
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
}  // namespace

TEST_CASE("pid_wait_until on an odom move returns once the move has ended short of the distance") {
  for (bool point_to_point : {false, true}) {
    CAPTURE(point_to_point);
    Drive chassis = make_chassis();
    configure_chassis(chassis);
    chassis.odom_xyt_set(0.0, 0.0, 0.0);
    chassis.drive_sensor_reset();

    start_move(chassis, point_to_point);
    robot_is_at(chassis, 24.0);  // Arrived and stopped, 6 inches short of what is waited for

    CHECK(wait_until_returns(chassis, 30.0, 500));
    CHECK(pid_wait_returns(chassis, 500));  // The wait leaves the move able to be waited on again
  }
}

TEST_CASE("pid_wait_until on a reverse odom move returns once the move has ended short of the distance") {
  for (bool point_to_point : {false, true}) {
    CAPTURE(point_to_point);
    Drive chassis = make_chassis();
    configure_chassis(chassis);
    chassis.odom_xyt_set(0.0, 0.0, 0.0);
    chassis.drive_sensor_reset();

    start_move(chassis, point_to_point, rev);
    robot_is_at(chassis, -24.0);

    CHECK(wait_until_returns(chassis, -30.0, 500));
  }
}

TEST_CASE("pid_wait_until on an odom move keeps waiting while the robot is still short of the target") {
  for (bool point_to_point : {false, true}) {
    CAPTURE(point_to_point);
    Drive chassis = make_chassis();
    configure_chassis(chassis);
    chassis.odom_xyt_set(0.0, 0.0, 0.0);
    chassis.drive_sensor_reset();

    start_move(chassis, point_to_point);
    robot_is_at(chassis, 14.0);  // Well past the look ahead point, 10 inches from the target

    CHECK_FALSE(wait_until_returns(chassis, 30.0, 500));
  }
}

TEST_CASE("odom moves aim each side's wait_until exit at its own sensor plus the look ahead") {
  for (bool point_to_point : {false, true}) {
    for (drive_directions direction : {fwd, rev}) {
      CAPTURE(point_to_point);
      CAPTURE(direction == fwd);
      Drive chassis = make_chassis();
      configure_chassis(chassis);
      chassis.odom_xyt_set(0.0, 0.0, 0.0);
      chassis.drive_sensor_reset();

      // The two sides start a long way apart, as they are after any turn.
      chassis.left_motors[0].fake().position = (std::int32_t)(5.0 * chassis.drive_tick_per_inch());
      chassis.right_motors[0].fake().position = (std::int32_t)(20.0 * chassis.drive_tick_per_inch());
      double left_start = chassis.drive_sensor_left();
      double right_start = chassis.drive_sensor_right();
      REQUIRE(left_start != doctest::Approx(right_start));

      start_move(chassis, point_to_point, direction);

      double look_ahead = direction == fwd ? 7.0 : -7.0;
      CHECK(chassis.leftPID.target_get() == doctest::Approx(left_start + look_ahead));
      CHECK(chassis.rightPID.target_get() == doctest::Approx(right_start + look_ahead));
    }
  }
}
