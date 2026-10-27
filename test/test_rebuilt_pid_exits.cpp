// The exit side of ez::PID after the waits and exits were rebuilt from v3.2.2.  One case per fix, each of which fails on v3.2.2's code:
// the big exit running with a small error set, the MotorGroup overload leaving the big timer alone, the velocity exit waiting for the
// robot to move, real-millisecond timers, the first-motor mA exit, and the stop speed.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// One 10 ms pass of a task that computes and then polls: the sensor reads `cur`, the error is `error`.
exit_output pass(PID& pid, double error, double cur, int period_ms = 10) {
  pid.compute_error(error, cur);
  exit_output out = pid.exit_condition();
  pros::delay(period_ms);
  return out;
}

// Runs passes until an exit fires or `max_ms` of fake time has gone by.  Returns the exit and how long it took.
exit_output run_until_exit(PID& pid, double error, double cur, int max_ms, int& took_ms, int period_ms = 10) {
  std::uint32_t start = pros::millis();
  exit_output out = RUNNING;
  while (out == RUNNING && pros::millis() - start < (std::uint32_t)max_ms) out = pass(pid, error, cur, period_ms);
  took_ms = (int)(pros::millis() - start);
  return out;
}
}  // namespace

TEST_CASE("the big exit runs when a small error is set") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  int took = 0;
  // 2 in off: outside small_error, inside big_error, not moving
  CHECK(run_until_exit(pid, 2.0, 0.0, 1000, took) == BIG_EXIT);
  CHECK(took >= 250);
  CHECK(took < 350);
}

TEST_CASE("the MotorGroup exit_condition overload does not reset the big exit timer") {
  test_stub::reset_all();
  pros::MotorGroup group({1, 2});
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(0, 0.0, 250, 3.0, 0, 0);
  std::uint32_t start = pros::millis();
  exit_output out = RUNNING;
  while (out == RUNNING && pros::millis() - start < 1000) {
    pid.compute_error(2.0, 0.0);
    out = pid.exit_condition(group);
    pros::delay(10);
  }
  CHECK(out == BIG_EXIT);
}

TEST_CASE("the velocity exit waits for the robot to move, or for a second") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  int took = 0;
  // Sitting still at the start is not a stall yet
  CHECK(run_until_exit(pid, 20.0, 0.0, 500, took) == RUNNING);

  // A robot that never moves is armed after a second so a wait cannot hang
  PID pinned(1, 0, 0, 0, "pinned");
  pinned.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  CHECK(run_until_exit(pinned, 20.0, 0.0, 3000, took) == VELOCITY_EXIT);
  CHECK(took >= 1000);

  // A robot that moved and then stopped exits after the velocity exit time
  PID moved(1, 0, 0, 0, "moved");
  moved.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  for (int i = 0; i < 30; i++) CHECK(pass(moved, 20.0, i * 1.0) == RUNNING);  // 100 units/s
  CHECK(run_until_exit(moved, 20.0, 29.0, 1000, took) == VELOCITY_EXIT);
  CHECK(took < 400);
}

TEST_CASE("the exit timers count real milliseconds when the PID loop runs at 20 ms") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(100, 1.0, 0, 0.0, 0, 0);
  int took = 0;
  CHECK(run_until_exit(pid, 0.5, 0.0, 1000, took, 20) == SMALL_EXIT);
  CHECK(took >= 100);
  CHECK(took <= 140);  // 220 ms when each call counts a flat 10
}

TEST_CASE("the mA exit looks at the one motor it is given, and a disconnected motor counts as over current") {
  test_stub::reset_all();
  pros::Motor first(1), second(2);
  second.fake().over_current = true;  // e.g. an intake on the power take-off
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  std::uint32_t start = pros::millis();
  exit_output out = RUNNING;
  while (out == RUNNING && pros::millis() - start < 500) {
    pid.compute_error(5.0, 0.0);
    out = pid.exit_condition(first);
    pros::delay(10);
  }
  CHECK(out == RUNNING);

  first.fake().disconnected = true;
  out = RUNNING;
  start = pros::millis();
  while (out == RUNNING && pros::millis() - start < 500) {
    pid.compute_error(5.0, 0.0);
    out = pid.exit_condition(first);
    pros::delay(10);
  }
  CHECK(out == mA_EXIT);
}

TEST_CASE("the small exit's timer pauses while the robot is moving and picks up where it left off") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(300, 1.0, 0, 0.0, 0, 0);
  pid.stop_speed_set(10.0);
  // Inside the error band but crawling along at 50 units/s: a speed over the stop speed, so no time counts
  double cur = 0.0;
  std::uint32_t start = pros::millis();
  exit_output out = RUNNING;
  while (out == RUNNING && pros::millis() - start < 1000) {
    cur += 0.5;
    out = pass(pid, 0.5, cur);
  }
  CHECK(out == RUNNING);
  // It stops: now the time counts, from where it was (it did not reset, and nothing was banked while moving)
  int took = 0;
  CHECK(run_until_exit(pid, 0.5, cur, 1000, took) == SMALL_EXIT);
  CHECK(took >= 300);
  CHECK(took < 500);
}

TEST_CASE("the velocity exit uses the stop speed") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  pid.stop_speed_set(20.0);
  // Moves first, then creeps along at 10 units/s: under a 20 units/s stop speed, so the velocity exit counts it as stopped
  double cur = 0.0;
  for (int i = 0; i < 30; i++) {
    cur += 1.0;
    pass(pid, 50.0, cur);
  }
  exit_output out = RUNNING;
  std::uint32_t start = pros::millis();
  while (out == RUNNING && pros::millis() - start < 1000) {
    cur += 0.1;
    out = pass(pid, 50.0, cur);
  }
  CHECK(out == VELOCITY_EXIT);
}

TEST_CASE("the drive's stop speeds have defaults, units overloads and getters, and refuse a speed that can't be one") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  CHECK(chassis.pid_distance_exit_stop_speed_get() == doctest::Approx(8.0));
  CHECK(chassis.pid_angle_exit_stop_speed_get() == doctest::Approx(13.0));

  chassis.pid_distance_exit_stop_speed_set(2.5);
  CHECK(chassis.pid_distance_exit_stop_speed_get() == doctest::Approx(2.5));
  chassis.pid_distance_exit_stop_speed_set(ez::QSpeed(3.0 * ez::inch / ez::second));
  CHECK(chassis.pid_distance_exit_stop_speed_get() == doctest::Approx(3.0));
  CHECK(chassis.leftPID.stop_speed_get() == doctest::Approx(3.0));
  CHECK(chassis.xyPID.stop_speed_get() == doctest::Approx(3.0));

  chassis.pid_angle_exit_stop_speed_set(ez::QAngularSpeed(6.0 * ez::degree / ez::second));
  CHECK(chassis.pid_angle_exit_stop_speed_get() == doctest::Approx(6.0));
  CHECK(chassis.swingPID.stop_speed_get() == doctest::Approx(6.0));
  CHECK(chassis.current_a_odomPID.stop_speed_get() == doctest::Approx(6.0));

  chassis.pid_angle_exit_stop_speed_set(0.0);
  chassis.pid_angle_exit_stop_speed_set(-1.0);
  chassis.pid_angle_exit_stop_speed_set(std::nan(""));
  CHECK(chassis.pid_angle_exit_stop_speed_get() == doctest::Approx(6.0));
}

TEST_CASE("the secondary (imu) velocity channel is off by default") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  CHECK_FALSE(chassis.turnPID.velocity_sensor_secondary_toggle_get());
  CHECK_FALSE(chassis.swingPID.velocity_sensor_secondary_toggle_get());
  CHECK_FALSE(chassis.xyPID.velocity_sensor_secondary_toggle_get());
  CHECK_FALSE(chassis.leftPID.velocity_sensor_secondary_toggle_get());
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 500, true);  // still there for a team that wants it
  CHECK(chassis.turnPID.velocity_sensor_secondary_toggle_get());
}
