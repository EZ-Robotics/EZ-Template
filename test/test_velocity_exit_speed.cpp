// The velocity exit gives up at the stop speed, the same number the small and big exits wait on.  A robot creeping against a force under the stop
// speed counts as stopped, and one moving over it does not.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// Runs 10 ms passes of a task that computes and then polls, with the sensor moving `speed` units per second, until an exit fires or `max_ms` is up
exit_output creep(PID& pid, double speed, int max_ms, int& took_ms) {
  double cur = 0.0;
  for (int i = 0; i < 30; i++) {  // gets going first, so the velocity exit has seen the robot move
    cur += 1.0;
    pid.compute_error(50.0, cur);
    pid.exit_condition();
    pros::delay(10);
  }
  std::uint32_t start = pros::millis();
  exit_output out = RUNNING;
  while (out == RUNNING && pros::millis() - start < (std::uint32_t)max_ms) {
    cur += speed / 100.0;
    pid.compute_error(50.0, cur);
    out = pid.exit_condition();
    pros::delay(10);
  }
  took_ms = (int)(pros::millis() - start);
  return out;
}
}  // namespace

TEST_CASE("a robot creeping under the stop speed ends the wait") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  pid.stop_speed_set(5.0);
  int took = 0;
  CHECK(creep(pid, 3.0, 2000, took) == VELOCITY_EXIT);
  CHECK(took < 600);
}

TEST_CASE("a robot moving over the stop speed does not end the wait") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  pid.stop_speed_set(5.0);
  int took = 0;
  CHECK(creep(pid, 8.0, 2000, took) == RUNNING);
}

TEST_CASE("velocity_sensor_main_exit_set sets the stop speed, in units per 10 ms tick") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  CHECK(pid.velocity_sensor_main_exit_get() == doctest::Approx(0.05));
  pid.stop_speed_set(1.5);
  pid.velocity_sensor_main_exit_set(0.02);
  CHECK(pid.velocity_sensor_main_exit_get() == doctest::Approx(0.02));
  CHECK(pid.stop_speed_get() == doctest::Approx(2.0));
  pid.exit_condition_set(0, 0.0, 0, 0.0, 100, 0);
  int took = 0;
  CHECK(creep(pid, 3.0, 1000, took) == RUNNING);  // over 2 units per second now
}
