// A velocity or mA exit that ends a wait inside big_error is a clean finish, at any speed: the robot got where it was going.  Outside
// big_error it reads interfered.  Only the verdict changes, not when the wait returns.
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void ready(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
}

// Runs `wait` with `hook` on every delay, at most `max_delays` delays.  Returns whether it came back.
bool run_wait(const std::function<void()>& wait, const std::function<void(int)>& hook, int max_delays) {
  static int g_pass;
  static const std::function<void(int)>* g_hook;
  g_pass = 0;
  g_hook = &hook;
  test_stub::g_clock.on_delay = [] { (*g_hook)(++g_pass); };
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  return returned;
}

// The sensor runs at 100 units per second for 30 passes, then at `creep` units per second
double sensor(int pass, double creep) { return pass < 30 ? pass * 1.0 : 30.0 + (pass - 30) * creep / 100.0; }
}  // namespace

TEST_CASE("a drive that stops inside big_error on the velocity exit is not interfered") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_drive_set(24.0, 100);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int pass) {
                             chassis.leftPID.compute_error(2.0, sensor(pass, 0.0));
                             chassis.rightPID.compute_error(2.0, sensor(pass, 0.0));
                           },
                           300);
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("a drive that stops outside big_error on the velocity exit is interfered") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_drive_set(24.0, 100);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int pass) {
                             chassis.leftPID.compute_error(5.0, sensor(pass, 0.0));
                             chassis.rightPID.compute_error(5.0, sensor(pass, 0.0));
                           },
                           300);
  REQUIRE(returned);
  CHECK(chassis.interfered);
}

TEST_CASE("a drive that ends on the velocity exit inside big_error but still creeping over the stop speed is not interfered") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_drive_set(24.0, 100);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int pass) {
                             chassis.leftPID.compute_error(2.0, sensor(pass, 3.0));
                             chassis.rightPID.compute_error(2.0, sensor(pass, 3.0));
                           },
                           300);
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("a turn that stops inside big_error on the velocity exit is not interfered, and outside it is") {
  for (double error : {2.0, 6.0}) {
    Drive chassis = make_chassis();
    ready(chassis);
    chassis.pid_turn_set(90.0, 100);
    chassis.pid_turn_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
    bool returned = run_wait([&] { chassis.pid_wait(); }, [&](int pass) { chassis.turnPID.compute_error(error, sensor(pass, 0.0)); }, 300);
    REQUIRE(returned);
    CHECK(chassis.interfered == (error > 3.0));
  }
}
