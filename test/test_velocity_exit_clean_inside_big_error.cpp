// A velocity exit or mA exit that ends a wait inside big_error is a clean finish, however fast the robot is still moving.  The velocity exit fires
// at 5 in/s (5 deg/s) while the stop speed is 1.5 in/s (4 deg/s), so a robot arriving between the two used to read interfered on a healthy run.
// Outside big_error the exit still reads interfered.  Only the verdict changes, not when the wait returns.
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

void over_current(Drive& chassis) {
  for (auto& m : chassis.left_motors) m.fake().over_current = true;
  for (auto& m : chassis.right_motors) m.fake().over_current = true;
}

// Between the stop speed and the velocity exit speed, for a drive in in/s and for a turn or swing in deg/s
constexpr double DRIVE_CREEP = 3.0;
constexpr double TURN_CREEP = 4.5;
constexpr double STOPPED = 0.0;

// pid_wait() on a drive that ends on the velocity exit with `error` left
bool drive_wait(double error, double creep, bool& interfered) {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_drive_set(24.0, 100);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int pass) {
                             chassis.leftPID.compute_error(error, sensor(pass, creep));
                             chassis.rightPID.compute_error(error, sensor(pass, creep));
                           },
                           400);
  interfered = chassis.interfered;
  return returned;
}

// pid_wait_until() on a drive whose target is never reached, so the velocity exit ends it
bool drive_wait_until(double error, double creep, bool& interfered) {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_drive_set(48.0, 100);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  bool returned = run_wait([&] { chassis.pid_wait_until(40_in); },
                           [&](int pass) {
                             chassis.leftPID.compute_error(error, sensor(pass, creep));
                             chassis.rightPID.compute_error(error, sensor(pass, creep));
                           },
                           400);
  interfered = chassis.interfered;
  return returned;
}

// pid_wait() on a turn or a swing that ends on the velocity exit with `error` left
bool turn_wait(bool swing, double error, double creep, bool& interfered) {
  Drive chassis = make_chassis();
  ready(chassis);
  if (swing) {
    chassis.pid_swing_set(ez::LEFT_SWING, 90.0, 100);
    chassis.pid_swing_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  } else {
    chassis.pid_turn_set(90.0, 100);
    chassis.pid_turn_exit_condition_set(0, 0.0, 0, 3.0, 100, 0);
  }
  PID& pid = swing ? chassis.swingPID : chassis.turnPID;
  bool returned = run_wait([&] { chassis.pid_wait(); }, [&](int pass) { pid.compute_error(error, sensor(pass, creep)); }, 400);
  interfered = chassis.interfered;
  return returned;
}

// pid_wait() on a drive that ends on the mA exit with `error` left, the velocity exit off
bool drive_ma_wait(double error, double creep, bool& interfered) {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_drive_set(24.0, 100);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 3.0, 0, 100);
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int pass) {
                             if (pass >= 40) over_current(chassis);  // after the ramp, so the mA window runs on a robot at `creep`
                             chassis.leftPID.compute_error(error, sensor(pass, creep));
                             chassis.rightPID.compute_error(error, sensor(pass, creep));
                           },
                           400);
  interfered = chassis.interfered;
  return returned;
}

// pid_wait_quick() on an odom point whose xy error stays at `error` while the robot crawls at `creep` in/s
bool odom_quick_wait(double error, double creep, bool& interfered) {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_odom_drive_exit_condition_set(0, 0.0, 0, 3.0, 500, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 0);  // the heading stays converged
  chassis.pid_odom_ptp_set({{0.0, 100.0, 0.0}, fwd, 100});
  bool returned = run_wait([&] { chassis.pid_wait_quick(); },
                           [&](int pass) {
                             double y = sensor(pass, creep);
                             DriveTestAccess::odom_current(chassis) = {0.0, y, 0.0};
                             chassis.xyPID.compute_error(error, y);
                             chassis.current_a_odomPID.compute_error(0.0, chassis.current_a_odomPID.cur);
                           },
                           500);
  interfered = chassis.interfered;
  return returned;
}
}  // namespace

TEST_CASE("a drive arriving between the stop speed and the velocity exit speed, inside big_error, is not interfered") {
  bool interfered = true;
  REQUIRE(drive_wait(2.0, DRIVE_CREEP, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("a turn arriving between the stop speed and the velocity exit speed, inside big_error, is not interfered") {
  bool interfered = true;
  REQUIRE(turn_wait(false, 2.0, TURN_CREEP, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("a swing arriving between the stop speed and the velocity exit speed, inside big_error, is not interfered") {
  bool interfered = true;
  REQUIRE(turn_wait(true, 2.0, TURN_CREEP, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("pid_wait_until() on a creeping drive inside big_error gives the same verdict as pid_wait()") {
  bool interfered = true;
  REQUIRE(drive_wait_until(2.0, DRIVE_CREEP, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("an odom quick wait on a creeping robot inside big_error gives the same verdict as pid_wait()") {
  bool interfered = true;
  REQUIRE(odom_quick_wait(2.0, DRIVE_CREEP, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("an mA exit inside big_error while the robot is still moving is not interfered") {
  bool interfered = true;
  REQUIRE(drive_ma_wait(2.0, DRIVE_CREEP, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("an mA exit inside big_error with the robot stopped is not interfered") {
  bool interfered = true;
  REQUIRE(drive_ma_wait(2.0, STOPPED, interfered));
  CHECK_FALSE(interfered);
}

TEST_CASE("an mA exit outside big_error is interfered") {
  bool interfered = false;
  REQUIRE(drive_ma_wait(5.0, STOPPED, interfered));
  CHECK(interfered);
  REQUIRE(drive_ma_wait(5.0, DRIVE_CREEP, interfered));
  CHECK(interfered);
}

TEST_CASE("a robot pinned outside big_error is interfered, in pid_wait(), pid_wait_until() and an odom quick wait") {
  bool interfered = false;
  REQUIRE(drive_wait(5.0, STOPPED, interfered));
  CHECK(interfered);
  interfered = false;
  REQUIRE(turn_wait(false, 6.0, STOPPED, interfered));
  CHECK(interfered);
  interfered = false;
  REQUIRE(turn_wait(true, 6.0, STOPPED, interfered));
  CHECK(interfered);
  interfered = false;
  REQUIRE(drive_wait_until(5.0, STOPPED, interfered));
  CHECK(interfered);
  interfered = false;
  REQUIRE(odom_quick_wait(5.0, STOPPED, interfered));
  CHECK(interfered);
}
