// tracking [H4, M3]: construct a Drive with fake IMEs and one fake left
// tracker (3.5 in offset) and a fake back tracker (2.0 in); mark calibration
// complete; call odom_xyt_set(0, 0, 90); run ez_tracking_task() once with no
// sensor change; assert x and y are within 1e-6 of 0. Then set odom
// disabled, advance the fake wheels 100 in, enable, run one pass, assert
// pose unchanged.
// competition [#365 fix]: with the fake status moving disabled ->
// autonomous, running the task body once after a pid_drive_set leaves
// mode == DRIVE; moving autonomous -> driver with no disabled gap sets
// mode == DISABLE once.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Simulates a single ez_auto_task() pass. pros::Task never actually runs
// its callable (see stub/pros/rtos.hpp), and the task body itself is
// `while (true) { ... pros::delay(...); }`, so the fake pros::delay() is
// set to throw right after the first pass completes and this catches it.
void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}
}  // namespace

TEST_CASE("tracking [H4] no sensor change after priming leaves x/y at 0") {
  Drive chassis = make_chassis();

  tracking_wheel left_tracker(1, 3.25, 3.5);
  tracking_wheel back_tracker(2, 3.25, 2.0);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_back_set(&back_tracker);

  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // Primes h_last/l_last/r_last/t_last to the sensor readings as they stand
  // right now (post odom_xyt_set, which also moves the fake IMU to 90 via
  // drive_angle_set -> drive_imu_reset). Without this, the first real
  // tracking pass below would see a spurious 90-degree "delta" from the
  // t_last=0 a freshly-constructed Drive starts with.
  chassis.drive_sensor_reset();

  chassis.ez_tracking_task();

  CHECK(chassis.odom_x_get() == doctest::Approx(0.0).epsilon(1e-6));
  CHECK(chassis.odom_y_get() == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("tracking [M3] disabling odom before a sensor jump prevents that jump from ever showing up") {
  Drive chassis = make_chassis();

  tracking_wheel left_tracker(1, 3.25, 3.5);
  tracking_wheel back_tracker(2, 3.25, 2.0);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_back_set(&back_tracker);

  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.drive_sensor_reset();
  chassis.ez_tracking_task();
  double x_before = chassis.odom_x_get();
  double y_before = chassis.odom_y_get();

  chassis.odom_enable(false);

  // Advance the fake wheels 100in worth of ticks (drive_tick_per_inch() is
  // the counts-per-inch conversion the real sensors would report).
  double ticks = 100.0 * chassis.drive_tick_per_inch();
  chassis.left_motors[0].fake().position = (std::int32_t)ticks;
  chassis.right_motors[0].fake().position = (std::int32_t)ticks;

  // ez_tracking_task() re-primes to the *current* sensor readings every time
  // it's called while odom is disabled (that's the whole mechanism this test
  // is pinning -- see tracking_prime()'s doc comment) -- so at least one call
  // has to land while still disabled for the jump to actually be absorbed.
  chassis.ez_tracking_task();
  chassis.odom_enable(true);
  chassis.ez_tracking_task();

  CHECK(chassis.odom_x_get() == doctest::Approx(x_before).epsilon(1e-6));
  CHECK(chassis.odom_y_get() == doctest::Approx(y_before).epsilon(1e-6));
}

TEST_CASE("competition [#365 fix] disabled -> autonomous does not cancel the first motion") {
  Drive chassis = make_chassis();

  test_stub::g_competition.disabled = true;
  test_stub::g_competition.autonomous = false;
  run_one_auto_task_pass(chassis);  // syncs last_was_autonomous to false

  // Field control moves to autonomous...
  test_stub::g_competition.disabled = false;
  test_stub::g_competition.autonomous = true;
  // ...and the autonomous task's first setter runs before the next
  // ez_auto_task() pass sees the status change (matches the comment in
  // pid_tasks.cpp: entering autonomous must NOT cancel this).
  chassis.pid_drive_set(12.0, 100);
  REQUIRE(chassis.drive_mode_get() == DRIVE);

  run_one_auto_task_pass(chassis);

  CHECK(chassis.drive_mode_get() == DRIVE);
}

TEST_CASE("competition [#365 fix] autonomous -> driver with no disabled gap forces DISABLE") {
  Drive chassis = make_chassis();

  test_stub::g_competition.disabled = false;
  test_stub::g_competition.autonomous = true;
  run_one_auto_task_pass(chassis);  // syncs last_was_autonomous to true

  chassis.pid_drive_set(12.0, 100);
  REQUIRE(chassis.drive_mode_get() == DRIVE);

  // Straight to driver control, no disabled period in between.
  test_stub::g_competition.autonomous = false;

  run_one_auto_task_pass(chassis);

  CHECK(chassis.drive_mode_get() == DISABLE);
}
