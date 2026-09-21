// chassis.interfered is set when a wait gives up because the robot stalled or its motors were pulling too
// much current.  pid_wait() and the distance and angle waits already do that.  The waits for a point and for a
// pure pursuit index have to as well, otherwise a check like `if (chassis.interfered)` after them never fires.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
// A Drive owns a lock and a background task that points back at it, so it cannot be copied.  It is built where the
// caller needs it (returning a prvalue copies nothing) and then set up in place by configure().
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // Position exits only.  They fire almost at once because the fake PIDs sit at zero error, which ends a wait
  // without interference.
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
}

// Leaves the motors pulling too much current as the only thing that can end a wait, so the wait ends with
// interference.  Call this after the move is started, starting a move copies the exit conditions.
void only_the_current_exit_ends_waits(Drive& chassis) {
  chassis.xyPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.current_a_odomPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
}

void motors_pull_too_much_current(Drive& chassis, bool over_current) {
  chassis.left_motors[0].fake().over_current = over_current;
  chassis.right_motors[0].fake().over_current = over_current;
}

// Runs `wait` with the fake pros::delay() set to throw after `max_delays` calls, so a wait that never returns
// fails the test instead of hanging it.
template <typename F>
bool returns(int max_delays, F&& wait) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool done = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    done = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return done;
}

void start_point_move(Drive& chassis) { chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 60}); }

void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}, {{0.0, 36.0, 0.0}, fwd, 110}});
}
}  // namespace

TEST_CASE("pid_wait_until_point sets interfered when the motors pull too much current") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_point_move(chassis);
  only_the_current_exit_ends_waits(chassis);
  motors_pull_too_much_current(chassis, true);
  REQUIRE_FALSE(chassis.interfered);

  CHECK(returns(500, [&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); }));
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait_until_point leaves interfered alone when the wait ends normally") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_point_move(chassis);
  motors_pull_too_much_current(chassis, false);

  CHECK(returns(500, [&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); }));
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until with a pose sets interfered the same way") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_point_move(chassis);
  only_the_current_exit_ends_waits(chassis);
  motors_pull_too_much_current(chassis, true);

  CHECK(returns(500, [&] { chassis.pid_wait_until(pose{0.0, 24.0, 0.0}); }));
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait_until_index_started sets interfered when the motors pull too much current") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  REQUIRE(DriveTestAccess::injected_pp_index(chassis).size() >= 2);
  only_the_current_exit_ends_waits(chassis);
  motors_pull_too_much_current(chassis, true);
  REQUIRE_FALSE(chassis.interfered);

  CHECK(returns(500, [&] { chassis.pid_wait_until_index_started(0); }));
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait_until_index_started leaves interfered alone when the wait ends normally") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  motors_pull_too_much_current(chassis, false);

  CHECK(returns(500, [&] { chassis.pid_wait_until_index_started(0); }));
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until_index sets interfered when the motors pull too much current") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  only_the_current_exit_ends_waits(chassis);
  motors_pull_too_much_current(chassis, true);

  CHECK(returns(1000, [&] { chassis.pid_wait_until_index(0); }));
  CHECK(chassis.interfered);
}

TEST_CASE("starting a new motion clears interfered") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_point_move(chassis);
  only_the_current_exit_ends_waits(chassis);
  motors_pull_too_much_current(chassis, true);
  returns(500, [&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); });
  REQUIRE(chassis.interfered);

  motors_pull_too_much_current(chassis, false);
  start_point_move(chassis);
  CHECK_FALSE(chassis.interfered);
}
