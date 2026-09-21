// chassis.interfered is set when a wait gives up because the robot stalled or its motors were pulling too
// much current.  pid_wait() and the distance and angle waits already do that.  The waits for a point and for a
// pure pursuit index have to as well, otherwise a check like `if (chassis.interfered)` after them never fires.
// When the pure pursuit index wait gives up it also prints where it gave up and where it was headed, that has to
// be the waypoint the caller asked about.
#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

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

TEST_CASE("pid_wait_until_index_started names the waypoint it was waiting for when it gives up") {
  Drive chassis = make_chassis();
  configure(chassis);
  // The path the robot follows has an injected point every half inch, so the injected points are nowhere near
  // the waypoints.  Waiting on waypoint 1 has to name (0, 36), not a point next to the start of the path.
  chassis.pid_odom_set({{{0.0, 12.0}, fwd, 110}, {{0.0, 36.0}, fwd, 110}, {{0.0, 60.0}, fwd, 110}});
  REQUIRE(DriveTestAccess::injected_pp_index(chassis).size() == 4);
  only_the_current_exit_ends_waits(chassis);
  motors_pull_too_much_current(chassis, true);
  REQUIRE(chassis.pid_print_toggle_get());

  // What the wait printed when it gave up on waypoint `index`.
  auto printed_waiting_for = [&](int index) {
    return test_stub::capture_stdout([&] { CHECK(returns(500, [&] { chassis.pid_wait_until_index_started(index); })); });
  };

  std::string printed = printed_waiting_for(1);
  INFO(printed);
  CHECK(printed.find("instead of (0.00, 36.00)") != std::string::npos);

  // The first and last waypoints name their own points too.
  printed = printed_waiting_for(0);
  INFO(printed);
  CHECK(printed.find("instead of (0.00, 12.00)") != std::string::npos);

  printed = printed_waiting_for(2);
  INFO(printed);
  CHECK(printed.find("instead of (0.00, 60.00)") != std::string::npos);
}
