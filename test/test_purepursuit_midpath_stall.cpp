// Before pure pursuit's last point, pid_wait() breaks out early once xy reports a stall
// (current draw, or zero velocity) and angle has settled -- PR #408 widened "settled" from
// "angle itself stalled too" to "angle exited for any reason", because requiring a stall
// specifically from angle left the break unreachable: on an ordinary mostly-straight segment
// angle latches SMALL_EXIT almost immediately, and once a PID's cached exit result stops being
// RUNNING, exit_condition() is never called on it again for the rest of the path.
//
// That widened condition still reads the same latched value, though. A pure pursuit corner
// sharp enough to push the heading error past odom_turn_bias's cutoff (~84 degrees at the
// library default of 0.9, see pid_tasks.cpp) legitimately drives xy's commanded output -- and so
// its own velocity reading -- to zero while the robot pivots to face the next segment. If
// angle's settled flag is still the one it latched on an earlier straight stretch, that ordinary
// cornering pause alone satisfies the break, and pid_wait() returns with the path nowhere near
// its last point while ez_auto_task keeps driving it in the background.
//
// (The first case below was reversed when pid_wait() moved to a progress-based stuck check; see its comment.)
//
// These tests never run ez_auto_task/pp_task: nothing advances pp_index or calls compute_error,
// so xyPID/current_a_odomPID.error stay exactly where the test sets them. That is enough to
// drive exit_condition() -- pid_wait() is pure PID/geometry logic here, no motion synthesis.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// xy only stalls on current draw; angle only settles on error (small_error/small_exit_time).
// Velocity and mA are left disabled on whichever side isn't meant to drive the test, the same
// way test_wait_until_point_interfered.cpp isolates a single exit path.
void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}, {{0.0, 36.0, 0.0}, fwd, 110}});
  // Starting a move doesn't touch xyPID/current_a_odomPID's exit conditions (only
  // pid_odom_..._exit_condition_set does), but re-asserting them here keeps this test's intent
  // explicit and matches the pattern in test_wait_until_point_interfered.cpp.
  chassis.xyPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
}

// Runs `wait` with the fake pros::delay() set to throw after `max_delays` calls, so a wait that
// never returns fails the test instead of hanging it. Mirrors the helper in
// test_wait_until_point_interfered.cpp.
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

Drive* g_chassis = nullptr;
int g_delays = 0;
bool g_simulate_corner = false;

// Scripts a mid-path scenario across the single blocking pid_wait() call via the fake clock's
// on_delay hook. Angle starts aligned (error 0) long enough to latch its cached exit the way an
// ordinary straight segment does. If g_simulate_corner, angle then swings to a large heading
// error (simulating a sharp corner) well before xy ever stalls. xy's stall (current draw) always
// starts later still, after either branch's angle behavior is already established.
void scripted_corner_then_stall() {
  ++g_delays;
  if (g_simulate_corner && g_delays == 15) {
    g_chassis->current_a_odomPID.error = 90.0;
  }
  if (g_delays == 25) {
    g_chassis->left_motors[0].fake().over_current = true;
    g_chassis->right_motors[0].fake().over_current = true;
  }
}

void run_scenario(Drive& chassis, bool simulate_corner) {
  g_chassis = &chassis;
  g_delays = 0;
  g_simulate_corner = simulate_corner;
  chassis.current_a_odomPID.error = 0.0;
  test_stub::g_clock.on_delay = scripted_corner_then_stall;
}
}  // namespace

// This case used to assert that pid_wait() never returns here.  But the script is a robot stuck against
// something, drawing too much current for 750 ms, with its heading 90 degrees off: that is a stall at a
// corner, not a pause between segments, and waiting on it hung the auton for the rest of the match.  A stall
// has to end the wait at any heading error, so it now asserts the opposite.  test_pp_wait_stuck.cpp covers
// the healthy pause the old assertion was protecting (a slow pivot at a corner does not end the wait).
TEST_CASE("pid_wait ends a pure pursuit path when xy stalls on current while angle is mid-corner") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  run_scenario(chassis, /*simulate_corner=*/true);

  bool done = returns(300, [&] { chassis.pid_wait(); });
  test_stub::g_clock.on_delay = nullptr;

  // Nothing in this test drives pp_task, so the path never advances past its first point.
  // Returning at all can only be the mid-path early break firing.
  REQUIRE(DriveTestAccess::pp_index(chassis) != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  CHECK(done);
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait still ends a pure pursuit path when xy stalls while angle stays settled") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  run_scenario(chassis, /*simulate_corner=*/false);

  bool done = returns(300, [&] { chassis.pid_wait(); });
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(DriveTestAccess::pp_index(chassis) != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  // This is PR #408's fix: a real mid-path stall (angle settled, xy stuck) must still end the
  // wait instead of holding the drivetrain at full voltage for the rest of the path.
  CHECK(done);
  CHECK(chassis.interfered);
}

TEST_CASE("pid_wait ends a pure pursuit path once angle catches back up to the target after a corner") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);
  run_scenario(chassis, /*simulate_corner=*/true);

  // Straighten back out well before xy's mA timeout would fire (pass 25 + 10), so the break can
  // only succeed because angle is currently close again, not because it never left.
  g_chassis = &chassis;
  test_stub::g_clock.on_delay = [] {
    scripted_corner_then_stall();
    if (g_delays == 22) g_chassis->current_a_odomPID.error = 0.0;
  };

  bool done = returns(300, [&] { chassis.pid_wait(); });
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(DriveTestAccess::pp_index(chassis) != (int)DriveTestAccess::pp_movements(chassis).size() - 1);
  CHECK(done);
}
