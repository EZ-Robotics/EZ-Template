// The waits after they were rebuilt from v3.2.2.  One case per fix, each of which fails on v3.2.2's code (or on the first rebuild commit).  Nothing
// steps the drive task: each case scripts the PIDs' errors and sensors from the fake clock's on_delay hook, and the fake pros::delay() throws
// after a set number of calls so a wait that never returns fails the case instead of hanging it.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  detail::print_sink = nullptr;
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Runs `wait` with `hook` on every delay and at most `max_delays` delays.  Returns whether it came back, and how many passes it took.
struct Outcome {
  bool returned;
  int passes;
  std::string printed;
};
Outcome run_wait(const std::function<void()>& wait, const std::function<void(int)>& hook, int max_delays) {
  static int g_pass;
  static const std::function<void(int)>* g_hook;
  g_pass = 0;
  g_hook = &hook;
  test_stub::g_clock.on_delay = [] { (*g_hook)(++g_pass); };
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  Outcome out{true, 0, ""};
  out.printed = test_stub::capture_stdout([&] {
    try {
      wait();
    } catch (test_stub::StopLoop&) {
      out.returned = false;
    }
  });
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  out.passes = g_pass;
  return out;
}

void set_sensors(Drive& chassis, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * chassis.drive_tick_per_inch());
  chassis.left_motors[0].fake().position = ticks;
  chassis.right_motors[0].fake().position = ticks;
}

void ready(Drive& chassis) { DriveTestAccess::imu_calibration_complete(chassis) = true; }

std::vector<odom> path3() { return {{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110}}; }
int last_index(Drive& chassis) { return (int)DriveTestAccess::pp_movements(chassis).size() - 1; }
}  // namespace

TEST_CASE("the wait_until loops delay once per pass") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_turn_exit_condition_set(100, 2.0, 0, 0.0, 0, 0);
  chassis.pid_turn_set(0.0, 100);
  std::uint32_t start = test_stub::g_clock.now_ms;
  Outcome o = run_wait([&] { chassis.pid_wait_until(0.0); }, [&](int) { chassis.turnPID.compute_error(0.5, 0.0); }, 60);
  CHECK(o.returned);
  CHECK(test_stub::g_clock.now_ms - start < 160);
}

TEST_CASE("a drive wait_until prints the distance it was given next to the distance driven") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_print_toggle(true);
  chassis.pid_drive_exit_condition_set(90, 1.0, 0, 0.0, 0, 0);
  set_sensors(chassis, 50.0);  // the encoders are not at zero when the motion starts
  chassis.pid_drive_set(24.0, 100);
  Outcome o = run_wait([&] { chassis.pid_wait_until(6.0); },
                       [&](int) {
                         chassis.leftPID.compute_error(0.2, 50.0);
                         chassis.rightPID.compute_error(0.2, 50.0);
                       },
                       60);
  REQUIRE(o.returned);
  CHECK(o.printed.find("instead of 6.00") != std::string::npos);
  CHECK(o.printed.find("instead of 56") == std::string::npos);
}

TEST_CASE("pid_wait_until_index returns on an index outside the path") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_odom_pp_set(path3());
  Outcome o = run_wait([&] { chassis.pid_wait_until_index(9); }, [](int) {}, 30);
  CHECK(o.returned);
  CHECK(o.passes <= 2);
  CHECK(o.printed.find("not within range") != std::string::npos);
}

TEST_CASE("the point and index waits set interfered when they give up, and the mode is checked") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{0.0, 24.0}, fwd, 100});
  chassis.xyPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.current_a_odomPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.left_motors[0].fake().over_current = true;
  chassis.right_motors[0].fake().over_current = true;
  Outcome o = run_wait([&] { chassis.pid_wait_until_point({0.0, 200.0, 0.0}); }, [](int) {}, 100);
  CHECK(o.returned);
  CHECK(chassis.interfered);

  // Called outside an odom motion it says so and returns at once
  Drive drive = make_chassis();
  ready(drive);
  drive.pid_drive_set(10.0, 100);
  Outcome m = run_wait([&] { drive.pid_wait_until_point({5.0, 5.0, 0.0}); }, [](int) {}, 30);
  CHECK(m.returned);
  CHECK(m.passes <= 2);
  CHECK(m.printed.find("Mode needs") != std::string::npos);
}

TEST_CASE("pid_wait_quick_chain on a turn to a point waits for the turn") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 110);
  REQUIRE(chassis.mode == TURN_TO_POINT);
  Outcome o = run_wait([&] { chassis.pid_wait_quick_chain(); }, [](int) {}, 500);
  CHECK(o.printed.find("Not in a supported drive mode") == std::string::npos);
  CHECK(o.passes > 0);
}

TEST_CASE("an odom wait leaves the heading target at the angle nearest the IMU") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0e6, 250, 1.0e6, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 1.0e6, 250, 1.0e6, 0, 0);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.imu->fake_rotation = 270.0;
  chassis.pid_odom_ptp_set({{0.0, 12.0, -90.0}, fwd, 70});  // drives along 0 degrees, ends facing -90 (the IMU's 270)
  Outcome o = run_wait([&] { chassis.pid_wait(); },
                       [&](int) {
                         chassis.xyPID.compute_error(0.0, 0.0);
                         chassis.current_a_odomPID.compute_error(0.0, 0.0);
                       },
                       60);
  REQUIRE(o.returned);
  CHECK(chassis.headingPID.target_get() == doctest::Approx(270.0));
}

TEST_CASE("an odom pid_wait_until(distance) returns when the move ends short of it, and not before the distance") {
  // The move ends (xy settled at the point) 6 inches short of the 30 inches waited for
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.drive_sensor_reset();
  chassis.pid_odom_ptp_set({{0.0, 24.0}, fwd, 60});
  chassis.xyPID.exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.leftPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 0);
  chassis.rightPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 0);
  set_sensors(chassis, 24.0);
  Outcome o = run_wait([&] { chassis.pid_wait_until(30.0); },
                       [&](int) {
                         chassis.xyPID.compute_error(0.2, 0.0);
                         chassis.current_a_odomPID.compute_error(0.0, 0.0);
                       },
                       100);
  CHECK(o.returned);
  CHECK_FALSE(chassis.interfered);

  // Driving at 0.1 in a pass, the robot passes 7.5 inches (where leftPID and rightPID are aimed) without the wait for 20 inches ending
  Drive moving = make_chassis();
  ready(moving);
  moving.odom_xyt_set(0.0, 0.0, 0.0);
  moving.drive_sensor_reset();
  moving.pid_odom_ptp_set({{0.0, 40.0}, fwd, 60});
  moving.leftPID.exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  moving.rightPID.exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  moving.xyPID.exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  double look_ahead = moving.odom_look_ahead_get();
  Outcome m = run_wait([&] { moving.pid_wait_until(20.0); },
                       [&](int pass) {
                         double at = 0.1 * pass;
                         set_sensors(moving, at);
                         moving.xyPID.compute_error(40.0 - at, at);
                         moving.leftPID.compute(at);
                         moving.rightPID.compute(at);
                       },
                       400);
  REQUIRE(m.returned);
  CHECK(m.passes * 0.1 > 19.5);
  CHECK(m.passes * 0.1 > look_ahead + 2.0);
}

TEST_CASE("a turn wait_until checkpoint already passed at entry returns at once") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_turn_set(90.0, 100);
  chassis.imu->fake_rotation = 60.0;  // already past 45 by the time the wait is called
  Outcome o = run_wait([&] { chassis.pid_wait_until(45.0); }, [](int) {}, 40);
  CHECK(o.returned);
  CHECK(o.passes <= 3);
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("a blocked pid_wait_until_index does not restart its timers for the second stage") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_odom_pp_set(path3());
  chassis.xyPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 200);
  chassis.current_a_odomPID.exit_condition_set(0, 0.0, 0, 0.0, 0, 200);
  chassis.left_motors[0].fake().over_current = true;
  chassis.right_motors[0].fake().over_current = true;
  std::uint32_t start = test_stub::g_clock.now_ms;
  Outcome o = run_wait([&] { chassis.pid_wait_until_index(0); }, [](int) {}, 200);
  CHECK(o.returned);
  CHECK(chassis.interfered);
  CHECK(test_stub::g_clock.now_ms - start < 330);  // one over-current window, not two
}

TEST_CASE("the pure pursuit angle exit starts over on the last point") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_odom_pp_set(path3());
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 0, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
  int last = last_index(chassis);
  // 1-20: far from the first point, aimed right.  21-45: on the point before the last, aimed right long enough to latch the angle exit.
  // 46 on: on the last point, xy settled but the heading 60 degrees off.
  Outcome o = run_wait([&] { chassis.pid_wait(); },
                       [&](int pass) {
                         DriveTestAccess::pp_index(chassis) = pass <= 20 ? 0 : pass <= 45 ? last - 1 : last;
                         chassis.xyPID.compute_error(pass <= 45 ? 5.0 : 0.2, 0.0);
                         chassis.current_a_odomPID.compute_error(pass <= 45 ? 0.0 : 60.0, 0.0);
                       },
                       100);
  CHECK_FALSE(o.returned);  // still waiting for the heading
}

TEST_CASE("a robot pinned partway through a pure pursuit path ends the wait as interfered") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_odom_pp_set(path3());
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 500, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 500, 0);
  DriveTestAccess::pp_index(chassis) = 0;
  Outcome o = run_wait([&] { chassis.pid_wait(); },
                       [&](int) {
                         chassis.xyPID.compute_error(5.0, 0.0);              // far from the path point and not moving
                         chassis.current_a_odomPID.compute_error(0.0, 0.0);  // heading right, so its small exit latches
                       },
                       400);
  CHECK(o.returned);
  CHECK(chassis.interfered);
}

TEST_CASE("an odom move that pivots in place is not ended by xy reading stopped") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_odom_ptp_set({{24.0, 0.0}, fwd, 100});
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 200, 0);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 200, 0);
  // 100 passes turning at 30 degrees a second with translation zeroed by turn bias, then aimed at the point and driving off at 20 in/s
  Outcome o = run_wait([&] { chassis.pid_wait(); },
                       [&](int pass) {
                         bool pivoting = pass < 100;
                         DriveTestAccess::xy_translation_bias_gated(chassis) = pivoting;
                         chassis.xyPID.compute_error(10.0, pivoting ? 0.0 : 0.2 * (pass - 100));
                         if (pivoting)
                           chassis.current_a_odomPID.compute_error(30.0 - 0.3 * pass, 0.3 * pass);
                         else
                           chassis.current_a_odomPID.compute_error(1.0, 30.0);
                       },
                       300);
  CHECK_FALSE(o.returned);
}

TEST_CASE("a motion set from another task during a wait is followed, not abandoned") {
  Drive chassis = make_chassis();
  ready(chassis);
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24.0, 100);
  chassis.pid_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
  Outcome o = run_wait([&] { chassis.pid_wait(); },
                       [&](int pass) {
                         if (pass == 20) {
                           chassis.pid_turn_set(90.0, 100);  // another task starts a turn
                           chassis.pid_turn_exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
                         }
                         if (pass < 20)
                           chassis.leftPID.compute_error(10.0, 0.0);
                         else
                           chassis.turnPID.compute_error(pass < 60 ? 40.0 : 1.0, 0.0);
                       },
                       200);
  REQUIRE(o.returned);
  CHECK(o.passes >= 60);  // it waited for the turn to settle, not for the drive
}
