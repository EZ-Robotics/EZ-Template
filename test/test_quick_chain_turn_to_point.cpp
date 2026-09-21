// pid_wait_quick_chain() after a turn to a point. It used to print "Not in a supported drive mode!"
// and return at once, without waiting for the turn. It has to wait for the turn like it does for a
// normal turn, and chain into the next motion by aiming a little past the point.
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  detail::print_sink = nullptr;  // these tests read stdout
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// One pass of ez_auto_task(). pros::Task never runs its callable in the host build, so the fake
// pros::delay() throws right after the first pass to unwind the otherwise-infinite loop.
void run_one_auto_task_pass(Drive& chassis) {
  test_stub::g_clock.delay_calls_until_stop = 0;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}

// What the turn PID is driving the left motor with, as a -127 to 127 speed.
double turn_output(Drive& chassis) { return chassis.left_motors[0].fake().voltage / (12000.0 / 127.0); }

int delays_seen = 0;
void count_delay() { delays_seen++; }

// pid_wait_quick_chain() with the fake pros::delay() set to throw after `max_delays` calls, so a wait that
// never returns fails the test instead of hanging it.  Returns what it printed.
std::string quick_chain(Drive& chassis, int max_delays, bool* returned = nullptr) {
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool done = true;
  std::string out = test_stub::capture_stdout([&] {
    try {
      chassis.pid_wait_quick_chain();
    } catch (test_stub::StopLoop&) {
      done = false;
    }
  });
  test_stub::g_clock.delay_calls_until_stop = -1;
  if (returned) *returned = done;
  return out;
}
}  // namespace

TEST_CASE("pid_wait_quick_chain on a turn to a point waits for the turn instead of returning at once") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 110);
  REQUIRE(chassis.mode == TURN_TO_POINT);

  delays_seen = 0;
  test_stub::g_clock.on_delay = count_delay;
  bool returned = false;
  std::string printed = quick_chain(chassis, 500, &returned);
  test_stub::g_clock.on_delay = nullptr;

  CHECK(printed.find("Not in a supported drive mode") == std::string::npos);
  CHECK(returned);
  CHECK(delays_seen > 0);
}

TEST_CASE("pid_wait_quick_chain on a turn to a point aims past the point by the turn chain constant") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_constants_set(3.0, 0.0, 0.0);
  chassis.slew_turn_set(false);
  chassis.pid_turn_chain_constant_set(4.0);

  chassis.imu->fake_rotation = 80.0;
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 127);  // faces 90 degrees, 10 degrees away from the robot
  REQUIRE(chassis.mode == TURN_TO_POINT);

  run_one_auto_task_pass(chassis);
  run_one_auto_task_pass(chassis);
  CHECK(turn_output(chassis) == doctest::Approx(3.0 * 10.0).epsilon(0.01));

  quick_chain(chassis, 5);  // the robot never reaches the point, the bounded wait gives up
  run_one_auto_task_pass(chassis);
  CHECK(turn_output(chassis) == doctest::Approx(3.0 * (10.0 + 4.0)).epsilon(0.01));
}

TEST_CASE("pid_wait_quick_chain on a turn to a point aims past the point in the direction of the turn") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_constants_set(3.0, 0.0, 0.0);
  chassis.slew_turn_set(false);
  chassis.pid_turn_chain_constant_set(4.0);

  chassis.imu->fake_rotation = 100.0;
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 127);  // faces 90 degrees, so the turn goes the negative way
  REQUIRE(chassis.mode == TURN_TO_POINT);

  run_one_auto_task_pass(chassis);
  run_one_auto_task_pass(chassis);
  CHECK(turn_output(chassis) == doctest::Approx(3.0 * -10.0).epsilon(0.01));

  quick_chain(chassis, 5);
  run_one_auto_task_pass(chassis);
  CHECK(turn_output(chassis) == doctest::Approx(3.0 * -14.0).epsilon(0.01));
}

TEST_CASE("a turn to a point that isn't chained doesn't aim past the point") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_constants_set(3.0, 0.0, 0.0);
  chassis.slew_turn_set(false);
  chassis.pid_turn_chain_constant_set(4.0);

  chassis.imu->fake_rotation = 80.0;
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 127);
  run_one_auto_task_pass(chassis);
  run_one_auto_task_pass(chassis);
  CHECK(turn_output(chassis) == doctest::Approx(3.0 * 10.0).epsilon(0.01));

  // A later turn to a point starts clean, without the offset a chain left behind
  quick_chain(chassis, 5);
  chassis.pid_turn_set({10.0, 0.0, 0.0}, fwd, 127);
  run_one_auto_task_pass(chassis);
  CHECK(turn_output(chassis) == doctest::Approx(3.0 * 10.0).epsilon(0.01));
}
