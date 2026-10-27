// The speed every exit reads is measured over a window a team can change, 30 ms by default.
#include <cmath>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
// Runs 10 ms passes with the sensor moving `speed` units per second for `ms`, then returns what the PID reads as its speed
double speed_after(PID& pid, double speed, int ms) {
  double cur = 0.0;
  for (int t = 0; t < ms; t += 10) {
    cur += speed / 100.0;
    pid.compute_error(50.0, cur);
    pros::delay(10);
  }
  return pid.speed_get();
}
}  // namespace

TEST_CASE("the speed window is 30 ms by default and the speed is read over it") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  CHECK(pid.speed_window_get() == 30);
  CHECK(speed_after(pid, 20.0, 200) == doctest::Approx(20.0).epsilon(0.05));
}

TEST_CASE("a shorter window sees a robot stop sooner than a longer one") {
  test_stub::reset_all();
  PID fast(1, 0, 0, 0, "fast");
  PID slow(1, 0, 0, 0, "slow");
  fast.speed_window_set(30);
  slow.speed_window_set(200);
  speed_after(fast, 20.0, 400);
  speed_after(slow, 20.0, 400);
  CHECK(speed_after(fast, 0.0, 40) == doctest::Approx(0.0));
  CHECK(speed_after(slow, 0.0, 40) > 1.0);
}

TEST_CASE("a window under one tick is refused with a warning and the old window is kept") {
  test_stub::reset_all();
  PID pid(1, 0, 0, 0, "test");
  pid.speed_window_set(50);
  for (int bad : {9, 0, -5}) {
    std::string out = test_stub::capture_stdout([&] { pid.speed_window_set(bad); });
    CHECK(pid.speed_window_get() == 50);
    CHECK(out.find("speed_window_set") != std::string::npos);
  }
}

TEST_CASE("pid_exit_speed_window_set sets every exit's window, with or without units") {
  test_stub::reset_all();
  Drive d({1, -2, -3, -4}, {-5, 6, 7, 8}, 9, 3.25, 360);
  CHECK(d.pid_exit_speed_window_get() == 30);
  d.pid_exit_speed_window_set(60);
  CHECK(d.pid_exit_speed_window_get() == 60);
  for (PID* pid : {&d.leftPID, &d.rightPID, &d.xyPID, &d.turnPID, &d.swingPID, &d.current_a_odomPID}) CHECK(pid->speed_window_get() == 60);
  d.pid_exit_speed_window_set(80_ms);
  CHECK(d.pid_exit_speed_window_get() == 80);
  std::string out = test_stub::capture_stdout([&] { d.pid_exit_speed_window_set(5); });
  CHECK(d.pid_exit_speed_window_get() == 80);
  CHECK(out.find("pid_exit_speed_window_set") != std::string::npos);
}

TEST_CASE("a robot chattering between two positions every tick reads as moving over an odd number of ticks and as still over an even number") {
  // The simulated robots in the exit tests can settle into this: the heading flips between two values on every 10 ms pass.  Over a window of an
  // even number of ticks the flips cancel and the robot reads as still.  Over an odd number they leave one full flip in the window, so it reads as
  // moving at flip size / window.  Tests of those robots set a 40 ms window for that reason.
  test_stub::reset_all();
  PID odd(1, 0, 0, 0, "odd");
  PID even(1, 0, 0, 0, "even");
  odd.speed_window_set(30);
  even.speed_window_set(40);
  for (int t = 0; t < 400; t += 10) {
    double cur = (t / 10) % 2 == 0 ? 0.0 : 2.0;
    odd.compute_error(50.0, cur);
    even.compute_error(50.0, cur);
    pros::delay(10);
  }
  CHECK(odd.speed_get() == doctest::Approx(2.0 * 1000.0 / 30.0).epsilon(0.01));
  CHECK(even.speed_get() == doctest::Approx(0.0));
}
