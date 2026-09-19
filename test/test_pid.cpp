// Exit conditions: with small 1 in / 90 ms and big 3 in / 250 ms, an error
// held at 2 in returns BIG_EXIT after 26 passes and never SMALL_EXIT; an
// error held at 0.5 in returns SMALL_EXIT after 10 passes.
// Integral: with ki set, integral accumulates while error keeps sign and
// resets when error sign flips; a position sign flip with constant error
// sign does NOT reset it; motion_reset(c) zeroes the integral and makes the
// next derivative 0.
// MotorGroup overload: exit_condition(pros::MotorGroup) produces BIG_EXIT at
// the same pass count as the plain overload.
// Velocity exit arming: with velocity_exit_time 50, the velocity timer does not
// run until the main sensor's derivative has exceeded its zero threshold once
// (the robot has actually moved), so a robot that hasn't started moving yet is
// not exited early.  After that it exits on the 6th stationary pass, as before.
// A robot that never moves still velocity-exits once the 1000 ms fallback window
// passes (101 passes to arm, then the same 6 passes), so pid_wait can't hang.
// timers_reset() disarms.  The secondary sensor is gated by the same flag.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID BIG_EXIT after 26 passes at a held 2in error, never SMALL_EXIT") {
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pid.error = 2.0;  // within big_error (3), outside small_error (1)

  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
  CHECK(pid.exit_condition() == BIG_EXIT);
}

TEST_CASE("PID SMALL_EXIT after 10 passes at a held 0.5in error") {
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pid.error = 0.5;  // within both small_error (1) and big_error (3)

  for (int pass = 1; pass < 10; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
  CHECK(pid.exit_condition() == SMALL_EXIT);
}

TEST_CASE("PID exit_condition(MotorGroup) matches the plain overload's pass count") {
  // mA_timeout is left at 0 (unset), so the MotorGroup-specific mA check
  // is skipped entirely and this falls straight through to the same
  // exit_condition(print) as the BIG_EXIT case above -- so it should take
  // the same 26 passes to BIG_EXIT, not the ~20 passes a naive estimate
  // (ignoring the mA-timeout skip) might suggest.
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pid.error = 2.0;
  pros::MotorGroup mg({1, 2});

  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(mg) == RUNNING);
  }
  CHECK(pid.exit_condition(mg) == BIG_EXIT);
}

TEST_CASE("PID integral accumulates while error keeps sign, resets on sign flip") {
  PID pid(1.0, 1.0, 0.0, 1000.0);  // start_i huge: integral always active
  pid.target_set(10);

  // First call: prev_error starts at 0 (sgn 0), which differs from the new
  // error's sgn -- that's treated as a sign flip and zeroes the integral
  // right back out, so this call is just priming a consistent sign.
  pid.compute(0);
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(0);  // error=10, same sign as prev_error(10): integral += 10
  CHECK(pid.integral == doctest::Approx(10));

  pid.compute(2);  // error=8, still positive: integral += 8
  CHECK(pid.integral == doctest::Approx(18));

  pid.compute(20);  // error=10-20=-10, sign flips: integral resets to 0
  CHECK(pid.integral == doctest::Approx(0));
}

TEST_CASE("PID a position sign flip with constant error sign does not reset integral") {
  PID pid(1.0, 1.0, 0.0, 1000.0);
  pid.target_set(100);  // far away: error stays positive even as current crosses 0

  pid.compute(5);  // prime (same first-call reset as above)
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(5);  // error=95: integral += 95
  CHECK(pid.integral == doctest::Approx(95));

  pid.compute(-5);  // position flips sign (5 -> -5), error=105, still positive: integral += 105
  CHECK(pid.integral == doctest::Approx(200));
}

TEST_CASE("PID motion_reset zeroes the integral and primes the next derivative to 0") {
  PID pid(1.0, 1.0, 0.0, 1000.0);
  pid.target_set(10);
  pid.compute(0);
  pid.compute(0);
  CHECK(pid.integral == doctest::Approx(10));

  pid.motion_reset(3.0);
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(3.0);  // same value motion_reset primed prev_current to
  CHECK(pid.derivative == doctest::Approx(0));
}

// ---- Velocity exit arming -------------------------------------------------
// exit_condition_set(small_time, small_err, big_time, big_err, velocity_time, mA)
// velocity_time 50 ms with DELAY_TIME 10 ms: the timer passes 50 on its 6th
// counted pass.  The fallback window is 1000 ms (100 passes) before arming.

TEST_CASE("PID velocity exit does not run before the robot has moved") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.error = 10.0;
  pid.derivative = 0.0;  // hasn't started moving yet

  // Well past the 6 passes an unarmed timer would have exited on
  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit ignores sensor noise below the zero threshold") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.error = 10.0;
  pid.derivative = 0.04;  // under the 0.05 default, so not movement

  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit fires on the 6th stationary pass once the robot has moved") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.error = 10.0;

  pid.derivative = 1.0;
  for (int pass = 1; pass <= 3; pass++) CHECK(pid.exit_condition() == RUNNING);

  pid.derivative = 0.0;  // robot has stopped, e.g. hit a wall
  for (int pass = 1; pass < 6; pass++) {
    INFO("stationary pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID velocity timer restarts when the robot moves again") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.error = 10.0;

  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);
  pid.derivative = 0.0;
  for (int pass = 1; pass <= 5; pass++) CHECK(pid.exit_condition() == RUNNING);  // 1 pass short of exiting
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // moving again, timer resets
  pid.derivative = 0.0;
  for (int pass = 1; pass <= 5; pass++) CHECK(pid.exit_condition() == RUNNING);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit still fires for a robot that never moves, after the fallback window") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.error = 10.0;
  pid.derivative = 0.0;  // pinned from the start

  // 100 passes fill the 1000 ms window, pass 101 arms and starts the timer,
  // and the timer passes 50 ms on the 6th of those.
  for (int pass = 1; pass < 106; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID timers_reset disarms velocity exit for the next motion") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.error = 10.0;

  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // armed
  pid.timers_reset();                      // next motion starts

  pid.derivative = 0.0;
  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID secondary velocity sensor is gated by the same arming") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // reads as stationary
  pid.error = 10.0;
  pid.derivative = 0.0;

  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
}
