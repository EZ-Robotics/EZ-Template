// Exit conditions: the exit timers count real milliseconds (a test pass here is one 10 ms step of the fake clock).  With small 1 in / 90 ms and big 3 in /
// 250 ms, an error held at 2 in returns BIG_EXIT after 26 passes and never SMALL_EXIT; an error held at 0.5 in returns SMALL_EXIT after 10 passes.  Polling
// again in the same millisecond credits nothing, and extra computes between polls change nothing.
// Integral: with ki set, integral accumulates while error keeps sign and
// resets when error sign flips; a position sign flip with constant error
// sign does NOT reset it; motion_reset(c) zeroes the integral and makes the
// next derivative 0.
// MotorGroup overload: exit_condition(pros::MotorGroup) produces BIG_EXIT at
// the same pass count as the plain overload.
// Velocity exit arming: the speed is the net movement of the sensor passed to compute_error() over the last 100 ms, so it only changes on a 100 ms
// boundary.  The velocity timer does not run until that speed has exceeded the main sensor's zero threshold (5 units/s) once (the robot has actually
// moved) or 1000 ms have passed, so a robot that hasn't started moving yet is not exited early.  After that the timer counts real milliseconds while the
// speed reads at or under the threshold, and any window above it clears the timer at once.  timers_reset() disarms.  The secondary sensor is gated by
// the same arming flag.
// A non-finite secondary reading (no imu, or before the first update) never counts as stopped.
#include <cmath>
#include <limits>

#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {
void clock_start() {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
}

// One 10 ms pass of a bare PID loop: the clock moves on, compute_error() sees the new reading, then exit_condition() polls.
exit_output run_pass(PID& pid, double current, double error = 10.0) {
  test_stub::g_clock.now_ms += util::DELAY_TIME;
  pid.compute_error(error, current);
  return pid.exit_condition();
}

// Moves the sensor 1 unit per pass (100 units/s) for `passes` passes starting at `current`, and returns where it ends up.  The speed window closes every
// 10 passes from a clock that started at 0, so 10 moving passes make the first window read 100 units/s and arm the velocity exit.
double move_passes(PID& pid, double current, int passes) {
  for (int p = 0; p < passes; p++) {
    current += 1.0;
    CHECK(run_pass(pid, current) == RUNNING);
  }
  return current;
}
}  // namespace

TEST_CASE("PID BIG_EXIT after 26 passes at a held 2in error, never SMALL_EXIT") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  // The sensor sits still (speed 0 reads as stopped), so the held error counts 10 ms per pass: 260 ms is the first value over big_exit_time.
  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    CHECK(run_pass(pid, 0.0, 2.0) == RUNNING);  // within big_error (3), outside small_error (1)
  }
  CHECK(run_pass(pid, 0.0, 2.0) == BIG_EXIT);
}

TEST_CASE("PID SMALL_EXIT after 10 passes at a held 0.5in error") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  for (int pass = 1; pass < 10; pass++) {
    INFO("pass ", pass);
    CHECK(run_pass(pid, 0.0, 0.5) == RUNNING);  // within both small_error (1) and big_error (3)
  }
  CHECK(run_pass(pid, 0.0, 0.5) == SMALL_EXIT);
}

TEST_CASE("PID exit_condition(MotorGroup) matches the plain overload's pass count") {
  // mA_timeout is left at 0 (unset), so the MotorGroup-specific mA check
  // is skipped entirely and this falls straight through to the same
  // exit_condition(print) as the BIG_EXIT case above -- so it should take
  // the same 26 passes to BIG_EXIT, not the ~20 passes a naive estimate
  // (ignoring the mA-timeout skip) might suggest.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pros::MotorGroup mg({1, 2});

  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    pid.compute_error(2.0, 0.0);
    CHECK(pid.exit_condition(mg) == RUNNING);
  }
  test_stub::g_clock.now_ms += util::DELAY_TIME;
  pid.compute_error(2.0, 0.0);
  CHECK(pid.exit_condition(mg) == BIG_EXIT);
}

// ---- Polling cadence --------------------------------------------------------
// The small/big exit timers credit the real milliseconds since the last call, not one flat step per call.  Two things this needs to not break: a caller
// polling exit_condition() faster than compute() runs, and one polling slower.

TEST_CASE("PID small exit still settles when the caller polls twice for every one real compute") {
  // The second poll lands in the same millisecond as the first, so it credits nothing: it neither fires too early nor delays the exit.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  exit_output result = RUNNING;
  int fresh_computes = 0;
  while (result == RUNNING) {
    REQUIRE(fresh_computes <= 15);  // don't hang the suite if this regresses
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    pid.compute_error(0.5, 0.0);  // one real compute: error held at 0.5in (inside small_error)
    ++fresh_computes;
    result = pid.exit_condition();  // the poll for this compute
    if (result != RUNNING) break;
    result = pid.exit_condition();  // a second poll, no time has passed since the one just above
  }
  CHECK(result == SMALL_EXIT);
  // Exactly the same 10 real computes as the 1:1 cadence case above -- the extra poll neither speeds this up nor slows it down.
  CHECK(fresh_computes == 10);
}

TEST_CASE("PID small exit still settles when the caller polls once for every two real computes") {
  // The caller here polls SLOWER than compute() runs. Only the clock decides the credit, so this still exits after the same 10 polls as the 1:1
  // case, just with twice as many computes behind it.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  exit_output result = RUNNING;
  int polls = 0;
  while (result == RUNNING) {
    REQUIRE(polls <= 15);
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    pid.compute_error(0.5, 0.0);
    pid.compute_error(0.5, 0.0);  // two real computes between polls
    ++polls;
    result = pid.exit_condition();
  }
  CHECK(result == SMALL_EXIT);
  CHECK(polls == 10);
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
// The speed window closes every 10 passes (100 ms), so a robot that moves for 10 passes and then stops first reads as stopped on the boundary 10 passes
// later (pass 20).  With velocity_time 50 ms the timer holds 10 on that pass and passes 50 on the fifth pass after it, so the exit comes on pass 25.
// The fallback window is 1000 ms (100 passes) before arming without movement.

TEST_CASE("PID velocity exit does not run before the robot has moved") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  // The sensor never moves; 500 ms is well past the 6 passes an unarmed timer would have exited on, and short of the 1000 ms fallback
  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    CHECK(run_pass(pid, 0.0) == RUNNING);
  }
}

TEST_CASE("PID velocity exit ignores sensor noise below the zero threshold") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  // 0.04 units per pass is 4 units/s over a window, under the 5 units/s default, so not movement
  double cur = 0.0;
  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    cur += 0.04;
    CHECK(run_pass(pid, cur) == RUNNING);
  }
}

TEST_CASE("PID velocity exit fires after enough stationary passes once the robot has moved") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  double cur = move_passes(pid, 0.0, 10);  // the first window reads 100 units/s: arms

  // Resting from pass 11, but the window still holds the old speed until pass 20 closes it
  int pass = 10;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 40);
    result = run_pass(pid, cur);
  }
  CHECK(result == VELOCITY_EXIT);
  CHECK(pass == 25);  // pass 20 reads stopped (timer 10), exit once the timer passes 50 on pass 25
}

TEST_CASE("PID velocity timer clears on a single moving window, no debounce") {
  // One window with net movement above the zero threshold clears the timer at once.  velocity_exit_time is 300 so the timer is still running when the
  // movement is seen.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 300, 0);

  double cur = move_passes(pid, 0.0, 10);  // arms on pass 10

  int pass = 10;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 100);
    if (pass == 25) cur += 2.0;  // a single step: the window that closes on pass 30 reads 20 units/s
    result = run_pass(pid, cur);
  }
  CHECK(result == VELOCITY_EXIT);
  // Without the step the timer would hold 10 on pass 20 and pass 300 on pass 50.  The step clears it on pass 30 and the next window (pass 40) is the
  // first still one, so the timer holds 10 on pass 40 and passes 300 on pass 70.
  CHECK(pass == 70);
}

TEST_CASE("PID velocity exit does not fire on a healthy motion whose sensor only refreshes every other poll") {
  // A sensor that refreshes slower than the poll rate reports the SAME raw value on alternating polls.  The speed is the net movement over 100 ms, so
  // the repeat polls do not read as stopped.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  double real_position = 0.0;
  for (int pass = 1; pass <= 200; pass++) {
    INFO("pass ", pass);
    if (pass % 2 == 1) real_position += 0.3;  // the sensor only actually advances on odd polls: 15 units/s overall
    CHECK(run_pass(pid, real_position) == RUNNING);
  }
}

TEST_CASE("PID velocity exit does not fire on real motion whose sensor refreshes slower than the poll, at a realistic ratio") {
  // A sensor refreshing every 30ms, polled every 10ms, with a realistic velocity_exit_time (500ms, not the artificially tight 50ms most of this file's
  // other tests use to keep loops short).  Two out of every three polls repeat the last value, but the net movement over each 100 ms window is about
  // 10 units/s, so the timer never runs.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 500, 0);

  double position = 0.0;
  for (int pass = 1; pass <= 200; pass++) {  // 2000ms of continuous real motion
    INFO("pass ", pass);
    if (pass % 3 == 0) position += 0.3;  // the sensor's own refresh: a real step every 30ms
    CHECK(run_pass(pid, position) == RUNNING);
  }
}

TEST_CASE("PID velocity exit fires once a repeated reading has genuinely lasted longer than velocity_exit_time") {
  // A reading that never changes again is treated as stopped.  A bare PID with a realistic velocity_exit_time (500ms) still only fires after that much
  // real stillness, counted from the first window that reads the robot as stopped.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 500, 0);
  double cur = move_passes(pid, 0.0, 10);  // arms

  int pass = 10;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 100);
    result = run_pass(pid, cur);  // the raw value never moves again from here on
  }
  CHECK(result == VELOCITY_EXIT);
  CHECK(pass == 70);  // the window closing on pass 20 is the first still one: timer 10 there, 510 (over 500) on pass 70
}

TEST_CASE("PID timers_reset clears the velocity timer for the next motion") {
  // A new motion can start with the sensor sitting still.  Without timers_reset() clearing the timer, leftover time from the old motion's own stall
  // would carry straight into the new motion and fire far sooner than a fresh motion should.  The new motion never moves, so it arms through the
  // 1000 ms fallback, which is the only way to reach the timer without a moving window clearing it.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 300, 0);

  double cur = move_passes(pid, 0.0, 10);  // first motion arms
  for (int pass = 11; pass <= 40; pass++) {
    CHECK(run_pass(pid, cur) == RUNNING);  // the timer holds 210 by pass 40
  }

  pid.timers_reset();  // a new motion starts

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 200);
    result = run_pass(pid, cur);
  }
  CHECK(result == VELOCITY_EXIT);
  // The 1000 ms fallback arms on the 101st pass after the reset (the timer holds 10 there), and the timer passes 300 on the 131st.  Left at 210 it
  // would have exited on the 110th.
  CHECK(pass == 131);
}

TEST_CASE("PID velocity exit: a mechanism moving above the stopped floor every pass never exits, even with two computes per poll") {
  // A caller whose task calls compute() more than once between the wait's own polls still sees the speed measured over the clock, not per compute.
  // Each pass here moves 0.1, 10 units/s, twice the 5 units/s zero threshold, so this must never exit.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  double real_position = 0.0;
  for (int pass = 1; pass <= 40; pass++) {
    INFO("pass ", pass);
    real_position += 0.1;
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    pid.compute_error(10.0, real_position);
    pid.compute_error(10.0, real_position);  // immediately re-read the same value before the poll
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit: two real computes landing before one poll still exits promptly on a genuine stop") {
  // Once the mechanism has actually stopped (both computes between polls read the same, unchanging value), the exit still fires on the same pass as
  // with one compute per poll.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  double cur = move_passes(pid, 0.0, 10);  // arms

  int pass = 10;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 40);
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    pid.compute_error(10.0, cur);  // stopped: both computes this pass read the same, already-held value
    pid.compute_error(10.0, cur);
    result = pid.exit_condition();
  }
  CHECK(pass == 25);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity timer does not double-count when exit_condition is polled twice in the same millisecond") {
  // A caller polling FASTER than the clock moves can call exit_condition() twice for the same instant.  The second call elapsed no time, so it must not
  // credit it a second time.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  double cur = move_passes(pid, 0.0, 10);  // arms

  for (int pass = 11; pass <= 24; pass++) {
    INFO("pass ", pass);
    test_stub::g_clock.now_ms += util::DELAY_TIME;
    pid.compute_error(10.0, cur);
    CHECK(pid.exit_condition() == RUNNING);
    CHECK(pid.exit_condition() == RUNNING);  // same millisecond, no new credit
  }

  test_stub::g_clock.now_ms += util::DELAY_TIME;
  pid.compute_error(10.0, cur);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);  // pass 25, same as with a single poll per pass
}

TEST_CASE("PID velocity exit treats a non-finite sensor reading as stopped") {
  // A dead sensor feeding compute_error() a non-finite current (e.g. from a disconnected motor's PROS_ERR_F position) must not dodge this exit
  // indefinitely on a bare PID with no other backstop: a failed read reads as stopped, so it counts toward the timer like any other stall.
  // See test_sensorfault_mA_disconnect.cpp for the matching Motor-level coverage (a disconnected motor's mA timer independently catches this too).
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  move_passes(pid, 0.0, 10);  // arms with a real, finite speed first

  int pass = 10;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 60);
    result = run_pass(pid, std::numeric_limits<double>::quiet_NaN());  // sensor died
  }
  CHECK(result == VELOCITY_EXIT);
  CHECK(pass <= 30);  // the window that closes at pass 20 reads stopped, then the 50 ms velocity exit time
}

TEST_CASE("PID timers_reset disarms velocity exit for the next motion") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 500, 0);

  double cur = move_passes(pid, 0.0, 10);  // armed
  for (int pass = 11; pass <= 25; pass++) {
    CHECK(run_pass(pid, cur) == RUNNING);  // stopped from pass 20, the timer holds 60 by pass 25
  }
  pid.timers_reset();  // next motion starts

  // Armed it would exit on the 51st pass after the reset; disarmed it waits for the 1000 ms fallback
  for (int pass = 1; pass <= 55; pass++) {
    INFO("pass ", pass);
    CHECK(run_pass(pid, cur) == RUNNING);
  }
}

TEST_CASE("PID secondary velocity sensor is gated by the same arming") {
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // reads as stationary

  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    CHECK(run_pass(pid, 0.0) == RUNNING);
  }
}

TEST_CASE("PID secondary velocity sensor never exits on a reading that was never taken") {
  // second_sensor defaults to NaN (no imu, or velocity_sensor_secondary_set() was never called).
  // drive_imu_accel_get() also returns NaN with no imu (see Drive::drive_imu_accel_get). Either
  // way this must never be misread as "0 acceleration", which would falsely count as stopped.
  clock_start();
  PID pid;
  pid.speed_window_set(100);  // These timelines are worked out for a 100 ms speed window
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);

  CHECK_FALSE(std::isfinite(pid.velocity_sensor_secondary_get()));

  // real, continuous movement: arms after the first window, keeps the main channel from firing too
  double cur = 0.0;
  for (int pass = 1; pass <= 200; pass++) {
    INFO("pass ", pass);
    cur += 1.0;
    CHECK(run_pass(pid, cur) == RUNNING);
  }

  // A real reading afterwards is still honored by the secondary channel (main stays non-zero throughout).
  pid.velocity_sensor_secondary_set(0.0);
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 50);  // don't hang the suite if this regresses
    cur += 1.0;
    result = run_pass(pid, cur);
  }
  CHECK(result == VELOCITY_EXIT);
}
