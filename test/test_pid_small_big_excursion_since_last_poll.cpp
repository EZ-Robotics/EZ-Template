// The small/big exit timers each credit real elapsed wall-clock milliseconds on a fresh poll, gated
// on whether `error` currently reads inside that timer's own band. Looked at only once per poll, that
// check has a blind spot: the caller's own wait loop and whatever drives compute() are separate tasks
// that don't share a schedule, so a real excursion outside the band can happen entirely between two
// polls -- neither poll samples it -- while still being a real excursion that should reset the timer.
// Without tracking it, the whole gap (including the invisible out-of-band portion) gets credited as if
// the mechanism had been continuously settled the entire time. small_excursion_count/big_excursion_count
// (PID.hpp) close this the same way compute_count/last_checked_compute already track plain freshness:
// bumped once per real compute whose own reading crossed the line, compared against a last-seen
// snapshot in exit_condition(), so "did this happen since I last checked" is answered correctly even
// when the specific poll that would have caught it never landed.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("SMALL_EXIT does not credit a real excursion that happened entirely between two polls as clean dwell") {
  PID pid;
  pid.exit_condition_set(90, 5.0, 0, 0);  // small_exit_time=90ms, small_error=5.0; big/velocity/mA off

  test_stub::g_clock.now_ms = 0;
  pid.compute_error(2.0, 0.0);  // settled, inside small_error
  CHECK(pid.exit_condition() == RUNNING);

  // A real shove happens on a compute() tick with no accompanying poll: error jumps outside
  // small_error, then recovers, all before the wait loop's own next poll ever runs.
  test_stub::g_clock.now_ms = 40;
  pid.compute_error(9.0, 0.0);  // real excursion outside small_error=5.0 -- no poll observes this tick
  test_stub::g_clock.now_ms = 50;
  pid.compute_error(2.0, 0.0);  // recovered, back inside small_error -- still no poll yet

  // The wait loop's own next poll lands here, 90ms after the original settle. Naively crediting the
  // whole 90ms gap (since `error` reads inside small_error right now) would fire SMALL_EXIT -- but the
  // mechanism was only actually, continuously settled for the last 40ms of that gap (50->90), well
  // under the configured 90ms.
  test_stub::g_clock.now_ms = 90;
  pid.compute_error(2.0, 0.0);
  CHECK(pid.exit_condition() == RUNNING);

  // Confirm it's not simply stuck: continuing to hold settled from here does eventually exit.
  bool fired = false;
  for (int t = 100; t <= 400; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(2.0, 0.0);
    if (pid.exit_condition() == SMALL_EXIT) {
      fired = true;
      break;
    }
  }
  CHECK(fired);
}

TEST_CASE("BIG_EXIT does not credit a real excursion outside big_error that happened entirely between two polls") {
  // Same fix, same shape, against big_error instead of small_error -- a real excursion big enough to
  // leave big_error's own band, happening entirely between two polls, must not be invisible just
  // because the poll that would have caught it never landed.
  PID pid;
  pid.exit_condition_set(0, 0, 120, 50.0);  // big_exit_time=120ms, big_error=50.0; small/velocity/mA off

  test_stub::g_clock.now_ms = 0;
  pid.compute_error(10.0, 0.0);  // settled, inside big_error
  CHECK(pid.exit_condition() == RUNNING);

  // A real, larger disturbance happens on a compute() tick with no accompanying poll.
  test_stub::g_clock.now_ms = 60;
  pid.compute_error(80.0, 0.0);  // real excursion outside big_error=50.0 -- no poll observes this tick
  test_stub::g_clock.now_ms = 70;
  pid.compute_error(10.0, 0.0);  // recovered, back inside big_error -- still no poll yet

  // The wait loop's own next poll lands here, 120ms after the original settle -- naively crediting the
  // whole gap would fire BIG_EXIT, but the mechanism was only actually, continuously settled for the
  // last 50ms of it (70->120).
  test_stub::g_clock.now_ms = 120;
  pid.compute_error(10.0, 0.0);
  CHECK(pid.exit_condition() == RUNNING);

  bool fired = false;
  for (int t = 130; t <= 400; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10.0, 0.0);
    if (pid.exit_condition() == BIG_EXIT) {
      fired = true;
      break;
    }
  }
  CHECK(fired);
}

TEST_CASE("velocity arming is not fooled by a compute cadence that aliases the sensor's own refresh cadence") {
  // Companion to the STOPPED-check fix: the ARMED gate has the identical blind spot if it only looks
  // at the instantaneous derivative on whichever compute happens to run right before a poll. A sensor
  // that only refreshes every 3rd compute produces derivative==0 on the computes in between, even
  // while the mechanism is moving continuously from the very first real compute. If arming missed
  // this and fell back to the flat ~1000ms VELOCITY_ARM_FALLBACK instead, a genuine stop right after
  // this continuous-movement stretch would exit far later than the configured 300ms.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 300, 0);  // velocity_exit_time=300ms only
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  double sensor = 0.0;
  for (int t = 0; t <= 200; t += 4) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    if (t % 12 == 0) sensor += 24.0;  // sensor refreshes only every 3rd compute -- continuous real movement
    pid.compute_error(10000.0 - sensor, sensor);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // Now genuinely stop: every subsequent compute reads the same, unrefreshing value.
  bool fired = false;
  int t_exit = -1;
  for (int t = 204; t <= 700; t += 4) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10000.0 - sensor, sensor);
    if (pid.exit_condition() == VELOCITY_EXIT) {
      fired = true;
      t_exit = t;
      break;
    }
  }
  CHECK(fired);
  // Stopped at t=200; configured 300ms means exit by ~500ms if arming caught the movement in real
  // time. If arming had missed it and fallen back to the ~1000ms flat fallback instead, this would
  // land past t=1000 -- comfortably distinguishable from the expected ~500ms.
  CHECK(t_exit <= 550);
}
