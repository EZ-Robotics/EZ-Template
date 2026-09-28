// The small/big/velocity exit timers each credit real elapsed wall-clock milliseconds on a fresh
// poll (see PID.cpp's exit_condition() and wall_credit()), instead of a flat util::DELAY_TIME, so a
// caller whose poll cadence doesn't match its compute cadence still gets correctly-timed exits. That
// crediting must only ever cover time the channel was actually in its own band: an earlier version of
// this fix computed one wall-clock credit up front, shared by all three channels, before checking
// whether any of them were even in band yet. That let a channel which had just crossed into its band
// on THIS poll inherit the whole real-time gap since the last fresh poll -- including however long it
// spent still out of band during that gap -- and exit far sooner than its configured time actually
// allows. Each channel now keeps its own baseline, credited only while that channel's own band
// condition holds, and reset the moment it doesn't.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("BIG_EXIT does not credit wall-clock time from before the error entered big_error's band") {
  PID pid;
  pid.exit_condition_set(0, 0, 84, 6.0);  // big_exit_time=84ms, big_error=6.0; small/velocity/mA off

  test_stub::g_clock.now_ms = 0;
  // Ten fresh, 1:1-cadence polls held well outside big_error -- establishes a real, stale baseline
  // gap without ever entering the band.
  for (int t = 0; t <= 96; t += 12) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(20.0, 0.0);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // A single background compute lands well after the last poll, crossing into big_error's band.
  test_stub::g_clock.now_ms = 179;
  pid.compute_error(5.5, 0.0);

  // The very next poll, ~2ms after the error actually entered the band -- nowhere near the
  // configured 84ms -- must not exit. The stale 96->179 gap (spent entirely out of band) must not
  // be credited toward big_exit_time just because this poll happens to be the first fresh one where
  // the error reads in-band.
  test_stub::g_clock.now_ms = 181;
  CHECK(pid.exit_condition() == RUNNING);

  // Genuinely holding in-band from here does eventually exit, once real in-band dwell reaches the
  // configured time -- this isn't a channel that's stopped crediting altogether.
  bool fired = false;
  for (int t = 191; t <= 400; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(5.5, 0.0);
    if (pid.exit_condition() == BIG_EXIT) {
      fired = true;
      break;
    }
  }
  CHECK(fired);
}

TEST_CASE("SMALL_EXIT still credits real elapsed time correctly for a channel that stays continuously in-band") {
  // Regression guard for the mismatched-cadence case this whole wall-clock-crediting fix exists for:
  // per-channel baselines must not reintroduce the old flat-DELAY_TIME-per-poll undercounting bug.
  PID pid;
  pid.exit_condition_set(80, 50.0, 0, 0);  // small_exit_time=80ms, small_error=50; big/velocity/mA off

  test_stub::g_clock.now_ms = 0;
  pid.compute_error(10.0, 0.0);  // already inside small_error from the very first poll
  CHECK(pid.exit_condition() == RUNNING);

  // Poll every 90ms while staying continuously in-band the whole time -- a single poll's real gap
  // (90ms) alone exceeds the configured 80ms, so this must exit on the very next fresh in-band poll,
  // not be starved down to crediting only a flat DELAY_TIME per poll.
  test_stub::g_clock.now_ms = 90;
  pid.compute_error(10.0, 0.0);
  CHECK(pid.exit_condition() == SMALL_EXIT);
}
