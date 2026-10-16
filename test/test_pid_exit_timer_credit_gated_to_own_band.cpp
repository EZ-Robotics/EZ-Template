// The small/big/velocity exit timers credit the real milliseconds since the last exit_condition() call, not a flat util::DELAY_TIME, so a caller whose
// poll cadence doesn't match DELAY_TIME (a team's own PID loop at 20 ms, say) still gets the configured exit times.  A gap between two calls is capped at
// ten DELAY_TIMEs, and the first call after a reset counts one DELAY_TIME.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("SMALL_EXIT still credits real elapsed time correctly for a channel that stays continuously in-band") {
  // The mismatched-cadence case real-millisecond crediting exists for: a flat DELAY_TIME per poll would undercount.
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
