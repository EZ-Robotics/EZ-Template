// Issue #529: PID::exit_condition()'s hold_timer (the velocity_exit_hold/VELOCITY_EXIT_HOLD_FALLBACK
// safety valve) credited a flat util::DELAY_TIME per CALL to exit_condition(), not real elapsed
// wall-clock milliseconds -- unlike the small/big/velocity/mA timers PR #526 already fixed. A bare
// ez::PID caller polling faster than DELAY_TIME(10ms)/call released the hold roughly 10x sooner than
// the documented ~2000ms; a caller polling slower released it roughly 5x later. These drive a bare
// PID through a virtual millisecond timeline, with test_stub::g_clock.now_ms advanced to match, at
// two different poll cadences, and check the hold's fallback releases within about one poll of real
// VELOCITY_EXIT_HOLD_FALLBACK ms in both cases -- not after a fixed number of calls.
//
// The secondary-velocity-sensor timer (m) has the same flat-credit shape (deliberately without an
// error_fresh gate, since second_sensor is caller-supplied rather than derived from a real compute())
// and is fixed the same way here. Fixing it also means its own wall-clock baseline (have_m_fresh_ms/
// last_m_fresh_ms) must get the same held-span resync the main velocity channel's baseline already
// gets in exit_condition() -- otherwise a hold spanning the secondary channel would let it inherit
// the whole held span as settled dwell in one shot on release, which the last test case here checks
// for directly.
#include <cstdlib>

#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {

// Drives a bare PID with the (main) velocity channel held and derivative pinned at "stopped"
// throughout, polling exit_condition() at a fixed real millisecond cadence, and returns the real
// elapsed wall-clock ms at which the hold's fallback lets VELOCITY_EXIT fire -- i.e. the moment the
// hold actually released, not a call count. -1 if it never fires within the scan window.
int hold_release_ms(int poll_period_ms) {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;

  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, /*velocity_exit_time=*/10, 0);

  // One real above-floor compute arms the velocity channel; every compute after this keeps the
  // sensor reading unchanged (derivative == 0, i.e. stopped) for the rest of the run.
  pid.compute_error(10.0, 1.0);
  CHECK(pid.exit_condition() == RUNNING);

  pid.velocity_exit_hold_set(true);
  // 12000ms comfortably covers both the correct real-time release (~2000ms) and the old flat-credit
  // bug's slow-poll case (~10000-10100ms at a 50ms cadence), so a run against unfixed code reports a
  // real (wrong) release time here instead of just timing out.
  for (int t = poll_period_ms; t <= 12000; t += poll_period_ms) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10.0, 1.0);  // stopped every poll: the hold is the only thing masking the exit
    if (pid.exit_condition() == VELOCITY_EXIT) return t;
  }
  return -1;
}

// Same idea for the secondary-velocity-sensor timer (m): arms via a direct derivative write (no
// compute() call at all, so compute_count/error_fresh never advances and the main channel's own
// k-block -- gated on error_fresh -- can never fire), then holds the secondary sensor stationary at
// a fixed real millisecond cadence and returns the real elapsed ms at which VELOCITY_EXIT fires.
int secondary_exit_ms(int poll_period_ms, int velocity_exit_time) {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;

  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, velocity_exit_time, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // stationary throughout
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms; also credits m's own nominal first-poll DELAY_TIME
  pid.derivative = 0.0;

  for (int t = poll_period_ms; t <= 6000; t += poll_period_ms) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    if (pid.exit_condition() == VELOCITY_EXIT) return t;
  }
  return -1;
}

}  // namespace

TEST_CASE("bare PID: a fast poll cadence no longer releases the velocity-exit hold ~10x sooner than the documented fallback") {
  int t = hold_release_ms(1);
  REQUIRE(t > 0);
  // Real VELOCITY_EXIT_HOLD_FALLBACK is 2000ms; a flat-credit bug releases this at 2000 / DELAY_TIME
  // (10) == 200 calls, each only 1ms apart in real time here -- measured at t=202 against unfixed code.
  CHECK(t >= 1900);
  CHECK(t <= 2200);
}

TEST_CASE("bare PID: a slow poll cadence no longer holds the velocity-exit ~5x longer than the documented fallback") {
  int t = hold_release_ms(50);
  REQUIRE(t > 0);
  // A flat-credit bug holds this for 2000 / DELAY_TIME(10) == 200 calls, each 50ms apart in real
  // time here -- measured at t=10100 against unfixed code (the scan window above exists for this),
  // matching issue #529's own "~10,100ms measured" repro almost exactly.
  CHECK(t >= 1900);
  CHECK(t <= 2200);
}

TEST_CASE("bare PID: the secondary-velocity-sensor timer credits real elapsed wall-clock time, matching across poll cadences") {
  // A flat per-call credit makes a fast cadence exit far sooner than a slow cadence for the same
  // configured velocity_exit_time; a real wall-clock credit converges on (roughly) the same real
  // elapsed time regardless of cadence.
  int fast = secondary_exit_ms(1, 500);
  int slow = secondary_exit_ms(50, 500);
  REQUIRE(fast > 0);
  REQUIRE(slow > 0);
  CHECK(fast >= 480);
  CHECK(fast <= 520);
  CHECK(slow >= 480);
  CHECK(slow <= 520);
  CHECK(std::abs(fast - slow) <= 60);
}

TEST_CASE("bare PID: a hold spanning the secondary-velocity-sensor timer does not credit the held span as settled dwell on release") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, /*velocity_exit_time=*/50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // stationary throughout, hold or not

  test_stub::g_clock.now_ms = 0;
  pid.error = 10.0;
  pid.derivative = 1.0;
  CHECK(pid.exit_condition() == RUNNING);  // arms; m credited its own nominal first-poll DELAY_TIME
  pid.derivative = 0.0;

  test_stub::g_clock.now_ms = 10;
  pid.velocity_exit_hold_set(true);
  CHECK(pid.exit_condition() == RUNNING);  // held

  // A long held span, but well inside VELOCITY_EXIT_HOLD_FALLBACK(2000ms) so the hold's OWN fallback
  // never engages here -- this isolates the secondary channel's wall-clock baseline resync from the
  // hold's own release mechanism (covered separately above).
  test_stub::g_clock.now_ms = 1500;
  pid.velocity_exit_hold_set(false);
  // This specifically guards the fix's own held-span resync for the secondary channel's wall-clock
  // baseline (have_m_fresh_ms), not the flat-credit bug itself -- a flat per-call credit can't
  // overcredit a held span at all, so this only catches the wall-clock fix being applied to m
  // without also resyncing have_m_fresh_ms on every held poll (see exit_condition()'s `if (held)`
  // block). Without that resync, the baseline is left stale from before the hold started, so on
  // release it measures the ENTIRE ~1490ms held span in one shot (capped at WALL_CLOCK_CREDIT_CAP),
  // crediting far more than velocity_exit_time(50) at once and firing VELOCITY_EXIT immediately.
  // Correctly resynced, release credits only the nominal DELAY_TIME, same as any other
  // fresh-after-reset poll.
  CHECK(pid.exit_condition() == RUNNING);

  // Genuinely settling from here takes the full configured ~50ms, not 0ms.
  bool fired = false;
  for (int t = 1510; t <= 1800; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    if (pid.exit_condition() == VELOCITY_EXIT) {
      fired = true;
      CHECK(t >= 1500 + 40);
      break;
    }
  }
  CHECK(fired);
}
