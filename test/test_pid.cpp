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
// not exited early.  After that it exits on the 6th genuinely fresh stationary
// pass.  "Fresh" is judged the same way the small/big exit timers are: whether
// a real compute()/compute_error() call has landed since the last check
// (error_fresh, PID.cpp) -- there is no separate raw-value staleness state or
// moving-tick debounce for this channel any more: any single fresh sample
// above the threshold clears the timer at once, and a repeated reading is
// treated as stopped from the very first fresh repeat, not after some grace
// window. timers_reset() disarms.  The
// secondary sensor is gated by the same arming flag, but not by error_fresh --
// see its own comment in PID.cpp for why.
// A non-finite secondary reading (no imu, or before the first update) never counts as stopped.
// velocity_exit_hold freezes both velocity timers while true, but only for up to
// VELOCITY_EXIT_HOLD_FALLBACK ms of continuous hold, so a caller that holds indefinitely still
// resolves instead of hanging the caller waiting on exit_condition().
#include <limits>

#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

namespace {
// Feeds compute_error() a raw reading that's always different from the tick before -- real sensor
// dither at a resting position, not a raw value repeating because the sensor hasn't refreshed yet.
// Alternating between two points a hair apart keeps |derivative| inside the stalled band on every
// tick while every tick is still a genuinely fresh sample (a bit-identical repeat would instead be
// indistinguishable from a stale re-read -- see the velocity-exit tests below).
void dither_tick(PID& pid, double error, bool& toggle, double base, double amplitude = 0.01) {
  toggle = !toggle;
  // Alternates base+amplitude / base-amplitude -- never equal to `base` itself or to the other
  // side of the swing -- so this is fresh regardless of what raw value came immediately before it
  // (a jump target that happens to equal `base`, or the opposite phase of a previous dither call).
  pid.compute_error(error, base + (toggle ? amplitude : -amplitude));
}
}  // namespace

TEST_CASE("PID BIG_EXIT after 26 passes at a held 2in error, never SMALL_EXIT") {
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  // A real compute_error() call every pass, not a direct `.error =` write: the small/big exit timers
  // only credit `error` when a real compute has landed since the last check (see PID.cpp), so a "held"
  // error has to mean a real compute repeatedly landing on the same value, the same as a real motion
  // sitting at a steady-state error would. `current` is irrelevant here (velocity_exit_time is 0, so
  // the derivative it produces is never read) -- held fixed for simplicity.
  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    pid.compute_error(2.0, 0.0);  // within big_error (3), outside small_error (1)
    CHECK(pid.exit_condition() == RUNNING);
  }
  pid.compute_error(2.0, 0.0);
  CHECK(pid.exit_condition() == BIG_EXIT);
}

TEST_CASE("PID SMALL_EXIT after 10 passes at a held 0.5in error") {
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  for (int pass = 1; pass < 10; pass++) {
    INFO("pass ", pass);
    pid.compute_error(0.5, 0.0);  // within both small_error (1) and big_error (3)
    CHECK(pid.exit_condition() == RUNNING);
  }
  pid.compute_error(0.5, 0.0);
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
  pros::MotorGroup mg({1, 2});

  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    pid.compute_error(2.0, 0.0);
    CHECK(pid.exit_condition(mg) == RUNNING);
  }
  pid.compute_error(2.0, 0.0);
  CHECK(pid.exit_condition(mg) == BIG_EXIT);
}

// ---- Small/big exit staleness -----------------------------------------------
// exit_condition()'s small/big exit timers only advance on a poll where a real compute() call has
// landed since the timer last checked (mirrors k_prev_checked/k_unchanged_time's freshness gate for
// the velocity channel, PID.cpp). Two things this needs to not break: a caller polling exit_condition()
// faster than compute() runs, and one polling slower.

TEST_CASE("PID small exit still settles when the caller polls twice for every one real compute") {
  // A stale poll (no new compute since the last check) must be held, not treated as regress: it
  // neither advances the timer (would fire too early) nor resets it (would take far longer than the
  // real compute cadence to ever settle).
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  exit_output result = RUNNING;
  int fresh_computes = 0;
  while (result == RUNNING) {
    REQUIRE(fresh_computes <= 15);  // don't hang the suite if this regresses
    pid.compute_error(0.5, 0.0);    // one real compute: error held at 0.5in (inside small_error)
    ++fresh_computes;
    result = pid.exit_condition();  // the fresh poll for this compute
    if (result != RUNNING) break;
    result = pid.exit_condition();  // a stale poll -- no compute happened since the one just above
  }
  CHECK(result == SMALL_EXIT);
  // Exactly the same 10 real computes as the 1:1 cadence case above -- a stale poll neither speeds
  // this up nor slows it down.
  CHECK(fresh_computes == 10);
}

TEST_CASE("PID small exit still settles when the caller polls once for every two real computes") {
  // The caller here polls SLOWER than compute() runs. Freshness is a plain boolean (did at least one
  // compute land since the last check), not a count, so this still credits exactly one DELAY_TIME per
  // poll -- same poll count as the 1:1 case, just with twice as many (uncounted) computes behind it.
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  exit_output result = RUNNING;
  int polls = 0;
  while (result == RUNNING) {
    REQUIRE(polls <= 15);
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

TEST_CASE("PID motion_reset resyncs the small/big exit staleness baseline for the next motion") {
  // Mirrors the gap a real Drive motion setter leaves: the SAME PID keeps getting computed against
  // the OLD, just-finished motion (ez_auto_task doesn't stop just because a wait returned) right up
  // until a new pid_*_set() calls motion_reset(). Without motion_reset() resyncing
  // last_checked_compute, that leftover compute would still read as "fresh" on the new motion's very
  // first poll, crediting an old-target compute toward the new motion's own timer.
  PID pid;
  pid.exit_condition_set(0, 1.0, 0, 0);  // small only, small_exit_time=0: any credit at all fires

  pid.compute_error(0.5, 0.0);  // the old motion settling
  REQUIRE(pid.exit_condition() == SMALL_EXIT);

  // The old motion's own PID keeps getting computed a little longer (still against the old target),
  // the same as ez_auto_task would between the wait returning and the next setter running.
  pid.compute_error(0.5, 0.0);

  // A new motion starts: timers_reset() + motion_reset(), no compute yet for the new target.
  pid.timers_reset();
  pid.motion_reset(0.0);

  // Without a fresh compute for the new motion, the leftover compute above must not count.
  CHECK(pid.exit_condition() == RUNNING);
}

TEST_CASE("PID BIG_EXIT does not accumulate on a stale error with no fresh compute since a dead ez_auto_task's last pass") {
  // The small-exit staleness tests above all seed an error inside small_error, where `i = 0` on every
  // pass anyway -- they never actually exercise the big timer's own `if (error_fresh) i +=` gate. This
  // seeds an error inside big_error but OUTSIDE small_error, the only way to reach that gate at all,
  // then never computes again -- the same dead/deadlocked ez_auto_task shape the small-exit tests cover.
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);

  pid.compute_error(2.0, 0.0);  // outside small_error (1), inside big_error (3) -- one real compute
  REQUIRE(pid.exit_condition() == RUNNING);  // first poll is fresh, but 1 pass is nowhere near 250ms

  // No further compute lands from here on -- ez_auto_task is dead. Comfortably more than the 25
  // passes (250ms / DELAY_TIME) a live task would need to reach BIG_EXIT.
  for (int pass = 0; pass < 50; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
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

TEST_CASE("PID velocity exit fires after enough fresh but sub-threshold passes once the robot has moved") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // a real jump: arms
  CHECK(pid.exit_condition() == RUNNING);

  // Resting now, but every pass is still a genuinely fresh reading -- real sensor dither around
  // 1.0, not a raw value that simply repeats -- so this is the "actually still" case the exit is
  // supposed to catch, not a stale re-read (see the never-changes test below for that case).
  bool toggle = false;
  for (int pass = 1; pass < 6; pass++) {
    INFO("stationary pass ", pass);
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  dither_tick(pid, 10.0, toggle, 1.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID velocity timer clears on a single fresh moving tick, no debounce") {
  // An earlier design needed two CONSECUTIVE fresh moving ticks to clear k, to protect Drive's own
  // waits from an isolated noisy/jitter tick. That debounce is gone: Drive no longer routes any of
  // its own waits through this channel at all (without_velocity(), every DRIVE/TURN/SWING/odom
  // wait, exit_conditions.cpp), so its only remaining effect was letting k accumulate on the zero-
  // derivative ticks of a mechanism polled faster than its sensor refreshes and fire a false exit on
  // something still moving (see the sensor-refresh tests below). Any single fresh, above-threshold
  // sample now clears k at once, matching this PID's own pre-debounce behavior.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  bool toggle = false;
  for (int pass = 1; pass <= 4; pass++) {
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);  // k=40
  }

  pid.compute_error(10.0, 2.0);             // a single fresh moving tick
  CHECK(pid.exit_condition() == RUNNING);   // cleared at once: k=0, not "not yet cleared"

  // Proof it was actually cleared, not just still under threshold by coincidence: a fresh countdown
  // from here takes the full 5 stationary passes again, not just 1 more.
  for (int pass = 1; pass < 6; pass++) {
    dither_tick(pid, 10.0, toggle, 2.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  dither_tick(pid, 10.0, toggle, 2.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID velocity timer: a blip recurring more often than velocity_exit_time can defeat the exit indefinitely") {
  // Direct consequence of removing the debounce: a mechanism that is otherwise stalled but produces
  // one fresh, above-threshold sample often enough (here, one blip every 80ms against a 50ms exit
  // time) resets k before it can ever reach velocity_exit_time. This is expected, matching the
  // parent commit's identical shape -- a bare PID with no other progress backstop (unlike Drive's
  // own StuckWatch/SingleStuckWatch-backed waits) genuinely cannot tell "recurring contact jitter"
  // from "still slowly moving" from k alone. See the next test for a blip spaced out far enough not
  // to have this effect.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  double base = 1.0;
  bool toggle = false;
  for (int pass = 1; pass <= 200; pass++) {
    INFO("pass ", pass);
    if (pass % 8 == 0) {
      base += 0.5;  // an isolated blip: one fresh, well-above-threshold tick, up
      pid.compute_error(10.0, base);
    } else if (pass % 8 == 4) {
      base -= 0.5;  // ...and back down a few ticks later -- zero net displacement, still stuck
      pid.compute_error(10.0, base);
    } else {
      dither_tick(pid, 10.0, toggle, base);  // fresh, real dither, always within the stalled band
    }
    CHECK(pid.exit_condition() == RUNNING);  // never exits: a blip every 80ms keeps resetting k
  }
}

TEST_CASE("PID velocity exit does not fire on a healthy motion whose sensor only refreshes every other poll") {
  // A sensor that refreshes slower than the poll rate reports the SAME raw value on alternating
  // polls. Each poll is still a real, fresh compute() -- so the derivative it produces genuinely
  // reads 0 on the repeat half -- but the OTHER half's real advance resets k before the repeat half
  // can ever accumulate enough to fire.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  double real_position = 0.0;
  for (int pass = 1; pass <= 200; pass++) {
    INFO("pass ", pass);
    if (pass % 2 == 1) real_position += 0.1;  // the sensor only actually advances on odd polls
    pid.compute_error(10.0, real_position);   // even polls re-report the same value
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit does not fire on real motion whose sensor refreshes slower than the poll, at a realistic ratio") {
  // The realistic shape this fix's own "what could go wrong" line calls for: a sensor refreshing
  // every 30ms, polled every 10ms, with a realistic velocity_exit_time (500ms, not the artificially
  // tight 50ms most of this file's other tests use to keep loops short). Each poll is a real, fresh
  // compute(); two out of every three read a zero derivative (no new sample yet), but the third's
  // real step resets k well before 500ms could ever accumulate.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 500, 0);

  double position = 0.0;
  pid.compute_error(10.0, position);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  for (int pass = 1; pass <= 200; pass++) {  // 2000ms of continuous real motion
    INFO("pass ", pass);
    if (pass % 3 == 0) position += 0.3;  // the sensor's own refresh: a real step every 30ms
    pid.compute_error(10.0, position);
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit fires once a repeated reading has genuinely lasted longer than velocity_exit_time") {
  // The other half of the same fix: nothing distinguishes "sensor hasn't refreshed yet" from
  // "genuinely stopped" by duration alone any more -- an earlier design's 1000ms staleness grace
  // window before treating a repeated reading as stopped is gone. A repeated reading is simply
  // treated as stopped from the very first fresh repeat. A bare PID
  // with a realistic velocity_exit_time (500ms, comfortably above any real sensor's refresh gap)
  // still only fires after that much real stillness, same as before -- it just no longer waits an
  // extra second past that to start counting.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 500, 0);
  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 60);
    pid.compute_error(10.0, 1.0);  // the raw value never moves again from here on
    result = pid.exit_condition();
  }
  CHECK(pass == 51);  // 50 credits reaches exactly velocity_exit_time (not >), the 51st exceeds it
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity_exit_hold freezes k regardless of whether the raw reading is repeating or changing") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  pid.velocity_exit_hold_set(true);
  // 1500ms of a frozen raw value, well past several exit countdowns' worth -- held skips the
  // velocity block entirely (see exit_condition()'s `!held` gate), so k neither accumulates nor
  // resets while this runs.
  for (int pass = 1; pass <= 150; pass++) {
    INFO("held pass ", pass);
    pid.compute_error(10.0, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  pid.velocity_exit_hold_set(false);

  // Resuming behaves like a freshly-armed stall from here, not like anything accumulated while held.
  // velocity_armed was already true before the hold (set on the very first call above), so this
  // resumes straight into crediting -- no re-arm needed.
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 10);
    pid.compute_error(10.0, 1.0);
    result = pid.exit_condition();
  }
  CHECK(pass == 6);  // 6 fresh credits: 60 > velocity_exit_time (50)
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID timers_reset clears the velocity timer for the next motion even if it reads the same raw value as before") {
  // A new motion can happen to start reading the exact same raw value as where the previous one
  // left off (e.g. a lift that always re-arms from the same physical position). Without
  // timers_reset() clearing k, leftover credit from the old motion's own stall would carry straight
  // into the new motion and fire far sooner than a fresh motion should.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // first motion arms
  CHECK(pid.exit_condition() == RUNNING);
  for (int pass = 1; pass <= 4; pass++) {
    pid.compute_error(10.0, 1.0);
    CHECK(pid.exit_condition() == RUNNING);  // k=40
  }

  pid.timers_reset();  // a new motion starts

  // Re-arm with a real jump first: timers_reset() clears velocity_armed but not prev_current, so
  // immediately repeating the old value here (with no jump) would instead exercise the separate
  // 1000ms "hasn't moved" arm fallback rather than isolating this test to k specifically. A real
  // setter's own motion_reset() call reseeds prev_current the same way.
  pid.compute_error(10.0, 5.0);
  CHECK(pid.exit_condition() == RUNNING);

  // The real proof k was cleared, not left at 40: a fresh 6-credit countdown from here, not 1 more.
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 10);
    pid.compute_error(10.0, 5.0);  // now repeats: stopped
    result = pid.exit_condition();
  }
  CHECK(pass == 6);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit: a mechanism moving above the stopped floor every pass never exits, even when a same-value re-read lands right before the poll") {
  // A caller whose task calls compute() more than once between the wait's own polls sees its LATEST
  // compute's derivative masked to 0 if that specific call happens to re-read an already-seen value --
  // but the channel must still judge on whatever happened across every real compute since the last
  // poll, not just the last one (see velocity_moving_count's comment in PID.hpp). Each
  // pass here moves 0.1, twice the 0.05 stopped floor -- a mechanism that has never actually stopped --
  // so this must never exit, no matter how many passes run.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);  // real motion: arms
  CHECK(pid.exit_condition() == RUNNING);

  double real_position = 1.0;
  for (int pass = 1; pass <= 20; pass++) {
    INFO("pass ", pass);
    real_position += 0.1;                        // a real, fresh jump -- comfortably above the floor
    pid.compute_error(10.0, real_position);
    pid.compute_error(10.0, real_position);       // immediately re-read the same value before the poll
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit: two real computes landing before one poll still exits promptly on a genuine stop") {
  // Companion to the above: once the mechanism has actually stopped (both computes between polls read
  // the same, unchanging value), the exit still fires within the configured time -- this isn't a
  // channel that stopped crediting real stalls, just one that no longer judges solely on the very last
  // compute before a poll.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);  // real motion: arms, holds at the same value the loop below repeats
  CHECK(pid.exit_condition() == RUNNING);

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 10);
    pid.compute_error(10.0, 1.0);  // stopped: both computes this pass read the same, already-held value
    pid.compute_error(10.0, 1.0);
    result = pid.exit_condition();
  }
  CHECK(pass == 6);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit: an above-floor jitter compute mixed into an otherwise stalled cadence still resets the timer") {
  // What could go wrong with judging on the whole window instead of just the latest compute: a single
  // above-floor blip anywhere in a poll's window must still reset k, exactly as an above-floor LATEST
  // compute already did before this fix -- the window is strictly more sensitive to real movement, not
  // less. Two computes land per poll throughout; every 6th poll, the first of its two computes jitters
  // 1.0 above the previous position (comfortably over the 0.05 floor) before the second re-settles back
  // to the same held value -- still comfortably below the 500ms exit time, so this must never exit.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 500, 0);
  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  double held = 1.0;
  for (int pass = 1; pass <= 40; pass++) {
    INFO("pass ", pass);
    if (pass % 6 == 0) {
      pid.compute_error(10.0, held + 1.0);  // jitter: a real, above-floor movement mid-window
      pid.compute_error(10.0, held);        // settles back to the held value before the poll
    } else {
      pid.compute_error(10.0, held);
      pid.compute_error(10.0, held);
    }
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity timer does not double-count one real stalled sample across two checks between one compute") {
  // What could go wrong with keying k purely off error_fresh: a caller polling FASTER than compute()
  // runs can call exit_condition() twice for the same one real sample. The second check must not
  // credit that same sample a second time -- error_fresh (compute_count vs. last_checked_compute,
  // PID.hpp) is what catches this.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  bool toggle = false;
  for (int pass = 1; pass <= 4; pass++) {
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);  // k=40
  }

  dither_tick(pid, 10.0, toggle, 1.0);       // one more real, fresh, stalled sample: k reaches 50
  CHECK(pid.exit_condition() == RUNNING);    // 1st check of it: credited once, k=50, not yet >50
  CHECK(pid.exit_condition() == RUNNING);    // 2nd check, no new compute since: must not credit again

  dither_tick(pid, 10.0, toggle, 1.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);  // one more fresh credit: k=60, now fires
}

TEST_CASE("PID velocity exit treats a non-finite derivative as stopped") {
  // A dead sensor feeding compute_error() a non-finite current (e.g. from a disconnected motor's
  // PROS_ERR_F position) produces a non-finite derivative. Left unsanitized, that satisfies neither
  // "moving" nor a clean "stopped" reading and could dodge this exit indefinitely on a bare PID with
  // no other backstop. It is now explicitly sanitized to 0.0 -- stopped -- so it counts toward the
  // timer like any other stall. See test_sensorfault_mA_disconnect.cpp for the matching Motor-level
  // coverage (a disconnected motor's mA timer independently catches this too).
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);  // arms with a real, finite jump first
  CHECK(pid.exit_condition() == RUNNING);

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 10);
    pid.compute_error(10.0, std::numeric_limits<double>::quiet_NaN());  // sensor died
    result = pid.exit_condition();
  }
  CHECK(pass == 6);
  CHECK(result == VELOCITY_EXIT);
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

TEST_CASE("PID secondary velocity sensor never exits on a reading that was never taken") {
  // second_sensor defaults to NaN (no imu, or velocity_sensor_secondary_set() was never called).
  // drive_imu_accel_get() also returns NaN with no imu (see Drive::drive_imu_accel_get). Either
  // way this must never be misread as "0 acceleration", which would falsely count as stopped.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.error = 10.0;
  pid.derivative = 1.0;  // real, continuous movement: arms immediately, keeps the main channel from firing too

  CHECK_FALSE(std::isfinite(pid.velocity_sensor_secondary_get()));

  for (int pass = 1; pass <= 200; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // A real reading afterwards is still honored by the secondary channel (main stays non-zero throughout).
  pid.velocity_sensor_secondary_set(0.0);
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 50);  // don't hang the suite if this regresses
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity_exit_hold freezes both velocity timers while true") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.velocity_sensor_secondary_toggle_set(true);
  pid.velocity_sensor_secondary_set(0.0);  // reads as stationary too, same as the main sensor
  pid.error = 10.0;
  pid.derivative = 1.0;

  CHECK(pid.exit_condition() == RUNNING);  // real movement: arms immediately

  // Now reads exactly like a stall on both channels, but held -- neither may fire even though 50ms
  // (5 passes) would normally be well past both.  100 passes is 1000ms, safely under the 2000ms
  // fallback below.
  pid.derivative = 0.0;
  pid.velocity_exit_hold_set(true);
  for (int pass = 1; pass <= 100; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // Releasing the hold resumes from wherever the timers were left (0, since they were frozen, not
  // reset), so it takes another full velocity_exit_time to fire, same as a fresh stall would.
  pid.velocity_exit_hold_set(false);
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 50);
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity_exit_hold cannot be held forever") {
  // A caller holding this because of its own condition that never resolves (for example, a
  // genuinely double-stalled robot that can neither translate nor turn) must not be able to hang
  // whatever is waiting on exit_condition() forever.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);
  CHECK(pid.exit_condition() == RUNNING);  // arm

  pid.velocity_exit_hold_set(true);  // held indefinitely by the caller; never released in this test

  // While held, the main channel's own block is skipped outright (guarded by !held), so what this
  // tick reports doesn't matter until the fallback releases the hold below -- kept as a real,
  // fresh, resting-band dither for consistency with the resumed phase.
  double base = 1.0;
  bool toggle = false;
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    // VELOCITY_EXIT_HOLD_FALLBACK (2000ms) plus velocity_exit_time (50ms) must resolve well inside
    // this; 250 passes is 2500ms.
    REQUIRE(pass <= 250);
    dither_tick(pid, 10.0, toggle, base);
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);
}
