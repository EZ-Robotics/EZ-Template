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
// pass.  A raw reading that stays bit-identical for a short while is still
// treated as a possible stale re-read and ignored, the same as before -- but
// once it's stayed bit-identical for longer than any real sensor could
// plausibly take to refresh, further repeats of it count as fresh,
// zero-velocity samples, so a genuinely, permanently stalled bare PID (a lift,
// claw, catapult -- the mechanisms with no StuckWatch/SingleStuckWatch
// backstop the way Drive's own waits have) still exits instead of hanging.
// timers_reset() disarms.  The secondary sensor is gated by the same flag.
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

TEST_CASE("PID velocity timer needs two consecutive fresh moving passes to clear, not one") {
  // A single moving tick no longer fully resets the timer on its own: an isolated noisy/jitter
  // reading -- contact defense, drivetrain backlash under a sustained push -- must not be able to
  // masquerade as "moving again" and erase real accumulated stillness. Two CONSECUTIVE fresh
  // moving ticks still do, since that's what actually distinguishes genuine renewed motion from a
  // one-tick blip at a 10ms tick rate.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  bool toggle = false;
  for (int pass = 1; pass <= 5; pass++) {
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);  // k=50, 1 pass short of exiting
  }

  pid.compute_error(10.0, 2.0);
  CHECK(pid.exit_condition() == RUNNING);  // a single fresh moving tick: k is NOT cleared (still 50)
  pid.compute_error(10.0, 3.0);
  CHECK(pid.exit_condition() == RUNNING);  // a second consecutive one: NOW it clears

  for (int pass = 1; pass <= 5; pass++) {
    dither_tick(pid, 10.0, toggle, 3.0);
    CHECK(pid.exit_condition() == RUNNING);  // fresh 5-pass countdown
  }
  dither_tick(pid, 10.0, toggle, 3.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID velocity timer is not defeated by an isolated fresh blip amid genuine dither") {
  // A robot with zero net progress (genuinely pinned/stuck) whose sensor keeps dithering near its
  // resting point -- fresh, real readings, always within the stalled band -- except for the
  // occasional isolated tick that reads well above the threshold (contact jitter, drivetrain
  // backlash under a sustained push) must still eventually accumulate past velocity_exit_time.
  // Only two CONSECUTIVE fresh moving ticks clear the accumulator; a lone blip, spaced out from
  // the next one, never reaches that.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  double base = 1.0;
  bool toggle = false;
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 200);  // don't hang the suite if this regresses
    if (pass % 8 == 0) {
      base += 0.5;  // an isolated blip: one fresh, well-above-threshold tick, up
      pid.compute_error(10.0, base);
    } else if (pass % 8 == 4) {
      base -= 0.5;  // ...and back down a few ticks later -- zero net displacement, still stuck
      pid.compute_error(10.0, base);
    } else {
      dither_tick(pid, 10.0, toggle, base);  // fresh, real dither, always within the stalled band
    }
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit does not fire on a healthy motion whose sensor only refreshes every other poll") {
  // A sensor that refreshes slower than the poll rate reports the SAME raw value on alternating
  // polls -- a stale re-read, not a new sample -- even while the robot is continuously, genuinely
  // making progress. A stale re-read must not count as evidence of a stall.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  double real_position = 0.0;
  for (int pass = 1; pass <= 200; pass++) {
    INFO("pass ", pass);
    if (pass % 2 == 1) real_position += 0.1;  // the sensor only actually advances on odd polls
    pid.compute_error(10.0, real_position);   // even polls re-report the same value: a stale re-read
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity exit does not fire while the raw sensor is frozen mid-motion, and resumes after") {
  // The sensor stops producing new samples for an extended stretch while the robot keeps moving
  // underneath it (a firmware hiccup, not a genuine stall) -- must not falsely stall during the
  // freeze, and must behave normally again the moment a fresh, differing value arrives.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // real motion: arms
  CHECK(pid.exit_condition() == RUNNING);

  // 80 consecutive stale re-reads of the same raw value, standing in for the sensor going quiet
  // while the real robot (not modeled here -- the PID only ever sees what the sensor reports)
  // keeps advancing underneath it.
  for (int pass = 1; pass <= 80; pass++) {
    INFO("frozen pass ", pass);
    pid.compute_error(10.0, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // A fresh value finally arrives, reflecting all the real motion that happened during the freeze.
  pid.compute_error(10.0, 9.0);
  CHECK(pid.exit_condition() == RUNNING);

  // Normal operation resumes: a genuine stall from here on is still caught.
  double base = 9.0;
  bool toggle = false;
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 50);
    dither_tick(pid, 10.0, toggle, base);
    result = pid.exit_condition();
  }
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit does not fire on a short run of repeated readings") {
  // A raw value that briefly repeats itself, tick after tick, is indistinguishable from a stale
  // re-read using the raw value alone -- there's no way to tell "the robot stopped" from "the
  // sensor hasn't refreshed yet" this early. Short runs like this (see also the "healthy motion
  // whose sensor only refreshes every other poll" and "frozen mid-motion" tests above, which run
  // this same shape for up to 800ms while real motion continues underneath) must still be ignored
  // rather than counted as a stall.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // one real jump: arms
  CHECK(pid.exit_condition() == RUNNING);

  // 50 consecutive repeats of the same raw value -- comfortably inside the staleness window --
  // must not be read as a stall.
  for (int pass = 1; pass <= 50; pass++) {
    INFO("pass ", pass);
    pid.compute_error(10.0, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }

  // Genuine, fresh motion afterward is unaffected: a short stale run doesn't secretly advance the
  // stall timer, so this still takes the normal 6 fresh stationary passes to exit.
  bool toggle = false;
  for (int pass = 1; pass < 6; pass++) {
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  dither_tick(pid, 10.0, toggle, 1.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit eventually fires on a raw reading that never changes, once it's stayed that way far longer than any real sensor gap") {
  // Unlike a short run, a raw value that stays bit-identical for far longer than a real sensor
  // could plausibly take to refresh is no longer distinguishable from a genuine, permanent stall
  // (jammed against a hard stop) -- this is exactly the case a bare ez::PID with no other progress
  // backstop (a lift, claw, catapult; unlike Drive's own waits, which StuckWatch/SingleStuckWatch
  // back up independently -- see "a raw sensor that never changes at all is still caught" in
  // test_jc1_non_odom_stuck.cpp) previously had no way to exit from. Past the staleness window,
  // further repeats of the same reading count as fresh, zero-velocity samples, so the ordinary
  // velocity_exit_time countdown can run and this exits instead of hanging forever.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);  // one real jump: arms
  CHECK(pid.exit_condition() == RUNNING);

  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 200);
    pid.compute_error(10.0, 1.0);  // the raw value never moves again from here on
    result = pid.exit_condition();
  }
  // Doesn't fire before the staleness window itself has elapsed (roughly 100 passes at 10ms
  // each), and doesn't take dramatically longer than staleness + velocity_exit_time either.
  CHECK(pass >= 100);
  CHECK(pass <= 115);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit does not fire on a genuinely slow motion whose raw reading changes just inside the staleness window") {
  // What could go wrong with judging a stall by duration alone: a mechanism moving genuinely
  // slowly (not stalled) could have its raw reading go unchanged for a while without actually
  // being stopped. As long as it changes again -- by more than jitter, comfortably above
  // velocity_zero_main -- before the staleness window elapses, it must keep reading as healthy,
  // never as stopped, no matter how many times that repeats.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  double position = 0.0;
  pid.compute_error(10.0, position);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  for (int step = 0; step < 3; step++) {
    for (int pass = 1; pass <= 90; pass++) {  // 900ms of repeats, inside the 1000ms staleness window
      INFO("step ", step, " pass ", pass);
      pid.compute_error(10.0, position);
      CHECK(pid.exit_condition() == RUNNING);
    }
    position += 0.1;  // a genuine, if slow, step -- well above velocity_zero_main's 0.05
    pid.compute_error(10.0, position);
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID raw-value staleness is frozen while velocity_exit_hold is set, not secretly advancing") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  pid.velocity_exit_hold_set(true);
  // 1500ms of a frozen raw value, well past the 1000ms staleness window, all held -- must not
  // fire while held, the same as k/m are frozen (see velocity_exit_hold_set()'s doc).
  for (int pass = 1; pass <= 150; pass++) {
    INFO("held pass ", pass);
    pid.compute_error(10.0, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }
  pid.velocity_exit_hold_set(false);

  // Resuming behaves like a freshly-armed stall from here, not like the staleness window had
  // already secretly elapsed while held -- it still takes the full staleness window plus
  // velocity_exit_time to fire, not a handful of passes.
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 150);
    pid.compute_error(10.0, 1.0);
    result = pid.exit_condition();
  }
  CHECK(pass >= 100);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID timers_reset clears the raw-value staleness streak for the next motion") {
  // A new motion can happen to start reading the exact same raw value as where the previous one
  // left off (e.g. a lift that always re-arms from the same physical position, or simply hasn't
  // been touched between calls). Without clearing the staleness streak here, the leftover time
  // from the old motion's frozen stretch would carry straight into the new motion's own repeats
  // and cross the staleness window far earlier than a fresh motion should.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // first motion arms
  CHECK(pid.exit_condition() == RUNNING);
  for (int pass = 1; pass <= 90; pass++) {  // 900ms, just inside the staleness window
    pid.compute_error(10.0, 1.0);
    CHECK(pid.exit_condition() == RUNNING);
  }

  pid.timers_reset();  // a new motion starts

  // The next reading is the exact same 1.0 as before the reset, so its own derivative reads 0 --
  // not a real jump -- and this motion only arms via the 1000ms "hasn't moved" fallback, same as
  // any other motion that starts out reading a constant value. With the streak properly cleared,
  // that fallback arming (~pass 101) plus a fresh staleness window (~1000ms) plus
  // velocity_exit_time (50ms) takes until roughly pass 207 to exit. If the streak carried over
  // instead, arming would still land around pass 101, but the leftover ~900ms would push the
  // staleness window over by roughly pass 111, exiting by ~pass 116. Staying RUNNING at least to
  // pass 150 is real proof the streak was cleared, not just that fallback arming happened at all.
  int pass = 0;
  exit_output result = RUNNING;
  while (result == RUNNING) {
    pass++;
    REQUIRE(pass <= 230);
    pid.compute_error(10.0, 1.0);  // the exact same raw value as before the reset
    result = pid.exit_condition();
  }
  CHECK(pass >= 150);
  CHECK(result == VELOCITY_EXIT);
}

TEST_CASE("PID velocity exit does not fire when checked slower than a sensor that refreshes every other tick") {
  // What could go wrong with keying "fresh" off the raw value alone: a caller whose own poll loop
  // runs slower than compute() (or that just happens to land after the stale half of a sensor
  // that refreshes every other tick) can see a raw value that HAS changed since its last check,
  // paired with THIS tick's derivative reading exactly 0 -- the artifact of the most recent
  // compute() itself having been a stale re-read. That combination must not count as fresh either:
  // catching it needs both signals (a changed raw value AND a nonzero latest derivative) to agree,
  // not just the raw-value comparison on its own.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // real motion: arms
  CHECK(pid.exit_condition() == RUNNING);

  double real_position = 1.0;
  for (int check = 1; check <= 200; check++) {
    INFO("check ", check);
    real_position += 0.1;
    pid.compute_error(10.0, real_position);  // a fresh sample the check below never sees on its own
    pid.compute_error(10.0, real_position);  // immediately re-read stale: derivative reads exactly 0
    CHECK(pid.exit_condition() == RUNNING);
  }
}

TEST_CASE("PID velocity timer does not double-count one real sample across two checks between one compute") {
  // The other half of what could go wrong with this fix: a caller polling FASTER than compute()
  // runs can call exit_condition() twice for the same one real sample. If freshness were judged by
  // the latest derivative alone, both calls would see that one sample's nonzero derivative and both
  // would count it -- a single blip read twice looking like two consecutive misses, clearing k the
  // same way an isolated blip must not (see the jitter test above). Tracking what THIS check last
  // saw (k_prev_checked), not raw_compute()'s own bookkeeping, is what catches this: the second of
  // two checks between one compute() sees a raw value that hasn't changed since the first checked it.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);

  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  bool toggle = false;
  for (int pass = 1; pass <= 5; pass++) {
    dither_tick(pid, 10.0, toggle, 1.0);
    CHECK(pid.exit_condition() == RUNNING);  // k=50, 1 pass short
  }

  pid.compute_error(10.0, 2.0);            // one real, fresh, moving sample
  CHECK(pid.exit_condition() == RUNNING);  // 1st check of it: counted once
  CHECK(pid.exit_condition() == RUNNING);  // 2nd check, no new compute in between: must not count again

  dither_tick(pid, 10.0, toggle, 2.0);
  CHECK(pid.exit_condition() == VELOCITY_EXIT);  // k was never cleared: still fires right on schedule
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
