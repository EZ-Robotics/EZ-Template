// StuckWatch's angle channel seeds its Channel's low/side directly from current_a_odomPID.error at
// construction, with no freshness guard -- unlike SingleStuckWatch (see
// test_drive_wait_stuck_watch_seed_survives_skipped_first_tick.cpp /
// test_turn_wait_stuck_watch_seed_survives_skipped_first_tick.cpp for the DRIVE/TURN/SWING version of
// this same underlying gap), whose own seeded_ guard defers trusting its construction-time error until
// a real background tick has landed. pid_wait()'s odom branch shares the exact same
// leading-delay-then-construct shape: a single, fixed pros::delay(DELAY_TIME) runs once before
// StuckWatch is constructed from xyPID/current_a_odomPID's current error (exit_conditions.cpp,
// "StuckWatch watch(xyPID, current_a_odomPID, ...)"), on the assumption that delay always contains at
// least one real background PID pass. It doesn't have to -- the wait loop and the background PID task
// are two independently scheduled periodic loops that are never lock-stepped, so an entirely ordinary
// single missed tick during that one delay (no starvation, nothing wrong afterward) seeds the angle
// channel from a stale reading instead of a real one. motion_reset()/timers_reset() never touch
// `error` (only a real compute_error() does), so this isn't limited to a fresh Drive's very first
// motion -- any ordinary motion whose own first tick happens to be missed is exposed the same way.
//
// xy is held at a constant zero error throughout (a real compute_error() call every pass it's fed, so
// its own small/big exit timers genuinely accumulate) so it settles almost immediately and never
// contributes progress of its own -- isolating the angle channel as the only thing StuckWatch's
// combined progress check can credit here, the same isolation
// test_stuck_watch_credits_recovery_after_a_shove.cpp uses for a single side. Every script drives
// current_a_odomPID through a real compute_error() call, not a direct `.error =` write, matching the
// idiom in test_stuckwatch_starvation_cadence.cpp.
//
// Both scripts share the same close-then-bump-then-slow-recover shape as the DRIVE/TURN versions of
// this test: a real heading disturbance survives cleanly when every tick lands (control), but
// false-stops the wait when only the very first tick of the wait being tested is skipped, because the
// stale-to-real jump on the next tick reads as a fake overshoot/shove and spends the underlying
// Channel's one-shot recovery allowance before the real disturbance ever happens.
#include <algorithm>
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_fresh_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  // Shipped defaults, confirmed from drive.cpp's own constructor call.
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
}

// Closes normally from 60 down to 16, then a real bump back up to 40, then a slow recovery -- slow
// enough that beating the PRE-BUMP low (16, so needs to reach under 13 at small_error's 3deg step)
// takes far longer than one progress-backstop window if the rebound was already spent, but comfortably
// fits inside one window if it wasn't (only needs to beat the bump's own low of 40, i.e. reach under
// 37).
double disturbance_error(int n) {
  if (n <= 2) return 60.0;
  if (n <= 13) return 60.0 - 4.0 * (n - 2);  // n=3:56 ... n=13:16
  if (n == 14) return 40.0;                  // a single real bump
  return std::fmax(40.0 - 0.3 * (n - 14), 0.0);
}

// The first tick this wait's own leading delay ever sees is skipped entirely -- no compute at all,
// and no auto_task_passes credit either -- simulating the background odom PID task simply not getting
// a pass in during the wait's fixed leading delay this one time. Both PIDs are computed together (or
// not at all), matching a real ez_auto_task tick, which recomputes every PID in one pass.
// n==1 is a genuinely missed tick.
void skip_first_tick(Drive& c, int n) {
  if (n == 1) return;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  c.xyPID.compute_error(0.0, 0.0);
  double e = disturbance_error(n);
  c.current_a_odomPID.compute_error(e, e);
}

// Control: the identical shape, but every tick lands and is credited.
void every_tick_lands(Drive& c, int n) {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  c.xyPID.compute_error(0.0, 0.0);
  double e = n == 1 ? 60.0 : disturbance_error(n);
  c.current_a_odomPID.compute_error(e, e);
}

Drive* g_chassis = nullptr;
void (*g_script)(Drive&, int) = nullptr;
int g_pass = 0;

void on_delay() {
  ++g_pass;
  g_script(*g_chassis, g_pass);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run_wait(Drive& chassis, void (*script)(Drive&, int), int max_passes) {
  g_chassis = &chassis;
  g_script = script;
  g_pass = 0;
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}
}  // namespace

TEST_CASE("pid_wait() odom: a real heading bump-and-slow-recovery survives when every tick lands") {
  Drive chassis = make_fresh_chassis();
  configure(chassis);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 60});
  // odom_start is snapshotted once, at motion start, above -- moving the live pose away from it here
  // (without touching odom_start, and without touching heading, which is sensor-derived and only
  // resynced by the background tracking task this test harness doesn't run) makes StuckWatch see real
  // travel since the motion's own start right from construction, the same as an ordinary motion that's
  // already been running a moment, so this test exercises StuckWatch's real window instead of its
  // "hasn't moved yet" startup allowance (which would otherwise swallow this whole scenario inside its
  // own grace period).
  chassis.odom_pose_set({0.0, 5.0, ANGLE_NOT_SET});

  Outcome o = run_wait(chassis, every_tick_lands, 300);
  MESSAGE("every tick lands: returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() odom: an ordinary missed first tick makes the same survivable heading bump false-stuck the wait") {
  Drive chassis = make_fresh_chassis();
  configure(chassis);
  chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 60});
  chassis.odom_pose_set({0.0, 5.0, ANGLE_NOT_SET});

  Outcome o = run_wait(chassis, skip_first_tick, 300);
  MESSAGE("first tick skipped: returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}
