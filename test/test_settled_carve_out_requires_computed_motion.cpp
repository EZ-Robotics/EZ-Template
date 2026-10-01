// pid_wait()'s stuck backstop has a "settled" carve-out: a stuck verdict whose PID error is already
// inside the big error window is counted as settled (a clean return, interfered stays false) instead
// of stuck. That reads `error` -- but motion_reset()/timers_reset() never touch `error`, only a real
// compute()/compute_error() does. If ez_auto_task never computes anything for a newly-set motion
// (starved or dead), `error` is still whatever the PREVIOUS motion's last compute left behind, and if
// that was small, the wall-clock fallback fires, the carve-out reads it as "settled at the new
// target", and pid_wait() reports clean success for a motion the robot never even started -- with the
// new target far away.
//
// Each of the first three tests leaves a small error behind from a "previous motion", sets a new
// far-away motion, and never computes again. The wait must still end (bounded, as before) but with
// interfered=true. The last test is the control: a healthy motion that hovers inside its big error
// window (so the carve-out is what ends the wait) under 1 and 0/1/2 computes per poll must still
// return cleanly.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
enum Kind {
  DRIVE_K,
  TURN_K,
  SWING_K
};

Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
Kind g_kind = DRIVE_K;
int g_pass = 0;
int g_pattern_phase = 0;

void compute_once(Drive& c, double error) {
  // A rate of 0.2 per compute keeps the derivative above the velocity floor so the velocity exit
  // cannot end the wait by itself: only the stuck backstop (and its settled carve-out) can.
  switch (g_kind) {
    case DRIVE_K:
      c.leftPID.compute_error(error, c.leftPID.cur + 0.2);
      c.rightPID.compute_error(error, c.rightPID.cur + 0.2);
      break;
    case TURN_K:
      c.turnPID.compute_error(error, c.turnPID.cur + 0.2);
      break;
    case SWING_K:
      c.swingPID.compute_error(error, c.swingPID.cur + 0.2);
      break;
  }
  ez::detail::stats.auto_task_passes.fetch_add(1);
}

void set_exits(Drive& c) {
  // Small exit needs to hold 100 ms inside 0.5; big exit disabled; velocity window 500 ms. A hover at
  // ~1.0 is outside small, inside big (3.0), and never stops moving, so it can only end via stuck.
  switch (g_kind) {
    case DRIVE_K:
      c.pid_drive_exit_condition_set(100, 0.5, 0, 3.0, 500, 0);
      break;
    case TURN_K:
      c.pid_turn_exit_condition_set(100, 0.5, 0, 3.0, 500, 0);
      break;
    case SWING_K:
      c.pid_swing_exit_condition_set(100, 0.5, 0, 3.0, 500, 0);
      break;
  }
}

void start_motion(Drive& c) {
  switch (g_kind) {
    case DRIVE_K:
      c.pid_drive_set(100, 100);
      break;
    case TURN_K:
      c.pid_turn_set(170, 100);
      break;
    case SWING_K:
      c.pid_swing_set(ez::LEFT_SWING, 170, 100);
      break;
  }
}

// The task never computes and never ticks.
void on_delay_dead() { ++g_pass; }

// 0/1/2 computes on successive polls, cycling.
void on_delay_jitter() {
  ++g_pass;
  int k = g_pattern_phase++ % 3;
  for (int i = 0; i < k; i++) compute_once(*g_chassis, (g_pass + i) % 2 ? 0.9 : 1.1);
}

// Exactly one compute per poll.
void on_delay_every() {
  ++g_pass;
  compute_once(*g_chassis, g_pass % 2 ? 0.9 : 1.1);
}

struct Result {
  bool returned;
  int passes;
  bool interfered;
};

Result run(Kind kind, void (*hook)(), bool seed_stale_small_error) {
  g_kind = kind;
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  set_exits(chassis);
  g_chassis = &chassis;
  g_pass = 0;
  g_pattern_phase = 0;
  if (seed_stale_small_error) {
    // The "previous motion" finished with a small error, then a new far-away motion is set and the
    // task never computes for it.
    compute_once(chassis, 0.2);
    ez::detail::stats.auto_task_passes.store(0);
  }
  start_motion(chassis);
  test_stub::g_clock.on_delay = hook;
  test_stub::g_clock.delay_calls_until_stop = 800;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  return {returned, g_pass, chassis.interfered};
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a motion the task never computed is not reported as settled off a stale small error") {
  Result r = run(DRIVE_K, on_delay_dead, true);
  MESSAGE("passes=", r.passes, " interfered=", r.interfered);
  REQUIRE(r.returned);
  CHECK(r.interfered);
}

TEST_CASE("pid_wait() TURN: a motion the task never computed is not reported as settled off a stale small error") {
  Result r = run(TURN_K, on_delay_dead, true);
  MESSAGE("passes=", r.passes, " interfered=", r.interfered);
  REQUIRE(r.returned);
  CHECK(r.interfered);
}

TEST_CASE("pid_wait() SWING: a motion the task never computed is not reported as settled off a stale small error") {
  Result r = run(SWING_K, on_delay_dead, true);
  MESSAGE("passes=", r.passes, " interfered=", r.interfered);
  REQUIRE(r.returned);
  CHECK(r.interfered);
}

TEST_CASE("pid_wait(): a healthy motion hovering inside the big window still counts as settled at 0/1/2 computes per poll") {
  for (Kind k : {DRIVE_K, TURN_K, SWING_K}) {
    for (void (*hook)() : {on_delay_every, on_delay_jitter}) {
      Result r = run(k, hook, false);
      MESSAGE("kind=", (int)k, " passes=", r.passes, " interfered=", r.interfered);
      REQUIRE(r.returned);
      CHECK_FALSE(r.interfered);
    }
  }
}
