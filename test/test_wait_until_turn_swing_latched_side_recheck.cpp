// wait_until_turn_swing_internal()'s TURN and SWING branches have the identical single-PID version
// of wait_until_drive()'s latch-with-no-gate gap (see
// test_wait_until_drive_final_target_settle_gate.cpp): once turn_exit/swing_exit latches a window
// exit (SMALL_EXIT/BIG_EXIT), the `else` branch used to return a clean result with no at_final_target
// gate at all (only mA_EXIT/VELOCITY_EXIT set interfered=true there) -- so a genuine stall between a
// wait_until() checkpoint and the motion's real final target (close enough to the real target for the
// PID's OWN exit to latch, but short of the checkpoint itself) was reported as a clean,
// uninterfered success.
//
// Both branches now gate a trusted clean window exit on this wait_until()'s target actually being
// the motion's final target, the same way wait_until_drive()'s else branch and the sibling
// stuck-detected path already do -- and recheck a latched exit against its own window using the live
// error right before trusting it at all, un-latching if it has drifted back outside (unreachable by a
// single-pass-timing test the way the two-sided DRIVE case is, since this branch is evaluated again
// on the very next pass after latching with nothing else to occupy the wait in between -- but the
// same live-error recheck a drift would need is in place regardless, matching the DRIVE branch's).
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Hovers across small_error(3deg) every pass -- comfortably inside big_error(7deg) the whole time,
// but PID.cpp's own exit timers reset every time error dips back under small_error, so turn_exit
// never latches at all; the only thing that can end this wait is the no-progress watch.
void hover_script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  double e = (g_pass % 2 == 0) ? 2.7 : 3.3;
  c.turnPID.error = e;
  c.turnPID.derivative = (g_pass % 2 == 0) ? 0.3 : -0.3;
}

// Held fixed at a stall well short of a wait_until() checkpoint, but close enough to the motion's
// real final target for the PID's own exit to latch -- a genuine stall/pin at a fixed error, not
// scripted convergence. g_stall_error is the (constant) distance from the stall position to the
// motion's real final target, matching what turnPID/swingPID.error would read at that position.
double g_stall_error = 0.0;
// A real compute_error() call every pass, not a direct `.error =` write -- the big exit timer only
// credits `error` when a real compute has landed since it last checked (see PID.cpp), so "held at a
// fixed stall" has to mean a real compute repeatedly landing on the same value.
void stalled_turn_pass() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_chassis->turnPID.compute_error(g_stall_error, 0.0);
  g_chassis->turnPID.derivative = 0.0;
}
void stalled_swing_pass() {
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_chassis->swingPID.compute_error(g_stall_error, 0.0);
  g_chassis->swingPID.derivative = 0.0;
}
}  // namespace

// drive_angle_get() (the stub's fake IMU heading) never moves in this harness -- only turnPID/
// swingPID.error is scripted directly -- so wait_until_turn_swing_internal()'s own crossing check
// never fires here, isolating that the result comes from the exit-condition/gate machinery this test
// is about, not the crossing check.
TEST_CASE("pid_wait_until() TURN: a stall between the checkpoint and the real target that latches BIG_EXIT inside big_error is reported clean") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Position exits only (velocity/mA off) -- isolates this to the small/big latch path, matching
  // test_wait_until_drive_final_target_settle_gate.cpp's own isolation.
  chassis.pid_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_turn_set(24.0, 100);
  REQUIRE(chassis.mode == TURN);
  double final_target = chassis.turnPID.target_get();

  const double checkpoint = final_target - 2.0;  // short of final_target, inside its big_error(7)
  g_stall_error = 5.0;                           // short of checkpoint too, and < big_error(7)

  g_chassis = &chassis;
  stalled_turn_pass();  // seed turnPID.error before the wait's own first settle delay reads it
  test_stub::g_clock.on_delay = stalled_turn_pass;
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_until(checkpoint * ez::degree);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " checkpoint=", checkpoint, " turnPID.error=", chassis.turnPID.error);
  REQUIRE(returned);
  // Rewritten for the one rule for "finished" (checkpoint_end() in exit_conditions.cpp): the robot settled inside big_error of the
  // motion's final target with this checkpoint between where it rested and that target, so the checkpoint counts as reached, the way
  // pid_wait() on the same motion says clean. This used to read interfered; outside big_error it still does (see test_exit_gate_verdicts.cpp).
  CHECK_FALSE(chassis.interfered);
}

TEST_CASE("pid_wait_until() SWING: a stall between the checkpoint and the real target that latches BIG_EXIT inside big_error is reported clean") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_swing_set(ez::LEFT_SWING, 24.0, 100);
  REQUIRE(chassis.mode == SWING);
  double final_target = chassis.swingPID.target_get();

  const double checkpoint = final_target - 2.0;
  g_stall_error = 5.0;

  g_chassis = &chassis;
  stalled_swing_pass();
  test_stub::g_clock.on_delay = stalled_swing_pass;
  test_stub::g_clock.delay_calls_until_stop = 200;

  bool returned = true;
  try {
    chassis.pid_wait_until(checkpoint * ez::degree);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " checkpoint=", checkpoint, " swingPID.error=", chassis.swingPID.error);
  REQUIRE(returned);
  // Rewritten for the one rule for "finished" (checkpoint_end() in exit_conditions.cpp): the robot settled inside big_error of the
  // motion's final target with this checkpoint between where it rested and that target, so the checkpoint counts as reached, the way
  // pid_wait() on the same motion says clean. This used to read interfered; outside big_error it still does (see test_exit_gate_verdicts.cpp).
  CHECK_FALSE(chassis.interfered);
}

// Control: a genuinely stuck-and-settled turn (hovers across small_error, never fully latches, ends
// only via the stuck watch) must still get the settled exemption -- proving the recheck/gate added
// above didn't regress the sibling stuck-detected path's own existing behavior.
TEST_CASE("pid_wait_until() TURN control: hovering settled at the final target without a clean latch still returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(90, 100);
  double final_target = chassis.turnPID.target_get();

  g_chassis = &chassis;
  g_pass = 0;
  hover_script();
  test_stub::g_clock.on_delay = hover_script;
  test_stub::g_clock.delay_calls_until_stop = 400;

  bool returned = true;
  try {
    chassis.pid_wait_until(final_target * ez::degree);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " passes=", g_pass);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
}
