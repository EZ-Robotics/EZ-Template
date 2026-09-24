// SingleStuckWatch (exit_conditions.cpp), the DRIVE/TURN/SWING progress backstop, has no equivalent
// to odom's StuckWatch "settled" carve-out: when the no-progress watch would otherwise call a wait
// stuck, but every side that's still RUNNING is already sitting inside its own big-error window
// (where a big exit would have left it), that's settled, not stuck. Odom's pid_wait() branch already
// makes this call (exit_conditions.cpp, the "Stopped inside both big error windows" comment); DRIVE/
// TURN/SWING did not.
//
// This only matters at a non-default configuration: small_exit_time set longer than the watch's own
// window. A robot whose error hovers back and forth across small_error, while staying comfortably
// inside big_error the whole time, resets both exit-condition timers every time it crosses the small
// window's edge -- small_exit_time never accumulates (the timer restarts every out-of-small pass) and
// big_exit_time never accumulates either (the exit-condition code zeroes the big timer's own progress
// on every in-small pass, "while [small] is running, don't run big thresh"). Neither ordinary exit can
// ever fire on its own. Before this fix, the no-progress watch (which also sees no progress, since the
// error never reaches a new low a full step below the last one) would eventually call that stuck --
// even though the robot has been sitting right where a big exit would have ended the motion cleanly
// the whole time.
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Error hovers back and forth across small_error's edge (0.9 / 1.1 against a default small_error of
// 1.0) every single pass -- comfortably inside big_error(3.0) throughout -- so small_exit_time and
// big_exit_time both keep getting reset before either can complete. Derivative alternates well above
// velocity_zero_main(0.05) so the velocity exit can't fire either; nothing but the no-progress watch
// can end this wait.
void hovering_settled(Drive& c, int n) {
  double e = (n % 2 == 0) ? 0.9 : 1.1;
  c.leftPID.error = e;
  c.leftPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
  c.rightPID.error = e;
  c.rightPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void hovering_settled_turn(Drive& c, int n) {
  double e = (n % 2 == 0) ? 2.7 : 3.3;  // straddles turn's default small_error of 3deg, inside big_error(7deg)
  c.turnPID.error = e;
  c.turnPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
}

// Same shape as pinned_jitter in test_jc1_non_odom_stuck.cpp: genuinely pinned far from target,
// outside big_error, while jittering just enough to avoid the velocity exit.
void pinned_outside_big_error(Drive& c, int n) {
  c.leftPID.error = 24.0;
  c.leftPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
  c.rightPID.error = 24.0;
  c.rightPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void on_delay_pinned() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  pinned_outside_big_error(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a robot hovering across its small-error window, but settled inside big-error, is not reported stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // small_exit_time (2000ms) set past the watch's own window (velocity_exit_time, 500ms by default) --
  // the non-default configuration this carve-out exists for.
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  hovering_settled(chassis, 0);
  test_stub::g_clock.on_delay = [] { on_delay(); hovering_settled(*g_chassis, g_pass); };
  test_stub::g_clock.delay_calls_until_stop = 400;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  // Real fire (before this fix credited it as settled) is ~150 passes; a generous cap that would
  // still catch an outright hang.
  CHECK(g_pass < 300);
}

TEST_CASE("pid_wait() TURN: a robot hovering across its small-error window, but settled inside big-error, is not reported stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(90, 100);

  g_chassis = &chassis;
  g_pass = 0;
  hovering_settled_turn(chassis, 0);
  test_stub::g_clock.on_delay = [] { on_delay(); hovering_settled_turn(*g_chassis, g_pass); };
  test_stub::g_clock.delay_calls_until_stop = 400;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_pass < 300);
}

// Regression guard: a robot genuinely pinned FAR from target (outside big_error, same shape as the
// existing pinned_jitter tests) must still be reported stuck -- the settled carve-out must not mask a
// real stall just because it happens to hover.
TEST_CASE("pid_wait() DRIVE: a robot pinned outside big-error is still reported stuck even while its error jitters") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  pinned_outside_big_error(chassis, 0);
  test_stub::g_clock.on_delay = on_delay_pinned;
  test_stub::g_clock.delay_calls_until_stop = 400;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);

  CHECK(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass < 300);
}
