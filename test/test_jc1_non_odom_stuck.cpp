// DRIVE/TURN/SWING pid_wait() and wait_until_drive()/wait_until_turn_swing_internal() had no progress backstop
// at all (JC-1 in WAIT_BEHAVIOR_SPEC.md, confirmed by Step 3 finding #1/#15): only velocity and mA exits, both
// defeated by a sustained disturbance that never reads as fully "stopped" (so velocity never fires) and never
// draws over current (so mA never fires) -- a continuous spin, a defender holding the robot, or ordinary sensor
// jitter under contact.  These tests script that shape directly on each PID's own error/derivative, the same way
// test_pp_wait_stuck.cpp scripts xyPID/current_a_odomPID, and check the new SingleStuckWatch backstop catches it
// without disturbing a healthy, steadily-closing motion.
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
void (*g_script)(Drive&, int) = nullptr;

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_script(*g_chassis, g_pass);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run(Drive& chassis, void (*script)(Drive&, int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_pass = 0;
  g_script = script;
  script(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}

// Closes steadily to 0 -- a normal, healthy settle.  Small enough steps that it does NOT clear a full step
// (1 in default small_error) every single pass, so a real regression test for "does this false-flag a slow but
// genuinely progressing motion" too, not just a trivially-fast one.
void healthy_close(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  c.leftPID.error = e;
  c.leftPID.derivative = e > 0.0 ? -0.15 : 0.0;
  c.rightPID.error = e;
  c.rightPID.derivative = e > 0.0 ? -0.15 : 0.0;
}

// Pinned from the start: error frozen well outside small/big windows, derivative jitters just above
// velocity_zero_main (0.05) every other pass so the velocity exit's own accumulator can never build up --
// exactly the noisy-contact shape Step 3 finding #15 describes, not a literal silent stall.
void pinned_jitter(Drive& c, int n) {
  c.leftPID.error = 24.0;
  c.leftPID.derivative = (n % 2 == 0) ? 0.2 : -0.2;
  c.rightPID.error = 24.0;
  c.rightPID.derivative = (n % 2 == 0) ? 0.2 : -0.2;
}

void healthy_turn(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.error = e;
  c.turnPID.derivative = e > 0.0 ? -0.5 : 0.0;
}

void pinned_turn_jitter(Drive& c, int n) {
  c.turnPID.error = 60.0;
  c.turnPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void healthy_swing(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.error = e;
  c.swingPID.derivative = e > 0.0 ? -0.4 : 0.0;
}

void pinned_swing_jitter(Drive& c, int n) {
  c.swingPID.error = 45.0;
  c.swingPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a healthy, steadily-closing motion is not falsely flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, healthy_close, 500, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() DRIVE: a sustained disturbance that jitters past velocity's threshold is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  // 3000 passes = 30 s.  Before this fix, this scenario hung the full cap (confirmed in Step 3 finding #15's
  // own sim repro); the fix should end it in well under a second past the odom StuckWatch-equivalent window.
  Outcome o = run(chassis, pinned_jitter, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("wait_until_drive(): the same sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, pinned_jitter, 3000, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("pid_wait() TURN: a healthy turn is not falsely flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, healthy_turn, 500, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() TURN: a sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, pinned_turn_jitter, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("wait_until_turn_swing_internal() TURN: a sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_set(90, 100);
  Outcome o = run(chassis, pinned_turn_jitter, 3000, [&] { chassis.pid_wait_until(45_deg); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("pid_wait() SWING: a healthy swing is not falsely flagged stuck") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  Outcome o = run(chassis, healthy_swing, 500, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() SWING: a sustained disturbance is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);
  Outcome o = run(chassis, pinned_swing_jitter, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}
