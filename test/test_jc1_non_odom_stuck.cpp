// DRIVE/TURN/SWING pid_wait() and wait_until_drive()/wait_until_turn_swing() end on the velocity and mA exits.  A sustained disturbance that
// never draws over current (so mA never fires) and keeps the robot's speed above the stop speed -- a continuous spin, a defender holding the
// robot, or ordinary sensor jitter under contact -- still has to end: the speed is the net movement over the last 100 ms, so jitter that
// cancels out reads as stopped and the velocity exit ends the wait.  These tests script that shape directly on each PID's own
// compute_error(), the same way test_pp_wait_stuck.cpp scripts xyPID/current_a_odomPID, and check the wait ends without disturbing a
// healthy, steadily-closing motion.
#include <functional>

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

// Every script below drives its PID(s) through compute_error(), not a direct `.error =`/`.derivative =` write: PID::exit_condition()
// reads the speed from the `cur` that compute_error() is given, and a script that never moves `cur` reads as a robot standing still.
// This file's on_delay() runs once per simulated DELAY_TIME tick (bumping auto_task_passes), standing in for a real ez_auto_task pass.

// Closes steadily to 0 at 15 in/s, above the 5 in/s the velocity exit counts as stopped -- a normal, healthy settle.
void healthy_close(Drive& c, int n) {
  double e = std::fmax(0.0, 20.0 - 0.15 * n);
  // Feeding `current` the same closing value as `error` keeps the two moving together (both -0.15/pass while e > 0, then both flat),
  // without needing to track separate running state.
  c.leftPID.compute_error(e, e);
  c.rightPID.compute_error(e, e);
}

// Pinned from the start: error frozen well outside small/big windows, `cur` jitters by +-0.05 every other pass -- the noisy-contact shape,
// not a literal silent stall.  The jitter is small enough that its net movement over the 30 ms speed window stays under the 5 units per second velocity exit
// speed, so the robot reads as stopped.
void pinned_jitter(Drive& c, int n) {
  double cur = (n % 2 == 0) ? 0.05 : -0.05;  // alternating +-0.05
  c.leftPID.compute_error(24.0, cur);
  c.rightPID.compute_error(24.0, cur);
}

void healthy_turn(Drive& c, int n) {
  double e = std::fmax(0.0, 60.0 - 0.5 * n);
  c.turnPID.compute_error(e, e);
}

void pinned_turn_jitter(Drive& c, int n) {
  double cur = (n % 2 == 0) ? 0.05 : -0.05;  // same reasoning as pinned_jitter above
  c.turnPID.compute_error(60.0, cur);
}

void healthy_swing(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.compute_error(e, e);
}

void pinned_swing_jitter(Drive& c, int n) {
  double cur = (n % 2 == 0) ? 0.05 : -0.05;
  c.swingPID.compute_error(45.0, cur);
}

// A raw sensor reading that's bit-identical to itself every single poll, because the robot is genuinely, fully stalled.  `current` held
// fixed at 0 every tick is a real compute() call each tick (a genuinely stalled sensor is still polled every pass -- it just never
// changes), and the velocity exit ends the wait once its 1000 ms arming fallback has passed and velocity_exit_time has run out.
void pinned_frozen(Drive& c, int n) {
  c.leftPID.compute_error(24.0, 0.0);
  c.rightPID.compute_error(24.0, 0.0);
}

// The realistic shape of the frozen-sensor case above: a robot that drives normally for a while (real, fresh, closing readings), then hits a
// wall and pins there, at which point its sensor starts reading bit-identical every poll.  The robot moved, so the velocity exit is already
// armed and only has to wait out the speed window and velocity_exit_time.
void drives_then_pins(Drive& c, int n) {
  bool pinned = n > 20;
  double e = pinned ? 14.0 : std::fmax(14.0, 24.0 - 0.5 * n);
  // cur(n) = -0.5*min(n, 20): a running sum in closed form, so each pass' delta (this call's cur minus the previous call's) is the rate
  // (-0.5 while n<=20, 0 once pinned) without needing separate persistent state.
  double cur = -0.5 * std::fmin((double)n, 20.0);
  c.leftPID.compute_error(e, cur);
  c.rightPID.compute_error(e, cur);
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

TEST_CASE("pid_wait() DRIVE: a sustained disturbance of jittering sensor readings is caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  // 3000 passes = 30 s.  The velocity exit ends it once its 1000 ms arming fallback and the 500 ms velocity_exit_time have passed.
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

TEST_CASE("pid_wait() DRIVE: a raw sensor that never changes at all is still caught, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, pinned_frozen, 3000, [&] { chassis.pid_wait(); });
  MESSAGE("passes=", o.passes);
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 200);
}

TEST_CASE("pid_wait() DRIVE: drives normally, then pins at a wall with a frozen sensor -- still caught promptly") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, drives_then_pins, 3000, [&] { chassis.pid_wait(); });
  MESSAGE("passes=", o.passes);
  CHECK(o.returned);
  CHECK(o.interfered);
  // Pins at pass 20.  The velocity exit is already armed (the robot moved), but its speed is the net movement over the last 100 ms, so it
  // keeps reading the final strokes of the drive for up to 10 passes after the pin, then velocity_exit_time (500 ms, 50 passes) has to run
  // out, plus slack.
  CHECK(o.passes <= 20 + 10 + 50 + 10);
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

TEST_CASE("wait_until_turn_swing() TURN: a sustained disturbance is caught, not hung forever") {
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
