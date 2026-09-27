// pid_wait()'s DRIVE/TURN/SWING branches already treat "the no-progress watch fired, but every
// side that's still RUNNING is already sitting inside its own big-error window" as settled, not
// stuck -- correct for a wait_until() point the robot is meant to drive THROUGH (stopping short
// there really should report interfered=true), but that exemption was deliberately scoped away
// from wait_until_drive()/wait_until_turn_swing_internal(). That's wrong for the common case of
// e.g. pid_drive_set(48) followed by pid_wait_until(48), where 48 IS the motion's actual final
// target -- a normal big-error settle an inch short was being misreported as interfered there.
//
// Fix: the same exemption now also applies in wait_until_drive()/wait_until_turn_swing_internal(),
// but ONLY when the wait_until() target numerically equals the motion's final target (within
// FINAL_TARGET_TOLERANCE, exit_conditions.cpp). A genuine intermediate waypoint short of the
// final target keeps today's behavior (interfered=true on a stuck settle).
//
// All these tests script a "hovering across small_error, settled inside big_error" shape (same
// as test_single_stuck_watch_settled.cpp's pid_wait() coverage) so the only thing that can end the
// wait is the no-progress watch -- proving the settled exemption itself fired, not an ordinary
// small/big exit.
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
void (*g_script)(Drive&, int) = nullptr;
int g_pass = 0;

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

// Drive::pid_wait_until(double) dispatches by mode -- wait_until_drive() for DRIVE, or
// wait_until_turn_swing() for TURN/SWING/TURN_TO_POINT -- so the plain double overload exercises
// all three under test here.
Outcome run_wait_until(Drive& chassis, void (*script)(Drive&, int), int max_passes, double wait_target) {
  g_chassis = &chassis;
  g_script = script;
  g_pass = 0;
  script(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait_until(wait_target);
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}

// Hovers across small_error(1.0) every pass, settled well inside big_error(3.0) -- only the
// no-progress watch can end this wait.
void hovering_settled_drive(Drive& c, int n) {
  double e = (n % 2 == 0) ? 0.9 : 1.1;
  c.leftPID.error = e;
  c.leftPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
  c.rightPID.error = e;
  c.rightPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void hovering_settled_turn(Drive& c, int n) {
  double e = (n % 2 == 0) ? 2.7 : 3.3;  // straddles turn's default small_error(3), inside big_error(7)
  c.turnPID.error = e;
  c.turnPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void hovering_settled_swing(Drive& c, int n) {
  double e = (n % 2 == 0) ? 2.7 : 3.3;
  c.swingPID.error = e;
  c.swingPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

// A genuine stall, far outside big_error -- must never be read as settled regardless of target.
void pinned_far_drive(Drive& c, int n) {
  c.leftPID.error = 24.0;
  c.leftPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
  c.rightPID.error = 24.0;
  c.rightPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void pinned_far_turn(Drive& c, int n) {
  c.turnPID.error = 60.0;
  c.turnPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}

void pinned_far_swing(Drive& c, int n) {
  c.swingPID.error = 40.0;
  c.swingPID.derivative = (n % 2 == 0) ? 0.3 : -0.3;
}
}  // namespace

// --- DRIVE ---

TEST_CASE("pid_wait_until() DRIVE: settling inside big error AT the final target returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(48, 100);

  Outcome o = run_wait_until(chassis, hovering_settled_drive, 400, 48.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait_until() DRIVE: settling short of a genuine intermediate waypoint still returns interfered=true") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(48, 100);  // final target is 48

  // Stopping at ~46 while waiting for the 24in waypoint -- error to 24 is ~22, far outside
  // big_error(3), so even without any gate this should read interfered. This is the "not just an
  // ungated freebie" control the gate itself doesn't change.
  Outcome o = run_wait_until(chassis, pinned_far_drive, 400, 24.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK(o.interfered);
}

// The actual discriminating test: a waypoint CLOSE to the final target (within big_error of it,
// so an ungated fix would also read this as settled) but not equal to it. Ungated: interfered
// would read false here too, incorrectly. Gated: must still read interfered=true, since 47 != 48.
// Reuses hovering_settled_drive's exact shape (hovering across small_error, well inside
// big_error) -- the same scripted leftPID/rightPID.error the "AT the final target" test above
// uses, just waited on with a different wait_until() target. That isolates the one thing this
// test is actually about (does the gate correctly tell 47 apart from 48) from any difference in
// how the settle itself is scripted.
TEST_CASE("pid_wait_until() DRIVE: a waypoint close to, but not equal to, the final target still returns interfered=true") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(48, 100);  // final target is 48, big_error is 3

  Outcome o = run_wait_until(chassis, hovering_settled_drive, 400, 47.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK(o.interfered);  // 47 != 48 -- not the final target, gate must not exempt this
}

// Float tolerance doesn't make the gate flaky in the OTHER direction either: a target that's
// equal to the final target except for a ~1 ULP perturbation (the kind of difference ordinary
// double round-trip arithmetic -- e.g. l_start + target - l_start, or a QLength/QAngle unit
// conversion and back -- can introduce) must still count as equal, not fall just outside the
// tolerance. Built with std::nextafter rather than relying on some specific l_start value to
// happen to introduce rounding error incidentally (tried that: at some l_start values the
// round-trip lands back on 48.0 bit-for-bit and proves nothing) -- this is deterministic.
TEST_CASE("pid_wait_until() DRIVE: a target equal to the final target within float rounding still returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(48, 100);

  double perturbed_target = std::nextafter(std::nextafter(48.0, 100.0), 100.0);  // 48.0 + ~2 ULP
  REQUIRE(perturbed_target != 48.0);  // otherwise this test proves nothing about tolerance
  REQUIRE(std::fabs(perturbed_target - 48.0) < 1e-9);  // still comfortably inside FINAL_TARGET_TOLERANCE

  Outcome o = run_wait_until(chassis, hovering_settled_drive, 400, perturbed_target);
  MESSAGE("perturbed_target=" << perturbed_target << " returned=" << o.returned << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

// --- TURN ---

TEST_CASE("pid_wait_until() TURN: settling inside big error AT the final target returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(200, 100);  // a large turn, not near a wrap boundary

  Outcome o = run_wait_until(chassis, hovering_settled_turn, 400, 200.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait_until() TURN: settling short of a genuine intermediate waypoint still returns interfered=true") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_turn_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_turn_set(200, 100);

  Outcome o = run_wait_until(chassis, pinned_far_turn, 400, 100.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK(o.interfered);
}

// --- SWING ---

TEST_CASE("pid_wait_until() SWING: settling inside big error AT the final target returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);

  Outcome o = run_wait_until(chassis, hovering_settled_swing, 400, 45.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait_until() SWING: settling short of a genuine intermediate waypoint still returns interfered=true") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(2000, 3.0, 250, 7.0, 500, 500);
  chassis.pid_swing_set(ez::LEFT_SWING, 45, 100);  // final target is 45

  Outcome o = run_wait_until(chassis, pinned_far_swing, 400, 20.0);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK(o.interfered);
}

// --- pid_wait_quick() picks up the same exemption for a plain (non-chained) DRIVE wait, since
// its dispatch waits on the motion's own real final target (chain_target_start ==
// pid_drive_set()'s own target when nothing has chained onto it yet). pid_wait_quick_chain(),
// by contrast, first ADDS the chain scale to leftPID/rightPID's target before waiting -- so
// at_final_target reads false there and a chained wait gets no exemption, which is correct: a
// chained motion is explicitly meant to drive through that point, not stop there. ---

TEST_CASE("pid_wait_quick() DRIVE: settling inside big error at the final target returns interfered=false") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(2000, 1.0, 250, 3.0, 500, 500);
  chassis.pid_drive_set(48, 100);

  g_chassis = &chassis;
  g_script = hovering_settled_drive;
  g_pass = 0;
  hovering_settled_drive(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 400;
  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);
  CHECK(returned);
  CHECK_FALSE(chassis.interfered);
}
