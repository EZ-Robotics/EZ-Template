// Issue #533: a swing's held (non-swinging) side was never checked for over-current. pid_wait()'s
// SWING branch and wait_until_turn_swing_internal()'s SWING path both selected only the
// actively-swinging side's motors for the mA channel (`current_swing == LEFT_SWING ? left_motors :
// right_motors`), but swing_pid_task() (pid_tasks.cpp) actively drives the held side with its own
// PID output whenever swing_opposite_speed is 0 (the default for the plain pid_swing_set(type,
// target, speed) overload) -- so the held side can genuinely stall/over-current (e.g. a defender
// pinning it while the swinging side is unobstructed), but that motor was never polled. The fix
// polls both_sides(left_motors, right_motors) for the mA channel, the same as TURN already does
// (see test_turn_ma_checks_both_sides.cpp).
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}
}  // namespace

TEST_CASE("pid_wait() SWING mA exit catches a stall on the held side, not just the swinging side") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // mA exit only (100 ms/10 passes); small/big/velocity off, so a genuine over-current on any
  // polled motor is the only thing that can end this swing quickly. The SingleStuckWatch progress
  // backstop is still live (window_ falls back to mA_timeout when velocity_exit_time is 0) but
  // only fires after its own much longer 1000 ms start allowance plus window -- comfortably later
  // than 100 ms, so a fast return here can only be the mA exit.
  chassis.pid_swing_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  // Plain 3-arg overload -> opposite_speed defaults to 0, the held-side-holds-position case the
  // issue describes.
  chassis.pid_swing_set(ez::LEFT_SWING, 90.0, 100);

  // Held (right) side only -- the actively-swinging (left) side never reads as over current.
  chassis.right_motors[0].fake().over_current = true;

  test_stub::g_clock.delay_calls_until_stop = 15;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " interfered=", chassis.interfered);

  // Correct code: mA_timeout(100ms) is crossed at pass 11, well inside the 15-pass budget, via
  // the held-side motor alone.
  REQUIRE(returned);
  CHECK(chassis.interfered);
}

TEST_CASE("wait_until_turn_swing_internal() SWING mA exit catches a stall on the held side, not just the swinging side") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_swing_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_swing_set(ez::LEFT_SWING, 90.0, 100);

  // Held (right) side only.
  chassis.right_motors[0].fake().over_current = true;

  test_stub::g_clock.delay_calls_until_stop = 15;
  bool returned = true;
  try {
    chassis.pid_wait_until(45_deg);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  MESSAGE("returned=", returned, " interfered=", chassis.interfered);

  REQUIRE(returned);
  CHECK(chassis.interfered);
}

namespace {
Drive* g_chassis = nullptr;
int g_pass = 0;

// A normal, healthy swing convergence: the swinging side's own error closes steadily to 0 across
// the whole motion (45 degrees at 0.4 degrees/pass, so roughly 112 passes/1.1s to reach the small
// error window), the same shape a real, unobstructed swing produces. Not a stall on the swinging
// side at all -- the point of this test is that the OTHER side (held, not swinging) is
// continuously over-current the entire time, which this convergence has nothing to do with.
void healthy_swing_convergence(Drive& c, int n) {
  double e = std::fmax(0.0, 45.0 - 0.4 * n);
  c.swingPID.compute_error(e, e);
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  healthy_swing_convergence(*g_chassis, g_pass);
}
}  // namespace

// The issue's own repro, using the library's default swing exit constants (mA_timeout 500ms/50
// passes) rather than a stripped-down configuration, so the "mA_timeout well inside the
// convergence window" relationship the issue describes is real, not engineered: a held side that
// is continuously over-current from the very first pass of an otherwise perfectly healthy,
// converging swing must still trip mA_EXIT/interfered=true within roughly mA_timeout, long before
// the swinging side would have converged on its own around pass 112 -- not be masked by the
// swinging side's own clean convergence.
TEST_CASE("pid_wait() SWING: a held side continuously over-current during a healthy convergence still trips mA_EXIT (issue #533)") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Default swing exit constants (small/big/velocity/mA_timeout all as the Drive constructor
  // leaves them) -- deliberately not overridden, so this exercises the exact behavior a team
  // would see without any special exit-condition tuning of their own.
  chassis.pid_swing_set(ez::LEFT_SWING, 45.0, 100);

  // Held (right) side flagged over-current from pass 1 onward, exactly as the issue describes --
  // the swinging (left) side is never flagged, and its own error is healthily converging via
  // healthy_swing_convergence() above.
  chassis.right_motors[0].fake().over_current = true;

  g_chassis = &chassis;
  g_pass = 0;
  test_stub::g_clock.on_delay = on_delay;
  // Generous cap: comfortably past both the fixed behavior's expected mA trip (~pass 51) and the
  // swinging side's own unrelated convergence (~pass 112), so either outcome returns well before
  // the cap is ever reached -- a StopLoop here would mean neither happened and the test setup
  // itself is wrong, not evidence either way about the fix.
  test_stub::g_clock.delay_calls_until_stop = 200;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " passes=", g_pass, " interfered=", chassis.interfered);

  // Before the fix: the held side's over-current was never polled at all, so this swing runs its
  // full, healthy convergence (~pass 112) and completes as a clean, uninterfered success despite
  // the continuous over-current on the held side for its entire motion -- returned=true,
  // interfered=false, exactly the issue's own reported symptom. After the fix: the held side's
  // mA timer trips around pass 51, long before that convergence, ending the swing early with
  // interfered=true.
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass < 90);
}
