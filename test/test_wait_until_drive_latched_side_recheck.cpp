// wait_until_drive() has the same two-sided latch pid_wait()'s DRIVE branch has (see
// test_pid_wait_drive_latched_side_recheck.cpp), reached through its own `else` branch once both
// left_exit/right_exit are non-RUNNING. That branch used to return a clean result with no drift
// recheck at all, and it has nothing to do with the crossing check (drive_sensor_left()/_right() vs
// `target`): a robot that settles within small_error short of its wait_until() target never crosses
// it, so an ordinary final-target settle -- pid_wait_until(<the final target>), or pid_wait_quick(),
// which dispatches to exactly that call -- used to end through this same unguarded `else`.
//
// wait_until_drive()'s else branch now gets the identical recheck-before-trusting-a-clean-exit
// treatment pid_wait()'s DRIVE branch does.
//
// This protects the library's own documented "interfered==false means actually on target" contract
// (src/autons.cpp's interfered_example() chains the next motion only when interfered==false).
#include <cmath>
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
  double left_error_at_return;
};

Outcome run_wait(Drive& chassis, void (*script)(Drive&, int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_script = script;
  g_pass = 0;
  script(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  o.left_error_at_return = chassis.leftPID.error;
  return o;
}

// Same pass timeline as test_pid_wait_drive_latched_side_recheck.cpp's script(): left latches
// SMALL_EXIT early (pass 10) then gets shoved to 5.5in (outside the default 3in big_error) and held
// there; right keeps closing honestly and small-exits for real at pass 40.
//
// A DriveTestAccess::refresh() call every pass, not a bare `.error =` write -- PID.cpp's small/big
// exit timers only credit `error` when a real compute has landed since they last checked (see
// exit_condition()'s error_fresh comment), so without this left/right would sit at RUNNING forever
// and this test would only ever exercise wait_until_drive()'s StuckWatch-backed stuck-detected path,
// never the two-sided latch its own docstring is about.
void script(Drive& c, int n) {
  double left_error = (n <= 10) ? 0.5 : 5.5;
  c.leftPID.error = left_error;
  c.leftPID.derivative = 0.1;
  DriveTestAccess::refresh(c.leftPID);

  double right_error = std::fmax(0.4, 3.9 - 0.1 * n);
  c.rightPID.error = right_error;
  c.rightPID.derivative = -0.1;
  DriveTestAccess::refresh(c.rightPID);
}
}  // namespace

// drive_sensor_left()/_right() (the stub's fake encoders) never move in this harness -- only
// leftPID.error/rightPID.error are scripted directly, as test_wait_until_settled_at_final_target.cpp
// already does for the same reason -- so wait_until_drive()'s own "has the robot's real position
// crossed the target" check never fires here, isolating that the result comes from the exit-condition
// latch, not the crossing check. Waiting on 48 -- the motion's own final target -- so at_final_target
// is true and this is the common real case (a team's own pid_wait_until() call matching the
// pid_drive_set() distance it followed), not an edge case.
TEST_CASE("pid_wait_until() DRIVE at final target: a latched side is rechecked, so a later shove off-target is reported as interfered") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);

  Outcome o = run_wait(chassis, script, 500, [&] { chassis.pid_wait_until(48.0); });
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered
                       << " left_error_at_return=" << o.left_error_at_return);

  REQUIRE(o.returned);
  REQUIRE(o.left_error_at_return > chassis.leftPID.exit.big_error);
  CHECK(o.interfered);
}

// pid_wait_quick() on a plain (non-chained) DRIVE motion dispatches to pid_wait_until(chain_target_start),
// which for an un-chained motion is the same final target pid_drive_set() itself used -- this is the
// most common quick-wait usage in a real auton (TEAM_CORPUS.md's chain corpus uses
// pid_wait_quick_chain() specifically when it wants to hold momentum; a plain pid_wait_quick() call
// behaves like an ordinary final-target wait). Same latch, reached the same way.
TEST_CASE("pid_wait_quick() DRIVE: a latched side is rechecked, so a later shove off-target is reported as interfered") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);

  Outcome o = run_wait(chassis, script, 500, [&] { chassis.pid_wait_quick(); });
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered
                       << " left_error_at_return=" << o.left_error_at_return);

  REQUIRE(o.returned);
  REQUIRE(o.left_error_at_return > chassis.leftPID.exit.big_error);
  CHECK(o.interfered);
}

namespace {
Drive* g_chassis2 = nullptr;
int g_pass2 = 0;

// Left: closes fast (20in -> 0in by pass 10), sits at 0 through pass 19 -- long enough for its own
// small_exit_time (90ms/9 passes) to latch SMALL_EXIT around pass 19 -- then gets pinned by a
// defender from pass 20 onward: error ramps up to 4in over 20 passes and holds there. Right closes
// slowly enough (20in -> 0in by ~pass 57) that it is still genuinely RUNNING, and genuinely
// converging, the entire time left is pinned.
// A DriveTestAccess::refresh() call every pass -- see script()'s own comment above for why a bare
// `.error =` write can't reach a real SMALL_EXIT/BIG_EXIT latch under PID.cpp's freshness gate.
void pinned_script() {
  ++g_pass2;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis2;
  int n = g_pass2;

  double left_e;
  if (n <= 10) left_e = std::fmax(0.0, 20.0 - 2.0 * n);
  else if (n <= 19) left_e = 0.0;
  else if (n <= 39) left_e = 0.2 * (n - 19);
  else left_e = 4.0;
  c.leftPID.error = left_e;
  c.leftPID.derivative = n <= 10 ? -2.0 : (n > 19 && n <= 39 ? 0.2 : 0.0);
  DriveTestAccess::refresh(c.leftPID);

  double right_e = std::fmax(0.0, 20.0 - 0.35 * n);
  c.rightPID.error = right_e;
  c.rightPID.derivative = right_e > 0.0 ? -0.35 : 0.0;
  DriveTestAccess::refresh(c.rightPID);
}
}  // namespace

// pid_wait_quick() in DRIVE mode dispatches straight into wait_until_drive() (via
// pid_wait_until(chain_target_start)) -- the path heavy real-world chainers actually call, per
// TEAM_CORPUS.md (some autons call pid_wait_quick_chain()/pid_wait_quick() 90-120 times in one
// file). Real drive_sensor_left()/right() are never touched by this script (stay at 0 the whole
// test), so wait_until_drive()'s own crossed check (driven by real sensor position, not
// leftPID.error) never fires -- the only way out is its "both exits non-RUNNING" else branch, which
// has the identical latch shape as pid_wait()'s DRIVE branch.
TEST_CASE("pid_wait_quick() DRIVE: wait_until_drive()'s own else branch also rechecks a latched side before returning clean") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  REQUIRE(chassis.mode == DRIVE);

  g_chassis2 = &chassis;
  g_pass2 = 0;
  pinned_script();
  test_stub::g_clock.on_delay = pinned_script;
  test_stub::g_clock.delay_calls_until_stop = 300;
  bool returned = true;
  try {
    chassis.pid_wait_quick();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  CHECK((chassis.interfered || std::fabs(chassis.leftPID.error) < chassis.leftPID.exit.big_error));
  CHECK(std::fabs(chassis.leftPID.error) > chassis.leftPID.exit.big_error);
}
