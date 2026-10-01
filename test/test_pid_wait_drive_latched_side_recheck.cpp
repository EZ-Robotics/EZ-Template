// pid_wait()'s DRIVE branch used to latch left_exit/right_exit the first time each one went
// non-RUNNING:
//
//   left_exit  = left_exit  != RUNNING ? left_exit  : without_velocity(leftPID.exit_condition(left_motors));
//   right_exit = right_exit != RUNNING ? right_exit : without_velocity(rightPID.exit_condition(right_motors));
//
// Once a side latched, its ternary short-circuited: exit_condition() was never called on it again
// for the rest of the wait, so a disturbance landing AFTER that side had already latched was never
// looked at again. DRIVE's two sides routinely finish at different times (a scrubbing wheel, an
// uneven load, a slightly different gear mesh), so the realistic shape is: one side converges and
// latches SMALL_EXIT early, something shoves or pins that side off target (a defender, a collision,
// wheel slip) while the OTHER side is still closing normally, and the wait used to fall out of its
// loop reporting a clean, uninterfered finish -- even though the shoved side was sitting well
// outside its own big_error window.
//
// This was the exact mechanism pid_wait()'s odom (PURE_PURSUIT/POINT_TO_POINT) branch was already
// fixed for -- see test_odom_latched_axis_drift_after_exit.cpp and exit_conditions.cpp's own relatch
// comment there ("Recheck each axis that latched a window exit ... If it has drifted back outside,
// un-latch it"). DRIVE's own branch now gets the identical treatment: right before trusting a clean
// double-exit, each side that latched a window exit is rechecked against the live error, and
// un-latched (falling back to genuinely being watched) if it has drifted back outside the window it
// exited through.
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

// Pass timeline (shipped defaults: small_exit_time 90ms/9 passes, small_error 1in, big_exit_time
// 250ms/25 passes, big_error 3in):
//  1-10:  left sits at error 0.5in (inside small_error) -- latches SMALL_EXIT at pass 10. Right
//         starts at 3.9in, closing a realistic 0.1in/pass -- not physically implausible for the far
//         side of one drive motion (a scrub/load difference, not a 20in split). Right enters the
//         [1in,3in) big-error band around pass 11 and stays there ~20 passes (never falsely
//         BIG_EXITs), then drops under 1in around pass 31 and stays there (floored at 0.4in),
//         latching its own SMALL_EXIT at pass 40.
//  11+:   left is shoved to 5.5in -- well outside its own 3in big_error -- and held there for the
//         rest of the wait. Right keeps closing steadily throughout (never stuck: SingleStuckWatch's
//         window is 500ms/50 passes, and right clears a full 1in step every ~10 passes) until it
//         small-exits for real at pass 40.
// A DriveTestAccess::refresh() call every pass, not a bare `.error =` write -- PID.cpp's small/big
// exit timers only credit `error` when a real compute has landed since they last checked, so
// without this neither side could ever actually reach a latched SMALL_EXIT, and this test would
// only ever exercise pid_wait()'s DRIVE branch through its StuckWatch fallback, never through the
// two-sided latch this test is named for.
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

TEST_CASE("pid_wait() DRIVE: a side that already latched an exit is rechecked, so a later shove off-target is reported as interfered") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);  // shipped defaults: 90ms/1in/250ms/3in/500ms/500ms

  Outcome o = run_wait(chassis, script, 500, [&] { chassis.pid_wait(); });
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered << " left_error_at_return=" << o.left_error_at_return);

  REQUIRE(o.returned);  // the wait must actually end, not hang past max_passes
  // Sanity on the scripted shape itself: left really is outside its own big_error at return time.
  REQUIRE(o.left_error_at_return > chassis.leftPID.exit.big_error);
  // A side left well outside its own big_error window at return time must be reported interfered.
  CHECK(o.interfered);
}

// Control: with the SAME closing shape, if left's exit is never allowed to latch early (both sides
// exit at the same time, close together, and nothing shoves either one afterward), there's nothing
// to relatch -- this isolates that the case above is specifically about the EARLY-latch-then-drift
// shape, not a general problem with this exit_condition plumbing.
TEST_CASE("pid_wait() DRIVE control: both sides exiting close together at a genuinely settled target is a clean success") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);

  Outcome o = run_wait(
      chassis,
      [](Drive& c, int n) {
        double e = std::fmax(0.4, 3.9 - 0.1 * n);
        c.leftPID.error = e;
        c.leftPID.derivative = -0.1;
        c.rightPID.error = e;
        c.rightPID.derivative = -0.1;
      },
      500, [&] { chassis.pid_wait(); });

  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

namespace {
Drive* g_chassis2 = nullptr;
int g_pass2 = 0;
bool g_over_current = false;

// Left: closes fast (20in -> 0in by pass 10), sits at 0 through pass 19 -- long enough for its own
// small_exit_time (90ms/9 passes) to latch SMALL_EXIT around pass 19 -- then gets pinned by a
// defender from pass 20 onward: error ramps up to 4in over 20 passes (a ~20in/s shove, not an
// instantaneous teleport) and holds there, never recovering -- the "held, not just tapped" shape of
// sustained defense contact, outside the default 3in big_error.
//
// Right: closes slowly enough (20in -> 0in by ~pass 57) that it is still genuinely RUNNING, and
// genuinely converging, the entire time left is pinned -- left's disturbance is never what ends this
// wait; right's own honest exit is.
// A DriveTestAccess::refresh() call every pass -- see script()'s own comment above for why a bare
// `.error =` write can't reach a real SMALL_EXIT latch under PID.cpp's freshness gate.
void pinned_script() {
  ++g_pass2;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis2;
  int n = g_pass2;

  double left_e;
  if (n <= 10)
    left_e = std::fmax(0.0, 20.0 - 2.0 * n);
  else if (n <= 19)
    left_e = 0.0;
  else if (n <= 39)
    left_e = 0.2 * (n - 19);  // ramps 0 -> 4in over 20 passes
  else
    left_e = 4.0;
  c.leftPID.error = left_e;
  c.leftPID.derivative = n <= 10 ? -2.0 : (n > 19 && n <= 39 ? 0.2 : 0.0);
  DriveTestAccess::refresh(c.leftPID);

  double right_e = std::fmax(0.0, 20.0 - 0.35 * n);
  c.rightPID.error = right_e;
  c.rightPID.derivative = right_e > 0.0 ? -0.35 : 0.0;
  DriveTestAccess::refresh(c.rightPID);

  if (g_over_current && n >= 25) {
    for (auto& m : c.left_motors) m.fake().over_current = true;
  }
}

struct PinnedOutcome {
  bool returned;
  bool interfered;
  double final_left_error;
};

PinnedOutcome run_pinned(Drive& chassis, int max_passes) {
  g_chassis2 = &chassis;
  g_pass2 = 0;
  pinned_script();
  test_stub::g_clock.on_delay = pinned_script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  PinnedOutcome o{true, false, 0.0};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.interfered = chassis.interfered;
  o.final_left_error = chassis.leftPID.error;
  return o;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a side pinned after its own exit already latched is rechecked and reported as interfered") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  REQUIRE(chassis.mode == DRIVE);

  g_over_current = false;
  // Generous budget: right honestly finishes around pass ~65-70 at these rates.
  PinnedOutcome o = run_pinned(chassis, 300);

  REQUIRE(o.returned);
  // Left never recovers from the pass-20 pin in this script, so "genuinely on target" is impossible
  // on that side -- the only acceptable outcome is interfered == true (or left having somehow come
  // back inside big_error, which this script never lets happen).
  CHECK((o.interfered || std::fabs(o.final_left_error) < chassis.leftPID.exit.big_error));
  CHECK(std::fabs(o.final_left_error) > chassis.leftPID.exit.big_error);
}

TEST_CASE("pid_wait() DRIVE: over current on a side after its own exit already latched is still detected") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  REQUIRE(chassis.mode == DRIVE);

  g_over_current = true;
  PinnedOutcome o = run_pinned(chassis, 300);

  REQUIRE(o.returned);
  // Same pin, but this time the pinned side is also drawing over current the whole time (a motor
  // stalled against a defender, not just reading a stale position).
  CHECK((o.interfered || std::fabs(o.final_left_error) < chassis.leftPID.exit.big_error));
  CHECK(std::fabs(o.final_left_error) > chassis.leftPID.exit.big_error);
}

// NOTE on pid_wait_quick_chain(), deliberately NOT covered by a test here: Drive's constructor sets
// drive_forward_motion_chain_scale to a nonzero 3in by default (drive.cpp, pid_drive_chain_constant_set(3_in)),
// not 0 -- so pid_wait_quick_chain() genuinely bumps leftPID/rightPID's target past the final target
// before waiting. On real hardware that means the robot's actual sensor position crosses the
// ORIGINAL final target while still closing on the bumped one, so wait_until_drive()'s crossing
// check, which reads live drive_sensor_left()/_right(), fires before either side could latch an
// exit near the bumped target. This test harness scripts leftPID.error directly and never moves the
// stub encoders, which decouples error from sensor position in a way real hardware can't -- a
// scripted repro here would only "reproduce" by that decoupling, making it a harness artifact rather
// than a real repro of the chained case.
