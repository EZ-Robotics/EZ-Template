// GitHub issue #532: pid_wait()'s DRIVE branch gives each side (left/right) its own
// SingleStuckWatch, only fed while that side reads RUNNING. Once a side latches a clean
// SMALL_EXIT/BIG_EXIT, its watch stops being called entirely for as long as the OTHER side keeps
// running -- its internal progress clock freezes. The "latched side recheck" right before trusting
// a clean double-exit correctly flips a drifted-out latch back to RUNNING using live error, but
// used to leave that side's SingleStuckWatch un-reseeded: the very next stuck() call on it measured
// elapsed time against a clock that had been frozen since before it ever latched, which can already
// exceed the watch's own window purely from idle time spent waiting on the sibling -- with zero
// relation to whatever new disturbance is actually happening now.
//
// Effect: a side that finishes early (ordinary left/right asymmetry, nothing adversarial), then has
// a brief, ordinary, self-resolving blip right around when the sibling finally finishes, used to get
// an INSTANT "stuck" verdict -- zero grace period instead of a fresh window -- producing a false
// early interfered=true even though the same blip, given a few more passes, would have resolved on
// its own.
//
// The fix (accepting the tradeoff, see exit_conditions.cpp's own comment on the recheck): reseed the
// side's SingleStuckWatch right at the point the recheck un-latches it, the same way a motion's own
// first wait constructs one, so the newly-unlatched disturbance gets a genuinely fresh grace window.
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
bool g_resolve = true;
int g_pass = 0;

// Left: settles at 0.5in (inside the shipped 1in small_error) by pass ~9 and latches SMALL_EXIT
// almost immediately, then sits there completely idle -- genuinely on target, nothing wrong -- for
// roughly 190 more passes while right is still closing normally (an ordinary left/right asymmetry,
// not anything adversarial). Right around the pass where right itself finally settles, left takes a
// brief excursion PAST its own 3in big_error too -- timed to land on the pass where pid_wait()'s
// "both sides read as exited" recheck runs, so the recheck un-latches left back to RUNNING mid-blip.
// Going past big_error (not just small_error) matters here: a side caught "stuck" while still inside
// its own big_error reads as settled, not stalled (see the stuck-detection block's own comment on
// that carve-out, just above the recheck), so only a blip that clears big_error can expose a stale
// clock's false-instant-stuck verdict as an actual interfered=true, which is exactly the issue's own
// repro shape. g_resolve then selects what the blip does next:
//   true:  it resolves on its own within a handful of passes, well inside a fresh 500ms/50-pass
//          grace window -- the issue's own repro shape -- and should read as a clean,
//          uninterfered finish.
//   false: it is never allowed to resolve -- a genuinely sustained disturbance that outlasts even a
//          freshly reseeded grace window -- and should still be reported as interfered.
void script(Drive& c, int n) {
  double left_error;
  if (n <= 199) {
    left_error = 0.5;  // settled inside small_error(1in): latches early, then idles
  } else {
    left_error = g_resolve ? (n <= 205 ? 4.0 : 0.3) : 6.0;
  }
  c.leftPID.error = left_error;
  c.leftPID.derivative = 0.0;
  DriveTestAccess::refresh(c.leftPID);

  // Closes steadily the whole time (never stuck on its own), settling inside small_error(1in)
  // around pass ~195 and latching SMALL_EXIT itself shortly after -- this is what times the
  // recheck above to land right around pass 200-209, while left is mid-blip.
  double right_error = std::fmax(0.5, 20.0 - 0.1 * n);
  c.rightPID.error = right_error;
  c.rightPID.derivative = right_error > 0.5 ? -0.1 : 0.0;
  DriveTestAccess::refresh(c.rightPID);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
  double left_error_at_return;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  script(chassis, 0);
  test_stub::g_clock.on_delay = [] {
    ++g_pass;
    ez::detail::stats.auto_task_passes.fetch_add(1);
    script(*g_chassis, g_pass);
  };
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false, 0.0};
  try {
    chassis.pid_wait();
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
}  // namespace

TEST_CASE("pid_wait() DRIVE: a latched side's stuck watch is resynced on unlatch, giving a fresh self-resolving blip a genuine grace window") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);  // shipped defaults: 90ms/1in/250ms/3in/500ms/500ms

  g_resolve = true;
  Outcome o = run_wait(chassis, 600);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered
                       << " left_error_at_return=" << o.left_error_at_return);

  REQUIRE(o.returned);  // the wait must actually end, not hang past max_passes
  // The blip resolved well within a fresh grace window -- must not be reported as interfered.
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() DRIVE: a disturbance that outlasts even a resynced side's fresh grace window is still reported as interfered") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);

  g_resolve = false;
  Outcome o = run_wait(chassis, 600);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered
                       << " left_error_at_return=" << o.left_error_at_return);

  REQUIRE(o.returned);
  // Sanity on the scripted shape itself: left really never recovers, and is outside big_error.
  REQUIRE(std::fabs(o.left_error_at_return) > chassis.leftPID.exit.big_error);
  // The reseed only resets the clock's baseline -- it must not disable stuck detection outright.
  CHECK(o.interfered);
  // Confirms the fresh window was actually granted (not skipped/short-circuited): un-reseeded, the
  // stale clock fires almost immediately after the pass-~200 un-latch (observed ~202 passes before
  // this fix); reseeded, it has to wait out a genuinely fresh ~50-pass window first (observed ~252).
  CHECK(o.passes >= 240);
}
