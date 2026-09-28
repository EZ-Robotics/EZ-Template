// Companion to test_drive_latched_side_stuck_watch_resync.cpp: what could go wrong with
// unconditionally reseeding a latched side's SingleStuckWatch on every un-latch (see that recheck's
// own comment in exit_conditions.cpp) is a failure mode neither of that file's two tests can catch --
// a side sitting almost exactly on its own small_error boundary, with realistic sensor noise ticking
// it back and forth across that boundary, could latch, get un-latched by the recheck (and reseeded),
// re-latch, get un-latched again (and reseeded again), forever, and never let pid_wait() return at
// all. Measured, not just theorized: with reseeding left unconditional, this exact script left
// pid_wait() DRIVE still running past 3000 simulated passes (~30s) with no sign of resolving (see the
// discussion on GitHub issue #532's fix PR).
//
// STUCK_WATCH_REARM_CAP bounds this: past a small, fixed number of reseeds, a side falls back to the
// pre-fix un-reseeded (frozen-clock) behavior for the rest of that wait -- the same backstop that
// already bounded this exact scenario before the fix existed. This test drives BOTH sides with that
// boundary noise at once, on opposite half-cycles so a clean double-exit essentially never coincides
// (every meeting point at the recheck is a genuine relatch on one side or the other -- mirroring
// test_odom_relatch_boundary_noise_bounded.cpp's shape for odom's shared StuckWatch), and asserts the
// wait still returns within a bounded number of passes.
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

// left and right each spend half of a 24-pass cycle just inside their small_error window and half
// just outside it, on OPPOSITE halves: left is in-window (0.9in, under the shipped 1.0in small
// window) for the first 12 passes of each cycle and out (1.1in) for the second 12; right is the
// mirror image. Each side's in-window half is long enough (12 > the 10 consecutive passes
// exit_condition() needs at 90ms/10ms-per-pass) to genuinely re-latch SMALL_EXIT before its next
// out-of-window half un-latches it again -- but because the halves are disjoint, the two sides are
// never BOTH inside their window on the same pass, so the recheck below never sees a legitimately
// clean double-exit: every meeting point is a real relatch, on alternating sides, cycle after cycle.
// Neither side ever settles for good -- that's the point.
// A real compute via DriveTestAccess::refresh() every pass, not a bare `.error =` write -- the
// relatch behavior this test exercises depends on SMALL_EXIT genuinely firing during each in-window
// half, which only happens when a real compute has landed since exit_condition() last checked.
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int phase = g_pass % 24;
  double left_e = (phase < 12) ? 0.9 : 1.1;
  double right_e = (phase < 12) ? 1.1 : 0.9;
  c.leftPID.error = left_e;
  c.leftPID.derivative = 0.0;
  DriveTestAccess::refresh(c.leftPID);
  c.rightPID.error = right_e;
  c.rightPID.derivative = 0.0;
  DriveTestAccess::refresh(c.rightPID);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE does not hang when both sides oscillate across their exit window boundary") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);  // shipped defaults: 90ms/1in/250ms/3in/500ms/500ms

  // 1500 is generous relative to what STUCK_WATCH_REARM_CAP's own arithmetic predicts (a handful of
  // reseeds, each costing roughly one ~24-pass relatch cycle, followed by at most one more
  // un-reseeded ~50-pass window before the frozen clock catches it) -- there is no legitimate path
  // here that needs anywhere near this many passes to resolve.
  Outcome o = run_wait(chassis, 1500);
  MESSAGE("returned=" << o.returned << " passes=" << o.passes << " interfered=" << o.interfered);

  // The primary assertion: it returned at all. If reseeding were still unconditional, this throws
  // test_stub::StopLoop instead and o.returned is false -- confirmed by hand against unconditional
  // reseeding (the PR discussion on issue #532's fix has the measured numbers: still running past
  // 3000 passes with reseeding unconditional, vs. this cap).
  REQUIRE(o.returned);
  // Bounded, not just "didn't hit the passes ceiling".
  CHECK(o.passes < 500);
  // Neither side ever holds still long enough to be a genuine, sustained convergence, but this
  // noise's own amplitude (0.9-1.1) never leaves either side's big_error(3in) window either -- so
  // once the frozen-clock backstop (after the cap) ends this, it does so through the same
  // "stuck, but stopped inside the big error windows, counted as settled" path the pre-fix code
  // already used for this exact script (see the PR discussion on issue #532's fix: unconditional
  // un-reseeded code resolves this at pass 155, interfered=false). A capped side falling back to
  // that same pre-fix backstop should reach the same honest verdict, not a different one -- the
  // way it already did before this fix existed.
  CHECK_FALSE(o.interfered);
}
