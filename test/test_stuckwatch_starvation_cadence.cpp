// StuckWatch's starvation-confirmation (exit_conditions.cpp, the `expected_passes` check inside stuck(),
// gated on the wall-clock window_ already having elapsed) exists to tell "ez_auto_task really ran through
// this window and made no progress" (confirm stuck now) apart from "wall-clock time passed while the task
// barely got to run at all" (wait for the STUCK_STARVED_WINDOWS wall-clock fallback instead, same as a task
// that never runs again).  It derived "how fast the task should be ticking" from this watch's own observed
// passes-per-ms since it was constructed: elapsed real ms since construction divided by elapsed real passes
// since construction.
//
// During a temporary gap in ez_auto_task (busy with something else -- an SD-card write, a screen redraw, any
// other higher-priority task) its own pass counter freezes, but the WAITING task's clock (pros::millis(),
// read from its own un-starved loop) keeps advancing.  So elapsed_ms keeps growing while elapsed_passes stays
// flat, observed_delay = elapsed_ms/elapsed_passes keeps climbing, and expected_passes = window_/observed_delay
// keeps shrinking -- while the confirmation numerator, pass - last_progress_pass_, is frozen too (pass itself
// isn't moving).  Once the shrinking threshold drops below that frozen numerator, the check fires: "confirmed
// by real ez_auto_task passes" during a gap that produced zero of them, on a robot that was never actually
// stuck, only unreported on for a while.
//
// A gap's danger depends on how many passes had already elapsed since the wait started when it begins (call
// it P): expected_passes only drops below the frozen numerator N once elapsed_ms grows past roughly
// window_/N * P, i.e. a gap needs to be roughly proportional to P to trip this -- a wait already well
// established (large P) tolerates a longer gap than one still early in its own start-up allowance (small P).
// That's the opposite of "degrades the longer a wait has run": an earlier round's summary of this same
// mechanism said exactly that, but a verifier re-deriving it by hand and confirming by repro found early
// gaps are the dangerous ones, not late ones -- this file's comments and tests follow the verified mechanism,
// not the earlier paraphrase.
//
// Comparing the real pass count against a FIXED nominal-DELAY_TIME pass count instead removes the shrinking
// threshold entirely, so a frozen numerator can never catch up to it, at any P. Whether that was ever meant
// to give a busy task real extra leniency, or just to confirm it about as fast as a healthy one without
// waiting the full STUCK_STARVED_WINDOWS margin, is genuinely ambiguous from the code alone -- flagged for
// Jess, same as the Channel rebound latch's own flagged tradeoff -- but removing the shrinking threshold is
// unambiguously correct: it can only ever fire on real accumulated passes now, never on wall-clock time
// alone while frozen (that's what STUCK_STARVED_WINDOWS below is for).
//
// Tests: (1) the audit's own shape -- a real, single ~1s gap partway through an otherwise-healthy,
// continuously-progressing 2.5 in/s odom approach.  The PID error/odometry are held flat WHILE the gap is
// open (no fresh reading arrives), then catch up in one jump to wherever the robot truly is by real wall-
// clock time the instant the simulated task resumes -- real hardware keeps counting encoder ticks through a
// gap and ez_tracking_task's absolute-encoder-delta math (tracking.cpp) catches odometry up the same way, not
// gradually. (2) the latency tradeoff this fix trades for that: a robot pinned from the very start under a
// steady, moderately reduced (not dead) task duty, measuring detection latency since the last real progress
// credit old vs new. (3) a task that stops ticking entirely and never resumes -- genuinely dead, not just
// gapped -- which the unchanged STUCK_STARVED_WINDOWS wall-clock
// fallback still has to catch regardless of this fix.
#include <cmath>
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
void (*g_script)(Drive&, int, bool) = nullptr;
bool g_task_dead = false;  // once true, the simulated auto task never ticks again
bool (*g_should_tick)(int) = nullptr;

void on_delay() {
  ++g_pass;
  bool ticked = !g_task_dead && (g_should_tick == nullptr || g_should_tick(g_pass));
  if (ticked) ez::detail::stats.auto_task_passes.fetch_add(1);
  g_script(*g_chassis, g_pass, ticked);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run(Drive& chassis, void (*script)(Drive&, int, bool), bool (*should_tick)(int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_pass = 0;
  g_script = script;
  g_should_tick = should_tick;
  g_task_dead = false;
  script(chassis, 0, true);  // initial setup pass always "ticks" -- establishing starting state, not timing
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
  g_should_tick = nullptr;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}

constexpr double IN_PER_WALL_TICK = 0.025;  // 2.5 in/s at a 10ms tick -- the audit's own figure; matches the
                                             // small_error exit constant (1in) set below as the Channel step
constexpr double PATH_LENGTH_IN = 24.0;     // long enough to still be under way well past the gap

// A single, real gap in the simulated auto task: ticks normally, freezes for GAP_TICKS starting at
// GAP_START_TICK, then resumes -- not a steady reduced duty, the shape the audit itself used.
constexpr int GAP_START_TICK = 75;    // matches the audit's own repro phase
constexpr int GAP_TICKS = 100;        // a 1000ms gap -- the audit's own primary repro length
bool one_gap(int n) { return !(n >= GAP_START_TICK && n < GAP_START_TICK + GAP_TICKS); }

// 70% duty, steady: skips ticks 0, 1 and 2 of every block of 10 -- a moderate, never-fully-stopping slowdown,
// used only for the latency-tradeoff test below (not claimed to reproduce the audit's own gap numbers).
bool busy_70_percent(int n) { return (n % 10) >= 3; }

// Places the fake robot's odom position a shrinking distance short of the target, closing continuously at a
// steady 2.5 in/s of real wall-clock time -- the same "distance closes by a step every so often" progress
// StuckWatch's own credit is built around (see test_pp_wait_stuck.cpp).  Only writes state when the
// simulated task actually ticked this pass; the value it writes is always computed from the true wall-clock
// tick `n`, so a pass right after a gap catches the PID error/odometry up to wherever the robot truly is in
// one jump, the same way ez_tracking_task's absolute-encoder-delta math does on real hardware (it doesn't
// walk the gap's distance gradually pass by pass -- see the file header).  While the gap is open, nothing
// refreshes them at all: "the errors only change when that task runs" (see the class comment in
// exit_conditions.cpp).
// A real compute_error() call on every ticked pass, not a direct `.error =`/`.derivative =` write --
// the small exit this scenario needs to reach a clean finish only credits `error` when a real
// compute has landed since it last checked (see PID.cpp). Feeding `current` the same closing value
// as `error` reproduces the intended derivative on an ordinary pass, and -- just as intended -- a
// large one-time derivative right after a gap, when `remaining` jumps to catch up in one step (see
// this function's own header comment above).
void healthy_crawl(Drive& c, int n, bool ticked) {
  if (!ticked) return;
  double remaining = std::fmax(0.0, PATH_LENGTH_IN - IN_PER_WALL_TICK * n);
  DriveTestAccess::odom_current(c) = {0.0, PATH_LENGTH_IN - remaining, 0.0};
  // IN_PER_WALL_TICK (0.025) is below velocity_zero_main (0.05): feeding it as compute_error()'s
  // `current` the way the other conversions in this file do would make every ordinary pass read as
  // a fresh, stopped sample to xyPID's own velocity channel -- unlike the old direct-write script,
  // which never touched `cur` and so never armed that channel at all. Passing the unchanged `cur`
  // keeps compute_error() from moving it, then restoring `derivative` after the call reproduces the
  // old behavior exactly: a real, fresh compute for the staleness fix, but no velocity-channel
  // side effect this scenario was never about.
  c.xyPID.compute_error(remaining, c.xyPID.cur);
  c.xyPID.derivative = remaining > 0.0 ? -IN_PER_WALL_TICK : 0.0;
  c.current_a_odomPID.compute_error(0.0, 0.0);
}

// A brief healthy crawl (enough to clear the start allowance and set moved_), then pinned in place with a
// jittering derivative so the velocity exit can't end the wait on its own -- same idiom as
// test_jc1_non_odom_stuck.cpp's pinned_jitter(), just for xyPID/current_a_odomPID.
constexpr int PIN_AT_TICK = 60;  // ~1.5in in, comfortably past the start allowance and a full Channel step
void pinned_after_healthy_start(Drive& c, int n, bool ticked) {
  if (!ticked) return;
  if (n < PIN_AT_TICK) {
    healthy_crawl(c, n, ticked);
    return;
  }
  double e = PATH_LENGTH_IN - IN_PER_WALL_TICK * PIN_AT_TICK;
  double jitter_cur = (n % 2 == 0) ? 0.1 : -0.1;  // alternating +-0.1 -> a +-0.2 swing in derivative
  c.xyPID.compute_error(e, jitter_cur);
  c.current_a_odomPID.compute_error(0.0, 0.0);
}

// Same healthy crawl, but the simulated auto task stops ticking entirely partway through and never resumes
// -- genuinely dead, which the STUCK_STARVED_WINDOWS wall-clock fallback (unaffected by this fix) has to
// still catch regardless.
constexpr int DIE_AT_TICK = 150;
void healthy_then_task_dies(Drive& c, int n, bool ticked) {
  if (n == DIE_AT_TICK) g_task_dead = true;
  healthy_crawl(c, n, ticked);  // no-ops on its own once g_task_dead makes on_delay stop passing ticked=true
}
}  // namespace

TEST_CASE("pid_wait() odom: a single ~1s gap in ez_auto_task during an otherwise-healthy 2.5in/s approach does not false-stuck it") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, PATH_LENGTH_IN, ANGLE_NOT_SET}, fwd, 30});
  Outcome o = run(chassis, healthy_crawl, one_gap, (int)(PATH_LENGTH_IN / IN_PER_WALL_TICK) + GAP_TICKS + 100, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() odom: a steady, merely-busy (not dead) task's own genuinely stuck robot is still caught") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, PATH_LENGTH_IN, ANGLE_NOT_SET}, fwd, 30});
  Outcome o = run(chassis, pinned_after_healthy_start, busy_70_percent, 1200, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  // A loose upper bound of "since the pin" alone isn't proof this exercises the pass-count check rather than
  // the unrelated STUCK_STARVED_WINDOWS wall-clock fallback below it: busy_70_percent's own skipped ticks put
  // the last REAL progress credit a little before the pin (around tick 43, not tick 60), so the fallback
  // alone (waited > 4 * window_ = 2000ms past that) would already fire by roughly PIN_AT_TICK + 184 -- inside
  // a merely-generous 200 bound. Tightened well under that (roughly 2x the fix's own measured ~55-pass
  // since-pin result) so a deleted pass-count check (e.g. an always-huge expected_passes) would fail this.
  CHECK(o.passes - PIN_AT_TICK < 100);
}

TEST_CASE("pid_wait() odom: a genuinely dead auto task is still caught by the unchanged wall-clock fallback") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, PATH_LENGTH_IN, ANGLE_NOT_SET}, fwd, 30});
  // Generous cap: even the pre-fix 4x-window wall-clock fallback (2s = 200 passes past the task's death) has
  // to fit comfortably inside this, so a failure here is the fallback itself missing, not an unlucky cap.
  Outcome o = run(chassis, healthy_then_task_dies, nullptr, DIE_AT_TICK + 400, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
}
