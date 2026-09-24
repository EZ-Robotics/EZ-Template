// StuckWatch's starvation-confirmation (exit_conditions.cpp, the `expected_passes` check inside stuck(),
// gated on the wall-clock window_ already having elapsed) exists to tell "ez_auto_task really ran through
// this window and made no progress" (confirm stuck now) apart from "wall-clock time passed while the task
// barely got to run at all" (wait for the STUCK_STARVED_WINDOWS wall-clock fallback instead). It estimated
// "how fast should the task be ticking" from this watch's own observed passes-per-ms since it was
// constructed. That estimate shares its own denominator with the very stretch it's judging: expected_passes
// (window_ / observed_delay) times observed_delay is window_ again, by construction, whatever observed_delay
// turns out to be measured as -- so "real passes since last progress" exceeding "expected_passes" reduces to
// nothing more than waited > window_, the same test performed just above it, for any task ticking at a
// roughly steady rate. That gives a task running a little slower than nominal (ordinary scheduling overhead,
// not dead) essentially the same detection speed as a perfectly healthy one -- confirmed quickly, at roughly
// 1x window_, rather than being given the full STUCK_STARVED_WINDOWS margin a truly-dead task gets. The
// longer a wait has already run (the more precisely that observed rate has settled toward the task's true,
// steady cadence), the more exactly this collapse holds.
//
// Whether that was ever meant to be prompt-but-tight detection for a busy task, or real extra leniency for
// one, is genuinely ambiguous from the comment alone -- flagged for Jess, same as the rebound-latch tradeoff.
// What's unambiguous is the concrete case the round-2 audit raised: at shipped defaults, this can confirm
// "stuck" on a genuinely healthy, steadily-progressing slow approach in well under what round 1 had already
// found and accepted (1.5-6x the window) -- because the check's own tolerance shrinks toward bare window_ the
// longer a busy-but-not-dead task has been running, not because the robot ever stopped moving.
//
// These tests script a real, steady 70%-duty auto task (it ticks on 7 of every 10 of this loop's own passes,
// simulating ordinary competing-task overhead, never fully stopping) two ways: alongside a slow but
// genuinely healthy odom approach at the ~2.5 in/s the audit itself used, closing by a full step every 400ms
// of TASK time (not wall time -- the odometry and PID error only change on a pass the task actually runs,
// same as the real system, so a busy task's ticks land farther apart in real wall-clock time too); and
// alongside a robot that never moves at all, to see how promptly a merely-busy (not dead) task's own stuck
// robot gets caught. A last scenario has the task stop ticking entirely partway through -- genuinely dead,
// not just busy -- which the (unchanged) STUCK_STARVED_WINDOWS wall-clock fallback still has to catch.
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
int g_task_ticks = 0;  // how many times the simulated auto task has actually ticked so far
void (*g_script)(Drive&, int, bool, int) = nullptr;
bool g_task_dead = false;  // once true, the simulated auto task never ticks again
bool (*g_should_tick)(int) = nullptr;

// 70% duty: skips passes 0, 1 and 2 of every block of 10 -- a steady, moderate slowdown, never a full stop.
bool busy_70_percent(int n) { return (n % 10) >= 3; }

void on_delay() {
  ++g_pass;
  bool ticked = !g_task_dead && (g_should_tick == nullptr || g_should_tick(g_pass));
  if (ticked) {
    ez::detail::stats.auto_task_passes.fetch_add(1);
    ++g_task_ticks;
  }
  g_script(*g_chassis, g_pass, ticked, g_task_ticks);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run(Drive& chassis, void (*script)(Drive&, int, bool, int), bool (*should_tick)(int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_pass = 0;
  g_task_ticks = 0;
  g_script = script;
  g_should_tick = should_tick;
  g_task_dead = false;
  script(chassis, 0, true, 0);  // initial setup pass always "ticks" -- it's establishing starting state, not timing
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

constexpr double CRAWL_STEP_IN = 1.0;         // matches the small_error exit constant set below
constexpr int TASK_TICKS_PER_STEP = 40;       // 400ms of TASK time per inch => 2.5 in/s, the audit's own figure
constexpr int CRAWL_STEPS = 12;               // a long wait: 4.8s of task time, longer still in busy real time
constexpr double PATH_LENGTH_IN = CRAWL_STEPS * CRAWL_STEP_IN;
constexpr int TOTAL_TASK_TICKS = CRAWL_STEPS * TASK_TICKS_PER_STEP;
// Inches closed per TASK tick (not per wall tick) for a smooth, continuous crawl averaging one full
// CRAWL_STEP_IN every TASK_TICKS_PER_STEP task ticks. Continuous, not a staircase that lands exactly on
// each step boundary, so Channel::made()'s strict "< low - step" check (a size sitting exactly ON the
// boundary doesn't count, see exit_conditions.cpp) reliably credits progress once each step.
constexpr double IN_PER_TASK_TICK = CRAWL_STEP_IN / TASK_TICKS_PER_STEP;

// Places the fake robot's odom position a shrinking distance short of the target, closing smoothly and
// continuously at a slow but perfectly steady 2.5 in/s of TASK time -- the same "distance closes by a step
// every so often" progress StuckWatch's own credit is built around (see test_pp_wait_stuck.cpp), just slow.
// Only updates when the simulated task actually ticked this pass: "the errors only change when that task
// runs" (see the class comment in exit_conditions.cpp) -- a busy task's ticks land farther apart on the
// real wall clock too, exactly like the real system, not just a slower pass counter with the odometry still
// magically updating every wall-clock tick regardless.
void slow_healthy_crawl(Drive& c, int, bool ticked, int task_ticks) {
  if (!ticked) return;
  double remaining = std::fmax(0.0, PATH_LENGTH_IN - IN_PER_TASK_TICK * std::min(task_ticks, TOTAL_TASK_TICKS));
  DriveTestAccess::odom_current(c) = {0.0, PATH_LENGTH_IN - remaining, 0.0};
  c.xyPID.error = remaining;
  c.xyPID.derivative = remaining > 0.0 ? -IN_PER_TASK_TICK : 0.0;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}

// A robot that never moves at all, under the same steady 70%-duty (busy, not dead) task -- how promptly does
// a merely-busy task's own genuinely stuck robot get caught?  Jittered derivative so the velocity exit can't
// end the wait on its own, same idiom as test_jc1_non_odom_stuck.cpp's pinned_jitter().
void pinned_under_busy_task(Drive& c, int n, bool ticked, int) {
  if (!ticked) return;
  c.xyPID.error = 20.0;
  c.xyPID.derivative = (n % 2 == 0) ? 0.2 : -0.2;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}

// Same healthy crawl, but the simulated auto task stops ticking entirely partway through -- a genuinely dead
// task, which the STUCK_STARVED_WINDOWS wall-clock fallback (unaffected by this fix) has to still catch.
constexpr int DIE_AT_TICK = 400;  // ~half a step of real (busy) time into the crawl
void healthy_then_task_dies(Drive& c, int n, bool ticked, int task_ticks) {
  if (n == DIE_AT_TICK) g_task_dead = true;
  slow_healthy_crawl(c, n, ticked, task_ticks);  // no-ops once g_task_dead makes on_delay stop passing ticked=true
}
}  // namespace

TEST_CASE("pid_wait() odom: a steady, merely-busy (not dead) auto task does not false-stuck a genuinely healthy slow approach over a long wait") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, PATH_LENGTH_IN, ANGLE_NOT_SET}, fwd, 30});
  // At 70% duty, TOTAL_TASK_TICKS task ticks take TOTAL_TASK_TICKS / 0.7 real wall ticks; a healthy margin
  // on top covers the settling passes after the last step.
  int max_passes = (int)(TOTAL_TASK_TICKS / 0.7) + 100;
  Outcome o = run(chassis, slow_healthy_crawl, busy_70_percent, max_passes, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK_FALSE(o.interfered);
}

TEST_CASE("pid_wait() odom: a merely-busy (not dead) task's own genuinely stuck robot is still caught, sooner than the dead-task fallback") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, 48.0, ANGLE_NOT_SET}, fwd, 30});
  Outcome o = run(chassis, pinned_under_busy_task, busy_70_percent, 1200, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  // Comfortably below the dead-task fallback (STUCK_STARVED_WINDOWS * window_ = 4 * 500ms = 2000ms = 200
  // passes past the 1s start allowance, ~300 passes total) -- proof this exercises the pass-count check
  // this fix changed, not just the unrelated wall-clock-only fallback below it.
  CHECK(o.passes < 250);
}

TEST_CASE("pid_wait() odom: a genuinely dead auto task is still caught by the unchanged wall-clock fallback") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, PATH_LENGTH_IN, ANGLE_NOT_SET}, fwd, 30});
  // Generous cap: even the old, pre-fix 4x-window wall-clock fallback (500ms * 4 = 2s = 200 passes past the
  // task's death) has to fit comfortably inside this, so a failure here is the fallback itself missing, not
  // an unlucky cap.
  Outcome o = run(chassis, healthy_then_task_dies, busy_70_percent, DIE_AT_TICK + 400, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
}
