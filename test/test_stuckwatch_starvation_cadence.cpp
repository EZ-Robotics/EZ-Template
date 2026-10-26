// pid_wait() on an odom point-to-point move ends on the xy and angle exits, each of which counts real milliseconds from the waiting task's own
// clock, not ez_auto_task's passes.  So a busy ez_auto_task (ticking at a reduced duty) or a dead one (never ticking again) can't keep a
// wait from ending when the robot is stuck.
//
// Tests: (1) a robot pinned from the start under a steady, moderately reduced (not dead) task duty, still ending interfered.  (2) a task
// that stops ticking entirely and never resumes -- genuinely dead, not just busy -- which the wait still has to end regardless.
//
// The scripted xyPID never has its `cur` moved, so xyPID reads as stopped on every pass (see PID::exit_condition): both tests end on the
// velocity exit's 1000 ms arming fallback plus velocity_exit_time, whether or not the task ticks.
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

constexpr double IN_PER_WALL_TICK = 0.025;  // 2.5 in/s at a 10ms tick
constexpr double PATH_LENGTH_IN = 24.0;     // long enough to still be under way when the wait ends

// 70% duty, steady: skips ticks 0, 1 and 2 of every block of 10 -- a moderate, never-fully-stopping slowdown.
bool busy_70_percent(int n) { return (n % 10) >= 3; }

// Places the fake robot's odom position a shrinking distance short of the target, closing continuously at a steady 2.5 in/s of real
// wall-clock time.  Only writes state when the simulated task actually ticked this pass.
// A real compute_error() call on every ticked pass, not a direct `.error =`/`.derivative =` write.
void healthy_crawl(Drive& c, int n, bool ticked) {
  if (!ticked) return;
  double remaining = std::fmax(0.0, PATH_LENGTH_IN - IN_PER_WALL_TICK * n);
  DriveTestAccess::odom_current(c) = {0.0, PATH_LENGTH_IN - remaining, 0.0};
  // Passing the unchanged `cur` keeps xyPID's speed at zero: the robot's odom position moves, but xyPID reads as stopped.
  c.xyPID.compute_error(remaining, c.xyPID.cur);
  c.xyPID.derivative = remaining > 0.0 ? -IN_PER_WALL_TICK : 0.0;
  c.current_a_odomPID.compute_error(0.0, 0.0);
}

// A brief healthy crawl, then pinned in place with a jittering `cur` -- same idiom as test_jc1_non_odom_stuck.cpp's pinned_jitter(), just
// for xyPID/current_a_odomPID.
constexpr int PIN_AT_TICK = 60;  // ~1.5in in
void pinned_after_healthy_start(Drive& c, int n, bool ticked) {
  if (!ticked) return;
  if (n < PIN_AT_TICK) {
    healthy_crawl(c, n, ticked);
    return;
  }
  double e = PATH_LENGTH_IN - IN_PER_WALL_TICK * PIN_AT_TICK;
  double jitter_cur = (n % 2 == 0) ? 0.1 : -0.1;  // alternating +-0.1
  c.xyPID.compute_error(e, jitter_cur);
  c.current_a_odomPID.compute_error(0.0, 0.0);
}

// Same healthy crawl, but the simulated auto task stops ticking entirely partway through and never resumes -- genuinely dead.
constexpr int DIE_AT_TICK = 150;
void healthy_then_task_dies(Drive& c, int n, bool ticked) {
  if (n == DIE_AT_TICK) g_task_dead = true;
  healthy_crawl(c, n, ticked);  // no-ops on its own once g_task_dead makes on_delay stop passing ticked=true
}
}  // namespace

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
  // The velocity exit arms through its 1000 ms fallback (100 passes) and then needs velocity_exit_time (500 ms, 50 passes) at speed zero:
  // the wait ends around pass 152, which is under 100 passes after the pin.
  CHECK(o.passes - PIN_AT_TICK < 100);
}

TEST_CASE("pid_wait() odom: a genuinely dead auto task is still caught by the velocity exit") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
  chassis.pid_odom_ptp_set({{0.0, PATH_LENGTH_IN, ANGLE_NOT_SET}, fwd, 30});
  // Generous cap: the 1000 ms arming fallback and 500 ms velocity_exit_time end the wait around pass 152, so a failure here is the exit
  // itself missing, not an unlucky cap.
  Outcome o = run(chassis, healthy_then_task_dies, nullptr, DIE_AT_TICK + 400, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
}
