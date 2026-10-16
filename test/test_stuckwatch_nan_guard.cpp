// A NaN reading fed into a Channel (exit_conditions.cpp) -- e.g. from a caller-supplied NaN target, which
// makes a PID's error/distance compute to NaN every pass -- reads as permanent "progress" instead of no
// progress at all: every comparison against NaN is false, so Channel::made()'s own "is this still above the
// last low?" check (`size >= low - step`) is always false while NaN keeps arriving, which both credits a
// bogus new low every single pass AND overwrites that low with NaN. That defeats the whole progress backstop
// (StuckWatch/SingleStuckWatch) for as long as NaN keeps arriving: last_progress_ never goes stale, so the
// window this backstop exists to enforce never elapses, even on a robot that is otherwise genuinely, fully
// pinned (the same disturbance test_jc1_non_odom_stuck.cpp already covers, just with a NaN reading standing
// in for a real sensor value instead of a finite pinned one).
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

constexpr int NAN_START = 5;

// A brief healthy start (so the channel's low water mark is a real, finite number, and the start allowance
// clears the same way a real motion's would) then pinned with a NaN error every pass from then on, jittering
// the derivative like test_jc1_non_odom_stuck.cpp's pinned_jitter() so the velocity exit's own accumulator
// can't build up and end the wait on its own -- isolating the stuck backstop as the only thing that can ever
// end this wait.
void nan_pinned(Drive& c, int n) {
  if (n < NAN_START) {
    double e = std::fmax(0.0, 20.0 - 2.0 * n);
    c.leftPID.error = e;
    c.leftPID.derivative = e > 0.0 ? -2.0 : 0.0;
    c.rightPID.error = e;
    c.rightPID.derivative = e > 0.0 ? -2.0 : 0.0;
    return;
  }
  c.leftPID.error = std::nan("");
  c.leftPID.derivative = (n % 2 == 0) ? 0.2 : -0.2;
  c.rightPID.error = std::nan("");
  c.rightPID.derivative = (n % 2 == 0) ? 0.2 : -0.2;
}

// NaN from the very first pass -- the Channel constructor itself seeds `low` from the first reading it's
// given, so this covers the case where nothing finite was ever seen at all, not just a NaN that arrives
// after a real low is already on record.
void nan_from_start(Drive& c, int) {
  c.leftPID.error = std::nan("");
  c.leftPID.derivative = 0.2;
  c.rightPID.error = std::nan("");
  c.rightPID.derivative = -0.2;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a NaN error does not defeat the stuck backstop on a genuinely pinned wait") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  // 3000 passes = 30s, the same cap test_jc1_non_odom_stuck.cpp uses for its pinned-jitter case -- before the
  // fix, this hangs the full cap (NaN reads as permanent progress); the fix should end it well under that,
  // in the same ballpark as an ordinary (non-NaN) pinned disturbance.
  Outcome o = run(chassis, nan_pinned, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}

TEST_CASE("pid_wait() DRIVE: a NaN error present from the very first pass does not defeat the stuck backstop") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  Outcome o = run(chassis, nan_from_start, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}
