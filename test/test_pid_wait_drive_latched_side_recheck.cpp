// pid_wait()'s DRIVE branch latches each side's exit the first time it goes non-RUNNING and never calls exit_condition() on that side again, so
// a disturbance landing after a side has latched is not looked at.  DRIVE's two sides routinely finish at different times (a scrubbing wheel, an
// uneven load), so this file keeps the control for that shape: both sides latching close together at a genuinely settled target is a clean,
// uninterfered finish.
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
};

Outcome run_wait(Drive& chassis, void (*script)(Drive&, int), int max_passes, std::function<void()> wait) {
  g_chassis = &chassis;
  g_script = script;
  g_pass = 0;
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

}  // namespace

// Both sides close the same way and exit together, and nothing shoves either one afterward.
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
