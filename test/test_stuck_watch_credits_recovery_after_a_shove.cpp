// A defense push that shoves the robot's error a full step worse mid-motion, without ever crossing its target (no sign flip), is not a
// stall: the robot keeps moving through the shove and the recovery, so its speed stays above the stop speed and neither the velocity exit
// nor the small/big exit timers run out until it has genuinely arrived.  The wait must finish the motion and not report interfered.
//
// The other pid_wait() tests script either a steadily-closing motion, a robot fully pinned, or an overshoot past the target -- none
// scripts a mid-motion shove that pushes the error further from the target without crossing it.
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

// A healthy motion closing toward its target (passes 1-100, error 20 -> 5), then something shoves
// the robot backward for a moment without crossing the target (101-110, error 5 -> 15, same sign
// the whole time -- no sign flip), then the robot recovers and finishes closing normally (error
// 15 -> 0 over the next 150 passes).  Defense contact mid-motion, not a stall.  The sensor reading
// `cur` follows the error (target 24, cur = 24 - error), so the PID's speed sees the real movement:
// 15 in/s closing, 100 in/s shoved, 10 in/s recovering.
void shoved_then_recovers(Drive& c, int n) {
  double error;
  if (n <= 100) {
    error = 20.0 - 0.15 * n;
  } else if (n <= 110) {
    error = 5.0 + 1.0 * (n - 100);
  } else {
    error = std::fmax(0.0, 15.0 - 0.1 * (n - 110));
  }
  c.leftPID.compute_error(error, 24.0 - error);
  c.rightPID.compute_error(error, 24.0 - error);
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  shoved_then_recovers(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("pid_wait credits recovering from a mid-motion shove as progress, not just an overshoot") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  shoved_then_recovers(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 500;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  REQUIRE(returned);
  // A defense push that never actually stalled the robot -- it kept recovering and finished the
  // motion -- must not be reported as stuck.
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(chassis.leftPID.error) < 1.0);
}
