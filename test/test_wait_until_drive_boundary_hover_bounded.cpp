// A side that sits on the edge of its small_error window, with noise ticking it across, latches, is taken back, latches again, and so on.
// Giving the watch a fresh clock each time it is taken back would hold the wait for ever, because every fresh clock restarts the very time the
// watch needs to run out (the pid_wait() version of this is test_drive_boundary_hover_resolves_via_rearm_cap.cpp). pid_wait_until() on a drive
// reseeds the same way and is bounded the same way, by STUCK_WATCH_REARM_CAP: this is its test, the same script on both sides at once.
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

// left and right each spend half of a 24 pass cycle just inside their small_error window (0.9 in) and half just outside it (1.1 in), on opposite
// halves, so the two sides are never both latched on the same pass: every meeting point is a real relatch on one side or the other
void script() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  Drive& c = *g_chassis;
  int phase = g_pass % 24;
  c.leftPID.error = (phase < 12) ? 0.9 : 1.1;
  c.leftPID.derivative = 0.0;
  DriveTestAccess::refresh(c.leftPID);
  c.rightPID.error = (phase < 12) ? 1.1 : 0.9;
  c.rightPID.derivative = 0.0;
  DriveTestAccess::refresh(c.rightPID);
}
}  // namespace

TEST_CASE("pid_wait_until() on a drive does not hang when both sides oscillate across their exit window boundary") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(48, 100);  // the shipped exits: 90 ms / 1 in / 250 ms / 3 in / 500 ms / 500 ms
  g_chassis = &chassis;
  g_pass = 0;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = 1500;
  bool returned = true;
  try {
    chassis.pid_wait_until(48.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);
  REQUIRE(returned);
  CHECK(g_pass < 500);
}
