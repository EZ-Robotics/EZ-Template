// velocity_exit_hold_set(true) documents that held time counts neither toward nor against the exit,
// resuming from wherever it left off once released. The moving-count comparison and the wall-clock
// baseline are only touched while not held, so without an explicit resync on release, the first poll
// after a hold ends would compare against whatever they were left at before the hold started: a real
// disturbance during the hold would still count against the exit, and -- worse -- the wall-clock
// baseline would credit the entire held span as if continuously settled the moment the hold releases.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("a hold with no disturbance does not credit the held span as settled dwell on release") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  pid.compute_error(10.0, 1.0);  // arms
  CHECK(pid.exit_condition() == RUNNING);

  test_stub::g_clock.now_ms = 10;
  pid.velocity_exit_hold_set(true);
  pid.compute_error(10.0, 1.0);
  CHECK(pid.exit_condition() == RUNNING);  // held: no progress toward the exit while frozen

  test_stub::g_clock.now_ms = 60;  // 50ms held -- would alone satisfy velocity_exit_time if credited
  pid.compute_error(10.0, 1.0);
  CHECK(pid.exit_condition() == RUNNING);  // still held

  test_stub::g_clock.now_ms = 70;
  pid.velocity_exit_hold_set(false);
  pid.compute_error(10.0, 1.0);
  CHECK(pid.exit_condition() == RUNNING);  // release: must not credit the 60ms held span in one shot

  // Genuinely settling from here takes the full configured 50ms, not 0ms.
  bool fired = false;
  for (int t = 80; t <= 300; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10.0, 1.0);
    if (pid.exit_condition() == VELOCITY_EXIT) {
      fired = true;
      CHECK(t >= 70 + 40);  // roughly velocity_exit_time after release, not immediately
      break;
    }
  }
  CHECK(fired);
}

TEST_CASE("a disturbance injected only during a hold does not count against the exit on release") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 50, 0);
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  pid.compute_error(10.0, 1.0);
  CHECK(pid.exit_condition() == RUNNING);

  test_stub::g_clock.now_ms = 10;
  pid.velocity_exit_hold_set(true);
  pid.compute_error(10.0, 50.0);  // a real shove, but entirely inside the hold
  CHECK(pid.exit_condition() == RUNNING);

  test_stub::g_clock.now_ms = 20;
  pid.velocity_exit_hold_set(false);
  pid.compute_error(10.0, 50.0);  // settled again the instant the hold releases
  CHECK(pid.exit_condition() == RUNNING);

  bool fired = false;
  int t_exit = -1;
  for (int t = 30; t <= 300; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute_error(10.0, 50.0);
    if (pid.exit_condition() == VELOCITY_EXIT) {
      fired = true;
      t_exit = t;
      break;
    }
  }
  CHECK(fired);
  // Should take the full ~50ms from release (t=20), not be penalized by the shove that happened
  // entirely inside the hold.
  CHECK(t_exit <= 20 + 60);
}
