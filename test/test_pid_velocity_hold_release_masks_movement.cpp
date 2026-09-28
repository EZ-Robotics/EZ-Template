// velocity_exit_hold_set()'s documented contract: held time counts neither toward nor against the
// exit, resuming from wherever it left off once released. A caller that toggles
// velocity_exit_hold_set() at or faster than its own poll cadence must not turn that toggling itself
// into a false stop reading: every released poll still has to see its own real movement (if any) and
// reset the STOPPED timer, the same as an ordinary poll would.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("velocity_exit_hold toggled every poll does not false-VELOCITY_EXIT a mechanism that never stops moving") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 100, 0);  // velocity_exit_time=100ms only
  pid.target_set(1000000);

  test_stub::g_clock.now_ms = 0;
  double position = 0.0;

  // Arm: one real, well-above-floor movement.
  position += 5.0;
  pid.compute_error(1000000.0 - position, position);
  CHECK(pid.exit_condition() == RUNNING);

  // From here on, the mechanism moves by 5.0 (100x the 0.05 default floor) on every single fresh
  // compute -- continuous, unambiguous real motion, never stalled for even one tick. Toggle the hold
  // on and off every poll: held on even ticks, released on odd ticks, so every released tick is a
  // release poll.
  bool exited = false;
  int t_exit = -1;
  for (int t = 10; t <= 5000; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    bool hold_now = (t / 10) % 2 == 0;
    pid.velocity_exit_hold_set(hold_now);
    position += 5.0;  // real, continuous, well-above-floor movement every tick, hold or not
    pid.compute_error(1000000.0 - position, position);
    if (pid.exit_condition() == VELOCITY_EXIT) {
      exited = true;
      t_exit = t;
      break;
    }
  }

  // A mechanism that moves 5.0 units (100x the stopped floor) on every single fresh compute, with no
  // tick ever genuinely stalled, must never VELOCITY_EXIT -- the exit only exists to catch a real
  // stop. If this fires, the hold-toggle cadence alone manufactured a false stop reading out of
  // continuous, unambiguous real motion.
  INFO("false VELOCITY_EXIT at t=", t_exit, " despite continuous above-floor movement every tick");
  CHECK_FALSE(exited);
}

TEST_CASE("velocity_exit_hold toggled every poll does not falsely mask a genuine stall either (control)") {
  // Companion control: the same toggle-every-poll pattern against a mechanism that is genuinely
  // stalled throughout (never moves) should still exit close to velocity_exit_time -- confirms the
  // toggle pattern by itself isn't simply suppressing the exit outright in both directions, only
  // that it must not manufacture a false stop out of continuous real movement.
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 100, 0);
  pid.target_set(1000000);

  test_stub::g_clock.now_ms = 0;
  double position = 50.0;

  // Arm with one real movement, then genuinely stop for good.
  pid.compute_error(1000000.0 - position, position);
  position += 5.0;
  pid.compute_error(1000000.0 - position, position);
  CHECK(pid.exit_condition() == RUNNING);

  bool exited = false;
  int t_exit = -1;
  for (int t = 10; t <= 2000; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    bool hold_now = (t / 10) % 2 == 0;
    pid.velocity_exit_hold_set(hold_now);
    pid.compute_error(1000000.0 - position, position);  // never moves again
    if (pid.exit_condition() == VELOCITY_EXIT) {
      exited = true;
      t_exit = t;
      break;
    }
  }
  CHECK(exited);
}
