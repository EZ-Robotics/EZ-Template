// Shared fake VEX runtime state used by the test/stub/pros headers.
//
// Most per-device fake state (IMU rotation, encoder value, ...) lives
// directly on the stub device objects themselves (e.g. chassis.imu->fake())
// since tests can reach those objects through EZ-Template's own public
// members. pros::Motor is the exception -- its fake state is a global
// registry keyed by port (see stub/pros/motors.hpp) because EZ-Template
// copies Motor objects by value when driving them, and a real motor's state
// belongs to the port, not to any one handle to it. This file holds that
// registry plus the state that has no per-device object to live on at all:
// the fake clock and the fake competition status, both read by free
// functions (pros::millis(), pros::competition::is_disabled()).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pros/motors.hpp"

namespace test_stub {

// Thrown by the fake pros::delay() to unwind an otherwise-infinite task loop
// (e.g. Drive::ez_auto_task(), which is `while (true) { ... pros::delay(...); }`
// and is never actually scheduled by the fake pros::Task -- see stub/pros/rtos.hpp)
// after a caller-controlled number of passes. A test that wants to run such a
// task's body exactly once sets g_clock.delay_calls_until_stop = 0 and wraps
// the call in try { ... } catch (test_stub::StopLoop&) {}.
struct StopLoop {};

struct FakeClock {
  // Starts past private_drive_set()'s `if (pros::millis() < 1500) return;`
  // startup guard so tests don't silently no-op on the drive output.
  std::uint32_t now_ms = 2000;

  // -1 (default): pros::delay() never throws. >= 0: counts down on every
  // pros::delay() call and throws StopLoop when it reaches 0.
  int delay_calls_until_stop = -1;

  // Runs on every pros::delay() call, before the countdown above. Lets a test change something between two
  // passes of a task loop, such as the task's priority.
  void (*on_delay)() = nullptr;
};

struct FakeCompetitionStatus {
  bool disabled = false;
  bool autonomous = false;
};

// A fake task scheduler for the pros::c task calls: a few tasks, each with a base and an effective priority (they
// differ while a task is running on an inherited priority), one of them current, and a log of the priority calls
// made ("get", "set:15", ...) so a test can assert the exact sequence. Handle n is task n - 1; nullptr means the
// current task, as it does in PROS. Task 3 is the PROS system daemon, at priority 14.
struct FakeTask {
  std::uint32_t base = 8;
  std::uint32_t effective = 8;
};

struct FakeScheduler {
  FakeTask tasks[4];
  int current = 0;
  bool daemon_present = true;
  int ignore_sets = 0;  // the next N set_priority calls change nothing, as if a restore did not take
  std::vector<std::string> calls;

  FakeScheduler() { tasks[3] = {14, 14}; }
};

inline FakeClock g_clock;
inline FakeCompetitionStatus g_competition;
inline FakeScheduler g_sched;

// pros::motor_fake_registry() and the globals above all persist for the
// whole test binary's run, not per TEST_CASE. Any test that constructs a
// Drive and cares about motor output, the fake clock, or competition status
// should call this first so it isn't reading state a differently-ported (or
// same-ported) Drive from an earlier TEST_CASE left behind.
inline void reset_all() {
  g_clock = FakeClock{};
  g_competition = FakeCompetitionStatus{};
  g_sched = FakeScheduler{};
  pros::motor_fake_registry().clear();
}

}  // namespace test_stub
