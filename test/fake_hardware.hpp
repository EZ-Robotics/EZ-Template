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
};

struct FakeCompetitionStatus {
  bool disabled = false;
  bool autonomous = false;
};

inline FakeClock g_clock;
inline FakeCompetitionStatus g_competition;

// pros::motor_fake_registry() and the globals above all persist for the
// whole test binary's run, not per TEST_CASE. Any test that constructs a
// Drive and cares about motor output, the fake clock, or competition status
// should call this first so it isn't reading state a differently-ported (or
// same-ported) Drive from an earlier TEST_CASE left behind.
inline void reset_all() {
  g_clock = FakeClock{};
  g_competition = FakeCompetitionStatus{};
  pros::motor_fake_registry().clear();
}

}  // namespace test_stub
