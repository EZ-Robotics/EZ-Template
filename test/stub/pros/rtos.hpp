// Minimal stand-in for PROS's pros/rtos.hpp -- pros::Task, pros::Mutex,
// pros::RecursiveMutex, pros::delay, pros::millis, pros::competition.
#pragma once

#include <cstdint>

#include "fake_hardware.hpp"

namespace pros {

// Real PROS's pros::Task schedules the given callable on its own thread.
// Host tests have no scheduler and no threads: this stub constructs and
// discards the callable without ever invoking it, so e.g. Drive's
// `ez_auto([this] { this->ez_auto_task(); })` member does not spin up an
// infinite loop when a Drive is constructed in a test. Task bodies (like
// ez_auto_task()) are instead called directly through DriveTestAccess.
class Task {
 public:
  Task() = default;
  template <typename F>
  explicit Task(F&&) {}
  template <typename F>
  Task(F&&, const char*) {}

  void remove() {}
  void resume() {}
  void suspend() {}
  bool notify() { return true; }
};

class Mutex {
 public:
  bool take(std::uint32_t timeout = 0xFFFFFFFF) { return true; }
  bool give() { return true; }
  void lock() {}
  void unlock() {}
  bool try_lock() { return true; }
};

// Needs lock()/unlock() (not just take()/give()) to satisfy the BasicLockable
// requirement std::lock_guard<pros::RecursiveMutex> depends on, since
// EZ-Template guards drive_mutex with std::lock_guard throughout.
class RecursiveMutex {
 public:
  bool take(std::uint32_t timeout = 0xFFFFFFFF) { return true; }
  bool give() { return true; }
  void lock() {}
  void unlock() {}
  bool try_lock() { return true; }
};

inline std::uint32_t millis() { return test_stub::g_clock.now_ms; }

inline void delay(std::uint32_t ms) {
  test_stub::g_clock.now_ms += ms;
  if (test_stub::g_clock.delay_calls_until_stop >= 0) {
    if (test_stub::g_clock.delay_calls_until_stop == 0) throw test_stub::StopLoop{};
    test_stub::g_clock.delay_calls_until_stop--;
  }
}

namespace competition {
inline bool is_disabled() { return test_stub::g_competition.disabled; }
inline bool is_autonomous() { return test_stub::g_competition.autonomous; }
inline bool is_connected() { return true; }
}  // namespace competition

}  // namespace pros
