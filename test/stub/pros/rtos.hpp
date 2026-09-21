// Minimal stand-in for PROS's pros/rtos.hpp -- pros::Task, pros::Mutex,
// pros::RecursiveMutex, pros::delay, pros::millis, pros::competition.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>

#include "fake_hardware.hpp"

// Same values and spelling as PROS's rtos.h.
#define TIMEOUT_MAX ((std::uint32_t)0xffffffffUL)
#define TASK_PRIORITY_MAX 16
#define TASK_PRIORITY_DEFAULT 8

namespace pros {

typedef void* task_t;

// The task calls the library uses, backed by test_stub::g_sched. Same semantics as PROS's task_set_priority: the
// base priority always changes, the effective one only when the task is not on an inherited priority.
namespace c {
inline test_stub::FakeTask& fake_task(task_t task) {
  auto& s = test_stub::g_sched;
  return task == nullptr ? s.tasks[s.current] : s.tasks[reinterpret_cast<std::uintptr_t>(task) - 1];
}
inline task_t task_get_current() { return reinterpret_cast<task_t>(static_cast<std::uintptr_t>(test_stub::g_sched.current + 1)); }
inline std::uint32_t task_get_priority(task_t task) {
  test_stub::g_sched.calls.push_back("get");
  return fake_task(task).effective;
}
inline void task_set_priority(task_t task, std::uint32_t priority) {
  test_stub::g_sched.calls.push_back("set:" + std::to_string(priority));
  if (test_stub::g_sched.ignore_sets > 0) {
    test_stub::g_sched.ignore_sets--;
    return;
  }
  test_stub::FakeTask& t = fake_task(task);
  bool inherited = t.effective != t.base;
  t.base = priority;
  if (!inherited) t.effective = priority;
}
inline task_t task_get_by_name(const char* name) {
  bool found = test_stub::g_sched.daemon_present && std::strcmp(name, "PROS System Daemon") == 0;
  return found ? reinterpret_cast<task_t>(static_cast<std::uintptr_t>(4)) : nullptr;
}
}  // namespace c

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

// EZ-Template takes this through ez::Lock, which only needs take()/give(). lock()/unlock() are kept so a test can
// still hold it with a std::lock_guard.
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
  if (test_stub::g_clock.on_delay != nullptr) test_stub::g_clock.on_delay();
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
