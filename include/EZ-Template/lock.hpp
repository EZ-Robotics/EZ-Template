/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "pros/rtos.hpp"

#if defined(__GNUC__)
#define EZ_PRINTF_FORMAT(format_index, first_argument) __attribute__((format(printf, format_index, first_argument)))
#else
#define EZ_PRINTF_FORMAT(format_index, first_argument)
#endif

/**
 * Locks that a competition mode change cannot leave held.
 *
 * WHY THIS EXISTS
 *
 * PROS runs autonomous(), opcontrol() and disabled() as one task, and its system daemon deletes that task, wherever it
 * happens to be, every time the competition mode changes (field control, a competition switch, the robot being
 * disabled). task_delete does not release a mutex the task was holding. If the task was inside something EZ-Template
 * guards with a mutex, that mutex stays taken for good, and every other task that needs it waits forever. For the
 * chassis lock that means the background drive task stops, so odometry and every PID motion are dead until the brain
 * restarts.
 *
 * HOW THIS PREVENTS IT
 *
 * The system daemon runs at TASK_PRIORITY_MAX - 2, and a task cannot preempt one running at a higher priority. So a
 * KillSafeGuard raises its task to TASK_PRIORITY_MAX - 1 before taking the lock and puts the priority back after
 * giving it. While the task holds the lock and does not block, the daemon cannot run, so it cannot delete the task. A
 * task deleted while it is only waiting for the lock holds nothing, which is safe.
 *
 * THE RULES
 *
 *  1. Take every lock in the library with an ez::KillSafeGuard, or an ez::PlainGuard for a task PROS never deletes
 *     (ez_auto_task, the LVGL callback). Do not use pros::Mutex, pros::RecursiveMutex, std::lock_guard, or
 *     take()/give() on a bare mutex anywhere else in the library.
 *  2. Nothing inside a guard may block: no printf, no std::cout, no pros::delay, no pros::screen, no other lock. A
 *     task that blocks lets the daemon run, and the daemon may then delete it while it holds the lock. To print from
 *     inside a guard, call lock.print_after_unlock(...): the text is printed once the outermost guard has released.
 *
 * A host test (test/test_locking_rule.cpp) reads the library source and fails the build if either rule is broken.
 *
 * WHAT IT CANNOT DO
 *
 *  - It depends on the system daemon staying below TASK_PRIORITY_MAX - 1.
 *  - A task running on an inherited priority cannot be shielded. task_set_priority changes the priority the task is
 *    using only when it is not inheriting one, so the raise does not take. The guard notices, puts the priority back at
 *    once, counts it (detail::stats.unshielded_raises), and takes the lock anyway. Putting it back stores the priority
 *    the task was using as its base, because the public API cannot read the base, so the task keeps the inherited
 *    priority after the inheritance ends. This happens when a task calls a chassis function while holding its own
 *    contended mutex. Do not do that.
 *  - Reading a sensor inside a guard can block on that sensor's port mutex, and the daemon takes every port mutex
 *    each time it runs. A setter that blocks there during the same 2 ms as a mode change is still exposed.
 *  - PROS's own printf and pros::screen mutexes can be orphaned by a mode change. No library can fix that.
 */
namespace ez {
namespace detail {

/// Turns every KillSafeGuard into a plain lock when false. For debugging the guard, not for teams.
inline constexpr bool KILL_SAFE_ENABLED = true;

/// The priority a guard raises its task to: the highest one PROS has, one above the system daemon.
inline constexpr std::uint32_t KILL_SAFE_PRIORITY = TASK_PRIORITY_MAX - 1;

/// The most text print_after_unlock can hold at once, including the terminating null. Longer text is cut off.
inline constexpr std::size_t LOCK_PRINT_BUFFER_SIZE = 256;

/// False until one of EZ-Template's tasks is running, which means the scheduler is. Before that (global
/// constructors), task_get_current() is not the caller, so changing "its" priority would change another task's.
inline std::atomic<bool> scheduler_running{false};

/// Call as the first line of every task EZ-Template creates.
inline void mark_scheduler_running() { scheduler_running.store(true, std::memory_order_relaxed); }

/// Counters that stay at zero in normal use. Meant for tests and for a robot's own debug display.
struct LockStats {
  std::atomic<std::uint32_t> unshielded_raises{0};   // guards that could not raise their task's priority
  std::atomic<std::uint32_t> restore_mismatches{0};  // guards whose priority did not read back as saved after the restore
  std::atomic<std::uint32_t> guarded_sections{0};    // guards that took and released their lock
};
inline LockStats stats;

/// Where queued text goes once its lock is released. Null means stdout. Tests replace it to capture the text.
inline void (*print_sink)(const char* text) = nullptr;
inline void emit(const char* text) {
  if (print_sink != nullptr)
    print_sink(text);
  else
    fputs(text, stdout);
}

}  // namespace detail

/// Lets host tests reach a Lock's private state. Defined in the tests, never in the library.
struct LockTestAccess;

template <typename M, bool Boost>
class Guard;

/**
 * A mutex plus who holds it and the text waiting to be printed. M is the mutex being wrapped: pros::RecursiveMutex for
 * the chassis lock, pros::Mutex for the screen line lock (which, being non-recursive, cannot be nested on). It needs
 * `bool take(std::uint32_t timeout_ms)` and `bool give()`. Take it with a guard, not directly.
 */
template <typename M>
class Lock {
 public:
  Lock() = default;
  Lock(const Lock&) = delete;
  Lock& operator=(const Lock&) = delete;

  /**
   * printf, but when the calling task holds this lock the text is held back and printed once the outermost guard has
   * released it, so a slow terminal can never keep the lock held. Output order is kept. A task that holds no guard
   * has nothing to protect, so its text is printed at once.
   */
  void print_after_unlock(const char* format, ...) EZ_PRINTF_FORMAT(2, 3) {
    std::va_list arguments;
    va_start(arguments, format);
    if (held_by_caller()) {
      std::size_t room = detail::LOCK_PRINT_BUFFER_SIZE - pending_length_;  // includes the terminating null
      int wanted = room > 1 ? vsnprintf(pending_ + pending_length_, room, format, arguments) : 0;
      if (wanted > 0) pending_length_ += static_cast<std::size_t>(wanted) < room ? static_cast<std::size_t>(wanted) : room - 1;
    } else {
      char text[detail::LOCK_PRINT_BUFFER_SIZE];
      vsnprintf(text, sizeof(text), format, arguments);
      detail::emit(text);
    }
    va_end(arguments);
  }

 private:
  template <typename, bool>
  friend class Guard;
  friend struct LockTestAccess;

  // owner_ is read by tasks that do not hold the lock, so it is atomic. depth_ and the pending text are only touched
  // by the holder, and depth_ is only read after owner_ says the reader is the holder.
  bool held_by_caller() const { return owner_.load(std::memory_order_relaxed) == pros::c::task_get_current() && depth_ > 0; }

  M mutex_;
  std::atomic<pros::task_t> owner_{nullptr};  // the task holding the lock, or null
  int depth_ = 0;                             // guards the holder has open on it
  char pending_[detail::LOCK_PRINT_BUFFER_SIZE] = {};
  std::size_t pending_length_ = 0;
};

/**
 * Holds a Lock for the rest of the enclosing scope, like std::lock_guard. Not copyable or movable: it belongs on the
 * stack. Use the aliases below, not this directly.
 *
 * With Boost, the guard also raises its task's priority for the length of the lock (see the top of this file). Nothing
 * is raised until the scheduler is running, and a guard nested inside another one on the same lock does not raise
 * again.
 */
template <typename M, bool Boost>
class Guard {
 public:
  explicit Guard(Lock<M>& lock) : lock_(lock) {
    if constexpr (Boost) {
      if (detail::scheduler_running.load(std::memory_order_relaxed) && !lock_.held_by_caller()) raise();
    }
    locked_ = lock_.mutex_.take(TIMEOUT_MAX);
    if (locked_) {
      lock_.owner_.store(pros::c::task_get_current(), std::memory_order_relaxed);
      lock_.depth_++;
    }
  }

  ~Guard() {
    char text[detail::LOCK_PRINT_BUFFER_SIZE];
    std::size_t length = 0;
    if (locked_) {
      if (--lock_.depth_ == 0) {
        // The outermost guard. Copy the queued text out while the lock is still held, because another task may write
        // to it the moment the lock is given, and clear the owner first for the same reason.
        length = lock_.pending_length_;
        std::memcpy(text, lock_.pending_, length);
        lock_.pending_length_ = 0;
        lock_.owner_.store(nullptr, std::memory_order_relaxed);
      }
      lock_.mutex_.give();
    }
    if constexpr (Boost) {
      if (boosted_) {
        pros::c::task_set_priority(nullptr, saved_priority_);
        if (pros::c::task_get_priority(nullptr) != saved_priority_) detail::stats.restore_mismatches.fetch_add(1, std::memory_order_relaxed);
      }
    }
    if (locked_) detail::stats.guarded_sections.fetch_add(1, std::memory_order_relaxed);
    if (length > 0) {
      text[length] = '\0';
      detail::emit(text);  // after the lock and the priority are both back, so printing can block without harm
    }
  }

  Guard(const Guard&) = delete;
  Guard& operator=(const Guard&) = delete;

 private:
  void raise() {
    std::uint32_t before = pros::c::task_get_priority(nullptr);
    if (before >= detail::KILL_SAFE_PRIORITY) return;  // already at the top, or inheriting it: raising would strand the base there
    pros::c::task_set_priority(nullptr, detail::KILL_SAFE_PRIORITY);
    if (pros::c::task_get_priority(nullptr) == detail::KILL_SAFE_PRIORITY) {
      saved_priority_ = before;
      boosted_ = true;
      return;
    }
    pros::c::task_set_priority(nullptr, before);  // the raise did not take, and the base is now 15: put it back at once
    detail::stats.unshielded_raises.fetch_add(1, std::memory_order_relaxed);
  }

  Lock<M>& lock_;
  bool locked_ = false;
  bool boosted_ = false;
  std::uint32_t saved_priority_ = 0;
};

template <typename M>
using PlainGuard = Guard<M, false>;
template <typename M>
using KillSafeGuard = Guard<M, detail::KILL_SAFE_ENABLED>;

}  // namespace ez
