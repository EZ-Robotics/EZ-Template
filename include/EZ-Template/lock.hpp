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
 * Locks that can never be left held forever.
 *
 * WHY THIS EXISTS
 *
 * PROS runs autonomous(), opcontrol() and disabled() as one task, and deletes that task wherever it
 * happens to be every time the competition mode changes (field control, a competition switch, or the
 * robot being disabled). Deleting a task does not release a mutex it was holding. If the deleted task was
 * inside something EZ-Template guards with a mutex, that mutex stays taken for good and every other task
 * that needs it waits forever. For the chassis lock that means the background drive task stops running, so
 * odometry and every PID motion are dead until the brain restarts. This was reproduced on a real robot.
 *
 * THE RULES
 *
 *  1. No task may wait forever on a lock. Every lock in EZ-Template is an ez::RecoverableMutex, taken with
 *     an ez::LockGuard. Do not use pros::Mutex, pros::RecursiveMutex, std::lock_guard, or take()/give() on a
 *     bare mutex anywhere else in the library.
 *  2. Nothing slow happens while a lock is held. No printf, no delay, no waiting on hardware. To print from
 *     inside a lock, call guard.print_after_unlock(...): the text is printed once the outermost guard on
 *     that lock has released it.
 *
 * A host test (test/test_locking_rule.cpp) reads the library source and fails the build if either rule is broken.
 *
 * HOW RECOVERY WORKS
 *
 * A task waiting for the lock wakes every LOCK_WAIT_SLICE_MS. If the lock has stayed unavailable for
 * LOCK_RECOVER_AFTER_MS, the task holding it is gone (no legitimate hold in EZ-Template lasts anywhere near
 * that long), so the waiter installs a fresh lock and carries on. A lock that is still held by a task that no
 * longer exists guards nothing, which is why replacing it is safe.
 *
 *  - Other waiters notice the replacement on their next wake and move to the new lock.
 *  - A guard always releases the lock it actually took, even if that lock was replaced while it was held.
 *  - A replaced lock is never freed while the RecoverableMutex exists. Its owner may still be blocked on it.
 *  - A lock is replaced at most LOCK_MAX_RECOVERIES times. After that it behaves like a plain mutex, so a
 *    system that keeps failing cannot leak memory without bound.
 *
 * If a task really is alive and holds a lock for longer than LOCK_RECOVER_AFTER_MS, recovery lets two tasks
 * touch the guarded state at once. Rule 2 is what keeps that from happening.
 */
namespace ez {

/// How long a task waits for a lock before deciding the task holding it no longer exists.
inline constexpr std::uint32_t LOCK_RECOVER_AFTER_MS = 500;

/// A waiting task wakes this often to check whether another task already replaced the lock.
inline constexpr std::uint32_t LOCK_WAIT_SLICE_MS = 25;

/// The most times a single lock will be replaced.
inline constexpr std::uint32_t LOCK_MAX_RECOVERIES = 8;

/// The most text that can be queued by print_after_unlock() on one lock at a time. Longer text is cut off.
inline constexpr std::size_t LOCK_PRINT_BUFFER_SIZE = 256;

/// Where queued text goes once the lock is released. Null means stdout. Tests replace it to capture the text.
inline void (*lock_print_sink)(const char* text) = nullptr;

/// Lets host tests reach private parts of RecoverableMutex. Defined in the tests, never in the library.
struct LockTestAccess;

template <typename MutexType>
class LockGuard;

/**
 * A mutex whose waiters recover if the task holding it stops existing.
 *
 * MutexType is the mutex being wrapped: pros::RecursiveMutex for the chassis lock, pros::Mutex for the
 * screen lock. It needs `bool take(std::uint32_t timeout_ms)` and `bool give()`. Take it with ez::LockGuard,
 * not directly.
 */
template <typename MutexType>
class RecoverableMutex {
 public:
  /**
   * \param name
   *        Shown in the message printed when this lock is replaced, e.g. "chassis".
   */
  explicit RecoverableMutex(const char* name = "") : name_(name) { current_.store(new Slot()); }

  RecoverableMutex(const RecoverableMutex&) = delete;
  RecoverableMutex& operator=(const RecoverableMutex&) = delete;

  ~RecoverableMutex() {
    delete current_.load();
    std::uint32_t used = recoveries_.load();
    for (std::uint32_t i = 0; i < used && i < LOCK_MAX_RECOVERIES; i++) delete retired_[i];
  }

  /// How many times this lock has been replaced.
  std::uint32_t recovery_count() const { return recoveries_.load(); }

 private:
  friend class LockGuard<MutexType>;
  friend struct LockTestAccess;

  /// One generation of the lock. When the lock is replaced the whole slot is, so a replacement never
  /// inherits a stale depth or queued text from a task that no longer exists.
  struct Slot {
    MutexType mutex;
    int depth = 0;                   // guards currently holding this slot. Only the holder touches it.
    std::size_t pending_length = 0;  // text queued by print_after_unlock(), printed after the last unlock
    char pending[LOCK_PRINT_BUFFER_SIZE] = {};
  };

  /**
   * Takes the lock. Never waits forever: if the lock's owner is gone the lock is replaced and this returns
   * holding the new one.
   *
   * \return the slot that was actually taken. Pass it to unlock().
   */
  [[nodiscard]] Slot* lock() {
    Slot* slot = current_.load();
    std::uint32_t waiting_since = pros::millis();

    while (true) {
      if (slot->mutex.take(LOCK_WAIT_SLICE_MS)) {
        if (current_.load() == slot) {
          slot->depth++;
          return slot;
        }

        // The lock was replaced while this task was waiting. What was just taken no longer guards
        // anything, so hand it back and use the current one.
        slot->mutex.give();
        slot = current_.load();
        waiting_since = pros::millis();
        continue;
      }

      Slot* latest = current_.load();
      if (latest != slot) {  // another task already recovered
        slot = latest;
        waiting_since = pros::millis();
        continue;
      }

      std::uint32_t waited = pros::millis() - waiting_since;
      if (waited >= LOCK_RECOVER_AFTER_MS) {
        slot = recover(slot, waited);
        waiting_since = pros::millis();
      }
    }
  }

  /**
   * Releases a slot returned by lock(). The outermost release also prints any text queued while the lock
   * was held, after the lock is free.
   */
  void unlock(Slot* slot) {
    if (--slot->depth > 0) {
      slot->mutex.give();
      return;
    }

    // Copy the text out first: once the lock is given back another task may start writing to the slot.
    char text[LOCK_PRINT_BUFFER_SIZE];
    std::size_t length = slot->pending_length;
    if (length > 0) std::memcpy(text, slot->pending, length);
    slot->pending_length = 0;

    slot->mutex.give();

    if (length > 0) {
      text[length < LOCK_PRINT_BUFFER_SIZE ? length : LOCK_PRINT_BUFFER_SIZE - 1] = '\0';
      if (lock_print_sink != nullptr)
        lock_print_sink(text);
      else
        fputs(text, stdout);
    }
  }

  /// Queues text on a slot the caller holds, to be printed after the outermost unlock.
  static void queue(Slot* slot, const char* format, std::va_list arguments) {
    std::size_t room = LOCK_PRINT_BUFFER_SIZE - slot->pending_length;  // includes the terminating NUL
    if (room <= 1) return;
    int wanted = vsnprintf(slot->pending + slot->pending_length, room, format, arguments);
    if (wanted <= 0) return;
    slot->pending_length += static_cast<std::size_t>(wanted) < room ? static_cast<std::size_t>(wanted) : room - 1;
  }

  /// Replaces `stale` with a fresh slot. Returns the slot to use from now on.
  Slot* recover(Slot* stale, std::uint32_t waited_ms) {
    if (recoveries_.load() >= LOCK_MAX_RECOVERIES) {
      if (!gave_up_warned_.exchange(true))
        printf("EZ-Template: the %s lock keeps getting stuck. It has been replaced %lu times and will not be replaced again.\n", name_, (unsigned long)LOCK_MAX_RECOVERIES);
      return stale;
    }

    Slot* fresh = new Slot();
    Slot* expected = stale;
    if (!current_.compare_exchange_strong(expected, fresh)) {
      delete fresh;  // another task replaced it first
      return expected;
    }

    std::uint32_t index = recoveries_.fetch_add(1);
    if (index < LOCK_MAX_RECOVERIES) retired_[index] = stale;  // kept, not freed: its owner may still be blocked on it

    printf("EZ-Template: the %s lock was still held %lu ms after the task holding it stopped running. This happens when a task is deleted while holding a lock, as field control does when the competition mode changes. The lock was replaced so nothing waits forever.\n", name_, (unsigned long)waited_ms);
    return fresh;
  }

  std::atomic<Slot*> current_{nullptr};
  Slot* retired_[LOCK_MAX_RECOVERIES] = {};
  std::atomic<std::uint32_t> recoveries_{0};
  std::atomic<bool> gave_up_warned_{false};
  const char* name_;
};

/**
 * Holds a RecoverableMutex for the rest of the enclosing scope, like std::lock_guard.
 *
 * \code
 * ez::LockGuard lock(drive_mutex);
 * lock.print_after_unlock("Started... Target Value: %.2f\n", target);  // never printf while holding a lock
 * \endcode
 */
template <typename MutexType>
class LockGuard {
 public:
  explicit LockGuard(RecoverableMutex<MutexType>& mutex) : owner_(mutex), slot_(mutex.lock()) {}
  ~LockGuard() { owner_.unlock(slot_); }

  LockGuard(const LockGuard&) = delete;
  LockGuard& operator=(const LockGuard&) = delete;

  /**
   * printf, but the text is held back and printed once the outermost guard on this lock has released it, so
   * a slow terminal can never keep the lock held. Output order is kept. Nested guards on one lock print
   * together, when the last of them releases.
   */
  void print_after_unlock(const char* format, ...) EZ_PRINTF_FORMAT(2, 3) {
    std::va_list arguments;
    va_start(arguments, format);
    RecoverableMutex<MutexType>::queue(slot_, format, arguments);
    va_end(arguments);
  }

 private:
  RecoverableMutex<MutexType>& owner_;
  typename RecoverableMutex<MutexType>::Slot* slot_;  // the lock actually taken, which may since have been replaced
};

}  // namespace ez
