// Promise 5: a guard raises its task to priority 15 while it holds a lock, so a PROS mode change can't delete it mid-lock,
// and puts the priority back after. The host has no kernel: the scheduler here is a fake that follows PROS's priority rules,
// so these check what the guard asks for, not that 15 is safe on a real brain.
#include <cstdint>
#include <string>

#include "../doctest.h"
#include "EZ-Template/lock.hpp"
#include "lock_test_access.hpp"

using namespace ez;
using test_stub::g_sched;

namespace {

struct FakeMutex {
  bool take_result = true;
  int takes = 0, gives = 0;
  bool take(std::uint32_t) {
    takes++;
    return take_result;
  }
  bool give() {
    gives++;
    return true;
  }
};
using TestLock = Lock<FakeMutex>;

// The task's base priority when no guard is held. The fake scheduler starts every task at 8.
const std::uint32_t OWN_PRIORITY = 8;

std::string printed;
std::uint32_t priority_when_printed = 0;
int gives_when_printed = -1;
FakeMutex* printing_mutex = nullptr;
void sink(const char* text) {
  printed += text;
  priority_when_printed = g_sched.tasks[g_sched.current].effective;
  if (printing_mutex != nullptr) gives_when_printed = printing_mutex->gives;
}

void reset(bool scheduler_running) {
  test_stub::reset_all();
  detail::scheduler_running.store(false);
  if (scheduler_running) detail::mark_scheduler_running();
  printed.clear();
  printing_mutex = nullptr;
  detail::print_sink = sink;
}

struct SinkOff {
  ~SinkOff() { detail::print_sink = nullptr; }
};

}  // namespace

TEST_CASE("Promise 5: a guard holds the lock at priority 15, then gives back every take at its own priority") {
  reset(true);
  SinkOff off;
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> guard(lock);
    CHECK(LockTestAccess::mutex(lock).takes == 1);
    CHECK(g_sched.tasks[0].effective == detail::KILL_SAFE_PRIORITY);
    { KillSafeGuard<FakeMutex> nested(lock); }                        // a setter calling another setter
    CHECK(g_sched.tasks[0].effective == detail::KILL_SAFE_PRIORITY);  // the inner guard letting go doesn't drop it early
  }
  CHECK(LockTestAccess::mutex(lock).gives == LockTestAccess::mutex(lock).takes);
  CHECK(g_sched.tasks[0].effective == OWN_PRIORITY);
}

TEST_CASE("Promise 5: before the scheduler runs, in initialize(), a guard only locks") {
  reset(false);
  SinkOff off;
  TestLock lock;
  { KillSafeGuard<FakeMutex> guard(lock); }
  CHECK(LockTestAccess::mutex(lock).gives == 1);
  CHECK(g_sched.calls.empty());
}

TEST_CASE("Promise 5: a lock that can't be taken isn't given back, and the priority still comes back down") {
  reset(true);
  SinkOff off;
  TestLock lock;
  LockTestAccess::mutex(lock).take_result = false;
  { KillSafeGuard<FakeMutex> guard(lock); }
  CHECK(LockTestAccess::mutex(lock).gives == 0);
  CHECK(g_sched.tasks[0].effective == OWN_PRIORITY);
}

TEST_CASE("Promise 5: text from inside a guard prints after the lock is free and the priority is back, in order") {
  reset(true);
  SinkOff off;
  TestLock lock;
  printing_mutex = &LockTestAccess::mutex(lock);
  {
    KillSafeGuard<FakeMutex> outer(lock);
    lock.print_after_unlock("a");
    {
      KillSafeGuard<FakeMutex> inner(lock);
      lock.print_after_unlock("b%d", 1);
    }
    CHECK(printed.empty());
  }
  CHECK(printed == "ab1");
  CHECK(gives_when_printed == LockTestAccess::mutex(lock).takes);  // after the outermost guard gave the lock back
  CHECK(priority_when_printed == OWN_PRIORITY);

  // Printed once, not again by the next guard
  { KillSafeGuard<FakeMutex> again(lock); }
  CHECK(printed == "ab1");

  // Outside any guard it prints at once
  lock.print_after_unlock(" now");
  CHECK(printed == "ab1 now");
}

// Promise 2: a message longer than the buffer is cut off instead of writing past it
TEST_CASE("Promise 2: text longer than the print buffer is cut off, not overrun") {
  reset(false);
  SinkOff off;
  TestLock lock;
  {
    PlainGuard<FakeMutex> guard(lock);
    lock.print_after_unlock("%s", std::string(400, 'x').c_str());
  }
  CHECK(printed == std::string(detail::LOCK_PRINT_BUFFER_SIZE - 1, 'x'));
}
