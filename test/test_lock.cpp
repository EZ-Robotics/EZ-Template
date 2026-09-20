// ez::RecoverableMutex and ez::LockGuard (include/EZ-Template/lock.hpp): a task that waits for a lock
// must never wait forever, even when the task that held it was deleted without releasing it, and no text
// is printed while a lock is held.
//
// The stub pros::Mutex (stub/pros/rtos.hpp) always succeeds and never waits, so it cannot model a lock
// whose owner is gone. These tests wrap a FakeMutex instead: one that can be told its owner is gone, or
// that a live owner lets go at a given fake time. Time is the fake clock, so waits cost nothing.
//
// Pinned here:
//  - a free lock is taken and released without waiting, and nested guards work
//  - a lock whose owner is gone is replaced after LOCK_RECOVER_AFTER_MS and the caller carries on
//  - a slow but living owner is not mistaken for a dead one, nor is one that releases and retakes the lock
//    faster than the waiter gets a turn
//  - a lock left held by a deleted task is replaced even while the task PROS started in its place keeps using it
//  - a waiter notices when another task already replaced the lock, and does not replace it a second time
//  - a lock taken just as it was replaced is handed back, not used
//  - a guard releases the lock it took even when the lock was replaced while it was held
//  - a lock stops replacing itself after LOCK_MAX_RECOVERIES
//  - text queued with print_after_unlock() prints after the lock is released, once, in order, and never
//    overruns its buffer
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "EZ-Template/lock.hpp"
#include "doctest.h"
#include "fake_hardware.hpp"

namespace {

// True while the test is acting as the task PROS starts in place of the one it deleted. That task is the same
// task as far as a recursive mutex can tell, so it can take a lock the deleted task left held.
bool g_as_adoptee = false;

// Stands in for a PROS mutex whose owner may have been deleted.
struct FakeMutex {
  bool orphaned = false;         // the owner is gone: take() can never succeed
  std::uint32_t free_at_ms = 0;  // a live owner that lets go at this fake time
  int held = 0;                  // takes minus gives
  int gives = 0;
  bool adoptee_reenters = false;  // an orphaned lock still lets the replacement task in (see g_as_adoptee)
  std::function<void()> on_failed_take;        // runs once, inside a take() that times out
  std::function<void()> on_every_failed_take;  // runs inside every take() that times out
  std::function<void()> on_take_success;       // runs once, just after a take() succeeds

  bool blocked() const { return (orphaned && !(adoptee_reenters && g_as_adoptee)) || pros::millis() < free_at_ms; }

  bool take(std::uint32_t timeout_ms) {
    if (blocked()) {
      pros::delay(timeout_ms);
      if (blocked()) {
        if (on_failed_take) std::exchange(on_failed_take, nullptr)();
        if (on_every_failed_take) on_every_failed_take();
        return false;
      }
    }
    held++;
    if (on_take_success) std::exchange(on_take_success, nullptr)();
    return true;
  }
  bool give() {
    held--;
    gives++;
    return true;
  }
};

using Lock = ez::RecoverableMutex<FakeMutex>;

// Every test starts from a fresh fake clock that throws after 1000 waits. A lock that stopped recovering
// would otherwise wait forever and hang the whole suite; this makes it fail the test instead.
void fresh_clock() {
  test_stub::reset_all();
  test_stub::g_clock.delay_calls_until_stop = 1000;
}

// Captures what the lock prints, and how many holders it still had when it did.
std::vector<std::string> g_printed;
std::vector<int> g_held_when_printed;
const FakeMutex* g_watched = nullptr;

void capture_text(const char* text) {
  g_printed.push_back(text);
  g_held_when_printed.push_back(g_watched != nullptr ? g_watched->held : -1);
}

// Installs the capture for one test and removes it again even if the test fails.
struct CaptureText {
  explicit CaptureText(const FakeMutex* watched) {
    g_printed.clear();
    g_held_when_printed.clear();
    g_watched = watched;
    ez::lock_print_sink = capture_text;
  }
  ~CaptureText() {
    ez::lock_print_sink = nullptr;
    g_watched = nullptr;
  }
};

}  // namespace

namespace ez {
// Reaches the parts of RecoverableMutex a test needs but the library does not expose.
struct LockTestAccess {
  using Slot = Lock::Slot;
  static Slot* current(Lock& lock) { return lock.current_.load(); }
  static Slot* recover(Lock& lock, Slot* stale) { return lock.recover(stale, LOCK_RECOVER_AFTER_MS); }
};
}  // namespace ez

using ez::LockGuard;
using ez::LockTestAccess;
using Slot = ez::LockTestAccess::Slot;

TEST_CASE("a free lock is taken and released without waiting") {
  fresh_clock();
  Lock lock("test");
  Slot* slot = LockTestAccess::current(lock);
  std::uint32_t before = pros::millis();

  {
    LockGuard guard(lock);
    CHECK(slot->mutex.held == 1);
    CHECK(slot->depth == 1);
  }

  CHECK(slot->mutex.held == 0);
  CHECK(slot->depth == 0);
  CHECK(pros::millis() == before);
  CHECK(lock.recovery_count() == 0);
}

TEST_CASE("nested guards on one lock nest and unwind") {
  fresh_clock();
  Lock lock("test");
  Slot* slot = LockTestAccess::current(lock);

  {
    LockGuard outer(lock);
    {
      LockGuard inner(lock);
      CHECK(slot->mutex.held == 2);
      CHECK(slot->depth == 2);
    }
    CHECK(slot->mutex.held == 1);
    CHECK(slot->depth == 1);
    CHECK(slot->releases == 0);  // letting go of a nested guard is not the lock being released
  }

  CHECK(slot->mutex.held == 0);
  CHECK(slot->depth == 0);
  CHECK(slot->releases == 1);
  CHECK(lock.recovery_count() == 0);
}

TEST_CASE("a lock whose owner is gone is replaced after the recovery time and the caller carries on") {
  fresh_clock();
  Lock lock("test");
  Slot* dead = LockTestAccess::current(lock);
  dead->mutex.orphaned = true;
  std::uint32_t before = pros::millis();

  {
    LockGuard guard(lock);
    Slot* fresh = LockTestAccess::current(lock);
    CHECK(fresh != dead);
    CHECK(fresh->mutex.held == 1);
    CHECK(dead->mutex.held == 0);
  }

  std::uint32_t waited = pros::millis() - before;
  CHECK(waited >= ez::LOCK_RECOVER_AFTER_MS);
  CHECK(waited <= ez::LOCK_RECOVER_AFTER_MS + ez::LOCK_WAIT_SLICE_MS);
  CHECK(lock.recovery_count() == 1);
  CHECK(dead->mutex.gives == 0);  // nobody ever releases a lock that was never taken
}

TEST_CASE("after a lock is replaced, later locks are immediate") {
  fresh_clock();
  Lock lock("test");
  LockTestAccess::current(lock)->mutex.orphaned = true;
  { LockGuard first(lock); }

  std::uint32_t before = pros::millis();
  { LockGuard second(lock); }

  CHECK(pros::millis() == before);
  CHECK(lock.recovery_count() == 1);
}

TEST_CASE("a slow but living owner is waited for, not replaced") {
  fresh_clock();
  Lock lock("test");
  Slot* slot = LockTestAccess::current(lock);
  std::uint32_t before = pros::millis();
  slot->mutex.free_at_ms = before + 300;  // lets go well inside the recovery time

  {
    LockGuard guard(lock);
    CHECK(LockTestAccess::current(lock) == slot);
  }

  CHECK(pros::millis() - before >= 300);
  CHECK(pros::millis() - before < ez::LOCK_RECOVER_AFTER_MS);
  CHECK(lock.recovery_count() == 0);
}

TEST_CASE("a live owner that keeps letting go and retaking the lock is never mistaken for a dead one") {
  fresh_clock();
  Lock lock("test");
  Slot* slot = LockTestAccess::current(lock);
  std::uint32_t before = pros::millis();

  // Every time this task looks, the owner has the lock again, because it releases and retakes it faster than
  // this task gets a turn (a task looping on setters does this). The owner only stops after four recovery times.
  slot->mutex.free_at_ms = before + 4 * ez::LOCK_RECOVER_AFTER_MS;
  slot->mutex.on_every_failed_take = [&] { slot->releases.fetch_add(1); };

  {
    LockGuard guard(lock);
    CHECK(LockTestAccess::current(lock) == slot);
  }

  CHECK(pros::millis() - before >= 4 * ez::LOCK_RECOVER_AFTER_MS);
  CHECK(lock.recovery_count() == 0);
}

TEST_CASE("a lock the deleted task left held is still replaced while its replacement task keeps using it") {
  fresh_clock();
  Lock lock("test");
  Slot* dead = LockTestAccess::current(lock);
  dead->mutex.orphaned = true;
  dead->mutex.adoptee_reenters = true;
  dead->mutex.held = 1;  // the deleted task's take, which nothing will ever give back
  dead->depth = 1;

  // While the chassis task waits, the task PROS started in place of the deleted one keeps calling setters. Each
  // call takes the orphaned lock and releases it again, but never gets it below the deleted task's own hold.
  int adoptee_calls = 0;
  dead->mutex.on_every_failed_take = [&] {
    g_as_adoptee = true;
    {
      LockGuard setter(lock);
      CHECK(dead->depth == 2);
    }
    g_as_adoptee = false;
    CHECK(dead->depth == 1);
    adoptee_calls++;
  };
  std::uint32_t before = pros::millis();

  {
    LockGuard chassis_task(lock);
    CHECK(LockTestAccess::current(lock) != dead);
  }

  std::uint32_t waited = pros::millis() - before;
  CHECK(waited >= ez::LOCK_RECOVER_AFTER_MS);
  CHECK(waited <= ez::LOCK_RECOVER_AFTER_MS + ez::LOCK_WAIT_SLICE_MS);
  CHECK(adoptee_calls >= 10);  // it really was using the lock the whole time
  CHECK(dead->releases == 0);  // and none of that counted as the lock being let go
  CHECK(lock.recovery_count() == 1);
}

TEST_CASE("a waiter moves to the new lock when another task already replaced it") {
  fresh_clock();
  Lock lock("test");
  Slot* dead = LockTestAccess::current(lock);
  dead->mutex.orphaned = true;
  // While this task waits, another task gives up on the same lock and replaces it.
  dead->mutex.on_failed_take = [&] { LockTestAccess::recover(lock, dead); };
  std::uint32_t before = pros::millis();

  {
    LockGuard guard(lock);
    CHECK(LockTestAccess::current(lock) != dead);
    CHECK(LockTestAccess::current(lock)->mutex.held == 1);
  }

  CHECK(lock.recovery_count() == 1);  // replaced once, by the other task, not again by this one
  CHECK(pros::millis() - before <= 2 * ez::LOCK_WAIT_SLICE_MS);
}

TEST_CASE("a lock taken just as it was replaced is handed back, not used") {
  fresh_clock();
  Lock lock("test");
  Slot* old = LockTestAccess::current(lock);
  // The take succeeds, but another task replaced the lock in the meantime.
  old->mutex.on_take_success = [&] { LockTestAccess::recover(lock, old); };

  {
    LockGuard guard(lock);
    Slot* fresh = LockTestAccess::current(lock);
    CHECK(fresh != old);
    CHECK(fresh->mutex.held == 1);
    CHECK(old->mutex.held == 0);  // taken, then given straight back
    CHECK(old->mutex.gives == 1);
    CHECK(old->depth == 0);
  }

  CHECK(lock.recovery_count() == 1);
}

TEST_CASE("a guard releases the lock it took even if the lock was replaced while it was held") {
  fresh_clock();
  Lock lock("test");
  Slot* first = LockTestAccess::current(lock);

  {
    LockGuard guard(lock);
    Slot* fresh = LockTestAccess::recover(lock, first);  // another task replaces it mid-hold
    CHECK(fresh != first);
  }

  CHECK(first->mutex.held == 0);
  CHECK(first->mutex.gives == 1);
  CHECK(first->depth == 0);
  CHECK(LockTestAccess::current(lock)->mutex.gives == 0);  // the replacement was never touched
}

TEST_CASE("a lock stops replacing itself after LOCK_MAX_RECOVERIES and waits like a plain mutex") {
  fresh_clock();
  Lock lock("test");

  for (std::uint32_t i = 0; i < ez::LOCK_MAX_RECOVERIES; i++) {
    LockTestAccess::current(lock)->mutex.orphaned = true;
    LockGuard guard(lock);
  }
  CHECK(lock.recovery_count() == ez::LOCK_MAX_RECOVERIES);

  // One more failure. It must keep waiting well past the recovery time instead of replacing again.
  LockTestAccess::current(lock)->mutex.orphaned = true;
  test_stub::g_clock.delay_calls_until_stop = 100;  // 100 waits of a slice each, then unwind the loop
  std::uint32_t before = pros::millis();
  CHECK_THROWS_AS([&] { LockGuard guard(lock); }(), test_stub::StopLoop);
  CHECK(pros::millis() - before > 2 * ez::LOCK_RECOVER_AFTER_MS);
  CHECK(lock.recovery_count() == ez::LOCK_MAX_RECOVERIES);

  test_stub::g_clock.delay_calls_until_stop = -1;
}

TEST_CASE("text queued under a lock prints after the lock is released") {
  fresh_clock();
  Lock lock("test");
  Slot* slot = LockTestAccess::current(lock);
  CaptureText capture(&slot->mutex);

  {
    LockGuard guard(lock);
    guard.print_after_unlock("started %d", 7);
    CHECK(g_printed.empty());  // nothing while the lock is held
  }

  REQUIRE(g_printed.size() == 1);
  CHECK(g_printed[0] == "started 7");
  CHECK(g_held_when_printed[0] == 0);  // and the lock was already free when it did
}

TEST_CASE("nested guards print once, in order, after the outermost one releases") {
  fresh_clock();
  Lock lock("test");
  Slot* slot = LockTestAccess::current(lock);
  CaptureText capture(&slot->mutex);

  {
    LockGuard outer(lock);
    {
      LockGuard inner(lock);
      inner.print_after_unlock("first ");
    }
    CHECK(g_printed.empty());  // the outer guard still holds the lock
    outer.print_after_unlock("second");
  }

  REQUIRE(g_printed.size() == 1);
  CHECK(g_printed[0] == "first second");
  CHECK(g_held_when_printed[0] == 0);
}

TEST_CASE("a lock that queued nothing prints nothing") {
  fresh_clock();
  Lock lock("test");
  CaptureText capture(&LockTestAccess::current(lock)->mutex);

  { LockGuard guard(lock); }

  CHECK(g_printed.empty());
}

TEST_CASE("queued text that does not fit is cut off, not overrun, and the queue is reusable") {
  fresh_clock();
  Lock lock("test");
  CaptureText capture(&LockTestAccess::current(lock)->mutex);

  {
    LockGuard guard(lock);
    std::string too_long(1000, 'x');
    guard.print_after_unlock("%s", too_long.c_str());
    guard.print_after_unlock("dropped");  // no room left
  }
  {
    LockGuard guard(lock);
    guard.print_after_unlock("again");
  }

  REQUIRE(g_printed.size() == 2);
  CHECK(g_printed[0].size() == ez::LOCK_PRINT_BUFFER_SIZE - 1);
  CHECK(g_printed[0].find('d') == std::string::npos);
  CHECK(g_printed[1] == "again");
}

TEST_CASE("text queued under a lock that was replaced meanwhile is still printed once") {
  fresh_clock();
  Lock lock("test");
  Slot* old = LockTestAccess::current(lock);
  CaptureText capture(&old->mutex);

  {
    LockGuard guard(lock);
    LockTestAccess::recover(lock, old);  // another task replaces the lock mid-hold
    guard.print_after_unlock("kept");
  }

  REQUIRE(g_printed.size() == 1);
  CHECK(g_printed[0] == "kept");
  CHECK(g_held_when_printed[0] == 0);
}
