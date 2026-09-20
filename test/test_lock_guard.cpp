// ez::Lock, ez::PlainGuard and ez::KillSafeGuard (include/EZ-Template/lock.hpp).
//
// What these prove and what they cannot. The host has no kernel: pros::c::task_get_priority / task_set_priority
// are a fake scheduler (see test/fake_hardware.hpp) that follows PROS's rules for base and inherited priority,
// and the mutex is a fake. So these tests pin the guard's own logic: which calls it makes, in which order, and
// what it does when the answers are not the ones it hoped for. They cannot prove that a task at priority 15
// is really safe from PROS's task_delete. That needs the robot.
#include <string>
#include <type_traits>
#include <vector>

#include "EZ-Template/lock.hpp"
#include "doctest.h"
#include "lock_test_access.hpp"

using namespace ez;
using test_stub::g_sched;

namespace {

using Calls = std::vector<std::string>;

struct FakeMutex {
  bool take_result = true;
  int takes = 0;
  int gives = 0;
  const std::atomic<pros::task_t>* owner_probe = nullptr;  // what the lock's owner field held at the moment of the give
  pros::task_t owner_at_give = nullptr;

  bool take(std::uint32_t) {
    takes++;
    return take_result;
  }
  bool give() {
    gives++;
    if (owner_probe != nullptr) owner_at_give = owner_probe->load();
    return true;
  }
};

using TestLock = Lock<FakeMutex>;
using Access = LockTestAccess;

void reset() {
  test_stub::reset_all();
  detail::scheduler_running.store(false);
  detail::stats.unshielded_raises.store(0);
  detail::stats.restore_mismatches.store(0);
  detail::stats.guarded_sections.store(0);
  detail::print_sink = nullptr;
}

void scheduler_is_running() { detail::mark_scheduler_running(); }

// Text handed to the print sink, plus what the world looked like at the moment it arrived.
struct Printed {
  std::string text;
  int gives_when_printed = -1;
  std::uint32_t priority_when_printed = 0;
  FakeMutex* mutex = nullptr;
};
Printed printed;

void sink(const char* text) {
  printed.text += text;
  if (printed.mutex != nullptr) printed.gives_when_printed = printed.mutex->gives;
  printed.priority_when_printed = g_sched.tasks[g_sched.current].effective;
}

// Removes the sink when the test ends, so text printed by a later test goes to stdout again.
struct SinkScope {
  ~SinkScope() { detail::print_sink = nullptr; }
};

SinkScope capture_prints(FakeMutex* mutex = nullptr) {
  printed = Printed{};
  printed.mutex = mutex;
  detail::print_sink = sink;
  return {};
}

const Calls RAISE = {"get", "set:15", "get"};

}  // namespace

// D: stack only.
static_assert(!std::is_copy_constructible_v<KillSafeGuard<FakeMutex>>);
static_assert(!std::is_move_constructible_v<KillSafeGuard<FakeMutex>>);
static_assert(!std::is_copy_assignable_v<KillSafeGuard<FakeMutex>>);
static_assert(!std::is_move_assignable_v<KillSafeGuard<FakeMutex>>);
static_assert(!std::is_copy_constructible_v<PlainGuard<FakeMutex>>);
static_assert(!std::is_move_constructible_v<PlainGuard<FakeMutex>>);
static_assert(!std::is_copy_constructible_v<TestLock>);
static_assert(!std::is_move_constructible_v<TestLock>);
static_assert(std::is_same_v<KillSafeGuard<FakeMutex>, Guard<FakeMutex, detail::KILL_SAFE_ENABLED>>);
static_assert(detail::KILL_SAFE_PRIORITY == 15);

TEST_CASE("a guard takes the lock once and gives it back once") {
  reset();
  TestLock lock;
  {
    PlainGuard<FakeMutex> guard(lock);
    CHECK(Access::mutex(lock).takes == 1);
    CHECK(Access::mutex(lock).gives == 0);
  }
  CHECK(Access::mutex(lock).gives == 1);
}

TEST_CASE("the owner is the holding task, and is cleared before the lock is given") {
  reset();
  TestLock lock;
  Access::mutex(lock).owner_probe = &Access::owner_field(lock);
  {
    PlainGuard<FakeMutex> guard(lock);
    CHECK(Access::owner(lock) == pros::c::task_get_current());
    CHECK(Access::depth(lock) == 1);
    {
      PlainGuard<FakeMutex> nested(lock);
      CHECK(Access::depth(lock) == 2);
    }
    CHECK(Access::depth(lock) == 1);
    CHECK(Access::owner(lock) == pros::c::task_get_current());  // an inner guard letting go does not clear it
  }
  CHECK(Access::depth(lock) == 0);
  CHECK(Access::owner(lock) == nullptr);
  // Once another task can take the lock it will write its own name into the owner field, so ours must
  // already be gone by the time the lock is given.
  CHECK(Access::mutex(lock).owner_at_give == nullptr);
}

TEST_CASE("guarded sections are counted, one per guard") {
  reset();
  TestLock lock;
  {
    PlainGuard<FakeMutex> outer(lock);
    { PlainGuard<FakeMutex> inner(lock); }
  }
  CHECK(detail::stats.guarded_sections.load() == 2);
}

// A: no priority work until the scheduler is running.
TEST_CASE("before the scheduler is running a guard only locks") {
  reset();
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> guard(lock);
    CHECK(Access::mutex(lock).takes == 1);
  }
  CHECK(Access::mutex(lock).gives == 1);
  CHECK(g_sched.calls.empty());
}

TEST_CASE("once the scheduler is running a guard raises to 15 around the lock, then puts the priority back") {
  reset();
  scheduler_is_running();
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> guard(lock);
    CHECK(g_sched.calls == RAISE);
    CHECK(g_sched.tasks[0].effective == 15);
    CHECK(Access::mutex(lock).takes == 1);
  }
  CHECK(g_sched.calls == Calls{"get", "set:15", "get", "set:8", "get"});
  CHECK(g_sched.tasks[0].base == 8);
  CHECK(g_sched.tasks[0].effective == 8);
  CHECK(detail::stats.restore_mismatches.load() == 0);
  CHECK(detail::stats.unshielded_raises.load() == 0);
}

TEST_CASE("a guard acts on what it did at construction, not on the flag as it is at destruction") {
  reset();
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> before_the_scheduler(lock);
    scheduler_is_running();  // EZ's tasks start running while initialize() is inside this scope
  }
  CHECK(g_sched.calls.empty());  // it never raised, so it has nothing to put back

  reset();
  scheduler_is_running();
  {
    KillSafeGuard<FakeMutex> guard(lock);
    detail::scheduler_running.store(false);
  }
  CHECK(g_sched.tasks[0].effective == 8);  // it raised, so it puts the priority back whatever the flag says
}

// B: nesting.
TEST_CASE("a guard nested on the same lock does no priority work") {
  reset();
  scheduler_is_running();
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> outer(lock);
    CHECK(g_sched.calls == RAISE);
    {
      KillSafeGuard<FakeMutex> inner(lock);
      CHECK(g_sched.calls == RAISE);  // nothing new
      CHECK(g_sched.tasks[0].effective == 15);
    }
    CHECK(g_sched.tasks[0].effective == 15);  // the inner guard did not drop the outer one's priority
  }
  CHECK(g_sched.tasks[0].effective == 8);
}

TEST_CASE("a kill safe guard inside a plain guard does no priority work") {
  reset();
  scheduler_is_running();
  TestLock lock;
  {
    PlainGuard<FakeMutex> outer(lock);  // ez_auto_task
    KillSafeGuard<FakeMutex> inner(lock);
    CHECK(g_sched.calls.empty());
  }
  CHECK(g_sched.calls.empty());
}

TEST_CASE("a plain task on an inherited priority of 15 is not stranded at 15 by a nested guard") {
  reset();
  scheduler_is_running();
  g_sched.tasks[0] = {8, 15};  // base 8, inheriting 15 from a waiter
  TestLock lock;
  {
    PlainGuard<FakeMutex> outer(lock);
    KillSafeGuard<FakeMutex> inner(lock);
  }
  CHECK(g_sched.tasks[0].base == 8);
  CHECK(g_sched.tasks[0].effective == 15);
}

TEST_CASE("a task that does not hold the lock is not treated as nested, even while another task does") {
  reset();
  scheduler_is_running();
  auto sink_scope = capture_prints();
  TestLock lock;
  {
    PlainGuard<FakeMutex> holder(lock);  // task 0 holds it, depth 1
    g_sched.current = 1;

    lock.print_after_unlock("from task 1");
    CHECK(printed.text == "from task 1");  // task 1 holds nothing, so this printed now instead of queueing

    Access::mutex(lock).take_result = false;  // the fake lock is busy: task 1 raises, cannot take, and does not touch the bookkeeping
    {
      KillSafeGuard<FakeMutex> other(lock);
      CHECK(g_sched.calls == RAISE);  // it did priority work, so it was not mistaken for the holder
      CHECK(Access::owner(lock) == reinterpret_cast<pros::task_t>(1));
      CHECK(Access::depth(lock) == 1);
    }
    Access::mutex(lock).take_result = true;
    g_sched.current = 0;
  }
  CHECK(Access::owner(lock) == nullptr);
}

// C: check that the raise took.
TEST_CASE("a raise that did not take is put back at once, counted, and the lock is still taken") {
  reset();
  scheduler_is_running();
  g_sched.tasks[0] = {8, 12};  // on an inherited priority: set_priority changes the base but not what the task runs at
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> guard(lock);
    CHECK(g_sched.calls == Calls{"get", "set:15", "get", "set:12"});
    CHECK(Access::mutex(lock).takes == 1);
    CHECK(detail::stats.unshielded_raises.load() == 1);
  }
  CHECK(g_sched.calls == Calls{"get", "set:15", "get", "set:12"});  // the destructor did no priority work
  CHECK(Access::mutex(lock).gives == 1);
  CHECK(detail::stats.restore_mismatches.load() == 0);
  // The public API cannot read a task's base priority, so writing back what get returned stores the inherited 12 as
  // the base. Known and documented in lock.hpp: it is the least bad option, leaving it at 15 would strand the task.
  CHECK(g_sched.tasks[0].base == 12);
}

TEST_CASE("a task already at 15 is left alone") {
  reset();
  scheduler_is_running();
  TestLock lock;

  g_sched.tasks[0] = {15, 15};
  {
    KillSafeGuard<FakeMutex> guard(lock);
    CHECK(g_sched.calls == Calls{"get"});
  }
  CHECK(g_sched.calls == Calls{"get"});

  reset();
  scheduler_is_running();
  g_sched.tasks[0] = {8, 15};  // inherited 15: raising would store 15 as its base and strand it there
  {
    KillSafeGuard<FakeMutex> guard(lock);
  }
  CHECK(g_sched.calls == Calls{"get"});
  CHECK(g_sched.tasks[0].base == 8);
}

// K: leak counters.
TEST_CASE("a restore that does not read back as the saved priority is counted") {
  reset();
  scheduler_is_running();
  TestLock lock;
  {
    KillSafeGuard<FakeMutex> guard(lock);
    g_sched.ignore_sets = 1;  // the restore will not take
  }
  CHECK(detail::stats.restore_mismatches.load() == 1);
  CHECK(g_sched.tasks[0].effective == 15);
}

// D: check the take.
TEST_CASE("a take that fails leaves the bookkeeping alone and is not given back") {
  reset();
  TestLock lock;
  Access::mutex(lock).take_result = false;
  {
    PlainGuard<FakeMutex> guard(lock);
    CHECK(Access::depth(lock) == 0);
    CHECK(Access::owner(lock) == nullptr);
  }
  CHECK(Access::mutex(lock).gives == 0);
  CHECK(Access::depth(lock) == 0);
  CHECK(detail::stats.guarded_sections.load() == 0);
}

TEST_CASE("a kill safe guard whose take fails still puts its priority back") {
  reset();
  scheduler_is_running();
  TestLock lock;
  Access::mutex(lock).take_result = false;
  {
    KillSafeGuard<FakeMutex> guard(lock);
  }
  CHECK(g_sched.tasks[0].effective == 8);
  CHECK(Access::mutex(lock).gives == 0);
}

// J: the switch.
TEST_CASE("with the switch off a kill safe guard is a plain lock") {
  reset();
  scheduler_is_running();
  TestLock lock;
  {
    Guard<FakeMutex, false> guard(lock);  // what KillSafeGuard becomes when KILL_SAFE_ENABLED is false
    CHECK(Access::mutex(lock).takes == 1);
  }
  CHECK(g_sched.calls.empty());
  CHECK(Access::mutex(lock).gives == 1);
}

// print_after_unlock.
TEST_CASE("text is held until the outermost guard lets go, in order") {
  reset();
  TestLock lock;
  auto sink_scope = capture_prints();
  {
    PlainGuard<FakeMutex> outer(lock);
    lock.print_after_unlock("a");
    {
      PlainGuard<FakeMutex> inner(lock);
      lock.print_after_unlock("b%d", 1);
    }
    CHECK(printed.text.empty());  // the inner guard letting go is not the end of the lock
  }
  CHECK(printed.text == "ab1");
}

TEST_CASE("text is printed after the lock is given and after the priority is put back") {
  reset();
  scheduler_is_running();
  TestLock lock;
  auto sink_scope = capture_prints(&Access::mutex(lock));
  {
    KillSafeGuard<FakeMutex> guard(lock);
    lock.print_after_unlock("hello");
    CHECK(printed.text.empty());
  }
  CHECK(printed.text == "hello");
  CHECK(printed.gives_when_printed == 1);
  CHECK(printed.priority_when_printed == 8);
}

TEST_CASE("queued text is cleared once printed, so it is never printed twice") {
  reset();
  TestLock lock;
  auto sink_scope = capture_prints();
  {
    PlainGuard<FakeMutex> guard(lock);
    lock.print_after_unlock("a");
  }
  {
    PlainGuard<FakeMutex> guard(lock);
    lock.print_after_unlock("b");
  }
  CHECK(printed.text == "ab");
}

TEST_CASE("text printed by a task that holds no guard goes out at once") {
  reset();
  TestLock lock;
  auto sink_scope = capture_prints();
  lock.print_after_unlock("now %d", 7);
  CHECK(printed.text == "now 7");
}

TEST_CASE("text longer than the buffer is cut off, not overrun") {
  reset();
  TestLock lock;
  auto sink_scope = capture_prints();
  {
    PlainGuard<FakeMutex> guard(lock);
    lock.print_after_unlock("%s", std::string(400, 'x').c_str());
  }
  CHECK(printed.text == std::string(detail::LOCK_PRINT_BUFFER_SIZE - 1, 'x'));
}
