// Two tasks sharing one Drive: Task A starts a motion and waits on it. Task B's own pid_*_set()
// retargets A's still-running motion (a legitimate scenario A's retarget guard is meant to catch --
// A's wait correctly ends early reporting interfered=true). Task B then waits on its OWN new motion,
// which finishes completely cleanly with no problem of its own.
//
// Task B's setter clears interfered=false the moment it runs, before B's own motion has had any
// chance to fail. A's guard notices the retarget shortly after (at most one pass later) and writes
// interfered=true -- correctly describing A's own abandoned motion, but landing on the same
// Drive::interfered every wait reads and writes, with no way to tell whose motion it's talking about.
// That write reaches B's settle delay before B's own wait has any result of its own to report, so by
// the time B's wait evaluates its own motion, `interfered` already reads true for a completely
// unrelated reason.
//
// A caller gating its next motion on `!chassis.interfered` right after its own wait would wrongly
// treat B's successful, uninterrupted motion as a failure.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("a stale wait's late retarget notice does not leave a later, cleanly-finished motion reporting interfered=true") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Small exit only, short enough that both motions below settle well inside this test's pass
  // budget; big/velocity/mA off so nothing but the small-exit dwell timer can end either wait.
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);

  // Task A starts a 48in drive.
  chassis.pid_drive_set(48, 100);

  // Task A's own wait for that motion. Scripted so Task B's setter lands on pass 2 -- a normal,
  // well-past-the-first-settle-delay pass, so Task A's own guard reliably catches it (matching the
  // already-tested, already-correct shape in test_drive_retarget_guard.cpp; this call finishing
  // with interfered=true is expected, not the bug under test).
  int pass = 0;
  static Drive* s_chassis = &chassis;
  static int* s_pass = &pass;
  test_stub::g_clock.on_delay = +[] {
    ++(*s_pass);
    if (*s_pass == 2) {
      // Task B: starts its OWN new motion. This clears interfered=false -- correctly, since B's
      // motion hasn't run at all yet.
      s_chassis->pid_drive_set(6, 100);
    }
    s_chassis->leftPID.error = 10.0;
    s_chassis->rightPID.error = 10.0;
  };
  test_stub::g_clock.delay_calls_until_stop = 30;
  bool a_returned = true;
  try {
    chassis.pid_wait();  // Task A's stale wait: notices the retarget on pass 3, sets interfered=true, returns.
  } catch (test_stub::StopLoop&) {
    a_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  REQUIRE(a_returned);
  REQUIRE(chassis.interfered);  // sanity: Task A's own guard behaves exactly as already tested elsewhere

  // Task B's OWN code now waits for ITS OWN motion (already set up by the pid_drive_set(6, 100)
  // above, which is still current -- mode/target were never touched again since). Held cleanly
  // inside small_error the whole time: no stall, no retarget, nothing wrong with this motion at all.
  // A DriveTestAccess::refresh() call every pass, not a bare `.error =` write -- PID.cpp's small exit
  // timer only credits `error` when a real compute has landed since it last checked, so without this
  // small_exit could never actually latch and this test would time out for an unrelated reason (see
  // test_pid_wait_drive_latched_side_recheck.cpp's matching comment on the same gate).
  void (*hold_small)() = +[] {
    s_chassis->leftPID.error = 0.5;
    s_chassis->rightPID.error = 0.5;
    DriveTestAccess::refresh(s_chassis->leftPID);
    DriveTestAccess::refresh(s_chassis->rightPID);
  };
  hold_small();
  test_stub::g_clock.on_delay = hold_small;
  test_stub::g_clock.delay_calls_until_stop = 30;
  bool b_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    b_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("a_returned=" << a_returned << " b_returned=" << b_returned << " interfered_after_b=" << chassis.interfered);

  REQUIRE(b_returned);
  // Task B's motion had zero problems of its own -- it never even touched the retarget guard
  // (mode/target were stable across its whole wait) -- so interfered must read false here, not the
  // true left over from Task A's stale notice about a completely different motion.
  CHECK_FALSE(chassis.interfered);
}

// A narrower timing than the test above: A's stale write lands not before B's wait even starts, but
// DURING B's own wait's leading settle delay -- after B's wait has already taken its own baseline for
// this motion, but before B's own loop has produced any result of its own. A fix that only clears a
// foreign `interfered` value once, when a wait starts, would still pass the test above (there, A's
// write is long done before B's wait begins) but must fail here, since nothing re-checks after that
// point. The real fix has to keep disowning a foreign write for as long as B's own wait is still
// running, not just at the moment it started.
TEST_CASE("a stale wait's retarget notice landing during a later wait's own settle delay still doesn't stick") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);

  // Task A's motion, never waited on directly here -- only its generation matters, as the identity
  // a stale write would be tagged with.
  chassis.pid_drive_set(48, 100);
  std::uint32_t gen_a = DriveTestAccess::motion_generation(chassis);

  // Task B's own motion, current by the time its own wait below starts.
  chassis.pid_drive_set(6, 100);

  static Drive* s_chassis = &chassis;
  static std::uint32_t s_gen_a = gen_a;
  static int s_pass = 0;
  test_stub::g_clock.on_delay = +[] {
    ++s_pass;
    if (s_pass == 1) {
      // Simulates task A's own retarget guard writing interfered=true for ITS motion (gen_a) at the
      // exact moment B's wait is inside its own leading settle delay -- after B's wait already took
      // its baseline for gen_b, but before B's loop has run even once.
      s_chassis->interfered = true;
      DriveTestAccess::interfered_generation(*s_chassis) = s_gen_a;
    }
    s_chassis->leftPID.error = 0.5;
    s_chassis->rightPID.error = 0.5;
    DriveTestAccess::refresh(s_chassis->leftPID);
    DriveTestAccess::refresh(s_chassis->rightPID);
  };
  test_stub::g_clock.delay_calls_until_stop = 30;
  bool b_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    b_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("b_returned=" << b_returned << " interfered_after_b=" << chassis.interfered);

  REQUIRE(b_returned);
  CHECK_FALSE(chassis.interfered);
}
