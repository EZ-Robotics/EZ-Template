// A real, corpus-confirmed pattern: a team calls pid_wait_until() for an early checkpoint on a drive, then calls pid_wait() right after on
// that same still-running motion ("until then wait").  Both calls share the same leftPID/rightPID objects.
//
// When a wait ends on a velocity exit, PID::timers_reset() clears the exit timers and the velocity exit's arming (arm_timer,
// velocity_armed), so the next wait on the same motion arms the velocity exit again: it is armed once the robot has moved faster than
// 5 units/s, or after VELOCITY_ARM_FALLBACK (1000 ms) if it never does.  A pinned robot that drove normally for a while and then hit a
// wall therefore pays the 1000 ms arming again on every wait call chained onto that motion after the pin, plus velocity_exit_time.  This
// test checks that this second wait still ends, and that it ends within that arming plus velocity_exit_time.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Drives normally toward the 24in target for the first 20 passes (closing steadily at 50 in/s, well above the 5 in/s that arms the velocity
// exit), then pins hard at 14in for the rest of the test: error frozen well outside small/big windows, `cur` jittering by +-0.05 every other
// pass so the robot reads as stopped over the 30 ms speed window.
//
// Also advances the fake drive encoders in step with the scripted error, the same as a real robot's would: on real hardware, compute_error()
// is always fed FROM drive_sensor_left()/right(), so the two never drift apart the way they would if only PID.error were written here.
void drives_then_pins(Drive& c, int n) {
  bool pinned = n > 20;
  double e = pinned ? 14.0 : std::fmax(14.0, 24.0 - 0.5 * n);
  double jitter = (n % 2 == 0) ? 0.05 : -0.05;
  double cur = pinned ? jitter : -0.5 * std::fmin((double)n, 20.0);
  c.leftPID.compute_error(e, cur);
  c.rightPID.compute_error(e, cur);
  std::int32_t ticks = (std::int32_t)((24.0 - e) * c.drive_tick_per_inch());
  c.left_motors[0].fake().position = ticks;
  c.right_motors[0].fake().position = ticks;
}

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  drives_then_pins(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("chained wait calls on a drive that moved before pinning arm the velocity exit again on each call") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);

  g_chassis = &chassis;
  g_pass = 0;
  drives_then_pins(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;

  // First call: an early checkpoint well short of the real 24in target, so a genuine stall here must report interfered=true.  The robot
  // drives normally through pass 20, then pins; the velocity exit is armed by that movement, so it only waits out the speed window and
  // velocity_exit_time.
  test_stub::g_clock.delay_calls_until_stop = 3000;
  bool returned_1 = true;
  try {
    chassis.pid_wait_until(12.0);
  } catch (test_stub::StopLoop&) {
    returned_1 = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  int passes_after_first = g_pass;
  bool interfered_after_first = chassis.interfered;

  // Second call, immediately after, on the exact same still-running motion (no new pid_*_set() in between) -- the real "until then wait"
  // pattern.  The first call's exit cleared the arming, and the robot has been pinned, unresolved, ever since: the velocity exit arms
  // again through its 1000 ms fallback (100 passes) and then needs velocity_exit_time (500 ms, 50 passes) to confirm the same pin.
  test_stub::g_clock.delay_calls_until_stop = 3000;
  bool returned_2 = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned_2 = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  int passes_in_second_call = g_pass - passes_after_first;

  MESSAGE("passes_after_first=" << passes_after_first << " passes_in_second_call=" << passes_in_second_call
                                << " interfered_after_first=" << interfered_after_first << " interfered_after_second=" << chassis.interfered);

  CHECK(returned_1);
  CHECK(returned_2);
  CHECK(interfered_after_first);
  CHECK(chassis.interfered);

  // 1000 ms arming fallback + 500 ms velocity_exit_time is 150 passes, plus slack.
  CHECK(passes_in_second_call < 1000 / 10 + 500 / 10 + 10);
}
