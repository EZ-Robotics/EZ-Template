// A mechanism whose compute() cadence is a multiple of its sensor's own refresh cadence produces repeated raw values on most computes, with the real jump
// landing on whichever compute happens to follow a refresh -- which need not be the specific compute immediately before a given poll -- even while
// genuinely moving continuously the whole time. This is a real, deterministic hardware pattern (V5 sensors update on a fixed bus cadence), not a
// contrived one. The velocity channel judges the sensor's net movement over the last 100 ms, not the last compute, so a real jump anywhere in that
// window is caught even when the last compute before the poll happened to land on a repeated reading. The speed only changes when a window closes, so a
// real stall is noticed one window late.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("VELOCITY_EXIT does not fire on a continuously-moving mechanism whose compute rate aliases its sensor's refresh rate") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 300, 0);  // velocity_exit_time=300ms only
  pid.velocity_sensor_main_exit_set(0.05);
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  // Sensor refreshes every 12ms; compute() runs every 4ms (3x per refresh window), so 2 of every 3
  // computes see a bit-identical raw value (derivative == 0) purely from cadence, not a real stop.
  // Poll every 12ms, offset onto the third compute of each window.
  auto sensor_reading = [](int t) { return static_cast<double>((t / 12) * 24); };  // 24 units/12ms
  bool exited = false;
  for (int t = 0; t <= 2000 && !exited; t += 1) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    if (t % 4 == 0) pid.compute(sensor_reading(t));
    if (t % 12 == 8) {
      if (pid.exit_condition() == VELOCITY_EXIT) exited = true;
    }
  }
  CHECK_FALSE(exited);
}

TEST_CASE("VELOCITY_EXIT still fires promptly on a genuine stall at matched compute/poll cadence") {
  PID pid;
  pid.exit_condition_set(0, 0, 0, 0, 300, 0);
  pid.velocity_sensor_main_exit_set(0.05);
  pid.target_set(10000);

  test_stub::g_clock.now_ms = 0;
  pid.compute(0);  // the window starts here; the move to 50 below makes the first window arm

  bool exited = false;
  int t_exit = -1;
  for (int t = 10; t <= 2000; t += 10) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);
    pid.compute(t == 10 ? 50.0 : 50.0);  // moves once, then holds perfectly still
    if (pid.exit_condition() == VELOCITY_EXIT) {
      exited = true;
      t_exit = t;
      break;
    }
  }
  CHECK(exited);
  // The jump on t=10 lands in the first speed window (closes t=100, arms), the first fully still window closes on t=200 where the timer holds 10 ms, and
  // it passes the configured 300 ms on t=500; one extra poll of slack.
  CHECK(t_exit <= 510);
}
