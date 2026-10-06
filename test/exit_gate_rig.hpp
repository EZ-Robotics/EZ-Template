// Shared rig for the exit speed gate tests: a Drive on the sim with a ground truth trace, so what a wait returned on can be
// judged against how the simulated robot was really moving, not against what the library believes. Every speed these tests
// assert on comes from the sim (SimRobot::left()/right()/heading_deg()), never from the library's own tracker, or the check
// would be circular.
//
// Two archetypes live here and not in sim_physics.hpp:
//   classroom: light_fast with one motor a side on a 200 rpm cartridge at 6 kg. The robot that shows "fast robots exit while they
//              still have momentum" most clearly.
//   hunter:    see make_hunter(): PID constants tuned to limit cycle around the target.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

namespace gate {

inline sim::SimArchetype archetype_classroom() {
  sim::SimArchetype a = sim::archetype_light_fast();
  a.name = "classroom";
  a.motors_per_side = 1;
  a.cartridge_rpm = 200.0;
  a.mass_kg = 6.0;
  return a;
}

// Floors, restated for the asserts (exit_conditions.cpp keeps its own)
constexpr double FLOOR_DISTANCE = 1.5;
constexpr double FLOOR_ANGLE = 4.0;

template <typename F>
bool run_capped(F&& wait, int max_ticks) {
  test_stub::g_clock.delay_calls_until_stop = max_ticks;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

inline ez::Drive make_drive(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return ez::Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

struct Sample {
  double t_ms, left, right, heading, avg;
};

struct Rig {
  sim::SimArchetype a;
  ez::Drive chassis;
  sim::SimRobot sim;
  std::vector<Sample> trace;

  Rig(const sim::SimArchetype& arch, int passes = 1, bool noise = false, std::uint32_t seed = 1)
      : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{noise, seed}) {
    ez::DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim.passes_per_tick(passes);
    sim.after_pass = [this](int) { record(); };
  }

  // The robot's true state now, one sample per instant
  void record() {
    double t = sim.now_ms();
    if (!trace.empty() && trace.back().t_ms == t) return;
    trace.push_back({t, sim.left().position_in, sim.right().position_in, sim.heading_deg(), (sim.left().position_in + sim.right().position_in) / 2.0});
  }

  // Runs the wait for at most max_ticks polls. Returns whether it came back, and how long it took in ms
  template <typename F>
  bool wait(F&& f, int max_ticks, double* elapsed_ms = nullptr) {
    std::uint32_t t0 = pros::millis();
    bool ok = run_capped(std::forward<F>(f), max_ticks);
    if (elapsed_ms != nullptr) *elapsed_ms = pros::millis() - t0;
    // No sample is added here: the last one is the pass the wait decided on. The tick after it (the physics step the wait's last
    // delay ran) is something the wait never saw, and counting it would judge the wait on movement it could not have known about.
    return ok;
  }

  // Lets the robot run on for `ms` with whatever the drive is doing, and returns how far it travelled (path length, true
  // positions) over that time on the worst of the two sides, and the heading
  struct After {
    double distance;
    double angle;
  };
  After run_on(int ms) {
    record();
    size_t from = trace.size() - 1;
    for (int t = 0; t < ms; t += ez::util::DELAY_TIME) pros::delay(ez::util::DELAY_TIME);
    record();
    return {std::fmax(path(from, &Sample::left), path(from, &Sample::right)), path(from, &Sample::heading)};
  }

  double path(size_t from, double Sample::*field) const {
    double sum = 0;
    for (size_t i = from + 1; i < trace.size(); i++) sum += std::fabs(trace[i].*field - trace[i - 1].*field);
    return sum;
  }

  // Average speed (units per second) over the last window_ms of the trace, by path length of true positions: worst side
  double speed_over(double Sample::*field, int window_ms) const {
    if (trace.size() < 2) return 0.0;
    double end = trace.back().t_ms;
    double sum = 0, span = 0;
    for (size_t i = trace.size() - 1; i > 0 && end - trace[i - 1].t_ms <= window_ms + 1e-9; i--) {
      sum += std::fabs(trace[i].*field - trace[i - 1].*field);
      span = end - trace[i - 1].t_ms;
    }
    return span > 0 ? sum / (span / 1000.0) : 0.0;
  }
  // How many times the channel reversed direction over the last window_ms of the trace: a robot hunting about its target does it every
  // few ticks, a robot that overshoots once does it once
  int reversals(double Sample::*field, int window_ms) const {
    if (trace.size() < 3) return 0;
    double end = trace.back().t_ms;
    int n = 0;
    double last_delta = 0;
    for (size_t i = trace.size() - 1; i > 0 && end - trace[i - 1].t_ms <= window_ms + 1e-9; i--) {
      double delta = trace[i].*field - trace[i - 1].*field;
      if (std::fabs(delta) < 1e-9) continue;
      if (last_delta != 0 && (delta > 0) != (last_delta > 0)) n++;
      last_delta = delta;
    }
    return n;
  }
  bool hunting(int window_ms = 200) const {
    return reversals(&Sample::heading, window_ms) >= 6 || reversals(&Sample::left, window_ms) >= 6 || reversals(&Sample::right, window_ms) >= 6;
  }
  double drive_speed_over(int window_ms) const { return std::fmax(speed_over(&Sample::left, window_ms), speed_over(&Sample::right, window_ms)); }
  double angle_speed_over(int window_ms) const { return speed_over(&Sample::heading, window_ms); }

  // What "under the stop speed" means to the library over a window: the stop speed plus one sensor count per window, because
  // movement of less than a count is not counted as travel (see travel.hpp). At a 50 ms window that is 0.5 in/s on a 450 rpm drive.
  double drive_floor(int window_ms) { return FLOOR_DISTANCE + (1.0 / chassis.drive_tick_per_inch()) / (window_ms / 1000.0); }
  double angle_floor(int window_ms) const { return FLOOR_ANGLE + 0.01 / (window_ms / 1000.0); }

  // Instantaneous true speed, the faster side
  double drive_speed_now() const { return std::fmax(std::fabs(sim.left().velocity_in_s), std::fabs(sim.right().velocity_in_s)); }
  double angle_speed_now() const {
    if (trace.size() < 2) return 0.0;
    const Sample& b = trace.back();
    const Sample& p = trace[trace.size() - 2];
    return std::fabs(b.heading - p.heading) / ((b.t_ms - p.t_ms) / 1000.0);
  }
};

}  // namespace gate
