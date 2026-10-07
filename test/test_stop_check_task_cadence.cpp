// An undisturbed wait must come back about when the robot stops, whatever pace the auto task runs at. At a task that passes every 20, 40 or 50
// ms the sample before a 90 ms window is not 90 ms back but 100, 120 or 100; a stop check that charges the whole stretch back to that sample
// to the window reads the robot as moving faster than it is, and a robot slowing onto the stop speed is called stopped several samples later.
#include <cmath>
#include <string>

#include "EZ-Template/travel.hpp"
#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;
using ez::detail::PathTracker;

namespace {

// Runs the auto task on every `every`th tick only
struct Cadence {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline int every = 1;
  static inline int count = 0;
  static void tick() {
    rig->record();
    rig->sim.passes_per_tick(count++ % every == 0 ? 1 : 0);
    inner();
  }
  static void install(Rig& r, int n) {
    rig = &r;
    every = n;
    count = 0;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Cadence::tick;
  }
};

constexpr double FLOOR_IN_S = 1.5;
constexpr double COUNT_450_325 = 3.25 * M_PI / 400.0;
constexpr std::uint32_t T0 = 5000;

}  // namespace

TEST_CASE("PathTracker: a task passing every 20 to 50 ms reads a slowing thing as stopped as soon as the window's own travel says so") {
  for (int spacing : {20, 40, 50}) {
    for (int window : {90, 150}) {
      for (double tau : {100.0, 250.0}) {
        PathTracker t;
        t.band_set(COUNT_450_325);
        std::uint32_t pass = 0;
        // A thing slowing exponentially from 40 in/s, sampled every `spacing` ms. The reference is the travel over the stretch back to the
        // newest sample at or before the window's start divided by the time that stretch took, which is the stop check without any stall
        std::vector<std::pair<int, double>> samples;
        std::vector<double> followed;  // the position with the backlash band followed, as the tracker holds it
        double cum_pos = 0.0;
        int late = 0;
        for (int ms = 0; ms <= 3000; ms += spacing) {
          double x = 40.0 * tau / 1000.0 * (1.0 - std::exp(-ms / tau));
          t.sample(x, 0.0, T0 + ms, ++pass);
          double step = x - cum_pos;
          if (std::fabs(step) > COUNT_450_325 + 1e-9) cum_pos += step - std::copysign(COUNT_450_325, step);
          samples.push_back({ms, x});
          followed.push_back(cum_pos);
          if (ms < 200) continue;
          size_t base = samples.size() - 1;
          while (base > 0 && ms - samples[base].first < window) base--;
          if (ms - samples[base].first < window) continue;
          double travel = 0;
          for (size_t i = base + 1; i < samples.size(); i++) {
            travel += std::fabs(followed[i] - followed[i - 1]);
          }
          bool reference = travel < FLOOR_IN_S * (ms - samples[base].first) / 1000.0;
          bool got = t.stopped(window, FLOOR_IN_S, T0 + ms);
          if (reference && !got) late++;
        }
        CAPTURE(spacing);
        CAPTURE(window);
        CAPTURE(tau);
        CHECK(late == 0);
      }
    }
  }
}

namespace {

// How long after the robot really stopped (the last sample whose preceding 90 ms of travel was over the floor) the wait came back, and whether
// it was clean
struct Lag {
  bool returned;
  bool interfered;
  double ms;
  double lag;
};

Lag undisturbed_drive(const sim::SimArchetype& arch, double inches, int every) {
  Rig r(arch, 1, false, 1);
  Cadence::install(r, every);
  r.chassis.pid_drive_set(ez::QLength(inches * 1.0_in), 110);
  Lag l{};
  l.returned = r.wait([&] { r.chassis.pid_wait(); }, 1500, &l.ms);
  l.interfered = r.chassis.interfered;
  double moving_until = 0;
  for (size_t i = 1; i < r.trace.size(); i++) {
    double window_start = r.trace[i].t_ms - 90;
    size_t j = i;
    while (j > 0 && r.trace[j - 1].t_ms >= window_start) j--;
    double path = 0;
    for (size_t k = j + 1; k <= i; k++) path += std::fmax(std::fabs(r.trace[k].left - r.trace[k - 1].left), std::fabs(r.trace[k].right - r.trace[k - 1].right));
    if (path / 0.09 >= r.drive_floor(90)) moving_until = r.trace[i].t_ms;
  }
  l.lag = l.ms - moving_until;
  return l;
}

}  // namespace

TEST_CASE("an undisturbed wait comes back about when the robot stops at an auto task pace of 40 ms") {
  // What a task passing every tick takes is 50 ms after the robot stops. A slower task adds about its own spacing, not 100 ms more. (The 20 and 50 ms
  // paces are left to the tracker test above: one pass either way moves a return by the pace, and the bound here is a few passes wide.)
  std::vector<sim::SimArchetype> archetypes = {sim::archetype_light_fast(), sim::archetype_heavy_slow(), archetype_classroom()};
  for (size_t a = 0; a < archetypes.size(); a++)
    for (double inches : {6.0, 24.0})
      for (int every : {4}) {
        Lag l = undisturbed_drive(archetypes[a], inches, every);
        CAPTURE(a);
        CAPTURE(inches);
        CAPTURE(every);
        REQUIRE(l.returned);
        CHECK_FALSE(l.interfered);
        CHECK_MESSAGE(l.lag <= 170.0, "came back " << l.lag << " ms after the robot stopped");
      }
}

TEST_CASE("PathTracker: movement in a stall of more than a window and three passes is charged to the window in full") {
  for (int gap : {150, 300, 1000}) {
    PathTracker t;
    t.band_set(COUNT_450_325);
    std::uint32_t pass = 0;
    for (int ms = 0; ms <= 500; ms += 10) t.sample(0.0, 0.0, T0 + ms, ++pass);
    // The task stalls for `gap` ms; the robot is pushed over the last 60 ms of it at 3 in/s and the first sample after the gap sees it all
    int resume = 500 + gap;
    t.sample(0.18, 0.0, T0 + resume, ++pass);
    double travel = 0, span = 0;
    REQUIRE(t.travel_over(90, T0 + resume, travel, span));
    CAPTURE(gap);
    CHECK(span == doctest::Approx(90.0));
    CHECK(travel == doctest::Approx(0.18 - COUNT_450_325));
    CHECK_FALSE(t.stopped(90, FLOOR_IN_S, T0 + resume));
  }
}
