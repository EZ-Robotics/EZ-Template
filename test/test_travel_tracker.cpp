// ez::detail::PathTracker on its own: the "how far did it travel over the last W ms" that every wait's meaning of "stopped" is
// built on. See travel.hpp for what it promises; these tests are what hold it to that.
//
// The floors are the library's (exit_conditions.cpp keeps its own copies, these are for the asserts): 1.5 in/s for distance,
// 4 deg/s for angles.
#include <cmath>
#include <cstdio>
#include <vector>

#include "EZ-Template/travel.hpp"
#include "doctest.h"

using ez::detail::PathTracker;

namespace {
constexpr double FLOOR_IN_S = 1.5;
// One encoder count: 450 rpm 3.25 in drive (600 rpm cartridge, 4:3 gearing) and 200 rpm 4 in drive
constexpr double COUNT_450_325 = 3.25 * M_PI / 400.0;
constexpr double COUNT_200_4 = 4.0 * M_PI / 900.0;
constexpr std::uint32_t T0 = 5000;

struct Feed {
  PathTracker t;
  std::uint32_t pass = 0;
  explicit Feed(double band) { t.band_set(band); }
  void at(int ms, double x, double y = 0.0) { t.sample(x, y, T0 + ms, ++pass); }
  std::uint32_t now(int ms) const { return T0 + ms; }
};
}  // namespace

TEST_CASE("PathTracker: a constant speed reads as that speed for windows from 10 to 1000 ms") {
  for (double v : {5.0, 12.0, 40.0}) {
    Feed f(COUNT_450_325);
    int end = 2000;
    for (int ms = 0; ms <= end; ms += 10) f.at(ms, v * ms / 1000.0);
    for (int w : {10, 50, 100, 250, 1000}) {
      double travel = 0, span = 0;
      REQUIRE(f.t.travel_over(w, f.now(end), travel, span));
      CAPTURE(v);
      CAPTURE(w);
      // 10 ms is under the two-pass minimum and is asked as 20
      CHECK(span == doctest::Approx(std::max(w, 20)));
      CHECK(travel == doctest::Approx(v * span / 1000.0).epsilon(1e-9));
      CHECK_FALSE(f.t.stopped(w, FLOOR_IN_S, f.now(end)));
    }
  }
}

TEST_CASE("PathTracker: a sensor flickering one count at rest is stopped at every window from 50 ms") {
  for (double count : {COUNT_450_325, COUNT_200_4}) {
    for (int every : {1, 2}) {  // flickers every tick, and every other tick
      Feed f(count);
      int end = 2000;
      for (int ms = 0; ms <= end; ms += 10) f.at(ms, ((ms / 10) / every) % 2 == 0 ? 0.0 : count);
      for (int w : {50, 100, 250, 500, 1000}) {
        CAPTURE(count);
        CAPTURE(every);
        CAPTURE(w);
        CHECK(f.t.stopped(w, FLOOR_IN_S, f.now(end)));
      }
    }
  }
}

TEST_CASE("PathTracker: without the one-count band the same flicker would never read stopped (why the band exists)") {
  Feed f(0.0);
  int end = 2000;
  for (int ms = 0; ms <= end; ms += 10) f.at(ms, (ms / 10) % 2 == 0 ? 0.0 : COUNT_450_325);
  // 2.55 in/s of fake travel against a 1.5 in/s floor
  CHECK_FALSE(f.t.stopped(50, FLOOR_IN_S, f.now(end)));
  CHECK_FALSE(f.t.stopped(250, FLOOR_IN_S, f.now(end)));
}

TEST_CASE("PathTracker: a drift of one count that stays there is stopped, a swing of more than a count is counted") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 500; ms += 10) f.at(ms, 0.0);
  for (int ms = 510; ms <= 2000; ms += 10) f.at(ms, COUNT_450_325);
  CHECK(f.t.stopped(100, FLOOR_IN_S, f.now(2000)));
  CHECK(f.t.stopped(1000, FLOOR_IN_S, f.now(2000)));

  // Two counts away is one count of travel (the band follows the reading), once
  Feed g(COUNT_450_325);
  for (int ms = 0; ms <= 500; ms += 10) g.at(ms, 0.0);
  for (int ms = 510; ms <= 2000; ms += 10) g.at(ms, 2.0 * COUNT_450_325);
  double travel = 0, span = 0;
  REQUIRE(g.t.travel_over(1500, g.now(2000), travel, span));
  CHECK(travel == doctest::Approx(COUNT_450_325).epsilon(1e-9));
  CHECK(g.t.stopped(250, FLOOR_IN_S, g.now(2000)));
}

TEST_CASE("PathTracker: a 0.8 in, 2 Hz oscillation (10 in/s peak) never reads stopped, and 1 Hz is reported") {
  // The turnaround is the hard moment: the robot is momentarily at rest, and a window around it holds only a little path. Run
  // every window against every phase of the sample grid and keep the lowest speed seen, as a multiple of the floor.
  struct Row {
    double band;
    double hz;
    int w;
    double worst;
  };
  std::vector<Row> rows;
  for (double band : {COUNT_450_325, COUNT_200_4}) {
    for (double hz : {2.0, 1.0}) {
      for (int w = 50; w <= 500; w += 10) {
        double worst = 1e9;
        for (int phase = 0; phase < 10; phase++) {
          Feed f(band);
          for (int ms = 0; ms <= 4000; ms += 10) {
            f.at(ms, 0.8 * std::sin(2.0 * M_PI * hz * (ms + phase) / 1000.0));
            double travel = 0, span = 0;
            if (ms >= 1000 && f.t.travel_over(w, f.now(ms), travel, span)) worst = std::fmin(worst, travel / (FLOOR_IN_S * span / 1000.0));
          }
        }
        rows.push_back({band, hz, w, worst});
      }
    }
  }
  for (double band : {COUNT_450_325, COUNT_200_4}) {
    for (double hz : {2.0, 1.0}) {
      int longest_stopped = 0;
      for (const auto& r : rows)
        if (r.band == band && r.hz == hz && r.worst < 1.0) longest_stopped = std::max(longest_stopped, r.w);
      std::printf("  [travel tracker] 0.8 in %.0f Hz, count %.4f in: a turnaround reads stopped at windows up to %d ms%s\n", hz, band, longest_stopped,
                  longest_stopped == 0 ? " (never)" : "");
    }
  }
  // 2 Hz: never stopped from the shortest default exit window (the 90 ms small_exit_time) up. Shorter windows are reported
  // above, not asserted: at 50 ms the true path around a 2 Hz turnaround is 0.078 in against a 0.075 in allowance before the
  // band takes any of it, so no band at all can make that one hold.
  for (const auto& r : rows) {
    if (r.hz == 2.0 && r.w >= 90) {
      CAPTURE(r.band);
      CAPTURE(r.w);
      CHECK(r.worst >= 1.0);
    }
  }
}

TEST_CASE("PathTracker: a frozen pass counter, or a task that stopped sampling, is never stopped") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 0.0);
  REQUIRE(f.t.stopped(100, FLOOR_IN_S, f.now(1000)));
  // The task stopped: the same readings are no news, and nothing newer than the last sample can say it is still stopped
  CHECK_FALSE(f.t.stopped(100, FLOOR_IN_S, f.now(1000 + PathTracker::STALE_MS + 10)));
  CHECK_FALSE(f.t.stopped(100, FLOOR_IN_S, f.now(2000)));

  // The pass counter never moves while time does: not one fresh sample after the baseline
  PathTracker frozen;
  frozen.band_set(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) frozen.sample(0.0, 0.0, T0 + ms, 7);
  CHECK_FALSE(frozen.stopped(100, FLOOR_IN_S, T0 + 1000));
  double travel = 0, span = 0;
  CHECK_FALSE(frozen.travel_over(100, T0 + 1000, travel, span));
}

TEST_CASE("PathTracker: a window the history does not cover is not stopped, and a reset clears the history") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 200; ms += 10) f.at(ms, 0.0);
  CHECK(f.t.stopped(150, FLOOR_IN_S, f.now(200)));
  CHECK_FALSE(f.t.stopped(300, FLOOR_IN_S, f.now(200)));  // only 200 ms of motion so far

  f.t.reset();
  CHECK_FALSE(f.t.active());
  CHECK_FALSE(f.t.stopped(50, FLOOR_IN_S, f.now(200)));
  // Whatever it travelled before the reset is not in the first window after it
  Feed g(COUNT_450_325);
  for (int ms = 0; ms <= 500; ms += 10) g.at(ms, 20.0 * ms / 1000.0);  // fast
  g.t.reset();
  for (int ms = 510; ms <= 700; ms += 10) g.at(ms, 10.0);  // then still
  CHECK(g.t.stopped(100, FLOOR_IN_S, g.now(700)));
}

TEST_CASE("PathTracker: passes at the same instant are one sample, and a non finite reading is ignored") {
  // A task that catches up runs several passes in one tick, all reading the same sensors
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) {
    f.at(ms, 0.0);
    f.at(ms, 0.0);
    f.at(ms, 0.0);
  }
  double travel = 0, span = 0;
  REQUIRE(f.t.travel_over(100, f.now(1000), travel, span));
  CHECK(span == doctest::Approx(100.0));  // not 100 ms of 33 samples, 100 ms of 10

  Feed g(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) {
    g.at(ms, 0.0);
    g.t.sample(NAN, 0.0, T0 + ms, ++g.pass);
    g.t.sample(INFINITY, 0.0, T0 + ms, ++g.pass);
  }
  CHECK(g.t.stopped(100, FLOOR_IN_S, g.now(1000)));
}

TEST_CASE("PathTracker: a window longer than the stored history is answered over the history, never more loosely") {
  Feed f(COUNT_450_325);
  int end = 5000;
  double v = 5.0;
  for (int ms = 0; ms <= end; ms += 10) f.at(ms, v * ms / 1000.0);
  double travel = 0, span = 0;
  REQUIRE(f.t.travel_over(4000, f.now(end), travel, span));
  // 256 samples at 10 ms: 2.55 s of history
  CHECK(span == doctest::Approx((PathTracker::CAPACITY - 1) * 10.0));
  CHECK(travel == doctest::Approx(v * span / 1000.0).epsilon(1e-9));
  // 2 s windows are answered in full
  REQUIRE(f.t.travel_over(2000, f.now(end), travel, span));
  CHECK(span == doctest::Approx(2000.0));
}

TEST_CASE("PathTracker: two dimensions measure path length of the pose, so a circle at speed is never stopped") {
  Feed f(COUNT_450_325);
  // 6 in/s around a 3 in radius circle
  for (int ms = 0; ms <= 2000; ms += 10) f.at(ms, 3.0 * std::cos(2.0 * ms / 1000.0), 3.0 * std::sin(2.0 * ms / 1000.0));
  CHECK_FALSE(f.t.stopped(50, FLOOR_IN_S, f.now(2000)));
  CHECK_FALSE(f.t.stopped(500, FLOOR_IN_S, f.now(2000)));
}
