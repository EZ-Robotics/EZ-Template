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

// A stalled task leaves a hole in the samples. Whatever the robot did inside the hole shows up at once in the first sample after it,
// and has to be charged to the window that was asked about: spreading it over the hole would turn a 3 in/s shove into 1 in/s.
TEST_CASE("PathTracker: a robot that sits still through a gap in the samples is stopped after it") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 0.0);
  f.at(1300, 0.0);
  CHECK(f.t.stopped(90, FLOOR_IN_S, f.now(1300)));
  CHECK(f.t.stopped(250, FLOOR_IN_S, f.now(1300)));
}

TEST_CASE("PathTracker: movement that shows up at the end of a gap is charged to the window, not spread over the gap") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 0.0);
  f.at(1300, 0.3);  // 0.3 in over the 300 ms gap would be 1 in/s, under the floor, if spread over it
  double travel = 0, span = 0;
  REQUIRE(f.t.travel_over(90, f.now(1300), travel, span));
  CHECK(span == doctest::Approx(90.0));
  CHECK(travel == doctest::Approx(0.3 - COUNT_450_325));
  CHECK_FALSE(f.t.stopped(90, FLOOR_IN_S, f.now(1300)));
  // A longer window that reaches back past the gap sees the same inches over its own, longer, span
  CHECK(f.t.stopped(500, FLOOR_IN_S, f.now(1300)));
}

TEST_CASE("PathTracker: a gap's movement leaves the window once the samples after it are older than the window") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 0.0);
  f.at(1300, 0.3);
  for (int ms = 1310; ms <= 1500; ms += 10) {
    f.at(ms, 0.3);
    // The window still reaches the sample from before the gap until it is shorter than the time since the gap's end
    if (ms < 1390) CHECK_FALSE(f.t.stopped(90, FLOOR_IN_S, f.now(ms)));
    if (ms >= 1400) CHECK(f.t.stopped(90, FLOOR_IN_S, f.now(ms)));
  }
}

TEST_CASE("PathTracker: a window that is covered is never answered over more than its own length") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 0.0);
  f.at(1400, 0.0);
  for (int w : {20, 50, 90, 250}) {
    double travel = 0, span = 0;
    REQUIRE(f.t.travel_over(w, f.now(1400), travel, span));
    CAPTURE(w);
    CHECK(span == doctest::Approx(w));
  }
}

// Net displacement: has the thing gone anywhere over a window. A distance and not a path length, read between the average position of
// the first half of the window and that of the second

TEST_CASE("PathTracker: a thing oscillating about one place has a long path and no net displacement") {
  Feed f(COUNT_450_325);
  int end = 2000;
  for (int ms = 0; ms <= end; ms += 10) f.at(ms, 0.2 * std::sin(2.0 * M_PI * 3.0 * ms / 1000.0));
  for (int w : {667, 1000, 1500}) {
    double travel = 0, net = 0, span = 0;
    REQUIRE(f.t.travel_over(w, f.now(end), travel, span));
    REQUIRE(f.t.net_over(w, f.now(end), net, span));
    CAPTURE(w);
    CHECK(travel > 0.4);
    CHECK(net < 0.1);
    CHECK(span == doctest::Approx(w / 2.0));
    CHECK_FALSE(f.t.stopped(w, FLOOR_IN_S, f.now(end)));
    CHECK(f.t.in_place(w, FLOOR_IN_S, f.now(end)));
  }
}

TEST_CASE("PathTracker: a thing carried at a speed over the floor is not in place, one under it is") {
  for (double v : {0.5, 1.2, 1.8, 3.0, -3.0}) {
    Feed f(COUNT_450_325);
    int end = 2000;
    for (int ms = 0; ms <= end; ms += 10) f.at(ms, v * ms / 1000.0);
    for (int w : {250, 667, 1500}) {
      double net = 0, span = 0;
      REQUIRE(f.t.net_over(w, f.now(end), net, span));
      CAPTURE(v);
      CAPTURE(w);
      // A steady speed reads as that speed times the span between the two averages, which is half the window
      CHECK(span == doctest::Approx(w / 2.0));
      CHECK(net == doctest::Approx(std::fabs(v) * span / 1000.0).epsilon(0.02));
      CHECK(f.t.in_place(w, FLOOR_IN_S, f.now(end)) == (std::fabs(v) < FLOOR_IN_S));
    }
  }
}

TEST_CASE("PathTracker: a thing that ran past its target and is being dragged back is not in place, though the end points of the window are close") {
  Feed f(COUNT_450_325);
  // In to 23.4 at 17 in/s in the first 80 ms, then dragged back at 2.5 in/s
  for (int ms = 0; ms <= 80; ms += 10) f.at(ms, 22.0 + 1.4 * ms / 80.0);
  int end = 700;
  for (int ms = 90; ms <= end; ms += 10) f.at(ms, 23.4 - 2.5 * (ms - 80) / 1000.0);
  // The window opens while the thing is still coming in, so the way in cancels part of the way back at its two end points: they are 0.68 in
  // apart over 667 ms, under what the floor allows over it (1.0 in)
  CHECK(22.0 + 1.4 * 30 / 80.0 - (23.4 - 2.5 * (end - 80) / 1000.0) < FLOOR_IN_S * 0.667);
  double net = 0, span = 0;
  REQUIRE(f.t.net_over(667, f.now(end), net, span));
  CHECK(net >= FLOOR_IN_S * span / 1000.0);
  CHECK_FALSE(f.t.in_place(667, FLOOR_IN_S, f.now(end)));
}

TEST_CASE("PathTracker: a limit cycle of a few samples is in place however high its amplitude and whatever the window's sample count") {
  // A robot hunting by one pass (a sign flip every sample, or a short cycle of them) is on the same side of its centre at the two ends of
  // a window with a whole number of cycles in it and on opposite sides at the two ends of any other, and the end points read the whole
  // amplitude on every pass for ever. The averages over the two halves of the window do not: 3 in peak to peak is three times what the
  // floor allows over the window, and these have to read as going nowhere.
  struct Cycle {
    const char* name;
    std::vector<double> levels;
  };
  std::vector<Cycle> cycles = {
      {"2 samples", {0.0, 3.0}}, {"3 samples", {0.0, 1.5, 3.0}}, {"4 samples", {0.0, 3.0, 3.0, 0.0}}, {"5 samples", {0.0, 3.0, 1.0, 3.0, 2.0}}};
  for (const auto& c : cycles)
    for (int w : {333, 667, 750, 1000}) {
      Feed f(COUNT_450_325);
      int end = 3000;
      for (int ms = 0; ms <= end; ms += 10) f.at(ms, c.levels[(ms / 10) % c.levels.size()]);
      double net = 0, span = 0;
      REQUIRE(f.t.net_over(w, f.now(end), net, span));
      CAPTURE(c.name);
      CAPTURE(w);
      CHECK(net < 0.3 * 3.0);
      CHECK(f.t.in_place(w, FLOOR_IN_S, f.now(end)));
      // ...and on every one of the next few passes, which is the one thing that changes from one to the next for a lock-stepped cycle
      for (int more = 1; more <= 6; more++) {
        f.at(end + more * 10, c.levels[((end + more * 10) / 10) % c.levels.size()]);
        CHECK(f.t.in_place(w, FLOOR_IN_S, f.now(end + more * 10)));
      }
    }
  // The end points of a 75 sample window on the 2 sample cycle are a whole amplitude apart, on the same samples
  Feed g(COUNT_450_325);
  for (int ms = 0; ms <= 3000; ms += 10) g.at(ms, cycles[0].levels[(ms / 10) % 2]);
  CHECK(std::fabs(cycles[0].levels[(3000 / 10) % 2] - cycles[0].levels[((3000 - 750) / 10) % 2]) == doctest::Approx(3.0));
}

TEST_CASE("PathTracker: one count of flicker is not net displacement") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1500; ms += 10) f.at(ms, (ms / 10) % 2 == 0 ? 0.0 : COUNT_450_325);
  double net = 1.0, span = 0;
  REQUIRE(f.t.net_over(667, f.now(1500), net, span));
  CHECK(net == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(f.t.in_place(667, 0.05, f.now(1500)));
}

TEST_CASE("PathTracker: a window longer than the ring is answered over the history it has, never as 'cannot tell'") {
  // 3 s of 10 ms samples against a ring that holds 2.56 s
  for (bool carried : {false, true}) {
    Feed f(COUNT_450_325);
    int end = 3000;
    for (int ms = 0; ms <= end; ms += 10) f.at(ms, carried ? 0.6 * ms / 1000.0 : 0.2 * std::sin(2.0 * M_PI * 3.0 * ms / 1000.0));
    double net = -1, span = 0;
    REQUIRE(f.t.net_over(5000, f.now(end), net, span));
    CAPTURE(carried);
    CHECK(span > 1000.0);
    CHECK(span <= PathTracker::CAPACITY * 10.0 / 2.0);
    if (carried)
      CHECK(net == doctest::Approx(0.6 * span / 1000.0).epsilon(0.02));
    else
      CHECK(net < 0.1);
    // 0.4 in/s is under the carry and over the hunt
    CHECK(f.t.in_place(5000, 0.4, f.now(end)) == !carried);
  }
}

TEST_CASE("PathTracker: a motion younger than the window is answered over what it has") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 300; ms += 10) f.at(ms, 0.002 * ms);
  double travel = 0, net = 0, span = 0;
  CHECK_FALSE(f.t.travel_over(2000, f.now(300), travel, span));  // the path length keeps saying nothing here
  REQUIRE(f.t.net_over(2000, f.now(300), net, span));
  CHECK(span == doctest::Approx(150.0));
  CHECK(net == doctest::Approx(0.3).epsilon(0.02));
  CHECK_FALSE(f.t.in_place(2000, FLOOR_IN_S, f.now(300)));  // 2 in/s
}

TEST_CASE("PathTracker: with no history, or a stale one, net displacement is not answered and a thing is in place") {
  Feed f(COUNT_450_325);
  double net = 0, span = 0;
  CHECK_FALSE(f.t.net_over(100, f.now(0), net, span));
  CHECK(f.t.in_place(100, FLOOR_IN_S, f.now(0)));
  for (int ms = 0; ms <= 500; ms += 10) f.at(ms, 0.01 * ms);
  CHECK(f.t.net_over(100, f.now(500), net, span));
  CHECK_FALSE(f.t.net_over(100, f.now(500 + PathTracker::STALE_MS + 1), net, span));
  CHECK(f.t.in_place(100, FLOOR_IN_S, f.now(500 + PathTracker::STALE_MS + 1)));
}

TEST_CASE("PathTracker: net displacement across a gap in the samples is spread over the gap, and still reads as going somewhere") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 0.0);
  f.at(1300, 1.0);
  double net = 0, span = 0;
  REQUIRE(f.t.net_over(90, f.now(1300), net, span));
  CHECK(span == doctest::Approx(45.0));
  CHECK(net > 0.0);
  CHECK_FALSE(f.t.in_place(90, FLOOR_IN_S, f.now(1300)));  // 3.3 in/s over the gap
}

TEST_CASE("PathTracker: net displacement on two axes is the straight line between the two averages") {
  Feed f(COUNT_450_325);
  for (int ms = 0; ms <= 1000; ms += 10) f.at(ms, 3.0 * ms / 1000.0, 4.0 * ms / 1000.0);  // 5 in/s along a 3-4-5 line
  double net = 0, span = 0;
  REQUIRE(f.t.net_over(500, f.now(1000), net, span));
  CHECK(net == doctest::Approx(5.0 * span / 1000.0).epsilon(0.02));
  // Going round a circle of radius 0.5 at 3 in/s: a long path, no net displacement over a whole turn
  Feed g(COUNT_450_325);
  for (int ms = 0; ms <= 3000; ms += 10) g.at(ms, 0.5 * std::cos(6.0 * ms / 1000.0), 0.5 * std::sin(6.0 * ms / 1000.0));
  REQUIRE(g.t.net_over(2094, g.now(3000), net, span));  // two turns take 2094 ms, so each half is a whole turn
  CHECK(net < 0.05);
}
