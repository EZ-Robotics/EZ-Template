// What the stop speeds do to a wait, on the simulated robot (no noise, so every number here is the same on every machine).
//
// The table has one row per place the library asks "is the robot stopped": every public wait on a drive, a turn, a turn to a point, a
// swing and the odom waits. For each row:
//   - a lowered stop speed of the row's own kind ends the wait later, a raised one ends it earlier;
//   - the other kind's stop speed changes nothing, down to the millisecond, so a wait can never read the wrong kind's speed;
//   - setting the defaults explicitly (plain or with units) changes nothing either.
// There are two kinds: distance (drives and odom xy) and angle (turns, swings and odom heading).
// Then what a change in the middle of a motion does, and what the stop speeds do not touch.
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

enum class Kind {
  Distance,
  Angle
};
constexpr Kind ALL_KINDS[] = {Kind::Distance, Kind::Angle};

bool is_angle(Kind k) { return k == Kind::Angle; }

void stop_speed_set(Drive& c, Kind k, double v) {
  if (k == Kind::Distance)
    c.pid_distance_exit_stop_speed_set(v);
  else
    c.pid_angle_exit_stop_speed_set(v);
}

// The default stop speed of both kinds, written with units, so a team writing the defaults down gets exactly the numbers it already had
void stop_speed_defaults_with_units(Drive& c) {
  c.pid_distance_exit_stop_speed_set(8_in / 1_s);
  c.pid_angle_exit_stop_speed_set(13_deg / 1_s);
}
void stop_speed_defaults(Drive& c) {
  c.pid_distance_exit_stop_speed_set(8.0);
  c.pid_angle_exit_stop_speed_set(13.0);
}

double lowered(Kind k) { return is_angle(k) ? 1.0 : 0.4; }
double raised(Kind k) { return is_angle(k) ? 40.0 : 20.0; }

struct Row {
  const char* name;
  sim::SimArchetype arch;
  std::function<void(Drive&)> start;
  std::function<void(Drive&)> wait;
  std::vector<Kind> must;  // the stop speeds that change this wait's timing
  std::vector<Kind> may;   // the ones that may (the motion also turns, say) but need not
  bool hi_strict = true;   // a raised stop speed of a `must` kind ends the wait strictly sooner
  bool lo_strict = true;  // a lowered one ends it strictly later and clean; false where the robot is already stiller than the lowered speed at the band, or the
                          // velocity exit gives up first
  bool hi_clean = true;   // a raised stop speed of a `must` kind ends the wait clean; false where it also raises the velocity exit enough to end a slow wait as
                          // interfered
};

std::vector<Row> rows() {
  auto classroom = archetype_classroom();
  auto heavy = sim::archetype_heavy_slow();
  auto sticky = sim::archetype_sticky_high_friction();
  std::vector<Row> r;
  r.push_back({"drive, pid_wait", classroom, [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {Kind::Distance}, {}});
  r.push_back({"drive, pid_wait_until(final distance)",
               classroom,
               [](Drive& c) { c.pid_drive_set(24_in, 110); },
               [](Drive& c) { c.pid_wait_until(24.0); },
               {Kind::Distance},
               {}});
  r.push_back({"turn on a heavy robot, pid_wait",
               heavy,
               [](Drive& c) { c.pid_turn_set(90_deg, 110); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::Angle},
               {},
               true,
               false});
  r.push_back({"turn to a point, pid_wait",
               sticky,
               [](Drive& c) { c.pid_turn_set({12_in, 24_in}, fwd, 110); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::Angle},
               {},
               true,
               false,
               false});
  r.push_back(
      {"swing, pid_wait", classroom, [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {Kind::Angle}, {}});
  r.push_back({"swing, pid_wait_until(final angle)",
               classroom,
               [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); },
               [](Drive& c) { c.pid_wait_until(90.0); },
               {Kind::Angle},
               {}});
  r.push_back({"odom point, pid_wait",
               classroom,
               [](Drive& c) { c.pid_odom_set({{12_in, 24_in}, fwd, 110}); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::Distance},
               {Kind::Angle}});
  r.push_back({"odom point, pid_wait_until(distance)",
               classroom,
               [](Drive& c) { c.pid_odom_set({{0_in, 24_in}, fwd, 110}); },
               [](Drive& c) { c.pid_wait_until(24.0); },
               {Kind::Distance},
               {Kind::Angle}});
  r.push_back({"odom point with a heading, pid_wait_until(point)",
               classroom,
               [](Drive& c) { c.pid_odom_set({{12_in, 24_in, 90_deg}, fwd, 110}); },
               [](Drive& c) { c.pid_wait_until(pose{12, 24, 90}); },
               {Kind::Distance},
               {Kind::Angle},
               true,
               true,
               false});
  r.push_back({"odom path, pid_wait",
               classroom,
               [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in, 90_deg}, fwd, 110}}); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::Distance},
               {Kind::Angle},
               true,
               true,
               false});
  r.push_back({"odom path, pid_wait_until_index(last)",
               classroom,
               [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in, 90_deg}, fwd, 110}}); },
               [](Drive& c) { c.pid_wait_until_index(1); },
               {Kind::Distance},
               {Kind::Angle},
               true,
               true,
               false});
  // The heading is what a heavy robot is last to settle on a path, so this is where the odom heading's stop speed shows
  r.push_back({"odom path on a heavy robot, pid_wait",
               heavy,
               [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in, 90_deg}, fwd, 110}}); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::Distance, Kind::Angle},
               {},
               false,
               false,
               false});
  return r;
}

struct Run {
  bool returned = false;
  double elapsed = 0;
  bool interfered = false;
  double left = 0, right = 0, heading = 0;
  bool operator==(const Run& o) const {
    return returned == o.returned && elapsed == o.elapsed && interfered == o.interfered && left == o.left && right == o.right && heading == o.heading;
  }
};

struct Event {
  double at_ms;
  std::function<void(Drive&)> act;
};

constexpr int CAP_TICKS = 3000;  // 30 s of sim time

Run run(const Row& row, const std::function<void(Drive&)>& configure = nullptr, const std::vector<Event>& events = {}, int cap = CAP_TICKS) {
  // The table's heavy_slow rows run at 2 passes per poll, where that robot is stable (the other tests that use it say the same)
  Rig r(row.arch, std::string(row.name).find("heavy robot") != std::string::npos ? 2 : 1, false, 1);
  r.chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
  if (configure) configure(r.chassis);
  size_t next = 0;
  if (!events.empty()) {
    r.sim.before_pass = [&](int) {
      while (next < events.size() && r.sim.now_ms() >= events[next].at_ms) events[next++].act(r.chassis);
    };
  }
  row.start(r.chassis);
  Run out;
  test_stub::capture_stdout([&] { out.returned = r.wait([&] { row.wait(r.chassis); }, cap, &out.elapsed); });
  out.interfered = r.chassis.interfered;
  out.left = r.sim.left().position_in;
  out.right = r.sim.right().position_in;
  out.heading = r.sim.heading_deg();
  return out;
}

bool contains(const std::vector<Kind>& v, Kind k) {
  for (Kind x : v)
    if (x == k) return true;
  return false;
}

}  // namespace

TEST_CASE("a lowered stop speed ends a wait later and a raised one sooner, and only its own kind does") {
  for (const auto& row : rows()) {
    INFO(row.name);
    CAPTURE(std::string(row.name));
    Run base = run(row);
    REQUIRE(base.returned);
    REQUIRE_FALSE(base.interfered);
    for (Kind k : ALL_KINDS) {
      INFO("stop speed kind " << (int)k);
      Run lo = run(row, [&](Drive& c) { stop_speed_set(c, k, lowered(k)); });
      Run hi = run(row, [&](Drive& c) { stop_speed_set(c, k, raised(k)); });
      CAPTURE(base.elapsed);
      CAPTURE(lo.elapsed);
      CAPTURE(hi.elapsed);
      if (contains(row.must, k)) {
        CHECK(lo.returned);
        if (row.lo_strict) {
          CHECK_FALSE(lo.interfered);
          CHECK(lo.elapsed > base.elapsed);
        } else {
          CHECK(lo.elapsed >= base.elapsed);
        }
        CHECK(hi.returned);
        if (row.hi_clean) CHECK_FALSE(hi.interfered);
        if (row.hi_strict)
          CHECK(hi.elapsed < base.elapsed);
        else
          CHECK(hi.elapsed <= base.elapsed);
      } else if (contains(row.may, k)) {
        CHECK(lo.elapsed >= base.elapsed);
        CHECK(hi.elapsed <= base.elapsed);
      } else {
        // Every bit of the result, not just the time
        CHECK(lo == base);
        CHECK(hi == base);
      }
    }
  }
}

TEST_CASE("the default stop speeds set explicitly, plain or with units, change nothing") {
  for (auto arch : {archetype_classroom(), sim::archetype_light_fast(), sim::archetype_heavy_slow(), sim::archetype_sticky_high_friction()}) {
    std::vector<Row> rs = rows();
    // The same motions on every robot
    for (auto& row : rs) {
      row.arch = arch;
      INFO(row.name << " on " << arch.name);
      Run base = run(row);
      Run plain = run(row, [](Drive& c) { stop_speed_defaults(c); });
      Run units = run(row, [](Drive& c) { stop_speed_defaults_with_units(c); });
      CHECK(plain == base);
      CHECK(units == base);
    }
  }
}

TEST_CASE("a speed written with units ends a wait at the same millisecond as the plain number") {
  Row swing{"swing", archetype_classroom(), [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Row drive{"drive", archetype_classroom(), [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Row odom{"odom", archetype_classroom(), [](Drive& c) { c.pid_odom_set({{12_in, 24_in}, fwd, 110}); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Row turn{"turn", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  for (double v : {0.5, 2.0, 5.0}) {
    CAPTURE(v);
    CHECK(run(drive, [&](Drive& c) { c.pid_distance_exit_stop_speed_set(v); }) ==
          run(drive, [&](Drive& c) { c.pid_distance_exit_stop_speed_set(v * 1_in / 1_s); }));
    CHECK(run(odom, [&](Drive& c) { c.pid_distance_exit_stop_speed_set(v); }) ==
          run(odom, [&](Drive& c) { c.pid_distance_exit_stop_speed_set(v * 1_in / 1_s); }));
  }
  for (double v : {1.0, 6.0, 16.0}) {
    CAPTURE(v);
    CHECK(run(swing, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v); }) == run(swing, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v * 1_deg / 1_s); }));
    CHECK(run(turn, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v); }) == run(turn, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v * 1_deg / 1_s); }));
  }
  // 6 deg/s is also 1 rpm
  CHECK(run(swing, [](Drive& c) { c.pid_angle_exit_stop_speed_set(6.0); }) == run(swing, [](Drive& c) { c.pid_angle_exit_stop_speed_set(1_rpm); }));
}

TEST_CASE("a refused speed leaves the wait exactly as it was") {
  for (const auto& row : rows()) {
    INFO(row.name);
    Run base = run(row);
    for (Kind k : ALL_KINDS) {
      Run refused = run(row, [&](Drive& c) {
        test_stub::capture_stdout([&] {
          stop_speed_set(c, k, 0.0);
          stop_speed_set(c, k, -3.0);
          stop_speed_set(c, k, std::nan(""));
          stop_speed_set(c, k, std::numeric_limits<double>::infinity());
        });
      });
      CHECK(refused == base);
    }
  }
}

// ---- Changing a stop speed in the middle of a motion -------------------------------------------------------------------------------

TEST_CASE("a stop speed changed in the middle of a motion takes effect on the next poll and resets nothing") {
  // Rows whose wait is still running at 300 ms and 500 ms, on the robots where the answer is decided later than that
  for (const auto& row : rows()) {
    INFO(row.name);
    Run base = run(row);
    REQUIRE(base.elapsed > 700);
    for (Kind k : row.must) {
      INFO("kind " << (int)k);
      double def = is_angle(k) ? 13.0 : 8.0;
      // Set to the value it already has: nothing may change, not even by a millisecond
      CHECK(run(row, nullptr, {{300, [&](Drive& c) { stop_speed_set(c, k, def); }}}) == base);
      // Lowered, then put back before the wait is decided: as if it had never been touched
      CHECK(run(row, nullptr, {{300, [&](Drive& c) { stop_speed_set(c, k, lowered(k)); }}, {500, [&](Drive& c) { stop_speed_set(c, k, def); }}}) == base);
      // Raised, then put back
      CHECK(run(row, nullptr, {{300, [&](Drive& c) { stop_speed_set(c, k, raised(k)); }}, {500, [&](Drive& c) { stop_speed_set(c, k, def); }}}) == base);
      // Changed and left changed: the wait ends where a run that had it from the start ends, because it is read on every poll and no window,
      // timer or latch of the running wait is restarted by the change
      Run lo_from_start = run(row, [&](Drive& c) { stop_speed_set(c, k, lowered(k)); });
      Run lo_mid = run(row, nullptr, {{300, [&](Drive& c) { stop_speed_set(c, k, lowered(k)); }}});
      CHECK(lo_mid == lo_from_start);
      Run hi_from_start = run(row, [&](Drive& c) { stop_speed_set(c, k, raised(k)); });
      Run hi_mid = run(row, nullptr, {{300, [&](Drive& c) { stop_speed_set(c, k, raised(k)); }}});
      // Raised mid-motion it takes effect on the next poll: the wait ends, and no later than one that never had it raised.  It does not have to
      // match one that had the raised speed from the start, because the velocity exit arms the first time the robot is faster than the stop
      // speed it has (or after 1 second), and that already happened under the old speed
      CHECK(hi_mid.returned);
      CHECK(hi_mid.elapsed <= base.elapsed);
    }
  }
}

// ---- What the stop speeds do not touch --------------------------------------------------------------------------------------------

TEST_CASE("a robot held outside big_error is reported stuck, at the same time for every stop speed it moved faster than") {
  // The velocity exit arms the first time the robot is faster than the stop speed, or after 1 second if it never is.  The robot here moves for
  // 300 ms before it is held, so every stop speed under what it reached arms at the same time and ends the wait at the same time, and a stop
  // speed over what it reached waits for the 1 second fallback
  Row row{"drive held", archetype_classroom(), [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  auto held = [&](double v) {
    Rig r(row.arch, 1, false, 1);
    r.chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
    r.chassis.pid_distance_exit_stop_speed_set(v);
    r.sim.pin(300, 1e7);
    row.start(r.chassis);
    Run out;
    test_stub::capture_stdout([&] { out.returned = r.wait([&] { row.wait(r.chassis); }, CAP_TICKS, &out.elapsed); });
    out.interfered = r.chassis.interfered;
    return out;
  };
  Run base = held(8.0);
  REQUIRE(base.returned);
  CHECK(base.interfered);
  for (double v : {0.1, 0.4, 2.9, 5.0, 10.0}) {
    CAPTURE(v);
    Run o = held(v);
    CHECK(o.returned);
    CHECK(o.interfered);
    CHECK(o.elapsed == base.elapsed);
  }
  Run never_armed = held(100.0);
  CHECK(never_armed.returned);
  CHECK(never_armed.interfered);
  CHECK(never_armed.elapsed > base.elapsed);
}

TEST_CASE("a stop speed well over the default ends the wait no later than the default does") {
  // The velocity exit uses the stop speed too, so a raised one can end a slow wait as interfered, but it never ends it later
  for (const auto& row : rows()) {
    INFO(row.name);
    Run base = run(row);
    for (Kind k : row.must) {
      Run above = run(row, [&](Drive& c) { stop_speed_set(c, k, is_angle(k) ? 40.0 : 10.0); });
      CHECK(above.returned);
      CHECK(above.elapsed <= base.elapsed);
    }
  }
}

// ---- The cost of a lowered stop speed ---------------------------------------------------------------------------------------------

TEST_CASE("a lowered stop speed only ever ends a wait later") {
  Row turn{"heavy turn", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  double last = 0;
  for (double v : {16.0, 8.0, 4.0, 2.0, 1.0, 0.5}) {
    CAPTURE(v);
    Run o = run(turn, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v); });
    CHECK(o.returned);
    CHECK(o.elapsed >= last);
    last = o.elapsed;
  }
}

TEST_CASE("the same holds in pid_wait_until on a checkpoint past the target, for a turn and for a swing") {
  // A checkpoint past the final target is never crossed, so the wait ends when the motion settles
  Row turn{
      "heavy turn, until 100", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait_until(100.0); }, {}, {}};
  Row swing{"heavy swing, until 100",
            sim::archetype_heavy_slow(),
            [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); },
            [](Drive& c) { c.pid_wait_until(100.0); },
            {},
            {}};
  for (const Row* row : {&turn, &swing}) {
    INFO(row->name);
    double last = 0;
    for (double v : {8.0, 4.0, 2.0, 1.0}) {
      CAPTURE(v);
      Run o = run(*row, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v); });
      CHECK(o.returned);
      CHECK(o.elapsed >= last);
      last = o.elapsed;
    }
    // And the other kind's speed is not what does it
    Run base = run(*row);
    Run other = run(*row, [&](Drive& c) { c.pid_distance_exit_stop_speed_set(0.5); });
    CHECK(other == base);
  }
}

TEST_CASE("a stop speed far above anything real ends a wait no later than a large one, and clean") {
  Row drive{"classroom drive", archetype_classroom(), [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Row turn{"heavy turn", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Run d10 = run(drive, [](Drive& c) { c.pid_distance_exit_stop_speed_set(10.0); });
  for (double v : {1e3, 1e9, 1e300}) {
    CAPTURE(v);
    Run o = run(drive, [&](Drive& c) { c.pid_distance_exit_stop_speed_set(v); });
    CHECK(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(o.elapsed <= d10.elapsed);
  }
  Run t40 = run(turn, [](Drive& c) { c.pid_angle_exit_stop_speed_set(40.0); });
  for (double v : {1e3, 1e9, 1e300}) {
    CAPTURE(v);
    Run o = run(turn, [&](Drive& c) { c.pid_angle_exit_stop_speed_set(v); });
    CHECK(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(o.elapsed <= t40.elapsed + 50);  // a robot that never gets faster than the stop speed arms the velocity exit by its 1 second fallback
  }
}
