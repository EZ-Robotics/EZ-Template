// What the stop speeds do to a wait, on the simulated robot (no noise, so every number here is the same on every machine).
//
// The table has one row per place the library asks "is the robot stopped": every public wait on a drive, a turn, a turn to a point, a
// swing and the odom waits. For each row:
//   - a lowered stop speed of the row's own kind ends the wait later, a raised one ends it earlier;
//   - the other kinds' stop speeds change nothing, down to the millisecond, so a wait can never read the wrong kind's speed;
//   - setting the defaults explicitly (plain or with units) changes nothing either.
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
  Drive,
  Turn,
  Swing,
  OdomXY,
  OdomAngle
};
constexpr Kind ALL_KINDS[] = {Kind::Drive, Kind::Turn, Kind::Swing, Kind::OdomXY, Kind::OdomAngle};

bool is_angle(Kind k) { return k == Kind::Turn || k == Kind::Swing || k == Kind::OdomAngle; }

void stop_speed_set(Drive& c, Kind k, double v) {
  switch (k) {
    case Kind::Drive:
      c.pid_drive_exit_stop_speed_set(v);
      break;
    case Kind::Turn:
      c.pid_turn_exit_stop_speed_set(v);
      break;
    case Kind::Swing:
      c.pid_swing_exit_stop_speed_set(v);
      break;
    case Kind::OdomXY:
      c.pid_odom_drive_exit_stop_speed_set(v);
      break;
    case Kind::OdomAngle:
      c.pid_odom_turn_exit_stop_speed_set(v);
      break;
  }
}

// The default stop speed of every kind, written with units, so a team writing the defaults down gets exactly the numbers it already had
void stop_speed_defaults_with_units(Drive& c) {
  c.pid_drive_exit_stop_speed_set(1.5_in / 1_s);
  c.pid_turn_exit_stop_speed_set(4_deg / 1_s);
  c.pid_swing_exit_stop_speed_set(4_deg / 1_s);
  c.pid_odom_drive_exit_stop_speed_set(1.5_in / 1_s);
  c.pid_odom_turn_exit_stop_speed_set(4_deg / 1_s);
}
void stop_speed_defaults(Drive& c) {
  c.pid_drive_exit_stop_speed_set(1.5);
  c.pid_turn_exit_stop_speed_set(4.0);
  c.pid_swing_exit_stop_speed_set(4.0);
  c.pid_odom_drive_exit_stop_speed_set(1.5);
  c.pid_odom_turn_exit_stop_speed_set(4.0);
}

double lowered(Kind k) { return is_angle(k) ? 1.0 : 0.4; }
double raised(Kind k) { return is_angle(k) ? 16.0 : 6.0; }

struct Row {
  const char* name;
  sim::SimArchetype arch;
  std::function<void(Drive&)> start;
  std::function<void(Drive&)> wait;
  std::vector<Kind> must;  // the stop speeds that change this wait's timing
  std::vector<Kind> may;   // the ones that may (the motion also turns, say) but need not
  bool hi_strict = true;   // a raised stop speed of a `must` kind ends the wait strictly sooner
};

std::vector<Row> rows() {
  auto classroom = archetype_classroom();
  auto heavy = sim::archetype_heavy_slow();
  auto sticky = sim::archetype_sticky_high_friction();
  std::vector<Row> r;
  r.push_back({"drive, pid_wait", classroom, [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {Kind::Drive}, {}});
  r.push_back({"drive, pid_wait_until(final distance)",
               classroom,
               [](Drive& c) { c.pid_drive_set(24_in, 110); },
               [](Drive& c) { c.pid_wait_until(24.0); },
               {Kind::Drive},
               {}});
  r.push_back({"turn on a heavy robot, pid_wait", heavy, [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {Kind::Turn}, {}});
  r.push_back(
      {"turn to a point, pid_wait", sticky, [](Drive& c) { c.pid_turn_set({12_in, 24_in}, fwd, 110); }, [](Drive& c) { c.pid_wait(); }, {Kind::Turn}, {}});
  r.push_back(
      {"swing, pid_wait", classroom, [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {Kind::Swing}, {}});
  r.push_back({"swing, pid_wait_until(final angle)",
               classroom,
               [](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, 90_deg, 110); },
               [](Drive& c) { c.pid_wait_until(90.0); },
               {Kind::Swing},
               {}});
  r.push_back({"odom point, pid_wait",
               classroom,
               [](Drive& c) { c.pid_odom_set({{12_in, 24_in}, fwd, 110}); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::OdomXY},
               {Kind::OdomAngle}});
  r.push_back({"odom point, pid_wait_until(distance)",
               classroom,
               [](Drive& c) { c.pid_odom_set({{0_in, 24_in}, fwd, 110}); },
               [](Drive& c) { c.pid_wait_until(24.0); },
               {Kind::OdomXY},
               {Kind::OdomAngle}});
  r.push_back({"odom point with a heading, pid_wait_until(point)",
               classroom,
               [](Drive& c) { c.pid_odom_set({{12_in, 24_in, 90_deg}, fwd, 110}); },
               [](Drive& c) { c.pid_wait_until(pose{12, 24, 90}); },
               {Kind::OdomXY},
               {Kind::OdomAngle}});
  r.push_back({"odom path, pid_wait",
               classroom,
               [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in, 90_deg}, fwd, 110}}); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::OdomXY},
               {Kind::OdomAngle}});
  r.push_back({"odom path, pid_wait_until_index(last)",
               classroom,
               [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in, 90_deg}, fwd, 110}}); },
               [](Drive& c) { c.pid_wait_until_index(1); },
               {Kind::OdomXY},
               {Kind::OdomAngle}});
  // The heading is what a heavy robot is last to settle on a path, so this is where the odom heading's stop speed shows
  r.push_back({"odom path on a heavy robot, pid_wait",
               heavy,
               [](Drive& c) { c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in, 90_deg}, fwd, 110}}); },
               [](Drive& c) { c.pid_wait(); },
               {Kind::OdomXY, Kind::OdomAngle},
               {},
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
        CHECK_FALSE(lo.interfered);
        CHECK(lo.elapsed > base.elapsed);
        CHECK(hi.returned);
        CHECK_FALSE(hi.interfered);
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
    CHECK(run(drive, [&](Drive& c) { c.pid_drive_exit_stop_speed_set(v); }) == run(drive, [&](Drive& c) { c.pid_drive_exit_stop_speed_set(v * 1_in / 1_s); }));
    CHECK(run(odom, [&](Drive& c) { c.pid_odom_drive_exit_stop_speed_set(v); }) ==
          run(odom, [&](Drive& c) { c.pid_odom_drive_exit_stop_speed_set(v * 1_in / 1_s); }));
  }
  for (double v : {1.0, 6.0, 16.0}) {
    CAPTURE(v);
    CHECK(run(swing, [&](Drive& c) { c.pid_swing_exit_stop_speed_set(v); }) == run(swing, [&](Drive& c) { c.pid_swing_exit_stop_speed_set(v * 1_deg / 1_s); }));
    CHECK(run(turn, [&](Drive& c) { c.pid_turn_exit_stop_speed_set(v); }) == run(turn, [&](Drive& c) { c.pid_turn_exit_stop_speed_set(v * 1_deg / 1_s); }));
  }
  // 6 deg/s is also 1 rpm
  CHECK(run(swing, [](Drive& c) { c.pid_swing_exit_stop_speed_set(6.0); }) == run(swing, [](Drive& c) { c.pid_swing_exit_stop_speed_set(1_rpm); }));
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
      double def = is_angle(k) ? 4.0 : 1.5;
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
      CHECK(hi_mid == hi_from_start);
    }
  }
}

// ---- The odom stop test, on tracker samples fed by hand ---------------------------------------------------------------------------
//
// A sim wait does not isolate the odom heading's own stop test (the wait is ended by xy long before), so the helper every odom wait
// asks is checked directly: xy and heading each creep at a speed we choose, and each is judged against its own stop speed.
namespace {

// 600 ms of 10 ms samples, xy creeping along y at `xy_speed` in/s and the heading turning at `heading_speed` deg/s
void feed_odom_motion(Drive& d, double xy_speed, double heading_speed) {
  constexpr int XY = 4, HEADING = 3;
  DriveTestAccess::travel(d, XY).band_set(0.0);
  DriveTestAccess::travel(d, HEADING).band_set(0.0);
  std::uint32_t pass = 1;
  double y = 0, th = 0;
  for (int i = 0; i <= 60; i++) {
    DriveTestAccess::travel(d, XY).sample(0.0, y, pros::millis(), pass);
    DriveTestAccess::travel(d, HEADING).sample(th, 0.0, pros::millis(), pass);
    pass++;
    y += xy_speed * 0.01;
    th += heading_speed * 0.01;
    pros::delay(10);
  }
}

}  // namespace

TEST_CASE("an odom motion is stopped when its xy and its heading both are, each against its own stop speed") {
  auto stopped = [](double xy_speed, double heading_speed, const std::function<void(Drive&)>& configure) {
    test_stub::reset_all();
    Drive d({1, -2}, {-3, 4}, 5, 3.25, 360);
    configure(d);
    feed_odom_motion(d, xy_speed, heading_speed);
    return DriveTestAccess::odom_travel_stopped(d, 300);
  };
  auto none = [](Drive&) {};
  // The defaults: 1.5 in/s and 4 deg/s
  CHECK(stopped(1.0, 3.0, none));
  CHECK_FALSE(stopped(2.0, 3.0, none));
  CHECK_FALSE(stopped(1.0, 5.0, none));
  // The odom heading's own stop speed, and not the xy one or the turn's or the swing's
  CHECK_FALSE(stopped(1.0, 3.0, [](Drive& d) { d.pid_odom_turn_exit_stop_speed_set(2.0); }));
  CHECK(stopped(1.0, 5.0, [](Drive& d) { d.pid_odom_turn_exit_stop_speed_set(8.0); }));
  CHECK(stopped(1.0, 3.0, [](Drive& d) {
    d.pid_drive_exit_stop_speed_set(0.01);
    d.pid_turn_exit_stop_speed_set(0.01);
    d.pid_swing_exit_stop_speed_set(0.01);
  }));
  CHECK_FALSE(stopped(1.0, 5.0, [](Drive& d) {
    d.pid_drive_exit_stop_speed_set(100.0);
    d.pid_turn_exit_stop_speed_set(100.0);
    d.pid_swing_exit_stop_speed_set(100.0);
  }));
  // The odom xy stop speed, and not the heading's or the drive's
  CHECK_FALSE(stopped(1.0, 3.0, [](Drive& d) { d.pid_odom_drive_exit_stop_speed_set(0.5); }));
  CHECK(stopped(2.0, 3.0, [](Drive& d) { d.pid_odom_drive_exit_stop_speed_set(3.0); }));
  CHECK(stopped(1.0, 3.0, [](Drive& d) {
    d.pid_odom_turn_exit_stop_speed_set(100.0);
    d.pid_drive_exit_stop_speed_set(0.01);
  }));
  CHECK_FALSE(stopped(2.0, 3.0, [](Drive& d) {
    d.pid_odom_turn_exit_stop_speed_set(100.0);
    d.pid_drive_exit_stop_speed_set(100.0);
  }));
}

// ---- The mA hold ------------------------------------------------------------------------------------------------------------------
//
// An mA exit is held while the robot is still closing on its target (a heavy robot accelerating hard is over current the whole way), and
// released once, over one mA window, the error has not come down by the stop speed times the window. So the stop speed decides how fast a
// robot over current has to be closing to be left alone: here the robot is carried toward the target at a steady speed for two seconds with
// every motor reading over current, and the only exit is a 100 ms mA timeout.
namespace {

sim::SimRobot* g_ma_sim = nullptr;
Drive* g_ma_drive = nullptr;
void (*g_ma_previous_tick)() = nullptr;
void ma_tick() {
  g_ma_previous_tick();
  for (auto& m : g_ma_drive->left_motors) m.fake().over_current = true;
  for (auto& m : g_ma_drive->right_motors) m.fake().over_current = true;
}
struct OverCurrent {
  explicit OverCurrent(Rig& r) {
    g_ma_sim = &r.sim;
    g_ma_drive = &r.chassis;
    g_ma_previous_tick = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &ma_tick;
  }
  ~OverCurrent() { test_stub::g_clock.on_delay = g_ma_previous_tick; }
};

constexpr double CARRY_START_MS = 200;
constexpr double CARRY_MS = 2000;

// Which motion, carried at (v in/s, w deg/s), ended by pid_wait() or by a pid_wait_until() a checkpoint short of the target that is never crossed
Run run_ma(Kind kind, double v, double w, const std::function<void(Drive&)>& configure = nullptr, bool until = false) {
  Rig r(archetype_classroom(), 1, false, 1);
  auto& c = r.chassis;
  c.pid_drive_exit_condition_set(0, 0, 0, 0, 0, 100);
  c.pid_turn_exit_condition_set(0, 0, 0, 0, 0, 100);
  c.pid_swing_exit_condition_set(0, 0, 0, 0, 0, 100);
  c.pid_odom_drive_exit_condition_set(0, 0, 0, 0, 0, 100);
  c.pid_odom_turn_exit_condition_set(0, 0, 0, 0, 0, 100);
  if (configure) configure(c);
  switch (kind) {
    case Kind::Drive:
      c.pid_drive_set(48_in, 110);
      break;
    case Kind::Turn:
      c.pid_turn_set(180_deg, 110);
      break;
    case Kind::Swing:
      c.pid_swing_set(ez::LEFT_SWING, 180_deg, 110);
      break;
    case Kind::OdomXY:
      c.pid_odom_set({{0_in, 48_in}, fwd, 110});
      break;
    case Kind::OdomAngle:
      c.pid_odom_set({{0_in, 48_in, 180_deg}, fwd, 110});
      break;
  }
  r.sim.carry(v, w, CARRY_START_MS, CARRY_MS);
  OverCurrent guard(r);
  Run out;
  auto wait = [&] {
    if (!until)
      c.pid_wait();
    else if (kind == Kind::Drive)
      c.pid_wait_until(47.0);
    else
      c.pid_wait_until(179.0);
  };
  test_stub::capture_stdout([&] { out.returned = r.wait(wait, CAP_TICKS, &out.elapsed); });
  out.interfered = c.interfered;
  out.left = r.sim.left().position_in;
  out.right = r.sim.right().position_in;
  out.heading = r.sim.heading_deg();
  return out;
}

struct MaRow {
  const char* name;
  Kind kind;
  double held_v, held_w;  // closing faster than the default stop speed: held for as long as it is carried
  double free_v, free_w;  // closing slower than it: released at the first window
};
const MaRow MA_ROWS[] = {
    {"drive", Kind::Drive, 3.0, 0.0, 1.0, 0.0},
    {"turn", Kind::Turn, 0.0, 10.0, 0.0, 2.0},
    {"swing", Kind::Swing, 0.0, 10.0, 0.0, 2.0},
};

}  // namespace

TEST_CASE("an mA exit is held while the robot closes faster than the stop speed, and released once it does not") {
  for (const auto& row : MA_ROWS)
    for (bool until : {false, true}) {
      INFO(row.name << (until ? ", pid_wait_until" : ", pid_wait"));
      const Kind k = row.kind;
      auto run_ma = [&](Kind kind, double v, double w, const std::function<void(Drive&)>& configure = nullptr) {
        return ::run_ma(kind, v, w, configure, until);
      };
      // The default: closing faster than it is held for the whole carry, closing slower is released at the first window
      Run held = run_ma(k, row.held_v, row.held_w);
      Run freed = run_ma(k, row.free_v, row.free_w);
      REQUIRE(held.returned);
      REQUIRE(freed.returned);
      CHECK(held.elapsed >= CARRY_START_MS + CARRY_MS);
      CHECK(freed.elapsed < 800);
      // Raised above the carry speed, the robot that was held is released at the first window
      Run held_raised = run_ma(k, row.held_v, row.held_w, [&](Drive& c) { stop_speed_set(c, k, raised(k)); });
      CHECK(held_raised.returned);
      CHECK(held_raised.elapsed < 800);
      CHECK(held_raised.elapsed < held.elapsed);
      // Lowered under the carry speed, the robot that was released is held for the whole carry
      Run freed_lowered = run_ma(k, row.free_v, row.free_w, [&](Drive& c) { stop_speed_set(c, k, lowered(k)); });
      CHECK(freed_lowered.returned);
      CHECK(freed_lowered.elapsed >= CARRY_START_MS + CARRY_MS);
      CHECK(freed_lowered.elapsed > freed.elapsed);
      // The other kinds' stop speeds are not read here, in either direction
      for (Kind other : ALL_KINDS) {
        if (other == k) continue;
        INFO("other kind " << (int)other);
        CHECK(run_ma(k, row.held_v, row.held_w, [&](Drive& c) { stop_speed_set(c, other, raised(other)); }) == held);
        CHECK(run_ma(k, row.held_v, row.held_w, [&](Drive& c) { stop_speed_set(c, other, lowered(other)); }) == held);
        CHECK(run_ma(k, row.free_v, row.free_w, [&](Drive& c) { stop_speed_set(c, other, raised(other)); }) == freed);
        CHECK(run_ma(k, row.free_v, row.free_w, [&](Drive& c) { stop_speed_set(c, other, lowered(other)); }) == freed);
      }
    }
}

TEST_CASE("an odom move's mA hold follows the odom xy stop speed and no other") {
  Run base = run_ma(Kind::OdomXY, 3.0, 0.0);
  REQUIRE(base.returned);
  Run lo = run_ma(Kind::OdomXY, 3.0, 0.0, [](Drive& c) { c.pid_odom_drive_exit_stop_speed_set(0.4); });
  Run hi = run_ma(Kind::OdomXY, 3.0, 0.0, [](Drive& c) { c.pid_odom_drive_exit_stop_speed_set(6.0); });
  CHECK(lo.returned);
  CHECK(lo.elapsed > base.elapsed);
  CHECK(hi.returned);
  CHECK(hi.elapsed <= base.elapsed);
  for (Kind other : {Kind::Drive, Kind::Turn, Kind::Swing}) {
    CHECK(run_ma(Kind::OdomXY, 3.0, 0.0, [&](Drive& c) { stop_speed_set(c, other, lowered(other)); }) == base);
    CHECK(run_ma(Kind::OdomXY, 3.0, 0.0, [&](Drive& c) { stop_speed_set(c, other, raised(other)); }) == base);
  }
}

TEST_CASE("an odom move carried toward a point with an end heading follows the odom heading stop speed, and no other kind's") {
  // Carried toward a point that also has an end heading, closing on both: the xy hold is what keeps the wait going while xy closes, and the
  // heading's own hold is judged against its own stop speed. Lowered under the carry's 10 deg/s it holds a window longer than the default does.
  auto carried = [](const std::function<void(Drive&)>& configure) { return run_ma(Kind::OdomAngle, 3.0, 10.0, configure); };
  Run base = carried(nullptr);
  REQUIRE(base.returned);
  Run lo = carried([](Drive& c) { c.pid_odom_turn_exit_stop_speed_set(0.4); });
  Run hi = carried([](Drive& c) { c.pid_odom_turn_exit_stop_speed_set(16.0); });
  CHECK(lo.returned);
  CHECK(lo.elapsed > base.elapsed);
  CHECK(hi.returned);
  CHECK(hi.elapsed <= base.elapsed);
  for (Kind other : {Kind::Drive, Kind::Turn, Kind::Swing}) {
    CHECK(carried([&](Drive& c) { stop_speed_set(c, other, lowered(other)); }) == base);
    CHECK(carried([&](Drive& c) { stop_speed_set(c, other, raised(other)); }) == base);
  }
}

// ---- What the stop speeds do not touch --------------------------------------------------------------------------------------------

TEST_CASE("a robot held outside big_error is reported stuck at the same time whatever the stop speed is") {
  // The stuck check outside big_error (1 in over 350 ms, about 2.86 in/s) is its own number: it asks about progress, not about stopped, so
  // raising the drive stop speed above it or lowering it close to zero changes nothing there
  Row row{"drive held", archetype_classroom(), [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  auto held = [&](double v) {
    Rig r(row.arch, 1, false, 1);
    r.chassis.pid_drive_exit_stop_speed_set(v);
    r.sim.pin(300, 1e7);
    row.start(r.chassis);
    Run out;
    test_stub::capture_stdout([&] { out.returned = r.wait([&] { row.wait(r.chassis); }, CAP_TICKS, &out.elapsed); });
    out.interfered = r.chassis.interfered;
    return out;
  };
  Run base = held(1.5);
  REQUIRE(base.returned);
  CHECK(base.interfered);
  for (double v : {0.1, 0.4, 2.9, 5.0, 10.0, 100.0}) {
    CAPTURE(v);
    Run o = held(v);
    CHECK(o.returned);
    CHECK(o.interfered);
    CHECK(o.elapsed == base.elapsed);
  }
}

TEST_CASE("a stop speed over the velocity exit's own threshold ends the wait on the window exit, never later than the velocity exit would") {
  // The velocity exit (a speed under 0.05 in or deg in one 10 ms pass, which is 5 in/s or 5 deg/s, for velocity_exit_time) is a fixed number
  // of its own. A stop speed above it makes every window exit the first thing to fire, so a raised stop speed can only end a wait sooner
  for (const auto& row : rows()) {
    INFO(row.name);
    Run base = run(row);
    for (Kind k : row.must) {
      Run above = run(row, [&](Drive& c) { stop_speed_set(c, k, is_angle(k) ? 40.0 : 10.0); });
      CHECK(above.returned);
      CHECK_FALSE(above.interfered);
      CHECK(above.elapsed <= base.elapsed);
    }
  }
}

// ---- The cost of a lowered stop speed ---------------------------------------------------------------------------------------------

TEST_CASE("a lowered stop speed only ever ends a wait later, and by about step over stop speed for a robot that never gets that still") {
  // The stuck watch's backstop is a step (3 degrees, 1 inch) divided by the stop speed, so how long a robot that hunts about its target waits
  // follows the speed: a heavy robot's turn, whose last degrees never get under the speed, comes back after about 3 deg / speed plus the
  // 1 s start allowance (0.5 deg/s: 7 s; 1 deg/s: 4 s; 2 deg/s: 2.5 s; 4 deg/s, the default: 1.76 s)
  Row turn{"heavy turn", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  double last = 0;
  for (double v : {16.0, 8.0, 4.0, 2.0, 1.0, 0.5}) {
    CAPTURE(v);
    Run o = run(turn, [&](Drive& c) { c.pid_turn_exit_stop_speed_set(v); });
    CHECK(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(o.elapsed >= last);
    last = o.elapsed;
    if (v <= 2.0) {
      double step_over_speed_ms = 3.0 / v * 1000.0;
      CHECK(o.elapsed >= step_over_speed_ms);
      CHECK(o.elapsed <= step_over_speed_ms + 1500);
    }
  }
}

TEST_CASE("the same cost shows in pid_wait_until on a checkpoint past the target, for a turn and for a swing") {
  // pid_wait_until() has its own watch and gate. A checkpoint past the final target is never crossed, so it ends when the motion settles
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
    bool is_turn = row == &turn;
    double last = 0;
    for (double v : {8.0, 4.0, 2.0, 1.0}) {
      CAPTURE(v);
      Run o = run(*row, [&](Drive& c) {
        if (is_turn)
          c.pid_turn_exit_stop_speed_set(v);
        else
          c.pid_swing_exit_stop_speed_set(v);
      });
      CHECK(o.returned);
      CHECK(o.elapsed >= last);
      last = o.elapsed;
      if (v <= 2.0) CHECK(o.elapsed >= 3.0 / v * 1000.0);
    }
    // And the other kind's speed is not what does it
    Run base = run(*row);
    Run other = run(*row, [&](Drive& c) {
      if (is_turn)
        c.pid_swing_exit_stop_speed_set(0.5);
      else
        c.pid_turn_exit_stop_speed_set(0.5);
    });
    CHECK(other == base);
  }
}

TEST_CASE("a stop speed far below anything real makes the backstop very long, not short and not an overflow") {
  Row turn{"heavy turn", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  // 3 deg / 1e-9 deg/s is 3e9 s: far past an int of milliseconds, which has to saturate and not wrap to something short (or to 0, no backstop)
  for (double v : {1e-9, 1e-30, 1e-300}) {
    CAPTURE(v);
    Run o = run(turn, [&](Drive& c) { c.pid_turn_exit_stop_speed_set(v); }, {}, 1200);
    CHECK_FALSE(o.returned);  // still waiting after 12 s of sim time, where the default came back in 1.76 s
  }
}

TEST_CASE("a stop speed far above anything real ends a wait no later than a large one, and clean") {
  Row drive{"classroom drive", archetype_classroom(), [](Drive& c) { c.pid_drive_set(24_in, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Row turn{"heavy turn", sim::archetype_heavy_slow(), [](Drive& c) { c.pid_turn_set(90_deg, 110); }, [](Drive& c) { c.pid_wait(); }, {}, {}};
  Run d10 = run(drive, [](Drive& c) { c.pid_drive_exit_stop_speed_set(10.0); });
  for (double v : {1e3, 1e9, 1e300}) {
    CAPTURE(v);
    Run o = run(drive, [&](Drive& c) { c.pid_drive_exit_stop_speed_set(v); });
    CHECK(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(o.elapsed <= d10.elapsed);
  }
  Run t40 = run(turn, [](Drive& c) { c.pid_turn_exit_stop_speed_set(40.0); });
  for (double v : {1e3, 1e9, 1e300}) {
    CAPTURE(v);
    Run o = run(turn, [&](Drive& c) { c.pid_turn_exit_stop_speed_set(v); });
    CHECK(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(o.elapsed <= t40.elapsed);
  }
}
