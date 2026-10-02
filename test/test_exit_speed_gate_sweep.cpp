// The undisturbed sweep: every kind of motion a team runs, on every kind of robot, with every set of exits teams use, ended by
// every kind of wait, with nothing touching the robot. What a healthy auton does, and so what must never break.
//
// Every row must return, must not be reported interfered, and (for a return that settles, as opposed to one that crosses a
// checkpoint on purpose) must not settle while the robot is still moving at or above the stop speed over the window of the exit
// that ended it. After a pid_wait() the robot must also have stopped: it may travel at most 0.75 in (2 degrees) in the next
// 500 ms, which is the floor times 500 ms, rounded.
//
// The speed of the robot is the sim's own, never the library's. Set EZ_SWEEP_CSV=<path> to write one line per row (used to
// compare against another commit); the rows are the same on every commit.
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

struct ExitSet {
  const char* name;
  bool defaults;  // leave whatever drive_defaults_set() chose
  int small_t;
  double small_e;
  int big_t;
  double big_e;
  int vel_t;
  int mA_t;
};

const ExitSet EXIT_SETS[] = {
    {"defaults", true, 0, 0, 0, 0, 0, 0},   {"2550R", false, 90, 1, 200, 3, 100, 100},  {"fast", false, 40, 3, 150, 7, 150, 300},
    {"v50", false, 90, 1, 250, 3, 50, 500}, {"mA-only", false, 250, 1, 250, 3, 0, 100}, {"loose", false, 150, 3, 400, 6, 500, 750},
};

void apply(Drive& c, const ExitSet& e) {
  if (e.defaults) return;
  c.pid_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_turn_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_swing_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_odom_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  c.pid_odom_turn_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
}

enum class Kind {
  Drive,
  Turn,
  Swing,
  Point,
  Path
};

struct Motion {
  std::string name;
  Kind kind;
  double target;  // inches or degrees, signed; for a point, the distance from the origin
  pose point;     // Point: where
  std::function<void(Drive&)> set;
};

std::vector<Motion> motions() {
  std::vector<Motion> m;
  for (int len : {6, 24, 48})
    for (int speed : {40, 80, 110})
      for (int dir : {1, -1}) {
        double t = dir * len;
        m.push_back({"drive" + std::to_string((int)t) + "@" + std::to_string(speed), Kind::Drive, t, {}, [=](Drive& c) { c.pid_drive_set(t, speed); }});
      }
  for (int deg : {15, 90, 180})
    for (int speed : {60, 110})
      m.push_back(
          {"turn" + std::to_string(deg) + "@" + std::to_string(speed), Kind::Turn, (double)deg, {}, [=](Drive& c) { c.pid_turn_set((double)deg, speed); }});
  for (int deg : {45, 90}) {
    m.push_back({"lswing" + std::to_string(deg), Kind::Swing, (double)deg, {}, [=](Drive& c) { c.pid_swing_set(ez::LEFT_SWING, (double)deg, 110); }});
    m.push_back({"rswing" + std::to_string(deg), Kind::Swing, -(double)deg, {}, [=](Drive& c) { c.pid_swing_set(ez::RIGHT_SWING, -(double)deg, 110); }});
  }
  m.push_back({"pt(12,24)", Kind::Point, std::hypot(12.0, 24.0), {12, 24, 0}, [](Drive& c) { c.pid_odom_set({{12_in, 24_in}, fwd, 110}); }});
  m.push_back({"pt(-24,0)", Kind::Point, 24.0, {-24, 0, 0}, [](Drive& c) { c.pid_odom_set({{-24_in, 0_in}, fwd, 110}); }});
  m.push_back({"path3", Kind::Path, 0.0, {24, 24, 0}, [](Drive& c) {
                 c.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{12_in, 24_in}, fwd, 110}, {{24_in, 24_in}, fwd, 110}});
               }});
  return m;
}

struct Wait {
  std::string name;
  bool crossing;  // may return mid motion by crossing a checkpoint, by design
  int which;      // 0 pid_wait, 1 quick, 2 quick_chain, 3 until 50%, 4 until target - big/2, 5 until target
};

const Wait WAITS[] = {{"wait", false, 0},    {"quick", true, 1},          {"chain", true, 2},
                      {"until50%", true, 3}, {"until-half-big", true, 4}, {"until-target", true, 5}};

// What ended a wait, from what it printed
enum class End {
  Crossing,
  Small,
  Big,
  StuckSettled,
  mA,
  Other
};

End classify(const std::string& out) {
  if (out.find("Wait Until Exit Success") != std::string::npos) return End::Crossing;
  if (out.find("Small") != std::string::npos) return End::Small;
  if (out.find("Big") != std::string::npos) return End::Big;
  if (out.find("counted as settled") != std::string::npos) return End::StuckSettled;
  if (out.find("mA") != std::string::npos) return End::mA;
  return End::Other;
}

struct Row {
  std::string key;
  bool returned = false;
  double elapsed_ms = 0;
  bool interfered = false;
  double final_error = 0;
  End end = End::Other;
  double speed_at_end = 0;  // over the ending exit's window, in/s or deg/s
  double speed_limit = 0;
  double after = 0;  // travel in the next 500 ms, in or deg
  bool angular = false;
};

double wait_window(const PID& pid, End e) {
  switch (e) {
    case End::Small:
      return pid.exit.small_exit_time;
    case End::Big:
      return pid.exit.big_exit_time;
    case End::mA:
      return pid.exit.mA_timeout;
    default:
      return pid.exit.velocity_exit_time != 0 ? pid.exit.velocity_exit_time : pid.exit.mA_timeout;
  }
}

Row run_row(const sim::SimArchetype& arch, int passes, const ExitSet& es, const Motion& mo, const Wait& w, const std::string& arch_label) {
  Row row;
  row.key = arch_label + "," + es.name + "," + mo.name + "," + w.name;
  Rig r(arch, passes);
  r.chassis.pid_print_toggle(true);
  apply(r.chassis, es);

  auto big_of = [&]() {
    switch (mo.kind) {
      case Kind::Drive:
        return r.chassis.leftPID.exit.big_error;
      case Kind::Turn:
        return r.chassis.turnPID.exit.big_error;
      case Kind::Swing:
        return r.chassis.swingPID.exit.big_error;
      default:
        return r.chassis.xyPID.exit.big_error;
    }
  };
  auto do_wait = [&]() {
    switch (w.which) {
      case 0:
        r.chassis.pid_wait();
        break;
      case 1:
        r.chassis.pid_wait_quick();
        break;
      case 2:
        r.chassis.pid_wait_quick_chain();
        break;
      default: {
        if (mo.kind == Kind::Path) {
          r.chassis.pid_wait_quick();
          break;
        }
        if (mo.kind == Kind::Point) {
          double d = mo.target;
          double at = w.which == 3 ? d * 0.5 : (w.which == 4 ? d - big_of() / 2.0 : d);
          r.chassis.pid_wait_until(pose{mo.point.x * at / d, mo.point.y * at / d, 0});
          break;
        }
        double at = w.which == 3 ? mo.target * 0.5 : (w.which == 4 ? mo.target - std::copysign(big_of() / 2.0, mo.target) : mo.target);
        r.chassis.pid_wait_until(at);
      }
    }
  };

  std::string out = test_stub::capture_stdout([&]() {
    mo.set(r.chassis);
    row.returned = r.wait(do_wait, 4000, &row.elapsed_ms);
  });
  row.interfered = r.chassis.interfered;
  row.end = classify(out);

  // Where it ended up against where it was sent
  switch (mo.kind) {
    case Kind::Drive:
      row.final_error = std::fabs(mo.target - (r.sim.left().position_in + r.sim.right().position_in) / 2.0);
      break;
    case Kind::Turn:
    case Kind::Swing:
      row.final_error = std::fabs(mo.target - r.chassis.drive_angle_get());
      break;
    default:
      row.final_error = std::hypot(mo.point.x - r.chassis.odom_x_get(), mo.point.y - r.chassis.odom_y_get());
      break;
  }

  // How fast it was going when it settled, over the window of the exit that ended it
  if (row.returned && row.end != End::Crossing) {
    PID& pid = mo.kind == Kind::Drive   ? r.chassis.leftPID
               : mo.kind == Kind::Turn  ? r.chassis.turnPID
               : mo.kind == Kind::Swing ? r.chassis.swingPID
                                        : r.chassis.xyPID;
    int window = (int)std::max(wait_window(pid, row.end), 20.0);
    row.angular = mo.kind == Kind::Turn || mo.kind == Kind::Swing;
    if (mo.kind == Kind::Drive) {
      row.speed_at_end = r.drive_speed_over(window);
      row.speed_limit = FLOOR_DISTANCE;
    } else if (row.angular) {
      row.speed_at_end = r.angle_speed_over(window);
      row.speed_limit = FLOOR_ANGLE;
    } else {
      // An odom move: xy is the pose, the average of the two sides, and the heading is its own axis
      row.speed_at_end = r.speed_over(&Sample::avg, window);
      row.speed_limit = FLOOR_DISTANCE;
      if (r.angle_speed_over(window) >= FLOOR_ANGLE) row.speed_at_end = 1e9;  // the heading axis was still moving
    }
  }
  if (row.returned && w.which == 0) {
    auto after = r.run_on(500);
    row.angular = mo.kind == Kind::Turn || mo.kind == Kind::Swing;
    row.after = row.angular ? after.angle : after.distance;
  }
  return row;
}

struct Totals {
  std::vector<std::string> not_returned, interfered, fast_settle, after_big;
  int rows = 0, after_over_tight = 0, after_rows = 0;
  std::vector<double> elapsed, final_error;
};

void run_sweep(const sim::SimArchetype& arch, int passes, const std::string& label) {
  const char* csv_path = std::getenv("EZ_SWEEP_CSV");
  std::FILE* csv = csv_path != nullptr ? std::fopen(csv_path, "a") : nullptr;
  std::map<std::string, Totals> per_set;
  auto ms = motions();
  for (const auto& es : EXIT_SETS) {
    Totals& t = per_set[es.name];
    for (const auto& mo : ms)
      for (const auto& w : WAITS) {
        Row row = run_row(arch, passes, es, mo, w, label);
        t.rows++;
        t.elapsed.push_back(row.elapsed_ms);
        t.final_error.push_back(row.final_error);
        if (csv != nullptr)
          std::fprintf(csv, "%s,%d,%.0f,%d,%.3f,%d,%.3f,%.3f,%.3f\n", row.key.c_str(), row.returned, row.elapsed_ms, row.interfered, row.final_error,
                       (int)row.end, row.speed_at_end, row.speed_limit, row.after);
        if (!row.returned) t.not_returned.push_back(row.key);
        if (row.returned && row.interfered) t.interfered.push_back(row.key);
        if (row.returned && row.end != End::Crossing && row.speed_limit > 0 && row.speed_at_end >= row.speed_limit)
          t.fast_settle.push_back(row.key + " (" + std::to_string(row.speed_at_end) + " vs " + std::to_string(row.speed_limit) + ")");
        if (row.returned && w.which == 0) {
          t.after_rows++;
          double tight = row.angular ? 1.0 : 0.3;
          double loose = row.angular ? 2.0 : 0.75;
          if (row.after > tight) t.after_over_tight++;
          if (row.after > loose) t.after_big.push_back(row.key + " (" + std::to_string(row.after) + ")");
        }
      }
  }
  if (csv != nullptr) std::fclose(csv);

  for (const auto& es : EXIT_SETS) {
    Totals& t = per_set[es.name];
    std::vector<double> e = t.elapsed;
    std::sort(e.begin(), e.end());
    std::vector<double> fe = t.final_error;
    std::sort(fe.begin(), fe.end());
    auto pct = [](const std::vector<double>& v, double p) { return v.empty() ? 0.0 : v[std::min(v.size() - 1, (size_t)(p * v.size()))]; };
    std::printf(
        "  [sweep] %-22s %-9s rows=%d no_return=%zu interfered=%zu fast_settle=%zu after_big=%zu after>tight=%d/%d  return ms med/p95/max=%.0f/%.0f/%.0f  final err med/p95=%.2f/%.2f\n",
        label.c_str(), es.name, t.rows, t.not_returned.size(), t.interfered.size(), t.fast_settle.size(), t.after_big.size(), t.after_over_tight, t.after_rows,
        pct(e, 0.5), pct(e, 0.95), e.empty() ? 0.0 : e.back(), pct(fe, 0.5), pct(fe, 0.95));
    auto report = [&](const char* what, const std::vector<std::string>& v) {
      std::string text;
      for (size_t i = 0; i < v.size() && i < 8; i++) text += "\n    " + v[i];
      std::string message = label + " " + std::string(es.name) + ": " + std::string(what) + " (" + std::to_string(v.size()) + ")" + text;
      INFO(message);
      CHECK(v.empty());
    };
    report("a wait did not return", t.not_returned);
    report("interfered on an undisturbed run", t.interfered);
    report("settled while still moving at or above the stop speed", t.fast_settle);
    report("moved more than 0.75 in (2 deg) in the 500 ms after pid_wait() returned", t.after_big);
  }
}

}  // namespace

TEST_CASE("sweep: light_fast") { run_sweep(sim::archetype_light_fast(), 1, "light_fast"); }
TEST_CASE("sweep: sticky_high_friction") { run_sweep(sim::archetype_sticky_high_friction(), 1, "sticky_high_friction"); }
TEST_CASE("sweep: classroom") { run_sweep(archetype_classroom(), 1, "classroom"); }
TEST_CASE("sweep: heavy_slow at 2 passes per poll") { run_sweep(sim::archetype_heavy_slow(), 2, "heavy_slow/2"); }
TEST_CASE("sweep: heavy_slow at 3 passes per poll") { run_sweep(sim::archetype_heavy_slow(), 3, "heavy_slow/3"); }
