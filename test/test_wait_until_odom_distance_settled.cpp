// pid_wait_until(distance) at the end of an odom move settles like pid_wait() does on the move's last point. On an odom move the left and
// right exits aim at a look-ahead point, so the wait ends through the stuck watch, and the verdict used to force "not settled" for every odom
// move: a robot that drove the whole distance and came to rest inside its settle error read interfered when the checkpoint was the move's own
// distance, where pid_wait() and pid_wait_until(point) on the same motion read clean. On the last point the robot is now settled when it is
// inside settle_error() of the final target in xy and (when the move has a heading) in angle, and the checkpoint counts as reached when it is
// between where the robot came to rest and the final target. A robot that stopped short of a checkpoint it could reach, or anywhere but the
// last point, still reads interfered.
#include <cmath>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Setup {
  const char* name;
  sim::SimArchetype arch;
  int passes;
};

std::vector<Setup> setups() {
  return {{"light_fast", sim::archetype_light_fast(), 1},
          {"sticky", sim::archetype_sticky_high_friction(), 1},
          {"classroom", archetype_classroom(), 1},
          {"heavy_slow/2", sim::archetype_heavy_slow(), 2},
          {"heavy_slow/3", sim::archetype_heavy_slow(), 3}};
}

struct ExitSet {
  int small_t;
  double small_e;
  int big_t;
  double big_e;
  int vel_t;
  int mA_t;
};

// The exits the audit measured: the first is the one the issue was found with
const ExitSet EXIT_SETS[] = {{250, 1, 500, 3, 100, 250}, {90, 1, 250, 3, 50, 250}, {150, 1, 300, 3, 100, 750}};

struct Outcome {
  bool returned = false;
  bool interfered = false;
  double ms = 0;
};

Outcome wait_until_own_distance(const Setup& s, bool noise, const ExitSet& e, double distance, int speed) {
  Rig r(s.arch, s.passes, noise, 11);
  r.chassis.pid_odom_drive_exit_condition_set(e.small_t, e.small_e, e.big_t, e.big_e, e.vel_t, e.mA_t);
  Outcome o;
  r.chassis.pid_odom_set(distance, speed, true);
  o.returned = r.wait([&] { r.chassis.pid_wait_until(distance); }, 1500, &o.ms);
  o.interfered = r.chassis.interfered;
  return o;
}

void own_distance_reads_clean(const ExitSet& e, const char* label) {
  int runs = 0, interfered = 0;
  std::string first;
  for (auto& s : setups())
    for (bool noise : {false, true})
      for (int speed : {60, 110})
        for (double distance : {8.0, 24.0, 48.0, -24.0}) {
          Outcome o = wait_until_own_distance(s, noise, e, distance, speed);
          std::string what =
              std::string(s.name) + " noise=" + std::to_string(noise) + " speed=" + std::to_string(speed) + " D=" + std::to_string((int)distance);
          CHECK_MESSAGE(o.returned, what << ": did not return");
          runs++;
          if (o.interfered) {
            interfered++;
            if (first.empty()) first = what;
          }
        }
  CHECK_MESSAGE(interfered == 0, std::string(label) << ": " << interfered << " of " << runs << " runs read interfered, the first: " << first);
}

}  // namespace

TEST_CASE("pid_wait_until(its own distance) on a straight odom move that settled at the target reads clean") {
  own_distance_reads_clean(EXIT_SETS[0], "odom drive exits (250, 1, 500, 3, 100, 250)");
}

TEST_CASE("the same with the other two exit sets teams use") {
  own_distance_reads_clean(EXIT_SETS[1], "odom drive exits (90, 1, 250, 3, 50, 250)");
  own_distance_reads_clean(EXIT_SETS[2], "odom drive exits (150, 1, 300, 3, 100, 750)");
}

// Controls: the same wait, but the robot did not get there
TEST_CASE("control: a wall 6 in before the checkpoint still reads interfered") {
  for (auto& s : setups())
    for (bool noise : {false, true}) {
      Rig r(s.arch, s.passes, noise, 11);
      r.chassis.pid_odom_drive_exit_condition_set(250, 1, 500, 3, 100, 250);
      r.sim.wall(18.0);
      r.chassis.pid_odom_set(24.0, 110, true);
      double ms = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait_until(24.0); }, 1500, &ms);
      INFO(std::string(s.name) << " noise=" << noise);
      REQUIRE(ok);
      CHECK(r.chassis.interfered);
    }
}

TEST_CASE("control: a mid-path pid_wait_until(distance) on a three point path with the robot pinned before it still reads interfered") {
  for (auto& s : setups())
    for (bool noise : {false, true}) {
      Rig r(s.arch, s.passes, noise, 11);
      r.chassis.pid_odom_drive_exit_condition_set(250, 1, 500, 3, 100, 250);
      bool pinned = false;
      r.sim.before_pass = [&](int) {
        if (pinned || r.chassis.odom_y_get() < 20.0) return;
        pinned = true;
        r.sim.pin(r.sim.now_ms(), 1e7);
      };
      r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 110}, {{0_in, 24_in}, fwd, 110}, {{0_in, 36_in}, fwd, 110}}, true);
      double ms = 0;
      bool ok = r.wait([&] { r.chassis.pid_wait_until(30.0); }, 1500, &ms);
      INFO(std::string(s.name) << " noise=" << noise);
      REQUIRE(ok);
      CHECK(r.chassis.interfered);
    }
}

// What could go wrong with the rule: a robot held inside the settle error short of the checkpoint now reads clean. That is the rule the
// other waits already follow (inside big_error a stopped robot returns clean even if something is holding it there), and it is what this
// pins down, so it cannot change without somebody meaning it.
TEST_CASE("accepted: a wall 2 in before the checkpoint, inside the 3 in big_error, reads clean") {
  for (auto& s : setups()) {
    Rig r(s.arch, s.passes, false, 11);
    r.chassis.pid_odom_drive_exit_condition_set(250, 1, 500, 3, 100, 250);
    r.sim.wall(22.0);
    r.chassis.pid_odom_set(24.0, 110, true);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait_until(24.0); }, 1500, &ms);
    INFO(std::string(s.name));
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
  }
}
