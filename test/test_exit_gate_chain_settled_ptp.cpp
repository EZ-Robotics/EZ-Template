// A chain wait entered on a point to point move that has already settled must read clean. pid_wait_quick_chain() moves the move's
// target a few inches past the checkpoint, so a robot resting on the checkpoint is that far from the target the exits are measured
// to. The settle check credited a robot resting within big_error of that pushed target for paths and boomerangs, but not for a
// point to point move, so it measured the resting robot against the checkpoint alone and called a healthy arrival interfered
// whenever big_error was a little under the push distance (3 in).
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

sim::SimArchetype four_inch() {
  auto a = sim::archetype_light_fast();
  a.wheel_diameter_in = 4.0;
  return a;
}

enum class Kind {
  Ptp,
  PtpRev,
  Boomerang,
  BoomerangRev
};

struct Cfg {
  const char* name;
  bool shipped;  // leave the library's own exit constants alone
  double small_error;
  double big_error;
};

void start(Drive& c, Kind k, pose& point) {
  switch (k) {
    case Kind::Ptp:
      point = {12, 24};
      c.pid_odom_ptp_set(odom{point, fwd, 110});
      break;
    case Kind::PtpRev:
      point = {-12, -24};
      c.pid_odom_ptp_set(odom{point, rev, 110});
      break;
    case Kind::Boomerang:
      point = {12, 24, 45};
      c.pid_odom_boomerang_set(odom{point, fwd, 110});
      break;
    case Kind::BoomerangRev:
      point = {-10, -24, 200};
      c.pid_odom_boomerang_set(odom{point, rev, 110});
      break;
  }
}

}  // namespace

TEST_CASE("settled point to point move: pid_wait_quick_chain reads clean (fwd, rev, boomerang)") {
  const Cfg cfgs[] = {{"shipped", true, 1.0, 3.0}, {"small 1, big 3, windows 200/500", false, 1.0, 3.0}, {"small 0.3, big 2.8", false, 0.3, 2.8}};
  for (const Cfg& cfg : cfgs)
    for (Kind k : {Kind::Ptp, Kind::PtpRev, Kind::Boomerang, Kind::BoomerangRev})
      for (int seq = 0; seq < 3; seq++)  // 0: wait, 1: until_point, 2: quick; then the chain wait
        for (int passes : {1, 2, 3}) {
          Rig r(four_inch(), passes);
          if (!cfg.shipped) r.chassis.pid_odom_drive_exit_condition_set(100, cfg.small_error, 200, cfg.big_error, 200, 500);
          r.chassis.pid_odom_turn_exit_condition_set(0, 0.0, 250, 6.0, 200, 500);
          pose point;
          bool first = false, second = false;
          bool first_clean = false;
          std::string out = test_stub::capture_stdout([&]() {
            start(r.chassis, k, point);
            first = r.wait(
                [&] {
                  if (seq == 0) r.chassis.pid_wait();
                  if (seq == 1) r.chassis.pid_wait_until_point(point);
                  if (seq == 2) r.chassis.pid_wait_quick();
                },
                2500);
            first_clean = !r.chassis.interfered;
            second = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 2500);
          });
          INFO("config " << std::string(cfg.name) << " kind " << (int)k << " first wait " << seq << " passes " << passes);
          REQUIRE(first);
          REQUIRE(first_clean);
          REQUIRE(second);
          CHECK_FALSE(r.chassis.interfered);
        }
}

TEST_CASE("settled point to point move: a chain wait that comes well after the arrival reads clean") {
  for (Kind k : {Kind::Ptp, Kind::PtpRev})
    for (int passes : {1, 2, 3})
      for (int gap : {600, 2500}) {
        Rig r(four_inch(), passes);
        r.chassis.pid_odom_drive_exit_condition_set(100, 0.3, 200, 2.8, 200, 500);
        r.chassis.pid_odom_turn_exit_condition_set(0, 0.0, 250, 6.0, 200, 500);
        pose point;
        bool ok = false;
        std::string out = test_stub::capture_stdout([&]() {
          start(r.chassis, k, point);
          REQUIRE(r.wait([&] { r.chassis.pid_wait_quick(); }, 2500));
          for (int t = 0; t < gap; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
          ok = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 2500);
        });
        INFO("kind " << (int)k << " passes " << passes << " gap " << gap);
        REQUIRE(ok);
        CHECK_FALSE(r.chassis.interfered);
      }
}

// Controls: the robot has to be where the move goes. Held further than big_error short of the checkpoint, it is still interfered
TEST_CASE("point to point chain wait: a robot held more than big_error short of the checkpoint still reads interfered") {
  for (Kind k : {Kind::Ptp, Kind::PtpRev})
    for (double big : {2.8, 3.0})
      for (int late : {0, 1}) {
        Rig r(four_inch(), 2);
        r.chassis.pid_odom_drive_exit_condition_set(100, 0.3, 200, big, 200, 500);
        r.chassis.pid_odom_turn_exit_condition_set(0, 0.0, 250, 6.0, 200, 500);
        bool returned = false;
        std::string out = test_stub::capture_stdout([&]() {
          double sgn = k == Kind::Ptp ? 1.0 : -1.0;
          r.chassis.pid_odom_ptp_set(odom{pose{sgn * 24, sgn * 48}, k == Kind::Ptp ? fwd : rev, 110});
          r.sim.pin(200, 20000);
          if (late)
            for (int t = 0; t < 1500; t += util::DELAY_TIME) pros::delay(util::DELAY_TIME);
          returned = r.wait([&] { r.chassis.pid_wait_quick_chain(); }, 3000);
        });
        INFO("kind " << (int)k << " big " << big << " late " << late);
        REQUIRE(returned);
        CHECK(r.chassis.interfered);
      }
}
