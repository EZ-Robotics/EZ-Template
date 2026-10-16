// pid_wait_until(checkpoint) on a turn or swing must treat a checkpoint on the motion's own path as
// the point along that path, whatever current_angle_behavior the motion was set with.
//
// wait_until_turn_swing() used to re-resolve the checkpoint against the LIVE heading under the
// motion's current_angle_behavior. For cw/ccw/longest that pushes a checkpoint the robot has
// already passed (or one that belongs to the long way round) a full revolution away, so the
// "checkpoint crossed" sign never flipped: the wait blocked until the whole motion settled and the
// checkpoint-vs-real-target check then reported that healthy, settled motion as interfered=true.
// shortest was unaffected because the live-heading resolve lands on the same value there.
//
// Sim-backed (light_fast archetype, no noise, seed 1), a turn or swing to 90 degrees with each
// behavior; the checkpoint sits 30% of the way along the motion's own resolved path.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
enum class Kind {
  turn,
  swing_left,
  swing_right
};

const char* kind_name(Kind k) { return k == Kind::turn ? "turn" : (k == Kind::swing_left ? "swing_left" : "swing_right"); }
const char* behavior_name(e_angle_behavior b) {
  switch (b) {
    case shortest:
      return "shortest";
    case cw:
      return "cw";
    case ccw:
      return "ccw";
    case longest:
      return "longest";
    default:
      return "other";
  }
}

Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

struct Outcome {
  bool returned = false;
  bool interfered = false;
  std::uint32_t elapsed_ms = 0;
  double progress_at_return = 0.0;  // fraction of the resolved path covered when the wait returned
  double internal_target = 0.0;
};

// enter_after_progress < 0: call the wait right after the motion is set.  Otherwise let the sim run
// until that fraction of the path is covered first, then call it.
Outcome run_case(Kind kind, e_angle_behavior behavior, double checkpoint_frac, double enter_after_progress, int spelling_turns = 0) {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);
  chassis.pid_print_toggle(false);

  if (kind == Kind::turn) {
    chassis.pid_turn_set(90.0, 110, behavior);
  } else {
    chassis.pid_swing_set(kind == Kind::swing_left ? ez::LEFT_SWING : ez::RIGHT_SWING, 90.0, 110, behavior, true);
  }

  Outcome o;
  double start = chassis.drive_angle_get();
  o.internal_target = kind == Kind::turn ? chassis.turnPID.target_get() : chassis.swingPID.target_get();
  double path = o.internal_target - start;
  auto progress = [&] { return (chassis.drive_angle_get() - start) / path; };

  if (enter_after_progress >= 0.0) {
    for (int i = 0; i < 2000 && progress() < enter_after_progress; i++) pros::delay(10);
    REQUIRE(progress() >= enter_after_progress);
  }

  double checkpoint = start + checkpoint_frac * path + 360.0 * spelling_turns;
  std::uint32_t t0 = test_stub::g_clock.now_ms;
  test_stub::g_clock.delay_calls_until_stop = 3000;  // 30 s cap
  o.returned = true;
  try {
    chassis.pid_wait_until(checkpoint);
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  o.elapsed_ms = test_stub::g_clock.now_ms - t0;
  o.interfered = chassis.interfered;
  o.progress_at_return = progress();
  return o;
}

const e_angle_behavior kBehaviors[] = {shortest, cw, ccw, longest};
const Kind kKinds[] = {Kind::turn, Kind::swing_left, Kind::swing_right};
}  // namespace

TEST_CASE("pid_wait_until() turn/swing: an on-path checkpoint returns at the crossing, not interfered, for every angle behavior") {
  for (Kind k : kKinds) {
    for (e_angle_behavior b : kBehaviors) {
      Outcome o = run_case(k, b, 0.3, /*enter_after_progress=*/-1.0);
      INFO("kind=", std::string(kind_name(k)), " behavior=", std::string(behavior_name(b)), " internal_target=", o.internal_target, " returned=", o.returned,
           " interfered=", o.interfered, " elapsed_ms=", o.elapsed_ms, " progress_at_return=", o.progress_at_return);
      CHECK(o.returned);
      CHECK_FALSE(o.interfered);
      // Returned around the crossing (30%), well before the motion settles at 100%.
      CHECK(o.progress_at_return >= 0.3);
      CHECK(o.progress_at_return < 0.7);
    }
  }
}

TEST_CASE("pid_wait_until() turn/swing: a checkpoint already passed at entry returns promptly, not interfered, for every angle behavior") {
  for (Kind k : kKinds) {
    for (e_angle_behavior b : kBehaviors) {
      Outcome o = run_case(k, b, 0.3, /*enter_after_progress=*/0.5);
      INFO("kind=", std::string(kind_name(k)), " behavior=", std::string(behavior_name(b)), " internal_target=", o.internal_target, " returned=", o.returned,
           " interfered=", o.interfered, " elapsed_ms=", o.elapsed_ms, " progress_at_return=", o.progress_at_return);
      CHECK(o.returned);
      CHECK_FALSE(o.interfered);
      CHECK(o.elapsed_ms <= 30);
    }
  }
}

// The same on-path checkpoint spelled a full revolution away (e.g. -81 vs 279) is the same
// heading, so it must behave the same as the direct spelling.
TEST_CASE("pid_wait_until() turn/swing: an on-path checkpoint spelled a revolution away behaves like the direct spelling") {
  for (Kind k : kKinds) {
    for (e_angle_behavior b : {cw, ccw, longest}) {
      for (int turns : {-1, 1}) {
        Outcome o = run_case(k, b, 0.3, /*enter_after_progress=*/-1.0, turns);
        INFO("kind=", std::string(kind_name(k)), " behavior=", std::string(behavior_name(b)), " spelling_turns=", turns, " returned=", o.returned,
             " interfered=", o.interfered, " progress_at_return=", o.progress_at_return);
        CHECK(o.returned);
        CHECK_FALSE(o.interfered);
        CHECK(o.progress_at_return >= 0.3);
        CHECK(o.progress_at_return < 0.7);
      }
    }
  }
}
