// Regression table for the bare ez::PID velocity/mA-exit channel, covering stall, twitch, creep,
// disconnected-motor and compute/poll-cadence-mismatch shapes a lift/claw/catapult (a mechanism with
// no StuckWatch/SingleStuckWatch backstop the way Drive's own waits have) can be in. Every scenario
// here also ran clean against this library's own immediate parent commit, from before the velocity
// channel's raw-value staleness/debounce mechanism was removed in favor of gating on the same
// freshness signal the small/big timers use (see PID.cpp's exit_condition() and its own comments),
// and before a disconnected motor's non-finite position started counting toward the mA timer.
//
// Each scenario drives a bare PID (no Drive, no chassis) through a virtual 1ms timeline: `pos(t)` is
// the sensor reading the mechanism would report at time t, and a schedule decides which of those
// milliseconds land a real compute() and/or a poll (exit_condition() call). test_stub::g_clock.now_ms
// is advanced to match `t` every iteration so PID.cpp's own wall-clock crediting of real elapsed time
// (rather than a flat poll count) measures real elapsed time the same way it would against a real
// background task and a real wait loop, not a frozen test clock.
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "doctest.h"
#include "fake_hardware.hpp"

#include "EZ-Template/api.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

using namespace ez;

namespace {

struct Sched {
  std::function<int(int)> computes_at;
  std::function<bool(int)> poll_at;
};
Sched sched_periodic(int cp, int pp, int cphase = 0, int pphase = 0) {
  return {[=](int t) { return (t - cphase) >= 0 && (t - cphase) % cp == 0 ? 1 : 0; }, [=](int t) { return (t - pphase) >= 0 && (t - pphase) % pp == 0; }};
}
// Per-poll compute-count pattern: poll every 10ms, and just before poll n, pattern[n % len] computes
// land (all landing on the same virtual millisecond, so a pattern value of 2 feeds the SAME sensor
// reading into compute() twice in a row -- the "one real compute, checked/computed redundantly"
// shape test_pid.cpp's own unit tests cover in isolation).
Sched sched_pattern(std::vector<int> pat) {
  return {[=](int t) { return t % 10 == 0 ? pat[(t / 10) % pat.size()] : 0; }, [](int t) { return t % 10 == 0; }};
}

enum class MotorMode {
  NONE,
  SINGLE,
  VEC,
  GROUP
};

struct Result {
  exit_output e = RUNNING;
  int t_exit = -1;
};

struct Cfg {
  std::string name;
  std::function<double(int)> pos;  // sensor reading at time t
  int stop_ms;                     // when the mechanism physically stopped (for the "after stop" column)
  Sched sched;
  int small_t = 80, big_t = 300, vel_t = 500, mA_t = 500;
  double small_e = 50, big_e = 150;
  double target = 1000;
  double vzero = 0.05;
  MotorMode motor = MotorMode::NONE;
  bool over_current = false, disconnected = false;
  int max_ms = 8000;
  int over_current_from = -1;           // when >= 0, motor reads over current from this time on
  std::function<void(PID&, int)> hook;  // optional per-ms hook (velocity_exit_hold etc.)
};

Result run(const Cfg& c) {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;

  PID pid(1, 0, 0, 0, "lift");
  pid.exit_condition_set(c.small_t, c.small_e, c.big_t, c.big_e, c.vel_t, c.mA_t);
  pid.velocity_sensor_main_exit_set(c.vzero);
  pid.target_set(c.target);
  pros::Motor m1(11), m2(12);
  std::vector<pros::Motor> vec = {m1, m2};
  pros::MotorGroup grp({11, 12});
  Result r;
  for (int t = 0; t <= c.max_ms; ++t) {
    test_stub::g_clock.now_ms = static_cast<std::uint32_t>(t);  // keep pros::millis() in lockstep with t
    if (c.hook) c.hook(pid, t);
    bool oc = c.over_current || (c.over_current_from >= 0 && t >= c.over_current_from);
    m1.fake().over_current = oc;
    m2.fake().over_current = oc;
    m1.fake().disconnected = c.disconnected;
    m2.fake().disconnected = c.disconnected;
    double p = c.disconnected ? m1.get_position() : c.pos(t);
    m1.fake().position = (int)p;
    int n = c.sched.computes_at(t);
    for (int k = 0; k < n; ++k) pid.compute(p);
    if (c.sched.poll_at(t)) {
      exit_output e;
      switch (c.motor) {
        case MotorMode::NONE:
          e = pid.exit_condition();
          break;
        case MotorMode::SINGLE:
          e = pid.exit_condition(m1);
          break;
        case MotorMode::VEC:
          e = pid.exit_condition(vec);
          break;
        case MotorMode::GROUP:
          e = pid.exit_condition(grp);
          break;
      }
      if (e != RUNNING) {
        r.e = e;
        r.t_exit = t;
        return r;
      }
    }
  }
  return r;
}

std::string ename(exit_output e) {
  switch (e) {
    case RUNNING:
      return "HANG(none)";
    case SMALL_EXIT:
      return "SMALL";
    case BIG_EXIT:
      return "BIG";
    case VELOCITY_EXIT:
      return "VELOCITY";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "ERR_NO_CONST";
  }
  return "?";
}

Result report(const Cfg& c) {
  Result r = run(c);
  MESSAGE(c.name, " | ", ename(r.e), " | start+", r.t_exit, "ms | stop+", (r.t_exit < 0 ? -1 : r.t_exit - c.stop_ms), "ms");
  return r;
}

// --- traces ------------------------------------------------------------------
double ramp_settle(int t) { return t < 500 ? 2.0 * t : 1000.0; }
double stall_400(int t) { return t < 200 ? 2.0 * t : 400.0; }
std::function<double(int)> stall_400_blip_period(int P) {
  return [P](int t) { return t < 200 ? 2.0 * t : 400.0 + ((((t - 200) / 10) % (P / 10) == 0) ? 1 : 0); };
}
double crawl_1_per_50(int t) { return 900.0 + t / 50; }

}  // namespace

TEST_CASE("bare PID velocity/mA timing table: full sweep (diagnostic; see the CHECKed cases below for hard assertions)") {
  Sched s11 = sched_periodic(10, 10);
  Sched s20_10 = sched_periodic(20, 10);
  Sched s10_20 = sched_periodic(10, 20);
  Sched s10_5 = sched_periodic(10, 5);
  Sched unsync = sched_pattern({0, 2});

  report({"ideal ramp->settle, docs constants, 1:1", ramp_settle, 500, s11});
  report({"ideal ramp->settle, single motor overload", ramp_settle, 500, s11, 80, 300, 500, 500, 50, 150, 1000, 0.05, MotorMode::SINGLE});
  report({"claw: hard stop bit-identical, velocity-only (0,0,0,0,500,0)", stall_400, 200, s11, 0, 0, 500, 0, 0, 0});
  report({"lift stalls at 400 (bit-identical), docs consts, no mA", stall_400, 200, s11, 80, 300, 500, 500, 50, 150, 1000, 0.05, MotorMode::SINGLE});
  for (int P : {300, 520, 600, 800, 1000, 1200, 1500, 1520, 1600, 2000})
    report({"stall at 400, 1-tick +1 blip every " + std::to_string(P) + "ms, vel-only", stall_400_blip_period(P), 200, s11, 0, 0, 500, 0, 0, 0});
  report({"crawl 1deg/50ms from 900 toward 1000, docs consts", crawl_1_per_50, 99999, s11});
  report({"compute 20ms / poll 10ms, ramp->settle (small)", ramp_settle, 500, s20_10});
  report({"compute 10ms / poll 20ms, ramp->settle (small)", ramp_settle, 500, s10_20});
  report({"poll twice per compute (5ms), ramp->settle (small)", ramp_settle, 500, s10_5});
  report({"poll twice per compute, velocity-only hard stop", stall_400, 200, s10_5, 0, 0, 500, 0, 0, 0});
  report({"unsync worst case (0,2 computes per poll), small", ramp_settle, 500, unsync});
  report({"unsync worst case (0,2), velocity-only hard stop", stall_400, 200, unsync, 0, 0, 500, 0, 0, 0});
}

// ---- Hard-asserted cases, matching this channel's stated guarantees --------------

TEST_CASE("bare PID: a claw at a bit-identical hard stop, velocity-only, exits within one poll of velocity_exit_time after the stop") {
  Result r = report({"claw hard stop, velocity-only (0,0,0,0,500,0)", stall_400, 200, sched_periodic(10, 10), 0, 0, 500, 0, 0, 0});
  CHECK(r.e == VELOCITY_EXIT);
  CHECK(r.t_exit >= 200 + 500);
  CHECK(r.t_exit <= 200 + 520);
}

TEST_CASE("bare PID: a lift stalled mid-travel, docs constants, single motor overload, still exits (not a hang)") {
  Result r = report({"lift stalls at 400, docs consts, single motor overload", stall_400, 200, sched_periodic(10, 10), 80, 300, 500, 500, 50, 150, 1000, 0.05,
                     MotorMode::SINGLE});
  CHECK(r.e != RUNNING);
  CHECK(r.t_exit >= 0);
}

TEST_CASE("bare PID: a genuinely disconnected motor exits via mA, not by hanging, in every overload shape") {
  Cfg base{"disconnected, docs consts", ramp_settle, 0, sched_periodic(10, 10), 80, 300, 500, 500, 50, 150, 1000, 0.05, MotorMode::SINGLE};
  base.disconnected = true;

  Cfg single = base;
  Result r1 = report(single);
  CHECK(r1.e == mA_EXIT);
  CHECK(r1.t_exit >= 500);
  CHECK(r1.t_exit <= 520);

  Cfg vec = base;
  vec.name = "disconnected, vector overload";
  vec.motor = MotorMode::VEC;
  Result r2 = report(vec);
  CHECK(r2.e == mA_EXIT);
  CHECK(r2.t_exit >= 500);
  CHECK(r2.t_exit <= 520);

  Cfg grp = base;
  grp.name = "disconnected, MotorGroup overload";
  grp.motor = MotorMode::GROUP;
  Result r3 = report(grp);
  CHECK(r3.e == mA_EXIT);
  CHECK(r3.t_exit >= 500);
  CHECK(r3.t_exit <= 520);
}

TEST_CASE("bare PID: a disconnected motor with no motor-overload check still exits via the sanitized velocity channel") {
  // No MotorMode set (NONE): only PID::exit_condition() (no motor argument) runs, so the mA channel
  // is never reached at all -- this exercises the velocity channel's own derivative sanitization in
  // isolation, not the motor-argument overloads' mA handling.
  Cfg c{"disconnected, no-motor overload, docs consts", ramp_settle, 0, sched_periodic(10, 10), 80, 300, 500, 500, 50, 150};
  c.disconnected = true;
  Result r = report(c);
  CHECK(r.e == VELOCITY_EXIT);
  CHECK(r.t_exit >= 500);
  CHECK(r.t_exit <= 520);
}

TEST_CASE("bare PID: a legitimately over-current motor (not disconnected) still mA_EXITs normally") {
  Cfg c{"legit over current from t=0, mA only", ramp_settle, 0, sched_periodic(10, 10), 0, 0, 0, 500, 0, 0, 1000, 0.05, MotorMode::SINGLE, true};
  Result r = report(c);
  CHECK(r.e == mA_EXIT);
  CHECK(r.t_exit >= 500);
  CHECK(r.t_exit <= 520);
}

TEST_CASE("bare PID: a stall with a repeating blip at or above 530ms still eventually exits, matching the parent commit") {
  for (int P : {530, 600, 800, 1000, 1200, 1500, 1520, 1600, 2000}) {
    Cfg c{"stall at 400, blip every " + std::to_string(P) + "ms, vel-only", stall_400_blip_period(P), 200, sched_periodic(10, 10), 0, 0, 500, 0, 0, 0};
    Result r = run(c);
    INFO("period ", P, "ms: ", ename(r.e), " at t=", r.t_exit);
    CHECK(r.e == VELOCITY_EXIT);
  }
}

TEST_CASE("bare PID: mismatched compute/poll cadences no longer double the configured small-exit time") {
  // The small/big/velocity timers credit real elapsed wall-clock milliseconds per fresh poll, not a
  // flat util::DELAY_TIME -- a flat credit undercounts real elapsed time whenever the caller's own
  // poll cadence doesn't match DELAY_TIME.
  // A lift task computing at 20ms while the wait polls at 10ms used to take about 2x as long as the
  // configured 80ms small-exit time to fire. The real-world exit time also isn't simply
  // "stop_ms + small_exit_time" here: ramp_settle's error crosses into the small_error band while
  // still ramping, a few ms before stop_ms -- so this pins every cadence against the measured, known-
  // correct 1:1 baseline instead of a hand-derived formula.
  Cfg baseline_cfg{"1:1 baseline, ramp->settle (small)", ramp_settle, 500, sched_periodic(10, 10)};
  Result baseline = run(baseline_cfg);
  REQUIRE(baseline.e == SMALL_EXIT);

  auto check_matches_baseline = [&](const char* label, Sched sched) {
    Cfg c{label, ramp_settle, 500, sched};
    Result r = run(c);
    INFO(label, ": t_exit=", r.t_exit, " baseline=", baseline.t_exit);
    CHECK(r.e == SMALL_EXIT);
    // Within one poll period of the 1:1 baseline -- exact equality where the two cadences happen to
    // sample the same underlying millisecond, a small bounded difference where they don't.
    CHECK(std::abs(r.t_exit - baseline.t_exit) <= 20);
  };

  check_matches_baseline("compute 20ms / poll 10ms", sched_periodic(20, 10));
  check_matches_baseline("compute 10ms / poll 20ms", sched_periodic(10, 20));
  check_matches_baseline("poll twice per compute (5ms)", sched_periodic(10, 5));
  check_matches_baseline("unsync worst case (0,2 computes per poll)", sched_pattern({0, 2}));
}
