// The stop speed setters: what a team can set for "stopped" (in/s for drive and odom xy, deg/s for turn, swing and odom heading), with a
// plain number or an EZ-Units speed.
//
//   - Nothing set means the old values: 1.5 in/s and 4 deg/s, on every constructor.
//   - A number that is zero, negative, NaN or infinite is refused with a printed warning and the previous value stays. Any other number is taken.
//   - The units forms give the same number the plain form does.
//   - Each setter changes only its own kind of motion.
#include <cmath>
#include <limits>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {

Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double INF = std::numeric_limits<double>::infinity();

// The five stop speeds as a team sees them, in Drive::StopSpeed order: drive, turn, swing, odom xy, odom heading
struct Five {
  double v[5];
  bool operator==(const Five& o) const {
    for (int i = 0; i < 5; i++)
      if (v[i] != o.v[i]) return false;
    return true;
  }
};
Five read(Drive& d) {
  return {{d.pid_drive_exit_stop_speed_get(), d.pid_turn_exit_stop_speed_get(), d.pid_swing_exit_stop_speed_get(), d.pid_odom_drive_exit_stop_speed_get(),
           d.pid_odom_turn_exit_stop_speed_get()}};
}

// One row per setter: its name, how to set it with a number, with units, and which slot it owns
struct Setter {
  const char* name;
  int slot;
  bool angle;
  void (*set)(Drive&, double);
};
const Setter SETTERS[] = {
    {"pid_drive_exit_stop_speed_set", 0, false, [](Drive& d, double v) { d.pid_drive_exit_stop_speed_set(v); }},
    {"pid_turn_exit_stop_speed_set", 1, true, [](Drive& d, double v) { d.pid_turn_exit_stop_speed_set(v); }},
    {"pid_swing_exit_stop_speed_set", 2, true, [](Drive& d, double v) { d.pid_swing_exit_stop_speed_set(v); }},
    {"pid_odom_drive_exit_stop_speed_set", 3, false, [](Drive& d, double v) { d.pid_odom_drive_exit_stop_speed_set(v); }},
    {"pid_odom_turn_exit_stop_speed_set", 4, true, [](Drive& d, double v) { d.pid_odom_turn_exit_stop_speed_set(v); }},
};

}  // namespace

TEST_CASE("stop speeds default to 1.5 in/s and 4 deg/s on every constructor") {
  const Five defaults = {{1.5, 4.0, 4.0, 1.5, 4.0}};
  {
    Drive d = make_chassis();
    CHECK(read(d) == defaults);
  }
  {
    test_stub::reset_all();
    Drive d({1, -2}, {-3, 4});  // driver control only
    CHECK(read(d) == defaults);
  }
  {
    test_stub::reset_all();
    Drive d({1, -2}, {-3, 4}, std::vector<int>{5, 6}, 3.25, 360);  // two imus
    CHECK(read(d) == defaults);
  }
  {
    test_stub::reset_all();
    Drive d({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);  // the compatibility constructor with a ratio
    CHECK(read(d) == defaults);
  }
  // The constants are the same numbers
  CHECK(Drive::STOP_SPEED_DISTANCE_DEFAULT == 1.5);
  CHECK(Drive::STOP_SPEED_ANGLE_DEFAULT == 4.0);
}

TEST_CASE("each setter takes a plain number and changes only its own stop speed") {
  for (const auto& s : SETTERS) {
    INFO(s.name);
    Drive d = make_chassis();
    Five before = read(d);
    s.set(d, 7.25);
    Five after = read(d);
    for (int i = 0; i < 5; i++) {
      INFO("slot " << i);
      if (i == s.slot)
        CHECK(after.v[i] == 7.25);
      else
        CHECK(after.v[i] == before.v[i]);
    }
  }
}

TEST_CASE("the units forms give the same speed as the plain forms") {
  Drive d = make_chassis();

  d.pid_drive_exit_stop_speed_set(2_in / 1_s);
  CHECK(d.pid_drive_exit_stop_speed_get() == doctest::Approx(2.0).epsilon(1e-12));
  d.pid_odom_drive_exit_stop_speed_set(3_in / 1_s);
  CHECK(d.pid_odom_drive_exit_stop_speed_get() == doctest::Approx(3.0).epsilon(1e-12));
  // Another unit of speed, 0.0508 m/s is 2 in/s
  d.pid_drive_exit_stop_speed_set(0.0508_mps);
  CHECK(d.pid_drive_exit_stop_speed_get() == doctest::Approx(2.0).epsilon(1e-9));

  d.pid_turn_exit_stop_speed_set(6_deg / 1_s);
  CHECK(d.pid_turn_exit_stop_speed_get() == doctest::Approx(6.0).epsilon(1e-12));
  d.pid_swing_exit_stop_speed_set(8_deg / 1_s);
  CHECK(d.pid_swing_exit_stop_speed_get() == doctest::Approx(8.0).epsilon(1e-12));
  d.pid_odom_turn_exit_stop_speed_set(5_deg / 1_s);
  CHECK(d.pid_odom_turn_exit_stop_speed_get() == doctest::Approx(5.0).epsilon(1e-12));
  // 1 rpm is a full turn a minute: 6 deg/s
  d.pid_turn_exit_stop_speed_set(1_rpm);
  CHECK(d.pid_turn_exit_stop_speed_get() == doctest::Approx(6.0).epsilon(1e-9));

  // The other forms take a time that is not a second
  d.pid_swing_exit_stop_speed_set(3_deg / 500_ms);
  CHECK(d.pid_swing_exit_stop_speed_get() == doctest::Approx(6.0).epsilon(1e-9));
  d.pid_drive_exit_stop_speed_set(1_in / 500_ms);
  CHECK(d.pid_drive_exit_stop_speed_get() == doctest::Approx(2.0).epsilon(1e-9));
}

TEST_CASE("the default numbers written with units are exactly the defaults") {
  // default_constants() writes them this way, and a robot running it has to behave exactly like one that never set them, so the conversion
  // through metres and radians has to come back to the very same double, not a neighbour of it
  Drive d = make_chassis();
  d.pid_turn_exit_stop_speed_set(4_deg / 1_s);
  d.pid_swing_exit_stop_speed_set(4_deg / 1_s);
  d.pid_drive_exit_stop_speed_set(1.5_in / 1_s);
  d.pid_odom_turn_exit_stop_speed_set(4_deg / 1_s);
  d.pid_odom_drive_exit_stop_speed_set(1.5_in / 1_s);
  CHECK(read(d) == Five{{1.5, 4.0, 4.0, 1.5, 4.0}});
}

TEST_CASE("zero, negative, NaN and infinity are refused with a warning and the previous speed stays") {
  for (const auto& s : SETTERS) {
    for (double bad : {0.0, -0.0, -1.0, -1e-9, -1e9, NaN, INF, -INF}) {
      INFO(s.name << " " << bad);
      Drive d = make_chassis();
      s.set(d, 3.5);  // a value the team had set before
      Five before = read(d);
      std::string out = test_stub::capture_stdout([&] { s.set(d, bad); });
      CHECK(read(d) == before);
      CHECK(out.find(s.name) != std::string::npos);
      CHECK(out.find("rejected") != std::string::npos);
      CHECK(out.find("Keeping 3.5") != std::string::npos);
      CHECK(out.find(s.angle ? "deg/s" : "in/s") != std::string::npos);
      // and the setter still works after a refusal
      s.set(d, 4.5);
      CHECK(read(d).v[s.slot] == 4.5);
    }
  }
}

TEST_CASE("a refused speed keeps the default when none was set before") {
  for (const auto& s : SETTERS) {
    INFO(s.name);
    Drive d = make_chassis();
    Five before = read(d);
    test_stub::capture_stdout([&] { s.set(d, 0.0); });
    CHECK(read(d) == before);
  }
}

TEST_CASE("the units forms refuse the same bad speeds") {
  const std::string z = "rejected";
  {
    Drive d = make_chassis();
    d.pid_drive_exit_stop_speed_set(2.0);
    for (ez::QSpeed bad : {ez::QSpeed(0.0), ez::QSpeed(-1.0), ez::QSpeed(NaN), ez::QSpeed(INF), ez::QSpeed(0_in / 1_s), ez::QSpeed(-2_in / 1_s)}) {
      std::string out = test_stub::capture_stdout([&] { d.pid_drive_exit_stop_speed_set(bad); });
      CHECK(d.pid_drive_exit_stop_speed_get() == 2.0);
      CHECK(out.find(z) != std::string::npos);
    }
    for (ez::QSpeed bad : {ez::QSpeed(0.0), ez::QSpeed(NaN), ez::QSpeed(-2_in / 1_s)}) {
      d.pid_odom_drive_exit_stop_speed_set(2.5);
      std::string out = test_stub::capture_stdout([&] { d.pid_odom_drive_exit_stop_speed_set(bad); });
      CHECK(d.pid_odom_drive_exit_stop_speed_get() == 2.5);
      CHECK(out.find(z) != std::string::npos);
    }
  }
  {
    Drive d = make_chassis();
    d.pid_turn_exit_stop_speed_set(6.0);
    d.pid_swing_exit_stop_speed_set(6.0);
    d.pid_odom_turn_exit_stop_speed_set(6.0);
    for (ez::QAngularSpeed bad : {ez::QAngularSpeed(0.0), ez::QAngularSpeed(-1.0), ez::QAngularSpeed(NaN), ez::QAngularSpeed(INF),
                                  ez::QAngularSpeed(0_deg / 1_s), ez::QAngularSpeed(-5_deg / 1_s)}) {
      std::string a = test_stub::capture_stdout([&] { d.pid_turn_exit_stop_speed_set(bad); });
      std::string b = test_stub::capture_stdout([&] { d.pid_swing_exit_stop_speed_set(bad); });
      std::string c = test_stub::capture_stdout([&] { d.pid_odom_turn_exit_stop_speed_set(bad); });
      CHECK(d.pid_turn_exit_stop_speed_get() == 6.0);
      CHECK(d.pid_swing_exit_stop_speed_get() == 6.0);
      CHECK(d.pid_odom_turn_exit_stop_speed_get() == 6.0);
      CHECK(a.find(z) != std::string::npos);
      CHECK(b.find(z) != std::string::npos);
      CHECK(c.find(z) != std::string::npos);
    }
  }
}

TEST_CASE("every other positive number is the team's to choose, however small or large") {
  // No minimum and no maximum: the stop speed is the team's, and the timeouts made out of it follow it (see the header)
  for (const auto& s : SETTERS) {
    for (double v : {1e-9, 0.01, 0.1, 100.0, 1e6, 1e300}) {
      INFO(s.name << " " << v);
      Drive d = make_chassis();
      std::string out = test_stub::capture_stdout([&] { s.set(d, v); });
      CHECK(out.empty());
      CHECK(read(d).v[s.slot] == v);
    }
  }
}

TEST_CASE("accepted speeds print nothing") {
  Drive d = make_chassis();
  std::string out = test_stub::capture_stdout([&] {
    d.pid_drive_exit_stop_speed_set(2.0);
    d.pid_turn_exit_stop_speed_set(5_deg / 1_s);
    d.pid_swing_exit_stop_speed_set(5.0);
    d.pid_odom_drive_exit_stop_speed_set(2_in / 1_s);
    d.pid_odom_turn_exit_stop_speed_set(5.0);
  });
  CHECK(out.empty());
}

TEST_CASE("the exit condition setters do not touch the stop speeds, and the other way round") {
  Drive d = make_chassis();
  Five set = {{2.0, 5.0, 6.0, 2.5, 7.0}};
  d.pid_drive_exit_stop_speed_set(set.v[0]);
  d.pid_turn_exit_stop_speed_set(set.v[1]);
  d.pid_swing_exit_stop_speed_set(set.v[2]);
  d.pid_odom_drive_exit_stop_speed_set(set.v[3]);
  d.pid_odom_turn_exit_stop_speed_set(set.v[4]);

  d.pid_drive_exit_condition_set(40_ms, 2_in, 100_ms, 5_in, 200_ms, 300_ms);
  d.pid_turn_exit_condition_set(40_ms, 2_deg, 100_ms, 5_deg, 200_ms, 300_ms);
  d.pid_swing_exit_condition_set(40_ms, 2_deg, 100_ms, 5_deg, 200_ms, 300_ms);
  d.pid_odom_drive_exit_condition_set(40_ms, 2_in, 100_ms, 5_in, 200_ms, 300_ms);
  d.pid_odom_turn_exit_condition_set(40_ms, 2_deg, 100_ms, 5_deg, 200_ms, 300_ms);
  CHECK(read(d) == set);

  d.pid_drive_exit_stop_speed_set(3.0);
  CHECK(d.leftPID.exit.small_exit_time == 40);
  CHECK(d.leftPID.exit.mA_timeout == 300);
}
