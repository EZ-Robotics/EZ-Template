// A motion whose velocity exit and mA exit are both 0 has no backstop: the stop check is not armed and the wait ends on position alone.
// That is allowed, but it is rarely what a team means, so the setter that leaves a motion that way says so, once per call and always
// (not only with pid_print_toggle on). The shipped defaults must never say it.
//
// Odom waits fall back to the odom turn settings when the odom drive ones are 0 (and the other way round), so for odom the warning is
// only for the pair: both odom drive and odom turn at 0 and 0, whichever setter made it so.
#include <functional>
#include <string>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {

constexpr const char* KEY = "velocity and mA exits are both 0";

Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

size_t count_of(const std::string& text, const std::string& what) {
  size_t n = 0;
  for (size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + what.size())) n++;
  return n;
}

template <typename F>
std::string printed(F&& f) {
  return test_stub::capture_stdout(std::forward<F>(f));
}

}  // namespace

TEST_CASE("constructing a Drive with the shipped defaults prints no exit condition warning") {
  std::string out = printed([] { Drive chassis = make_chassis(); });
  CHECK(out.find(KEY) == std::string::npos);
}

TEST_CASE("a Drive's shipped odom defaults are not a warning to set again") {
  Drive chassis = make_chassis();
  std::string out = printed([&] {
    chassis.pid_odom_turn_exit_condition_set(90, 3, 250, 7, 500, 750);
    chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 500, 750);
  });
  CHECK(out.find(KEY) == std::string::npos);
}

TEST_CASE("drive, turn and swing exit condition setters warn once when velocity and mA are both 0") {
  Drive chassis = make_chassis();
  struct Case {
    const char* name;
    std::function<void(int, int)> set;
    std::function<void(ez::QTime, ez::QTime)> set_units;
  };
  std::vector<Case> cases = {
      {"pid_drive_exit_condition_set", [&](int v, int m) { chassis.pid_drive_exit_condition_set(90, 1, 250, 3, v, m); },
       [&](ez::QTime v, ez::QTime m) { chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, v, m); }},
      {"pid_turn_exit_condition_set", [&](int v, int m) { chassis.pid_turn_exit_condition_set(90, 3, 250, 7, v, m); },
       [&](ez::QTime v, ez::QTime m) { chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, v, m); }},
      {"pid_swing_exit_condition_set", [&](int v, int m) { chassis.pid_swing_exit_condition_set(90, 3, 250, 7, v, m); },
       [&](ez::QTime v, ez::QTime m) { chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, v, m); }},
  };
  for (auto& c : cases) {
    CAPTURE(c.name);
    std::string both = printed([&] { c.set(0, 0); });
    CHECK(count_of(both, KEY) == 1);
    CHECK(both.find(std::string(c.name) + ": ") != std::string::npos);
    CHECK(count_of(printed([&] { c.set(0, 0); }), KEY) == 1);  // once per call, not once per setter
    CHECK(count_of(printed([&] { c.set_units(0_ms, 0_ms); }), KEY) == 1);
    CHECK(count_of(printed([&] { c.set(500, 0); }), KEY) == 0);
    CHECK(count_of(printed([&] { c.set(0, 500); }), KEY) == 0);
    CHECK(count_of(printed([&] { c.set(500, 500); }), KEY) == 0);
    CHECK(count_of(printed([&] { c.set_units(500_ms, 0_ms); }), KEY) == 0);
  }
}

TEST_CASE("the warning is printed whether or not pid_print_toggle is on") {
  Drive chassis = make_chassis();
  for (bool toggle : {false, true}) {
    chassis.pid_print_toggle(toggle);
    CHECK(count_of(printed([&] { chassis.pid_turn_exit_condition_set(90, 3, 250, 7, 0, 0); }), KEY) == 1);
  }
}

TEST_CASE("odom exit condition setters warn only when odom drive and odom turn both have velocity and mA at 0") {
  {
    // One of the two still has a backstop: the odom waits use it for both
    Drive chassis = make_chassis();
    CHECK(count_of(printed([&] { chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 0, 0); }), KEY) == 0);
    CHECK(count_of(printed([&] { chassis.pid_odom_turn_exit_condition_set(90, 3, 250, 7, 500, 0); }), KEY) == 0);
    // Taking away the other one's leaves neither
    std::string out = printed([&] { chassis.pid_odom_turn_exit_condition_set(90, 3, 250, 7, 0, 0); });
    CHECK(count_of(out, KEY) == 1);
    CHECK(out.find("pid_odom_turn_exit_condition_set: ") != std::string::npos);
  }
  {
    // The same thing in the other order: whichever setter makes the pair 0 and 0 is the one that says so
    Drive chassis = make_chassis();
    CHECK(count_of(printed([&] { chassis.pid_odom_turn_exit_condition_set(90, 3, 250, 7, 0, 0); }), KEY) == 0);
    std::string out = printed([&] { chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 0, 0); });
    CHECK(count_of(out, KEY) == 1);
    CHECK(out.find("pid_odom_drive_exit_condition_set: ") != std::string::npos);
    // And once the pair is 0 and 0, setting either one again says it again
    CHECK(count_of(printed([&] { chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 0, 0); }), KEY) == 1);
    // Giving either one a backstop clears it
    CHECK(count_of(printed([&] { chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 0, 750); }), KEY) == 0);
    CHECK(count_of(printed([&] { chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 0_ms, 0_ms); }), KEY) == 0);
  }
  {
    Drive chassis = make_chassis();
    CHECK(count_of(printed([&] { chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 0_ms, 0_ms); }), KEY) == 0);
    CHECK(count_of(printed([&] { chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 0_ms, 0_ms); }), KEY) == 1);
  }
}

TEST_CASE("setting the other motions' exit conditions never warns about the odom pair") {
  Drive chassis = make_chassis();
  chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3, 250, 7, 0, 0);
  std::string out = printed([&] { chassis.pid_drive_exit_condition_set(90, 1, 250, 3, 500, 500); });
  CHECK(count_of(out, KEY) == 0);
}
