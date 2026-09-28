// pros::Motor::is_over_current() returns PROS_ERR, not a genuine 0/1, when the read itself fails
// (e.g. the motor is unplugged). An earlier fix made sure that alone couldn't force a false
// mA_EXIT (a transient read failure is not the same as genuine overcurrent). But a motor that has
// truly gone away -- unplugged, a cable pulled mid-wait -- fails every read, not just one: its
// get_position() comes back non-finite too. Treating THAT combination as "still running normally"
// left a bare (non-Drive) PID mechanism (a lift, claw, catapult -- Drive's own waits have
// StuckWatch/SingleStuckWatch as an independent backstop; a bare ez::PID does not) hanging
// forever the moment its motor disconnected, since neither the velocity channel (fed a
// non-finite derivative, sanitized to 0 -- reads as perfectly still, not stopped-and-exiting) nor
// the mA channel (PROS_ERR ignored outright) could ever end the wait. Fixed: PROS_ERR paired with
// a non-finite get_position() is a dead motor and counts toward the mA timer, same as genuine
// overcurrent. A PROS_ERR with a still-finite position -- a one-off read glitch, not the motor
// disappearing -- stays ignored, exactly as before.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("PID exit_condition(Motor): a genuinely disconnected motor (every read fails) mA_EXITs, same timing as real overcurrent") {
  test_stub::reset_all();  // motor fake state is a global port-keyed registry, shared across TEST_CASEs
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);  // mA_timeout=500ms
  pid.error = 10.0;                                   // outside both small_error(1) and big_error(3) the whole test
  pros::Motor m(1);
  m.fake().disconnected = true;  // is_over_current() -> PROS_ERR AND get_position() -> non-finite: a dead motor

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(m) == RUNNING);
  }
  CHECK(pid.exit_condition(m) == mA_EXIT);
}

TEST_CASE("PID exit_condition(Motor): a transient current-read failure with a still-finite position does not force mA_EXIT") {
  test_stub::reset_all();
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor m(1);
  m.fake().current_read_failed = true;  // is_over_current() -> PROS_ERR, but get_position() still reads fine

  for (int pass = 1; pass <= 80; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(m) == RUNNING);
  }
}

TEST_CASE("PID exit_condition(Motor): a genuinely over-current motor still mA_EXITs normally") {
  test_stub::reset_all();
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor m(1);
  m.fake().over_current = true;  // is_over_current() returns a genuine 1

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(m) == RUNNING);
  }
  CHECK(pid.exit_condition(m) == mA_EXIT);
}

TEST_CASE("PID exit_condition(vector<Motor>): one genuinely disconnected motor in the group mA_EXITs") {
  test_stub::reset_all();
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor healthy(1);
  pros::Motor disconnected(2);
  disconnected.fake().disconnected = true;
  std::vector<pros::Motor> group{healthy, disconnected};

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(group) == RUNNING);
  }
  CHECK(pid.exit_condition(group) == mA_EXIT);
}

TEST_CASE("PID exit_condition(vector<Motor>): one motor with a transient current-read failure does not force mA_EXIT") {
  test_stub::reset_all();
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor healthy(1);
  pros::Motor glitchy(2);
  glitchy.fake().current_read_failed = true;
  std::vector<pros::Motor> group{healthy, glitchy};

  for (int pass = 1; pass <= 80; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(group) == RUNNING);
  }
}

TEST_CASE("PID exit_condition(vector<Motor>): one genuinely over-current motor in the group still mA_EXITs") {
  test_stub::reset_all();
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor healthy(1);
  pros::Motor hot(2);
  hot.fake().over_current = true;
  std::vector<pros::Motor> group{healthy, hot};

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(group) == RUNNING);
  }
  CHECK(pid.exit_condition(group) == mA_EXIT);
}
