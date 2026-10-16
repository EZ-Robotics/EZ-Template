// pros::Motor::is_over_current() returns PROS_ERR (a nonzero value), not a genuine 0/1, when the read itself fails (e.g. the motor is unplugged).  A
// motor that has truly gone away -- unplugged, a cable pulled mid-wait -- fails every read, so the mA timer sees it as over current for as long as it is
// gone and ends the wait after mA_timeout, same as genuine overcurrent.  That keeps a bare (non-Drive) PID mechanism (a lift, claw, catapult) from
// hanging forever when its motor disconnects.  A Drive's own waits only look at the first motor of each side.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

namespace {
// One 10 ms pass of the wait loop: the clock moves on, then the PID polls the motor.
template <typename M>
exit_output poll(PID& pid, M& motor) {
  test_stub::g_clock.now_ms += util::DELAY_TIME;
  return pid.exit_condition(motor);
}
}  // namespace

TEST_CASE("PID exit_condition(Motor): a genuinely disconnected motor (every read fails) mA_EXITs, same timing as real overcurrent") {
  test_stub::reset_all();  // motor fake state is a global port-keyed registry, shared across TEST_CASEs
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);  // mA_timeout=500ms
  pid.error = 10.0;                                   // outside both small_error(1) and big_error(3) the whole test
  pros::Motor m(1);
  m.fake().disconnected = true;  // is_over_current() -> PROS_ERR AND get_position() -> non-finite: a dead motor

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(poll(pid, m) == RUNNING);
  }
  CHECK(poll(pid, m) == mA_EXIT);  // 510 ms is the first value over mA_timeout
}

TEST_CASE("PID exit_condition(Motor): a genuinely over-current motor still mA_EXITs normally") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor m(1);
  m.fake().over_current = true;  // is_over_current() returns a genuine 1

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(poll(pid, m) == RUNNING);
  }
  CHECK(poll(pid, m) == mA_EXIT);
}

TEST_CASE("PID exit_condition(vector<Motor>): one genuinely disconnected motor in the group mA_EXITs") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor healthy(1);
  pros::Motor disconnected(2);
  disconnected.fake().disconnected = true;
  std::vector<pros::Motor> group{healthy, disconnected};

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(poll(pid, group) == RUNNING);
  }
  CHECK(poll(pid, group) == mA_EXIT);
}

TEST_CASE("PID exit_condition(vector<Motor>): one genuinely over-current motor in the group still mA_EXITs") {
  test_stub::reset_all();
  test_stub::g_clock.now_ms = 0;
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor healthy(1);
  pros::Motor hot(2);
  hot.fake().over_current = true;
  std::vector<pros::Motor> group{healthy, hot};

  for (int pass = 1; pass < 51; pass++) {
    INFO("pass ", pass);
    CHECK(poll(pid, group) == RUNNING);
  }
  CHECK(poll(pid, group) == mA_EXIT);
}
