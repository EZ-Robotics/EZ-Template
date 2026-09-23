// Step 3 finding #8 (critical): pros::Motor::is_over_current() returns PROS_ERR, not a
// genuine 0/1, when the read itself fails (e.g. the motor is unplugged). Before this fix,
// PID::exit_condition()'s `if (sensor.is_over_current())` treated PROS_ERR exactly like a
// real overcurrent reading (both are nonzero), so a single disconnected motor forced an
// mA_EXIT at exactly mA_timeout regardless of the robot's actual motion. Fixed by checking
// for a genuine "1", not just "nonzero".
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("PID exit_condition(Motor): a disconnected motor does not force mA_EXIT while error is still converging normally") {
  test_stub::reset_all();  // motor fake state is a global port-keyed registry, shared across TEST_CASEs
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);  // mA_timeout=500ms
  pid.error = 10.0;                                   // outside both small_error(1) and big_error(3) the whole test
  pros::Motor m(1);
  m.fake().disconnected = true;  // is_over_current() now returns PROS_ERR, not 0 or 1

  // Old code: is_over_current() truthy (PROS_ERR is nonzero) -> mA timer runs -> mA_EXIT
  // at pass 51 (l crosses 500 at l=510, DELAY_TIME=10). Fixed code: a failed read is not
  // "over current", so this must stay RUNNING for as long as a healthy motor would.
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

TEST_CASE("PID exit_condition(vector<Motor>): one disconnected motor in the group does not force mA_EXIT") {
  test_stub::reset_all();
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  pid.error = 10.0;
  pros::Motor healthy(1);
  pros::Motor disconnected(2);
  disconnected.fake().disconnected = true;
  std::vector<pros::Motor> group{healthy, disconnected};

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
