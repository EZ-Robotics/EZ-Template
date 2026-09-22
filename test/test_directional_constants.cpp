// The forward and backward PID constants getters report the constants the next
// motion in that direction will use.  The plain setter stores its values apart
// from the directional ones, so the getters have to fall back to those when no
// directional constants were set.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

void check(PID::Constants got, double p, double i, double d, double start_i) {
  CHECK(got.kp == doctest::Approx(p));
  CHECK(got.ki == doctest::Approx(i));
  CHECK(got.kd == doctest::Approx(d));
  CHECK(got.start_i == doctest::Approx(start_i));
}
}  // namespace

TEST_CASE("drive forward and backward getters return the plain setter's constants") {
  Drive chassis = make_chassis();
  chassis.pid_drive_constants_set(20.0, 0.1, 100.0, 2.0);

  check(chassis.pid_drive_constants_forward_get(), 20.0, 0.1, 100.0, 2.0);
  check(chassis.pid_drive_constants_backward_get(), 20.0, 0.1, 100.0, 2.0);
  check(chassis.pid_drive_constants_get(), 20.0, 0.1, 100.0, 2.0);
}

TEST_CASE("swing forward and backward getters return the plain setter's constants") {
  Drive chassis = make_chassis();
  chassis.pid_swing_constants_set(6.0, 0.2, 65.0, 3.0);

  check(chassis.pid_swing_constants_forward_get(), 6.0, 0.2, 65.0, 3.0);
  check(chassis.pid_swing_constants_backward_get(), 6.0, 0.2, 65.0, 3.0);
  check(chassis.pid_swing_constants_get(), 6.0, 0.2, 65.0, 3.0);
}

TEST_CASE("a drive constructed with no setters called returns the constructor's defaults for both directions") {
  Drive chassis = make_chassis();

  check(chassis.pid_drive_constants_forward_get(), 20.0, 0.0, 100.0, 0.0);
  check(chassis.pid_drive_constants_backward_get(), 20.0, 0.0, 100.0, 0.0);
  check(chassis.pid_swing_constants_forward_get(), 6.0, 0.0, 65.0, 0.0);
  check(chassis.pid_swing_constants_backward_get(), 6.0, 0.0, 65.0, 0.0);
}

TEST_CASE("directional constants win over the plain setter, in the direction they were set for") {
  Drive chassis = make_chassis();
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_drive_constants_forward_set(25.0, 0.0, 110.0);

  check(chassis.pid_drive_constants_forward_get(), 25.0, 0.0, 110.0, 0.0);
  // Backward was never set apart from the plain setter
  check(chassis.pid_drive_constants_backward_get(), 20.0, 0.0, 100.0, 0.0);

  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_swing_constants_backward_set(7.0, 0.0, 70.0);

  check(chassis.pid_swing_constants_backward_get(), 7.0, 0.0, 70.0, 0.0);
  check(chassis.pid_swing_constants_forward_get(), 6.0, 0.0, 65.0, 0.0);
}

TEST_CASE("calling the plain setter again replaces earlier directional constants") {
  Drive chassis = make_chassis();
  chassis.pid_drive_constants_forward_set(25.0, 0.0, 110.0);
  chassis.pid_drive_constants_backward_set(26.0, 0.0, 111.0);
  chassis.pid_drive_constants_set(30.0, 0.0, 120.0);

  check(chassis.pid_drive_constants_forward_get(), 30.0, 0.0, 120.0, 0.0);
  check(chassis.pid_drive_constants_backward_get(), 30.0, 0.0, 120.0, 0.0);
}

TEST_CASE("the getters report the constants that the motion actually loads") {
  Drive chassis = make_chassis();
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_drive_constants_backward_set(24.0, 0.0, 105.0);
  chassis.slew_drive_set(false);

  chassis.pid_drive_set(-10.0, 127);
  check(chassis.pid_drive_constants_backward_get(), chassis.leftPID.constants.kp, chassis.leftPID.constants.ki,
        chassis.leftPID.constants.kd, chassis.leftPID.constants.start_i);

  chassis.pid_drive_set(10.0, 127);
  check(chassis.pid_drive_constants_forward_get(), chassis.leftPID.constants.kp, chassis.leftPID.constants.ki,
        chassis.leftPID.constants.kd, chassis.leftPID.constants.start_i);
}
