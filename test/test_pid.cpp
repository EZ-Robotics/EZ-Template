// PID [H1]: with small 1 in / 90 ms and big 3 in / 250 ms, an error held at
// 2 in returns BIG_EXIT after 26 passes and never SMALL_EXIT; an error held
// at 0.5 in returns SMALL_EXIT after 10 passes.
// PID [H2]: with ki set, integral accumulates while error keeps sign and
// resets when error sign flips; a position sign flip with constant error
// sign does NOT reset it; motion_reset(c) zeroes the integral and makes the
// next derivative 0.
// PID [M15]: exit_condition(pros::MotorGroup) produces BIG_EXIT at the same
// pass count as the plain overload.
#include "doctest.h"

#include "EZ-Template/api.hpp"

using namespace ez;

TEST_CASE("PID [H1] BIG_EXIT after 26 passes at a held 2in error, never SMALL_EXIT") {
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pid.error = 2.0;  // within big_error (3), outside small_error (1)

  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
  CHECK(pid.exit_condition() == BIG_EXIT);
}

TEST_CASE("PID [H1] SMALL_EXIT after 10 passes at a held 0.5in error") {
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pid.error = 0.5;  // within both small_error (1) and big_error (3)

  for (int pass = 1; pass < 10; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition() == RUNNING);
  }
  CHECK(pid.exit_condition() == SMALL_EXIT);
}

TEST_CASE("PID [M15] exit_condition(MotorGroup) matches the plain overload's pass count") {
  // mA_timeout is left at 0 (unset), so the MotorGroup-specific mA check
  // is skipped entirely and this falls straight through to the same
  // exit_condition(print) as the H1 case above -- so it should take the
  // same 26 passes to BIG_EXIT, not the ~20 the prompt that produced this
  // suite estimated (see PR body).
  PID pid;
  pid.exit_condition_set(90, 1.0, 250, 3.0);
  pid.error = 2.0;
  pros::MotorGroup mg({1, 2});

  for (int pass = 1; pass < 26; pass++) {
    INFO("pass ", pass);
    CHECK(pid.exit_condition(mg) == RUNNING);
  }
  CHECK(pid.exit_condition(mg) == BIG_EXIT);
}

TEST_CASE("PID [H2] integral accumulates while error keeps sign, resets on sign flip") {
  PID pid(1.0, 1.0, 0.0, 1000.0);  // start_i huge: integral always active
  pid.target_set(10);

  // First call: prev_error starts at 0 (sgn 0), which differs from the new
  // error's sgn -- that's treated as a sign flip and zeroes the integral
  // right back out, so this call is just priming a consistent sign.
  pid.compute(0);
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(0);  // error=10, same sign as prev_error(10): integral += 10
  CHECK(pid.integral == doctest::Approx(10));

  pid.compute(2);  // error=8, still positive: integral += 8
  CHECK(pid.integral == doctest::Approx(18));

  pid.compute(20);  // error=10-20=-10, sign flips: integral resets to 0
  CHECK(pid.integral == doctest::Approx(0));
}

TEST_CASE("PID [H2] a position sign flip with constant error sign does not reset integral") {
  PID pid(1.0, 1.0, 0.0, 1000.0);
  pid.target_set(100);  // far away: error stays positive even as current crosses 0

  pid.compute(5);  // prime (same first-call reset as above)
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(5);  // error=95: integral += 95
  CHECK(pid.integral == doctest::Approx(95));

  pid.compute(-5);  // position flips sign (5 -> -5), error=105, still positive: integral += 105
  CHECK(pid.integral == doctest::Approx(200));
}

TEST_CASE("PID [H2] motion_reset zeroes the integral and primes the next derivative to 0") {
  PID pid(1.0, 1.0, 0.0, 1000.0);
  pid.target_set(10);
  pid.compute(0);
  pid.compute(0);
  CHECK(pid.integral == doctest::Approx(10));

  pid.motion_reset(3.0);
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(3.0);  // same value motion_reset primed prev_current to
  CHECK(pid.derivative == doctest::Approx(0));
}
