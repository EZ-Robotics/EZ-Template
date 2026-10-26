// start_i is the error window that the integral accumulates inside.  A start_i of 0 means no window: with ki set, the integral accumulates on every pass,
// so PID(p, i) and constants_set(p, i) with the default start_i give a team integral.  A positive start_i still only accumulates while |error| < start_i.
#include "doctest.h"

#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

using namespace ez;

TEST_CASE("PID start_i 0 accumulates integral at any error") {
  test_stub::reset_all();
  PID pid(1.0, 0.5);  // start_i left at its default of 0
  pid.target_set(100);

  pid.compute(0);  // the first pass only primes the sign of the error, same as the other integral tests
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(0);  // error 100 is far from the target, and still accumulates
  CHECK(pid.integral == doctest::Approx(100));
  CHECK(pid.output == doctest::Approx(100 * 1.0 + 100 * 0.5));

  pid.compute(40);  // error 60
  CHECK(pid.integral == doctest::Approx(160));
}

TEST_CASE("PID constants_set with start_i 0 accumulates integral") {
  test_stub::reset_all();
  PID pid;
  pid.constants_set(1.0, 0.5, 0.0);
  pid.target_set(10);

  pid.compute(0);
  pid.compute(0);
  CHECK(pid.integral == doctest::Approx(10));
}

TEST_CASE("PID positive start_i only accumulates integral inside the window") {
  test_stub::reset_all();
  PID pid(1.0, 0.5, 0.0, 5.0);
  pid.target_set(10);

  pid.compute(0);
  pid.compute(0);  // error 10 is outside the window of 5
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(4);  // error 6 is still outside
  CHECK(pid.integral == doctest::Approx(0));

  pid.compute(7);  // error 3 is inside
  CHECK(pid.integral == doctest::Approx(3));

  pid.compute(5);  // error 5 is on the edge, which is outside
  CHECK(pid.integral == doctest::Approx(3));
}

TEST_CASE("PID ki 0 never accumulates integral, whatever start_i is") {
  test_stub::reset_all();
  PID pid(1.0, 0.0, 0.0, 0.0);
  pid.target_set(10);

  pid.compute(0);
  pid.compute(0);
  CHECK(pid.integral == doctest::Approx(0));
}
