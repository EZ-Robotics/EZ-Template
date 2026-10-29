// #601: a PID made with ki and no start_i never added up its integral.
#include "../doctest.h"
#include "EZ-Template/api.hpp"
#include "fake_hardware.hpp"

TEST_CASE("#601: a team's ez::PID with ki and no start_i pushes harder the longer it is held short of its target") {
  test_stub::reset_all();
  ez::PID lift{1, 0.003, 4};  // a lift PID written without a start_i
  lift.target_set(100);

  // The lift is held against something at 0, so only the integral can change the output
  double first = lift.compute(0);
  double later = first;
  for (int pass = 0; pass < 50; pass++) later = lift.compute(0);
  CHECK(later > first);
}
