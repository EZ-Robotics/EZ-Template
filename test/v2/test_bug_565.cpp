// #565: drive_imu_reset() left odom_theta_get() at the old heading until the next tracking pass.
#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#565: after drive_imu_reset(), odom reads the new heading at once") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  c.pid_turn_set(90_deg, 90);
  c.pid_wait();

  c.drive_imu_reset();
  CHECK(c.odom_theta_get() == doctest::Approx(0).epsilon(0.01));
}
