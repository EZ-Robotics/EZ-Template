// Stands in for a user's project (main.h + autons.cpp) so compile_matrix.sh can
// check what upgrading to EZ-Units looks like from the user's side. It is only
// ever compiled with -fsyntax-only, never linked or run.
//
//   STALE_MAIN_H_LINE          the project's main.h still says
//                              `using namespace okapi::literals;`
//   USER_INCLUDES_OKAPI_UNITS  the project also includes okapi's unit headers
//                              directly (e.g. via #include "okapi/api.hpp" for
//                              other okapi features)
//
// Whether okapi is installed at all is decided by whether test/fixtures is on
// the include path.
#include "EZ-Template/api.hpp"

#ifdef USER_INCLUDES_OKAPI_UNITS
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QTime.hpp"
#endif

#ifdef STALE_MAIN_H_LINE
using namespace okapi::literals;
#endif

extern ez::Drive chassis;

void user_autons() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait_until(6_in);
  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait_until(45_deg);
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110);
  chassis.pid_odom_set({{24_in, 24_in, 90_deg}, ez::fwd, 110});
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.slew_drive_constants_set(7_in, 80);

  ez::QLength distance = 24_in;
  ez::QAngle heading = 90_deg;
  ez::QTime timeout = 100_ms;
  (void)distance;
  (void)heading;
  (void)timeout;
}
