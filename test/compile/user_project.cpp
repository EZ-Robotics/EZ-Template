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

// Every constructor call shape a project can have today. None of these may warn.
ez::Drive upgraded_chassis({-5, -6, -7}, {11, 15, 16}, 21, 3.25, 360);
ez::Drive upgraded_redundant_chassis({-5, -6, -7}, {11, 15, 16}, std::vector<int>{21, 20}, 3.25, 450.0);
ez::Drive driver_only_chassis({-5, -6, -7}, {11, 15, 16});

#ifdef LEGACY_RATIO_CONSTRUCTOR
// What a 3.x or beta.1-3 project has: the sixth argument, ratio. It has to compile and tell the team what to change.
ez::Drive legacy_chassis({-5, -6, -7}, {11, 15, 16}, 21, 3.25, 600, 1.667);
ez::Drive legacy_redundant_chassis({-5, -6, -7}, {11, 15, 16}, std::vector<int>{21, 20}, 3.25, 600, 1.667);
#endif

void user_autons() {
#ifdef LEGACY_RATIO_SET
  // What a project that shifts with the ratio has. Removed in 4.0: it must stop compiling and name drive_rpm_set().
  chassis.drive_ratio_set(1.667);
#endif
#ifdef LEGACY_RATIO_GET
  double ratio = chassis.drive_ratio_get();
  (void)ratio;
#endif
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
