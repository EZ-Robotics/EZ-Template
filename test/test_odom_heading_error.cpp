// An odom motion steers with the difference between the heading to the point and the heading the
// robot has right now. That difference has to stay the short way round: the bearing to the point is
// recomputed every pass as the robot moves, and if the error is measured against where the robot
// was when the motion started instead, it steps by a full turn the moment the bearing crosses the
// edge of that heading's +/-180 range. The robot then reverses, spins a whole revolution, reaches
// the edge again, and repeats until the motion times out.
//
// The motions below are driven by a small kinematic model of a differential drive: each pass runs
// the odom task, reads the voltages it commanded, and moves the pose accordingly.
#include <cmath>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

struct Run {
  double error_at_end;  // Distance from the target when the run stopped, in inches
  double rotated;       // Furthest the heading got from where it started, in degrees
  bool arrived;         // Whether it got within ARRIVED inches of the last point in time
};

// Where a real motion would already have exited. Closer than this the bearing to the point swings
// wildly with every inch of drift, which is noise no motion is left running through.
constexpr double ARRIVED = 3.0;
constexpr double DT = 0.01;           // One pass of the task, in seconds
constexpr double MAX_SPEED = 60.0;    // Inches per second at full power
constexpr double TRACK_WIDTH = 12.0;  // Inches between the two sides of the drive
constexpr double SIM_SECONDS = 12.0;  // Long enough to finish a half turn and a 24 inch drive with room to spare

// Puts the robot at a pose the way the tracking task would: no task runs on the host, and it is the
// tracking task that copies the IMU heading into the odom pose.
void place(Drive& chassis, double x, double y, double theta) {
  chassis.imu->fake_rotation = theta;
  DriveTestAccess::odom_current(chassis) = {x, y, theta};
}

// Runs whatever odom motion is already set on `chassis` until it arrives or the time is up.
Run drive_until_arrived(Drive& chassis, pose end) {
  double x = chassis.odom_x_get();
  double y = chassis.odom_y_get();
  double theta = chassis.odom_theta_get();
  double start_theta = theta;
  double rotated = 0.0;
  double error = util::distance_to_point(end, {x, y, theta});

  for (int i = 0; i < (int)(SIM_SECONDS / DT) && error > ARRIVED; i++) {
    if (chassis.drive_mode_get() == PURE_PURSUIT)
      DriveTestAccess::pp_task(chassis);
    else
      DriveTestAccess::ptp_task(chassis);

    std::vector<int> out = chassis.drive_get();
    double left = out[0] / 127.0 * MAX_SPEED;
    double right = out[1] / 127.0 * MAX_SPEED;

    // Positive heading is clockwise and 0 faces +Y, so a faster left side turns the robot right
    double forward = (left + right) / 2.0;
    double d_theta = util::to_deg((left - right) / TRACK_WIDTH * DT);
    x += forward * DT * std::sin(util::to_rad(theta));
    y += forward * DT * std::cos(util::to_rad(theta));
    theta += d_theta;
    rotated = std::fmax(rotated, std::fabs(theta - start_theta));

    place(chassis, x, y, theta);
    error = util::distance_to_point(end, {x, y, theta});
  }
  return {error, rotated, error <= ARRIVED};
}

struct StartCase {
  const char* name;
  double start_theta;  // Heading the robot starts with
  odom movement;       // The single motion to run
};
}  // namespace

TEST_CASE("an odom move to a point behind the robot turns around once and drives there") {
  const StartCase cases[] = {
      {"point straight behind", 0.0, {{0.0, -24.0, ANGLE_NOT_SET}, fwd, 110}},
      {"reverse to a point in front", 0.0, {{0.0, 24.0, ANGLE_NOT_SET}, rev, 110}},
      {"point behind and to the side", 0.0, {{12.0, -20.0, ANGLE_NOT_SET}, fwd, 110}},
      {"start facing 179, point ahead", 179.0, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}},
      {"start facing -179, point ahead", -179.0, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}},
  };

  for (const StartCase& c : cases) {
    CAPTURE(c.name);
    Drive chassis = make_chassis();
    chassis.odom_xyt_set(0.0, 0.0, c.start_theta);
    chassis.pid_odom_set(c.movement);

    Run run = drive_until_arrived(chassis, c.movement.target);
    CAPTURE(run.error_at_end);
    CAPTURE(run.rotated);

    CHECK(run.arrived);
    // A half turn to face the point plus a little steering on the way. Spinning through whole
    // revolutions, which is what the unwrapped error does, is many times this.
    CHECK(run.rotated < 270.0);
  }
}

TEST_CASE("an odom move to a point straight ahead does not turn") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.pid_odom_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110});

  Run run = drive_until_arrived(chassis, {0.0, 24.0, 0.0});
  CAPTURE(run.error_at_end);
  CAPTURE(run.rotated);

  CHECK(run.arrived);
  CHECK(run.rotated < 5.0);
}

TEST_CASE("a path that doubles back gets to the last point") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.pid_odom_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 0.0, ANGLE_NOT_SET}, fwd, 110}});

  Run run = drive_until_arrived(chassis, {0.0, 0.0, 0.0});
  CAPTURE(run.error_at_end);
  CAPTURE(run.rotated);

  CHECK(run.arrived);
  CHECK(run.rotated < 270.0);
}

// cw, ccw and longest exist to choose the long way round, so their target is deliberately more than
// half a turn from the robot. The error must be left that size, not folded back to the short way.
TEST_CASE("an odom move keeps the turn direction it was asked for") {
  struct DirectionCase {
    const char* name;
    pose target;
    e_angle_behavior behavior;
    double min_rotated;
    double max_rotated;
  };
  const DirectionCase cases[] = {
      {"ccw to a point 90 clockwise goes 270 the other way", {24.0, 0.0, ANGLE_NOT_SET}, ccw, 250.0, 330.0},
      {"cw to a point 90 counterclockwise goes 270 the other way", {-24.0, 0.0, ANGLE_NOT_SET}, cw, 250.0, 330.0},
      {"cw to a point 90 clockwise takes the short way", {24.0, 0.0, ANGLE_NOT_SET}, cw, 60.0, 130.0},
      {"ccw to a point 90 counterclockwise takes the short way", {-24.0, 0.0, ANGLE_NOT_SET}, ccw, 60.0, 130.0},
      {"longest to a point 90 clockwise goes 270 the other way", {24.0, 0.0, ANGLE_NOT_SET}, longest, 250.0, 330.0},
  };

  for (const DirectionCase& c : cases) {
    CAPTURE(c.name);
    Drive chassis = make_chassis();
    chassis.odom_xyt_set(0.0, 0.0, 0.0);
    chassis.pid_odom_set({c.target, fwd, 110, c.behavior});

    Run run = drive_until_arrived(chassis, c.target);
    CAPTURE(run.error_at_end);
    CAPTURE(run.rotated);

    CHECK(run.arrived);
    CHECK(run.rotated > c.min_rotated);
    CHECK(run.rotated < c.max_rotated);
  }
}

// A turn to a point pivots about the middle of the drive, but odom tracks the tracking center, which is
// usually a little off it. The tracked position therefore drifts a few inches during the turn, and the
// bearing to a point behind the robot moves with it.
TEST_CASE("a turn to a point behind the robot turns around once with an off-center tracking wheel") {
  for (double offset : {-4.0, -2.0, 0.0, 2.0, 4.0}) {
    CAPTURE(offset);
    Drive chassis = make_chassis();
    place(chassis, 0.0, 0.0, 0.0);
    chassis.pid_turn_set({0.0, -24.0}, fwd, 110, shortest);

    double theta = 0.0;
    double rotated = 0.0;
    for (int i = 0; i < (int)(SIM_SECONDS / DT); i++) {
      DriveTestAccess::turn_pid_task(chassis);
      std::vector<int> out = chassis.drive_get();
      double d_theta = util::to_deg((out[0] - out[1]) / 127.0 * MAX_SPEED / TRACK_WIDTH * DT);
      theta += d_theta;
      rotated = std::fmax(rotated, std::fabs(theta));

      // The robot pivots about the origin. Odom sees a point `offset` inches behind it.
      place(chassis, -offset * std::sin(util::to_rad(theta)), -offset * std::cos(util::to_rad(theta)), theta);
    }

    // Facing the point means facing straight away from where it started, 180 degrees either way round
    CAPTURE(theta);
    CAPTURE(rotated);
    CHECK(std::fabs(std::fabs(theta) - 180.0) < 5.0);
    CHECK(rotated < 200.0);
  }
}
