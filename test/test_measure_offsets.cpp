// The example project's measure_offsets() routine (src/autons.cpp), run against the tracker rig.
//
// The example project is not part of the host build, so measure_offsets_copy() below is a verbatim copy of
// the function body in src/autons.cpp, with four substitutions the host needs:
//   * `chassis` is a reference to the rig's Drive
//   * the pid_turn_set / pid_wait / pros::delay lines that turn the robot become rig.turn(target)
//   * MOTOR_BRAKE_HOLD, a macro from the example project's main.h, is defined below
//   * rig.rezero() follows the resets, because the rig keeps its own record of where the robot is
// Change the routine and this copy together.  Anything the routine prints to the terminal is captured here.
#include <string>

#include "doctest.h"
#include "stdout_capture.hpp"
#include "tracker_rig.hpp"

using namespace ez;
using ez::tracker_rig::Cfg;
using ez::tracker_rig::Rig;

namespace {
constexpr auto MOTOR_BRAKE_HOLD = pros::E_MOTOR_BRAKE_HOLD;

void measure_offsets_copy(Rig& rig) {
  auto& chassis = rig.chassis;
  // ---- copy of measure_offsets() from src/autons.cpp starts here ----
  // Number of times to test
  int iterations = 10;

  // Our final offsets.  These keep their sign, which says if a tracker is wired the right way.
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;
  int turns_measured = 0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    rig.rezero();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.drive_angle_get();
    double target = i % 2 == 0 ? 90 : -90;  // Switch the turn direction every run

    // Turn to target at half power
    rig.turn(target);

    // Calculate delta in angle.  This is signed (clockwise is positive) and is not wrapped, because the
    // trackers saw the whole turn, not the angle it wraps to.  It is read from the imu, odom_theta_get() only
    // catches up with a reset when the tracking task next runs.
    double t_delta = ez::util::to_rad(chassis.drive_angle_get() - imu_start);
    if (fabs(t_delta) < ez::util::to_rad(10.0)) continue;  // The robot did not turn, nothing to measure

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
    turns_measured++;
  }

  if (turns_measured == 0) {
    printf("measure_offsets: the robot never turned, nothing was measured\n");
    ez::screen_print("The robot never turned", 0);
    return;
  }

  // Average all offsets
  l_offset /= turns_measured;
  r_offset /= turns_measured;
  b_offset /= turns_measured;
  f_offset /= turns_measured;

  // Turning clockwise, a vertical tracker on the left counts up and one on the right counts down.
  // A horizontal tracker counts up when the robot moves left, so it counts up at the back and down at the front.
  // A tracker with the other sign is wired backwards.
  int line = 0;
  auto report = [&](const char* name, ez::tracking_wheel* tracker, double offset, double expected_sign) {
    if (tracker == nullptr) return;

    char text[64];
    snprintf(text, sizeof(text), "%s tracker offset: %.2f in", name, fabs(offset));
    printf("%s\n", text);
    ez::screen_print(text, line++);

    if (offset * expected_sign < 0.0) {
      snprintf(text, sizeof(text), "%s tracker looks reversed, flip its port sign", name);
      printf("%s\n", text);
      ez::screen_print(text, line++);
    }

    // Set the new offset
    tracker->distance_to_center_set(fabs(offset));
  };
  report("left", chassis.odom_tracker_left, l_offset, 1.0);
  report("right", chassis.odom_tracker_right, r_offset, -1.0);
  report("back", chassis.odom_tracker_back, b_offset, 1.0);
  report("front", chassis.odom_tracker_front, f_offset, -1.0);
  // ---- copy of measure_offsets() from src/autons.cpp ends here ----
}

// Runs the routine with the trackers told wrong offsets to start with, returns what it printed.
std::string run(Rig& rig) {
  return test_stub::capture_stdout([&] { measure_offsets_copy(rig); });
}

double offset_of(tracking_wheel* t) { return std::fabs(t->distance_to_center_get()); }

// The number printed after "<name> tracker offset: ", or -1 if that line is missing
double printed(const std::string& out, const std::string& name) {
  const std::string key = name + " tracker offset: ";
  const auto at = out.find(key);
  return at == std::string::npos ? -1.0 : std::stod(out.substr(at + key.size()));
}

bool within_2_percent(double got, double truth) { return std::fabs(got - truth) <= 0.02 * truth; }
}  // namespace

TEST_CASE("measure_offsets: left, right and back trackers are measured within 2 percent of their true offsets") {
  Cfg c;
  c.left = 3.5;
  c.right = 1.0;
  c.horiz = 2.0;
  c.set_left = c.set_right = c.set_horiz = 0.5;  // start from wrong offsets so a measurement really replaces them
  Rig rig(c);
  const std::string out = run(rig);

  INFO("left " << offset_of(rig.tl.get()) << " right " << offset_of(rig.tr.get()) << " back " << offset_of(rig.th.get()));
  CHECK(within_2_percent(offset_of(rig.tl.get()), 3.5));
  CHECK(within_2_percent(offset_of(rig.tr.get()), 1.0));
  CHECK(within_2_percent(offset_of(rig.th.get()), 2.0));
  CHECK(out.find("reversed") == std::string::npos);
}

TEST_CASE("measure_offsets: a front tracker is measured within 2 percent of its true offset") {
  Cfg c;
  c.horiz = 2.0;
  c.horiz_front = true;
  c.set_horiz = 0.5;
  Rig rig(c);
  const std::string out = run(rig);

  INFO("front " << offset_of(rig.th.get()));
  CHECK(within_2_percent(offset_of(rig.th.get()), 2.0));
  CHECK(out.find("reversed") == std::string::npos);
}

TEST_CASE("measure_offsets: the offset to type into the constructor is printed for every tracker that exists") {
  Cfg c;
  c.left = 3.5;
  c.horiz = 2.0;
  Rig rig(c);
  const std::string out = run(rig);

  CHECK(within_2_percent(printed(out, "left"), 3.5));
  CHECK(within_2_percent(printed(out, "back"), 2.0));
  CHECK(out.find("right tracker") == std::string::npos);
  CHECK(out.find("front tracker") == std::string::npos);
}

TEST_CASE("measure_offsets: a reversed tracker is named, and its offset is still measured") {
  {
    Cfg c;
    c.horiz = 2.0;
    c.horiz_wired_left = false;  // counts up when the robot moves right, backwards
    Rig rig(c);
    const std::string out = run(rig);
    CHECK(out.find("back tracker looks reversed, flip its port sign") != std::string::npos);
    CHECK(within_2_percent(offset_of(rig.th.get()), 2.0));
  }
  {
    Cfg c;
    c.horiz = 2.0;
    c.horiz_front = true;
    c.horiz_wired_left = false;
    Rig rig(c);
    const std::string out = run(rig);
    CHECK(out.find("front tracker looks reversed, flip its port sign") != std::string::npos);
  }
  {
    Cfg c;
    c.left = 3.5;
    c.right = 1.0;
    c.right_reversed = true;
    Rig rig(c);
    const std::string out = run(rig);
    CHECK(out.find("right tracker looks reversed, flip its port sign") != std::string::npos);
    CHECK(out.find("left tracker looks reversed") == std::string::npos);
    CHECK(within_2_percent(offset_of(rig.tr.get()), 1.0));
  }
  {
    Cfg c;
    c.left = 3.5;
    c.left_reversed = true;
    Rig rig(c);
    const std::string out = run(rig);
    CHECK(out.find("left tracker looks reversed, flip its port sign") != std::string::npos);
  }
}
