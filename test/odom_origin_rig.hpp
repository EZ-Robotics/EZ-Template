// Shared rig for the odom origin tests (test_odom_origin_*.cpp).
//
// xyPID's "sensor" has to read how far the robot's own movement changed xyPID's error on each pass, with that pass's
// target held fixed, and nothing else: not where on the field the robot is, not a target that moved, not a pose set.
// The rig runs a light_fast sim robot (noise off unless a test asks for it), logs one row per auto task pass, and for each
// pass works out that movement itself (PassRow::dref) so a test can compare it to what xyPID actually read.
#pragma once

#include <cmath>
#include <functional>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

namespace origin {

using namespace ez;

struct PassRow {
  int n = 0;
  std::uint32_t ms = 0;  // pros::millis() when the pass ran
  e_mode mode = DISABLE;
  pose start{0, 0, 0};  // the odom pose at the start of the pass, after anything a test's hook set
  pose end{0, 0, 0};    // the odom pose after the pass
  double err = 0, deriv = 0;
  // How much the robot's movement over this pass changed xyPID's error against this pass's target: moving toward the
  // point reads positive, away negative, in the units xyPID's derivative has.
  double dref = 0;
  double l_mv = 0, r_mv = 0;
};

struct Outcome {
  bool returned = false;
  bool interfered = false;
  int ms = 0;  // pros::millis() from the first wait to the return
  pose end{0, 0, 0};
  double turned = 0;  // total heading change, degrees, summed over the passes
};

// An odom movement to (x, y), in inches and degrees; theta left unset is a plain point, set is a boomerang.
inline odom O(double x, double y, drive_directions d = fwd, int speed = 110, double theta = ANGLE_NOT_SET) { return odom{pose{x, y, theta}, d, speed}; }

inline Drive make_drive(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

struct Rig {
  sim::SimArchetype a;
  Drive chassis;
  sim::SimRobot sim;
  std::vector<PassRow> rows;
  // Runs before every pass, before the snapshot the row's dref is measured from. A pose set made here is not movement.
  std::function<void(int)> hook;

  // jitter: extra auto task passes per poll (0 is one pass per poll, 1 is two, ...)
  explicit Rig(const sim::SimArchetype& arch = sim::archetype_light_fast(), int jitter = 0, bool noise = false, std::uint32_t seed = 1)
      : a(arch), chassis(make_drive(arch)), sim(chassis, arch, sim::NoiseConfig{noise, seed}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 200_ms, 3_deg, 100_ms, 100_ms);
    sim.passes_per_tick(1 + jitter);
    sim.before_pass = [this](int n) {
      if (hook) hook(n);
      snap_ = chassis.odom_pose_get();
    };
    sim.after_pass = [this](int n) { record(n); };
  }

  // Exits that only let a wait return once the robot is within 1 in of its target (0.5 in for the small exit), so "arrived"
  // does not mean "inside the default 3 in big error band for 200 ms".
  void tight_exits() { chassis.pid_odom_drive_exit_condition_set(90_ms, 0.5_in, 200_ms, 1_in, 100_ms, 100_ms); }

  // Sets the pose and lets a few idle passes go by, so the next motion starts from a settled robot.
  void start_at(double x, double y, double theta) {
    chassis.odom_xyt_set(x, y, theta);
    idle(5);
    rows.clear();
  }
  void idle(int ticks) {
    for (int i = 0; i < ticks; i++) pros::delay(ez::util::DELAY_TIME);
  }

  // Runs `wait` until it returns or `max_ticks` polls have gone by.
  Outcome run(const std::function<void()>& wait, int max_ticks = 3000) {
    Outcome o;
    std::uint32_t t0 = pros::millis();
    size_t first_row = rows.size();
    test_stub::g_clock.delay_calls_until_stop = max_ticks;
    o.returned = true;
    try {
      wait();
    } catch (test_stub::StopLoop&) {
      o.returned = false;
    }
    test_stub::g_clock.delay_calls_until_stop = -1;
    o.ms = (int)(pros::millis() - t0);
    o.interfered = chassis.interfered;
    o.end = chassis.odom_pose_get();
    for (size_t i = first_row; i < rows.size(); i++) o.turned += std::fabs(rows[i].end.theta - rows[i].start.theta);
    return o;
  }

  // Everything below is read straight off the rows.
  double max_abs_deriv_error(size_t from = 0, size_t to = (size_t)-1) const {
    double worst = 0;
    for (size_t i = from; i < rows.size() && i < to; i++)
      if (rows[i].mode == POINT_TO_POINT || rows[i].mode == PURE_PURSUIT) worst = std::fmax(worst, std::fabs(rows[i].deriv - rows[i].dref));
    return worst;
  }
  double min_mv() const {
    double m = 1e9;
    for (const auto& r : rows) m = std::fmin(m, std::fmin(r.l_mv, r.r_mv));
    return m;
  }

private:
  pose snap_{0, 0, 0};

  void record(int n) {
    PassRow r;
    r.n = n;
    r.ms = pros::millis();
    r.mode = chassis.drive_mode_get();
    r.start = snap_;
    r.end = chassis.odom_pose_get();
    r.err = chassis.xyPID.error;
    r.deriv = chassis.xyPID.derivative;
    if (r.mode == POINT_TO_POINT || r.mode == PURE_PURSUIT) {
      pose t = DriveTestAccess::odom_target(chassis);
      r.dref = DriveTestAccess::drive_dir_sign(chassis) *
               (DriveTestAccess::is_past_target(chassis, t, r.end) - DriveTestAccess::is_past_target(chassis, t, r.start));
    }
    r.l_mv = chassis.left_motors.front().fake().voltage;
    r.r_mv = chassis.right_motors.front().fake().voltage;
    rows.push_back(r);
  }
};

// The robot's true field pose, kept from the sim's wheels and the IMU the way a GPS would know it, apart from anything the
// odom pose has been set to. step() adds what the wheels have moved since the last call, along the IMU's heading.
struct TruePose {
  Rig& r;
  double x = 0, y = 0;
  explicit TruePose(Rig& rig) : r(rig) {}
  void reset(double px, double py) {
    x = px;
    y = py;
    last_ = wheels();
  }
  void step() {
    double now = wheels();
    double th = r.chassis.drive_angle_get() * M_PI / 180.0;
    x += (now - last_) * std::sin(th);
    y += (now - last_) * std::cos(th);
    last_ = now;
  }

private:
  double last_ = 0;
  double wheels() const { return (r.sim.left().position_in + r.sim.right().position_in) / 2.0; }
};

}  // namespace origin
