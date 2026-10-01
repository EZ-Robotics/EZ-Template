// Setting the odom pose mid motion with EZ-Template's own tracking: a pose set is not movement.
//
// On a pose set pass the old code skipped the sensor update, so xyPID's derivative kept its old value: a team that
// relocalized on every pass while moving froze the derivative at a stale speed (the robot stopped 6 in short of a 24 in
// target and was marked interfered), and one odom_xyt_set() while moving spiked the derivative by up to 11 in. The fix
// moves the "last pose" along with the pose, so a set is never counted as movement and what the robot drives after it
// still is. light_fast, noise off.
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "odom_origin_rig.hpp"

using namespace ez;
using namespace origin;

namespace {

enum class Motion { ptp, pure_pursuit };

void start_motion(Rig& r, Motion m) {
  if (m == Motion::ptp) r.chassis.pid_odom_set(O(0, 48, fwd, 110));
  else r.chassis.pid_odom_set(std::vector<odom>{O(0, 24, fwd, 110), O(0, 48, fwd, 110)});
}

struct Set {
  const char* name;
  std::function<void(Drive&)> apply;
};

std::vector<Set> sets() {
  return {
      {"odom_xyt_set to the same pose", [](Drive& d) { d.odom_xyt_set(d.odom_x_get(), d.odom_y_get(), d.odom_theta_get()); }},
      {"odom_xyt_set, a 20 in jump", [](Drive& d) { d.odom_xyt_set(d.odom_x_get() + 12.0, d.odom_y_get() + 16.0, d.odom_theta_get()); }},
      {"odom_xy_set only", [](Drive& d) { d.odom_xy_set(d.odom_x_get() + 5.0, d.odom_y_get() - 3.0); }},
      {"odom_x_set only", [](Drive& d) { d.odom_x_set(d.odom_x_get() + 4.0); }},
      {"drive_angle_set(+30)", [](Drive& d) { d.drive_angle_set(d.odom_theta_get() + 30.0); }},
  };
}

}  // namespace

// About 300 ms into the motion, 30 passes, one pose set. On the pass after it xyPID's derivative is the movement measured from
// the set pose.
TEST_CASE("the pass after a pose set mid motion reads the real movement from the set pose" * doctest::should_fail()) {
  for (Motion m : {Motion::ptp, Motion::pure_pursuit}) {
    for (const Set& s : sets()) {
      Rig r;
      r.start_at(0, 0, 0);
      start_motion(r, m);
      r.hook = [&](int n) {
        if (n == 30) s.apply(r.chassis);
      };
      r.run([&] { r.chassis.pid_wait(); }, 40);
      const PassRow* row = nullptr;
      for (const auto& p : r.rows)
        if (p.n == 30) row = &p;
      CAPTURE(std::string(s.name));
      CAPTURE((int)m);
      REQUIRE(row != nullptr);
      CHECK(std::fabs(row->deriv - row->dref) < 0.05);
    }
  }
}

// A team relocalizing on every pass with the pose the robot is already at. Nothing about the motion may change.
TEST_CASE("relocalizing to the current pose on every pass changes nothing about a 24 in ptp" * doctest::should_fail()) {
  for (int jitter : {0, 1, 2}) {
    Outcome plain;
    {
      Rig r(sim::archetype_light_fast(), jitter);
      r.tight_exits();
      r.start_at(0, 0, 0);
      r.chassis.pid_odom_ptp_set(O(0, 24, fwd, 90));
      plain = r.run([&] { r.chassis.pid_wait(); });
    }
    Rig r(sim::archetype_light_fast(), jitter);
    r.tight_exits();
    r.start_at(0, 0, 0);
    r.chassis.pid_odom_ptp_set(O(0, 24, fwd, 90));
    r.hook = [&](int n) {
      if (n >= 20 * (jitter + 1)) r.chassis.odom_xyt_set(r.chassis.odom_x_get(), r.chassis.odom_y_get(), r.chassis.odom_theta_get());
    };
    Outcome set = r.run([&] { r.chassis.pid_wait(); });
    CAPTURE(jitter);
    CAPTURE(plain.ms);
    CAPTURE(set.ms);
    REQUIRE(plain.returned);
    REQUIRE(set.returned);
    CHECK_FALSE(set.interfered);
    CHECK(std::fabs(set.end.y - 24.0) < 1.0);
    CHECK(std::abs(set.ms - plain.ms) <= 30);
    CHECK(std::hypot(set.end.x - plain.end.x, set.end.y - plain.end.y) <= 0.3);
  }
}

// Relocalizing every pass while a wall holds the robot 8 in short of the target. The set pose is the robot's real pose, so
// nothing moves: xyPID's derivative has to read 0 once the robot has stopped, and the wait has to see that the robot is stuck.
TEST_CASE("relocalizing every pass against a wall reads no movement and the wait ends interfered") {
  Rig r;
  r.start_at(0, 0, 0);
  r.sim.wall(16.0);
  r.chassis.pid_odom_ptp_set(O(0, 24, fwd, 90));
  r.hook = [&](int n) {
    if (n >= 10) r.chassis.odom_xyt_set(r.chassis.odom_x_get(), r.chassis.odom_y_get(), r.chassis.odom_theta_get());
  };
  std::uint32_t contact_ms = 0;
  double worst_still = 0;
  int still_passes = 0;
  // this replaces the rig's own row logging, which this test does not use
  r.sim.after_pass = [&](int) {
    if (contact_ms == 0 && r.chassis.odom_y_get() >= 15.8) contact_ms = pros::millis();
    if (contact_ms != 0 && pros::millis() > contact_ms + 100) {
      worst_still = std::fmax(worst_still, std::fabs(r.chassis.xyPID.derivative));
      still_passes++;
    }
  };
  Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1000);
  REQUIRE(o.returned);
  REQUIRE(contact_ms != 0);
  CHECK(still_passes > 5);
  CHECK(worst_still < 0.05);
  CHECK(o.interfered);
  // the stuck window with these exits is the 350 ms floor
  CHECK(pros::millis() - contact_ms <= 350 + 250);
}

// A pose that is not a number mid motion, then put back. The sensor must not be poisoned for good.
TEST_CASE("a non finite pose mid motion does not poison the sensor and the robot arrives after it is put back") {
  Rig r;
  r.start_at(0, 0, 0);
  r.chassis.pid_odom_ptp_set(O(0, 36, fwd, 90));
  double good_x = 0;
  r.hook = [&](int n) {
    if (n == 29) good_x = r.chassis.odom_x_get();
    if (n == 30) r.chassis.odom_x_set(NAN);
    if (n == 33) r.chassis.odom_x_set(good_x);
  };
  Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1200);
  REQUIRE(o.returned);
  CHECK(std::isfinite(DriveTestAccess::new_current_fake(r.chassis)));
  CHECK(std::hypot(o.end.x - 0.0, o.end.y - 36.0) < 1.5);
}

// Tracking paused for 100 ms and a sensor reset: neither may show up as movement on the pass that follows. These are guards
// for the new sensor, not fixes: the old sensor survives them too.
TEST_CASE("paused tracking and a sensor reset mid motion do not spike the derivative or hang") {
  {
    Rig r;
    r.start_at(0, 0, 0);
    r.chassis.pid_odom_ptp_set(O(0, 36, fwd, 90));
    r.hook = [&](int n) {
      if (n == 30) r.chassis.odom_enable(false);
      if (n == 40) r.chassis.odom_enable(true);
    };
    Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
    CHECK(o.returned);
    for (const auto& p : r.rows)
      if (p.n == 40 || p.n == 41) CHECK(std::fabs(p.deriv) < 1.0);
  }
  {
    Rig r;
    r.start_at(0, 0, 0);
    r.chassis.pid_odom_ptp_set(O(0, 36, fwd, 90));
    int reset_pass = -1;
    r.hook = [&](int n) {
      if (reset_pass < 0 && n >= 30) {
        reset_pass = n;
        r.chassis.drive_sensor_reset();
        r.sim.tare_encoders();
      }
    };
    Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
    CHECK(o.returned);
    for (const auto& p : r.rows)
      if (p.n == reset_pass || p.n == reset_pass + 1) CHECK(std::fabs(p.deriv) < 1.0);
  }
}
