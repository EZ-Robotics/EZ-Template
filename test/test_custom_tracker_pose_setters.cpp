// Custom tracking functions (odom_tracking_set()) that write the pose through the pose setters.
//
// A team's tracking function can write the pose two ways: straight to odom_current, or through odom_x_set() / odom_y_set() /
// odom_xy_set() / odom_xyt_set() / odom_pose_set(). Both are the robot moving, and both have to read the same everywhere
// the library measures movement: xyPID's derivative and the "stopped" check the waits use (Travel::OdomXY).
//
// The setters move xy_last_pose with the pose so a pose set made by the team's own code is not counted as movement. Called
// from inside the tracking function, that made every pass read no movement at all: odom xy read "stopped" while the robot
// drove at full speed, and pid_wait() came back clean the first pass the robot was inside big_error (classroom: 1030 ms,
// 2.99 in short, still at about 20 in/s; the same tracker writing odom_current: 1440 ms, 0.18 in).
//
// A pose set from the team's own code, between passes, must still not count as movement: these tests check both.
#include <cmath>
#include <functional>
#include <string>

#include "exit_gate_rig.hpp"
#include "odom_origin_rig.hpp"

using namespace ez;

namespace {

// How a tracking function writes the pose it tracked
enum class Style { Direct, XThenY, XY, XYT, PoseSet, IncrementalSetters, Count };

std::string style_name(Style s) {
  switch (s) {
    case Style::Direct: return "odom_current";
    case Style::XThenY: return "odom_x_set + odom_y_set";
    case Style::XY: return "odom_xy_set";
    case Style::XYT: return "odom_xyt_set";
    case Style::PoseSet: return "odom_pose_set";
    case Style::IncrementalSetters: return "odom_x_set(odom_x_get() + dx)";
    default: return "?";
  }
}

// The robot's true field position, from the sim's wheels along the IMU heading (what a perfect GPS would read)
struct Truth {
  sim::SimRobot& sim;
  Drive& chassis;
  double x = 0, y = 0, last = 0;
  Truth(sim::SimRobot& s, Drive& d) : sim(s), chassis(d) { last = wheels(); }
  double wheels() const { return (sim.left().position_in + sim.right().position_in) / 2.0; }
  void reset(double px, double py) {
    x = px;
    y = py;
    last = wheels();
  }
  void step() {
    double now = wheels();
    double th = chassis.drive_angle_get() * M_PI / 180.0;
    x += (now - last) * std::sin(th);
    y += (now - last) * std::cos(th);
    last = now;
  }
};

// Installs a tracking function that tracks the true position and writes it in the given style
void install(Drive& chassis, Truth& truth, Style style) {
  chassis.odom_tracking_set([&chassis, &truth, style] {
    double ox = truth.x, oy = truth.y;
    truth.step();
    switch (style) {
      case Style::Direct:
        chassis.odom_current.x = truth.x;
        chassis.odom_current.y = truth.y;
        break;
      case Style::XThenY:
        chassis.odom_x_set(truth.x);
        chassis.odom_y_set(truth.y);
        break;
      case Style::XY:
        chassis.odom_xy_set(truth.x, truth.y);
        break;
      case Style::XYT:
        chassis.odom_xyt_set(truth.x, truth.y, chassis.drive_angle_get());
        break;
      case Style::PoseSet:
        chassis.odom_pose_set(pose{truth.x, truth.y, ANGLE_NOT_SET});
        break;
      case Style::IncrementalSetters:
        chassis.odom_x_set(chassis.odom_x_get() + (truth.x - ox));
        chassis.odom_y_set(chassis.odom_y_get() + (truth.y - oy));
        break;
      default:
        break;
    }
    chassis.odom_current.theta = chassis.drive_angle_get();
  });
}

enum class Motion { Ptp, PtpRev, PurePursuit, Boomerang, QuickChainThenWait, Count };

std::string motion_name(Motion m) {
  switch (m) {
    case Motion::Ptp: return "ptp";
    case Motion::PtpRev: return "ptp rev";
    case Motion::PurePursuit: return "pure pursuit";
    case Motion::Boomerang: return "boomerang";
    case Motion::QuickChainThenWait: return "quick chain then wait";
    default: return "?";
  }
}

struct End {
  bool returned = false;
  bool interfered = false;
  double ms = 0;
  pose odom{0, 0, 0};
  double true_short = 0;  // true distance from the robot to the final target when the wait returned
  double true_speed = 0;  // true speed of the faster side when the wait returned
};

End run(const sim::SimArchetype& a, Style style, int passes, Motion motion) {
  gate::Rig r(a, passes);
  Truth truth(r.sim, r.chassis);
  install(r.chassis, truth, style);
  r.chassis.odom_xyt_set(0, 0, 0);
  truth.reset(0, 0);
  for (int i = 0; i < 5; i++) pros::delay(ez::util::DELAY_TIME);

  double tx = 0, ty = 0;
  std::function<void()> wait;
  switch (motion) {
    case Motion::Ptp:
      tx = 12, ty = 24;
      r.chassis.pid_odom_set(odom{pose{tx, ty, ANGLE_NOT_SET}, fwd, 110});
      wait = [&] { r.chassis.pid_wait(); };
      break;
    case Motion::PtpRev:
      tx = -6, ty = -24;
      r.chassis.pid_odom_set(odom{pose{tx, ty, ANGLE_NOT_SET}, rev, 110});
      wait = [&] { r.chassis.pid_wait(); };
      break;
    case Motion::PurePursuit:
      tx = 24, ty = 36;
      r.chassis.pid_odom_set({odom{pose{0, 24, ANGLE_NOT_SET}, fwd, 110}, odom{pose{tx, ty, ANGLE_NOT_SET}, fwd, 110}});
      wait = [&] { r.chassis.pid_wait(); };
      break;
    case Motion::Boomerang:
      tx = 12, ty = 30;
      r.chassis.pid_odom_set(odom{pose{tx, ty, 45}, fwd, 110});
      wait = [&] { r.chassis.pid_wait(); };
      break;
    case Motion::QuickChainThenWait:
      tx = 0, ty = 48;
      r.chassis.pid_odom_set(odom{pose{0, 24, ANGLE_NOT_SET}, fwd, 110});
      wait = [&] {
        r.chassis.pid_wait_quick_chain();
        r.chassis.pid_odom_set(odom{pose{tx, ty, ANGLE_NOT_SET}, fwd, 110});
        r.chassis.pid_wait();
      };
      break;
    default:
      break;
  }
  End e;
  e.returned = r.wait(wait, 1500, &e.ms);
  e.interfered = r.chassis.interfered;
  e.odom = r.chassis.odom_pose_get();
  e.true_short = std::hypot(tx - truth.x, ty - truth.y);
  e.true_speed = r.drive_speed_now();
  return e;
}

}  // namespace

// The report's own repro: classroom, two passes per poll, no noise, default exits.
TEST_CASE("a tracker writing the pose with odom_x_set / odom_y_set does not return clean while the robot is still driving") {
  End e = run(gate::archetype_classroom(), Style::XThenY, 2, Motion::Ptp);
  REQUIRE(e.returned);
  CHECK_FALSE(e.interfered);
  CHECK(e.true_short < 1.0);
  CHECK(e.true_speed < 5.0);
}

// The tracked pose is the same either way, so everything the library does with it has to be the same too: when the wait
// returns, what it returns, and where odom ends. Compared exactly against the tracker that writes odom_current.
TEST_CASE("a tracker ends every odom motion exactly the same whether it writes odom_current or uses the pose setters") {
  sim::SimArchetype archs[] = {gate::archetype_classroom(), sim::archetype_light_fast(), sim::archetype_heavy_slow()};
  for (const auto& a : archs) {
    for (int passes : {1, 2, 3}) {
      for (int m = 0; m < (int)Motion::Count; m++) {
        End ref = run(a, Style::Direct, passes, (Motion)m);
        for (int s = 1; s < (int)Style::Count; s++) {
          End e = run(a, (Style)s, passes, (Motion)m);
          std::string arch = a.name;
          CAPTURE(arch);
          CAPTURE(passes);
          CAPTURE(motion_name((Motion)m));
          CAPTURE(style_name((Style)s));
          CHECK(e.returned == ref.returned);
          CHECK(e.interfered == ref.interfered);
          CHECK(e.ms == ref.ms);
          CHECK(std::fabs(e.odom.x - ref.odom.x) < 1e-9);
          CHECK(std::fabs(e.odom.y - ref.odom.y) < 1e-9);
          CHECK(std::fabs(e.true_short - ref.true_short) < 1e-9);
        }
      }
    }
  }
}

// xyPID's derivative is the robot's real movement on every ordinary pass, and 0 on the pass after the team's own code sets
// the pose (a custom tracker may write its own pose over a set, so the jump back cannot be trusted as movement).
TEST_CASE("a tracker using the pose setters: xy derivative is the real movement, and 0 on the pass after a pose set from team code") {
  for (Style style : {Style::XThenY, Style::XY, Style::XYT, Style::PoseSet, Style::IncrementalSetters}) {
    origin::Rig r;
    Truth truth(r.sim, r.chassis);
    install(r.chassis, truth, style);
    r.start_at(24, 24, 0);
    truth.reset(24, 24);
    r.chassis.pid_odom_ptp_set(origin::O(24, 84, fwd, 90));
    // An incremental tracker keeps the set pose, an absolute one writes the true pose straight back over it: either way the
    // pass after the set reads 0
    r.hook = [&](int n) {
      if (n == 30) r.chassis.odom_xyt_set(r.chassis.odom_x_get() + 12.0, r.chassis.odom_y_get() + 10.0, r.chassis.odom_theta_get());
    };
    origin::Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
    CAPTURE(style_name(style));
    REQUIRE(o.returned);
    bool saw_set_pass = false;
    int moving_passes = 0;
    for (const auto& p : r.rows) {
      if (p.mode != POINT_TO_POINT) continue;
      CAPTURE(p.n);
      if (p.n == 30) {
        saw_set_pass = true;
        CHECK(std::fabs(p.deriv) < 0.05);
      } else {
        CHECK(std::fabs(p.deriv - p.dref) < 0.05);
        if (std::fabs(p.dref) > 0.1) moving_passes++;
      }
    }
    CHECK(saw_set_pass);
    CHECK(moving_passes > 20);  // the robot really drove, so the derivative above was checked on real movement
  }
}

// A pose set from team code is still never movement: a robot sitting still, with the tracker writing its true pose through
// the setters and the team setting the pose somewhere else every few passes, reads no movement on any pass.
TEST_CASE("pose sets from team code are not movement when the tracker uses the pose setters") {
  for (Style style : {Style::XThenY, Style::XY, Style::XYT, Style::PoseSet, Style::IncrementalSetters}) {
    origin::Rig r;
    Truth truth(r.sim, r.chassis);
    install(r.chassis, truth, style);
    r.start_at(0, 0, 0);
    truth.reset(0, 0);
    double worst = 0;
    int sets = 0;
    r.sim.after_pass = [&](int) {
      worst = std::fmax(worst, std::hypot(DriveTestAccess::xy_pose_delta(r.chassis).x, DriveTestAccess::xy_pose_delta(r.chassis).y));
    };
    for (int t = 0; t < 100; t++) {
      if (t % 5 == 0) {
        r.chassis.odom_xy_set(r.chassis.odom_x_get() + 10.0, r.chassis.odom_y_get() - 7.0);
        sets++;
      }
      pros::delay(ez::util::DELAY_TIME);
    }
    CAPTURE(style_name(style));
    CHECK(sets == 20);
    CHECK(worst < 1e-9);
  }
}

// A GPS-style tracker that writes the true pose through odom_xy_set(), and the team also relocalizes 1.5 in off on every pass:
// the same case test_odom_origin_custom_tracking.cpp checks for a tracker writing odom_current.
TEST_CASE("a GPS style tracker using odom_xy_set, relocalized every pass, arrives within 1 in and is not marked interfered") {
  for (int jitter : {0, 1, 2}) {
    for (pose s : {pose{0, 0, 0}, pose{60, -60, 0}}) {
      origin::Rig r(sim::archetype_light_fast(), jitter);
      r.tight_exits();
      Truth truth(r.sim, r.chassis);
      install(r.chassis, truth, Style::XY);
      truth.reset(s.x, s.y);
      r.start_at(s.x, s.y, s.theta);
      r.chassis.pid_odom_ptp_set(origin::O(s.x, s.y + 24, fwd, 90));
      r.hook = [&](int n) {
        if (n >= 20) r.chassis.odom_xy_set(truth.x + 1.5, truth.y + 1.5);
      };
      origin::Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
      CAPTURE(jitter);
      CAPTURE(s.x);
      REQUIRE(o.returned);
      CHECK_FALSE(o.interfered);
      CHECK(std::hypot(truth.x - s.x, truth.y - (s.y + 24)) < 1.0);
    }
  }
}

// A pose set far from where a setter-style tracker says the robot is, right before a motion: the tracker writes its own pose
// back on the next pass, a 40 in jump that must not read as the robot moving.
TEST_CASE("a pose set far from a setter-style tracker's pose right before a motion does not drive the robot backwards") {
  origin::Rig r;
  Truth truth(r.sim, r.chassis);
  install(r.chassis, truth, Style::XY);
  r.start_at(0, 0, 0);
  truth.reset(0, 0);
  r.chassis.odom_xyt_set(0, -40, 0);
  r.chassis.pid_odom_ptp_set(origin::O(0, 100, fwd, 110));
  r.run([&] { r.chassis.pid_wait(); }, 1);
  REQUIRE(r.rows.size() >= 1);
  CHECK(r.rows[0].l_mv >= 0);
  CHECK(r.rows[0].r_mv >= 0);
}

// Installing a setter-style tracker in another frame while a motion runs: the jump on its first pass is not movement.
TEST_CASE("installing a setter-style tracker in another frame mid-motion does not read the jump as movement") {
  origin::Rig r;
  Truth truth(r.sim, r.chassis);
  r.start_at(0, 0, 0);
  r.chassis.pid_odom_ptp_set(origin::O(0, 60, fwd, 110));
  r.run([&] { r.chassis.pid_wait(); }, 25);  // get the robot moving on EZ-Template's own tracking
  REQUIRE(r.rows.size() >= 5);
  truth.reset(0, r.chassis.odom_y_get() + 30.0);  // the tracker's frame is 30 in ahead of odom
  install(r.chassis, truth, Style::XThenY);
  size_t installed_at = r.rows.size();
  r.run([&] { r.chassis.pid_wait(); }, 10);
  REQUIRE(r.rows.size() > installed_at + 2);
  CHECK(std::fabs(r.rows[installed_at].deriv) < 1.0);
  for (size_t i = installed_at + 1; i < r.rows.size(); i++) CHECK(std::fabs(r.rows[i].deriv - r.rows[i].dref) < 0.05);
}

// A team wrapping EZ-Template's own tracking in a custom function reads the same as EZ-Template's own tracking.
TEST_CASE("EZ-Template's own tracking wrapped in odom_tracking_set() ends the same as EZ-Template's own tracking") {
  for (int passes : {1, 2}) {
    End ends[2];
    for (int wrapped = 0; wrapped < 2; wrapped++) {
      gate::Rig r(gate::archetype_classroom(), passes);
      if (wrapped) r.chassis.odom_tracking_set([&r] { r.chassis.tracking_wheels_tracking(); });
      r.chassis.odom_xyt_set(0, 0, 0);
      for (int i = 0; i < 5; i++) pros::delay(ez::util::DELAY_TIME);
      r.chassis.pid_odom_set(odom{pose{12, 24, ANGLE_NOT_SET}, fwd, 110});
      End& e = ends[wrapped];
      e.returned = r.wait([&] { r.chassis.pid_wait(); }, 1500, &e.ms);
      e.interfered = r.chassis.interfered;
      e.odom = r.chassis.odom_pose_get();
    }
    CAPTURE(passes);
    REQUIRE(ends[0].returned);
    CHECK(ends[1].returned);
    CHECK(ends[1].interfered == ends[0].interfered);
    CHECK(ends[1].ms == ends[0].ms);
    CHECK(std::fabs(ends[1].odom.x - ends[0].odom.x) < 1e-9);
    CHECK(std::fabs(ends[1].odom.y - ends[0].odom.y) < 1e-9);
  }
}

namespace {

// What the team's own code does to the pose between passes, while the tracking function tracks the true position
enum class Team { Nothing, SetterEveryPass, DirectWriteEveryPassOffset, DirectWriteOnceBack4 };

struct TeamEnd : End {
  double d_at_write = 0;  // xyPID's derivative on the pass after the team's single write (DirectWriteOnceBack4)
};

TeamEnd run_team(const sim::SimArchetype& a, Style style, int passes, Team team) {
  gate::Rig r(a, passes);
  Truth truth(r.sim, r.chassis);
  install(r.chassis, truth, style);
  r.chassis.odom_xyt_set(0, 0, 0);
  truth.reset(0, 0);
  for (int i = 0; i < 5; i++) pros::delay(ez::util::DELAY_TIME);
  int pass = 0;
  TeamEnd e;
  r.sim.before_pass = [&](int) {
    pass++;
    switch (team) {
      case Team::SetterEveryPass:
        r.chassis.odom_xy_set(truth.x, truth.y);
        break;
      case Team::DirectWriteEveryPassOffset:  // relocalizing by writing odom_current from the team's code, 1.5 in off
        r.chassis.odom_current.x = truth.x + 1.5;
        r.chassis.odom_current.y = truth.y + 1.5;
        break;
      case Team::DirectWriteOnceBack4:
        if (pass == 25) r.chassis.odom_current.y -= 4.0;
        break;
      default:
        break;
    }
  };
  auto record = r.sim.after_pass;
  r.sim.after_pass = [&, record](int n) {
    record(n);
    if (team == Team::DirectWriteOnceBack4 && pass == 25) e.d_at_write = r.chassis.xyPID.derivative;
  };
  r.chassis.pid_odom_set(odom{pose{0, 40, ANGLE_NOT_SET}, fwd, 110});
  e.returned = r.wait([&] { r.chassis.pid_wait(); }, 1500, &e.ms);
  e.interfered = r.chassis.interfered;
  e.odom = r.chassis.odom_pose_get();
  e.true_short = std::hypot(0 - truth.x, 40 - truth.y);
  e.true_speed = r.drive_speed_now();
  return e;
}

}  // namespace

// The team's own code relocalizing on every pass (a GPS read in a separate task, say) while a custom tracking function runs:
// the library cannot tell how far the robot moved on a pass after a pose set, so it must not take that as "stopped" either.
TEST_CASE("a custom tracker with the pose set from team code on every pass does not return clean while the robot is still driving") {
  sim::SimArchetype archs[] = {gate::archetype_classroom(), sim::archetype_light_fast(), sim::archetype_heavy_slow()};
  for (const auto& a : archs) {
    for (int passes : {1, 2}) {
      for (Style style : {Style::Direct, Style::XY}) {
        TeamEnd e = run_team(a, style, passes, Team::SetterEveryPass);
        std::string arch = a.name;
          CAPTURE(arch);
        CAPTURE(passes);
        CAPTURE(style_name(style));
        REQUIRE(e.returned);
        if (!e.interfered) {
          CHECK(e.true_short < 1.5);
          CHECK(e.true_speed < 5.0);
        }
      }
    }
  }
}

// Team code that writes odom_current itself, outside the tracking function, with no setter. EZ-Template measures a custom
// tracker's movement from the pose the last pass ended on, so a tracker that writes the true pose back reads only the real
// movement: it arrives as if nothing was written, with no derivative kick from the write.
TEST_CASE("team code writing odom_current between passes reads the same for a tracker writing odom_current or using the setters") {
  for (int passes : {1, 2}) {
    for (Team team : {Team::DirectWriteEveryPassOffset, Team::DirectWriteOnceBack4}) {
      TeamEnd ref = run_team(sim::archetype_light_fast(), Style::Direct, passes, team);
      CAPTURE(passes);
      CAPTURE((int)team);
      REQUIRE(ref.returned);
      CHECK_FALSE(ref.interfered);
      CHECK(ref.true_short < 1.0);
      if (team == Team::DirectWriteOnceBack4) CHECK(std::fabs(ref.d_at_write) < 1.0);
      for (Style style : {Style::XThenY, Style::XY, Style::PoseSet}) {
        TeamEnd e = run_team(sim::archetype_light_fast(), style, passes, team);
        CAPTURE(style_name(style));
        CHECK(e.returned == ref.returned);
        CHECK(e.interfered == ref.interfered);
        CHECK(e.ms == ref.ms);
        CHECK(std::fabs(e.true_short - ref.true_short) < 1e-9);
        CHECK(std::fabs(e.d_at_write - ref.d_at_write) < 1e-9);
      }
    }
  }
}
