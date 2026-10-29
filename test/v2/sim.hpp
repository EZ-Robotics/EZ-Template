// A tank drive on a flat field, moved by its motors and by what the test does to it.
//
// The sim runs every time the library calls pros::delay(), in 10 ms steps up to the fake clock. Each step runs one pass of
// the real ez_auto_task() and then moves the robot for 10 ms with the voltage that pass wrote. Motor heat, battery sag,
// wheel slip and sensor noise are left out until there is robot log data to model them from.
#pragma once

#include <cmath>
#include <deque>
#include <functional>
#include <utility>
#include <vector>

#include "../drive_test_access.hpp"
#include "../fake_hardware.hpp"
#include "EZ-Template/api.hpp"

namespace sim {

// What the Drive isn't told: how heavy the robot is and how far apart its wheels are.
struct Body {
  double mass_kg;
  double track_width_in;
};

// Thrown out of pros::delay() when the robot has sat still past the deadline and the code is still waiting.
struct WaitNeverReturned {};

class Robot {
public:
  Robot(ez::Drive& drive, double wheel_rpm, double wheel_diameter_in, Body body)
      : drive_(drive), body_(body), wheel_rpm_(wheel_rpm), wheel_diameter_in_(wheel_diameter_in), sim_ms_(test_stub::g_clock.now_ms) {
    active_ = this;
    test_stub::g_clock.on_delay = [] { active_->catch_up(); };
  }
  ~Robot() {
    test_stub::g_clock.on_delay = nullptr;
    active_ = nullptr;
  }

  // Picks the robot up and sets it down, at rest, here. For a test that starts somewhere other than (0, 0, 0).
  void place(double x, double y, double heading) {
    x_in_ = x;
    y_in_ = y;
    heading_deg_ = heading;
    v_ = w_ = 0.0;
    history_.clear();
  }

  // Field situations. Times are ms from now.
  void pin(int start, int duration) { pins_.push_back({now() + start, now() + start + duration, 0}); }
  // As hard as this robot can push at full power: what an identical robot shoving it can do.
  double full_push_n() const { return 2.0 * stall_force_n(); }
  void push(double newtons, int start, int duration) { pushes_.push_back({now() + start, now() + start + duration, newtons}); }
  void wall_ahead(double inches) { wall_at_in_ = forward_in_ + inches; }
  void unplug_sensor_motor() { drive_.left_motors.front().fake().disconnected = true; }
  void fail_one_read(int start) { fail_read_at_ = now() + start; }
  // The drive's real gearing changes. The encoders keep counting, but each count is now a different distance.
  void shift_to(double new_wheel_rpm) { wheel_rpm_ = new_wheel_rpm; }
  // Someone pushes the robot forward by hand, at rest, and lets go. The wheels and encoders turn with it.
  void slide(double inches) { move(inches, 0.0); }

  // Where the robot really is. Heading is clockwise positive, like the IMU. y is forward at heading 0.
  double x() const { return x_in_; }
  double y() const { return y_in_; }
  double heading() const { return heading_deg_; }
  double forward_travel() const { return forward_in_; }
  int now() const { return test_stub::g_clock.now_ms; }

  // How long the robot has moved less than 0.5 in and 1 deg in every 500 ms, or 0 while it is moving.
  int stopped_for_ms() const { return history_.size() < 51 ? 0 : sim_ms_ - stopped_since_; }

  // A vertical tracking wheel the robot carries, right_of_center_in inches right of its turning center (negative is left).
  void track_with(ez::tracking_wheel* wheel, double right_of_center_in) { trackers_.push_back({wheel, right_of_center_in, 0.0}); }

  // Something the team's own code does between two library passes, like a second task starting a motion.
  void at(int start, std::function<void()> action) { actions_.push_back({now() + start, std::move(action)}); }

  // Throw WaitNeverReturned if the code is still in pros::delay() once the robot has been stopped for `stopped_ms`, or
  // `total_ms` from now. 0 turns a limit off.
  void give_up(int stopped_ms, int total_ms) {
    give_up_stopped_ms_ = stopped_ms;
    give_up_at_ms_ = total_ms > 0 ? now() + total_ms : 0;
  }

private:
  struct Window {
    int start, end;
    double force;
  };
  struct Sample {
    double x, y, heading;
  };
  struct Tracker {
    ez::tracking_wheel* wheel;
    double right_of_center_in;
    double ticks;
  };
  struct Action {
    int at;
    std::function<void()> run;
  };

  void catch_up() {
    while (sim_ms_ + ez::util::DELAY_TIME <= (int)test_stub::g_clock.now_ms) {
      sim_ms_ += ez::util::DELAY_TIME;
      for (auto& action : actions_)
        if (action.run && sim_ms_ >= action.at) std::exchange(action.run, nullptr)();
      run_library_pass();
      step(ez::util::DELAY_TIME / 1000.0);
      if (give_up_stopped_ms_ > 0 && stopped_for_ms() > give_up_stopped_ms_) throw WaitNeverReturned{};
      if (give_up_at_ms_ > 0 && sim_ms_ > give_up_at_ms_) throw WaitNeverReturned{};
    }
  }

  // ez_auto_task() ends in its own pros::delay(), so the hook is parked while it runs and the loop is unwound after one pass.
  void run_library_pass() {
    test_stub::g_clock.on_delay = nullptr;
    test_stub::g_clock.delay_calls_until_stop = 0;
    try {
      ez::DriveTestAccess::ez_auto_task(drive_);
    } catch (test_stub::StopLoop&) {}
    test_stub::g_clock.now_ms -= ez::util::DELAY_TIME;
    test_stub::g_clock.delay_calls_until_stop = -1;
    test_stub::g_clock.on_delay = [] { active_->catch_up(); };
  }

  double free_speed_m_s() const { return wheel_rpm_ / 60.0 * M_PI * wheel_diameter_in_ * 0.0254; }

  // One side's push at stall. Like EZ-Template, the motor's cartridge is taken to be the wheel rpm with 1:1 external gearing.
  // VEX lists the V5 motor at 2.1 N*m stall with the 100 rpm cartridge, and torque scales as 100 / rpm for any other cartridge.
  double stall_force_n() const {
    double wheel_torque = 2.1 * 100.0 / wheel_rpm_;
    return drive_.left_motors.size() * wheel_torque / (wheel_diameter_in_ / 2.0 * 0.0254);
  }
  // While the field has the robot disabled, VEXos keeps every motor limp whatever it was last told.
  static bool disabled() { return test_stub::g_competition.disabled; }
  static double duty(const std::vector<pros::Motor>& side) { return disabled() ? 0.0 : side.front().fake().voltage / 12000.0; }

  // A motor pushes stall_force * (duty - speed / free_speed). At 0 volts in brake or hold it is shorted and the speed
  // term still slows it. At 0 volts in coast it does nothing.
  static bool motor_acts(const std::vector<pros::Motor>& side) {
    if (disabled()) return false;
    return side.front().fake().voltage != 0.0 || side.front().fake().brake_mode != pros::E_MOTOR_BRAKE_COAST;
  }

  void step(double dt) {
    const double g = 9.81, m = body_.mass_kg, track = body_.track_width_in * 0.0254;
    const double inertia = m * 0.38 * 0.38 / 6.0;  // a flat 15 in square chassis, an estimate
    const double a = stall_force_n(), b = a / free_speed_m_s();
    double dl = duty(drive_.left_motors), dr = duty(drive_.right_motors);
    double damping = b * ((motor_acts(drive_.left_motors) ? 1 : 0) + (motor_acts(drive_.right_motors) ? 1 : 0));

    double push = 0.0;
    for (const auto& w : pushes_)
      if (sim_ms_ >= w.start && sim_ms_ < w.end) push += w.force;

    // Rolling resistance (0.05 g) and turning scrub (0.15 g) are guesses, enough that a robot coasts to a stop.
    v_ = integrate(v_, a * (dl + dr) + push, damping / m, 0.05 * m * g, m, dt);
    w_ = integrate(w_, a * (dl - dr) * track / 2.0, damping * track * track / 4.0 / inertia, 0.15 * m * g * track / 2.0, inertia, dt);

    for (const auto& p : pins_)
      if (sim_ms_ >= p.start && sim_ms_ < p.end) v_ = w_ = 0.0;
    double forward = v_ * dt / 0.0254;
    if (forward_in_ + forward > wall_at_in_) {
      forward = wall_at_in_ - forward_in_;
      v_ = 0.0;
    }
    move(forward, w_ * dt * 180.0 / M_PI);
  }

  // Implicit Euler on the speed term, so a light, fast robot settles instead of reversing every step. Friction can stop
  // the robot but never reverses it, and holds it still while the push on it is smaller than the friction.
  static double integrate(double v, double force, double damping, double friction, double mass, double dt) {
    if (v == 0.0 && std::fabs(force) <= friction) return 0.0;
    double direction = v != 0.0 ? std::copysign(1.0, v) : std::copysign(1.0, force);
    double next = (v + dt * (force - direction * friction) / mass) / (1.0 + dt * damping);
    bool reversed = next * v < 0.0;
    if (reversed && (force * next <= 0.0 || std::fabs(force) <= friction)) return 0.0;
    return next;
  }

  void move(double forward_in, double cw_deg) {
    double half_turn_in = cw_deg * M_PI / 180.0 * body_.track_width_in / 2.0;
    double mid = (heading_deg_ + cw_deg / 2.0) * M_PI / 180.0;
    x_in_ += forward_in * std::sin(mid);
    y_in_ += forward_in * std::cos(mid);
    heading_deg_ += cw_deg;
    forward_in_ += forward_in;

    // Encoders and the IMU move by the change, so a tare or set_rotation() the library made stays made.
    // A V5 motor counts 50 per turn of its motor shaft, which spins 3600 / wheel_rpm times per wheel turn.
    double ticks_per_in = 50.0 * 3600.0 / wheel_rpm_ / (M_PI * wheel_diameter_in_);
    add_ticks(drive_.left_motors, (forward_in + half_turn_in) * ticks_per_in, left_ticks_);
    add_ticks(drive_.right_motors, (forward_in - half_turn_in) * ticks_per_in, right_ticks_);
    drive_.imu->fake_rotation += cw_deg;
    write_motor_readings();
    for (auto& t : trackers_) turn_tracker(t, forward_in - cw_deg * M_PI / 180.0 * t.right_of_center_in);
    record_history();
  }

  void add_ticks(std::vector<pros::Motor>& side, double ticks, double& total) {
    auto& sensor = side.front().fake();
    if (sensor.position != (std::int32_t)std::lround(total)) total = sensor.position;  // the library tared it
    total += ticks;
    for (auto& motor : side) motor.fake().position = (std::int32_t)std::lround(total);
    if (&side == &drive_.left_motors && fail_read_at_ >= 0) sensor.disconnected = sim_ms_ == fail_read_at_;  // one PROS_ERR read
  }

  // A tracker counts what its wheel rolls, and keeps counting from wherever the library last reset it.
  static void turn_tracker(Tracker& t, double rolled_in) {
    bool rotation = t.wheel->smart_encoder.get_port() > 0;
    std::int32_t& count = rotation ? t.wheel->smart_encoder.fake_position : t.wheel->adi_encoder.fake_value;
    if (count != (std::int32_t)std::lround(t.ticks)) t.ticks = count;
    t.ticks += rolled_in * t.wheel->ticks_per_inch();
    count = (std::int32_t)std::lround(t.ticks);
  }

  void write_motor_readings() {
    const double track = body_.track_width_in * 0.0254;
    for (auto* side : {&drive_.left_motors, &drive_.right_motors}) {
      double v = side == &drive_.left_motors ? v_ + w_ * track / 2.0 : v_ - w_ * track / 2.0;
      double current_a = std::fmin(std::fabs(duty(*side) - v / free_speed_m_s()), 1.0) * 2.5;
      for (auto& motor : *side) {
        motor.fake().actual_velocity = v / (M_PI * wheel_diameter_in_ * 0.0254) * 60.0;
        motor.fake().current_draw = current_a * 1000.0;
        motor.fake().over_current = current_a >= 2.5;
      }
    }
  }

  void record_history() {
    history_.push_back({x_in_, y_in_, heading_deg_});
    if (history_.size() > 51) history_.pop_front();
    const auto& oldest = history_.front();
    if (std::hypot(oldest.x - x_in_, oldest.y - y_in_) >= 0.5 || std::fabs(oldest.heading - heading_deg_) >= 1.0) stopped_since_ = sim_ms_;
  }

  inline static Robot* active_ = nullptr;
  ez::Drive& drive_;
  Body body_;
  double wheel_rpm_, wheel_diameter_in_;
  int sim_ms_;
  double v_ = 0.0, w_ = 0.0;  // m/s forward, rad/s clockwise
  double x_in_ = 0.0, y_in_ = 0.0, heading_deg_ = 0.0, forward_in_ = 0.0;
  double left_ticks_ = 0.0, right_ticks_ = 0.0;
  double wall_at_in_ = 1e9;
  int fail_read_at_ = -1;
  int stopped_since_ = 0;
  int give_up_stopped_ms_ = 0, give_up_at_ms_ = 0;
  std::vector<Window> pins_, pushes_;
  std::vector<Action> actions_;
  std::deque<Sample> history_;
  std::vector<Tracker> trackers_;
};

}  // namespace sim
