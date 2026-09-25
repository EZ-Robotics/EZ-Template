// A from-scratch physics layer bolted onto the existing fake-hardware scaffold (see
// fake_hardware.hpp / stub/pros/motors.hpp / stub/pros/imu.hpp), built for the exit-conditions/
// pid-wait audit. There is no prior sim to extend here -- see SIM_FIDELITY.md's finding that the
// "existing harness" the audit task described could not be located in this repository.
//
// Scope and sourcing: every physical constant below is either cited to a source in
// SIM_FIDELITY.md (scratchpad, not in this repo) or is an explicit, commented simplification.
// Two things SIM_FIDELITY.md could not source are deliberately NOT modeled: the claimed
// "~2A cap after ~2s of sustained high current" and PWM dither ("issue 343") -- both are left
// out rather than guessed at. Wheel scrub/traction has no off-the-shelf reference model anywhere
// (not even WPILib's DifferentialDrivetrainSim covers it) but the audit's own hard constraints
// require modeling it, so SimArchetype::scrub_coefficient below is a first-principles
// approximation, explicitly flagged as an unvalidated placeholder pending real hardware
// measurement -- do not treat its numeric value as calibrated.
#pragma once

#include <array>
#include <cmath>
#include <random>

#include "EZ-Template/api.hpp"
#include "drive_test_access.hpp"
#include "fake_hardware.hpp"

namespace sim {

// A DC motor's torque-speed curve, linearized stall-to-free-speed (the simplification
// SIM_FIDELITY.md itself proposes -- real V5 motors aren't perfectly linear, but this is the
// standard first-pass approximation and matches what a torque-speed "curve" reduces to when
// only stall torque and free speed are known).
//
// stall_torque_nm is sourced for the green (200 rpm) cartridge specifically: ~2.1 N*m, per the
// VEX Forum torque-speed-curve thread cited in SIM_FIDELITY.md section 3. Red (100 rpm) and blue
// (600 rpm) cartridges are NOT independently sourced numbers -- they're derived here by assuming
// constant motor output power across cartridges (torque_speed_product is conserved, i.e.
// stall_torque scales as 1/free_speed_rpm relative to green). This is a reasonable first-order
// approximation for a planetary-geared motor family sharing the same core motor, not a sourced
// fact -- flagging so nobody mistakes the red/blue numbers for independently verified data.
struct MotorCurve {
  double free_speed_rpm;   // cartridge output shaft, unloaded
  double stall_torque_nm;  // cartridge output shaft, current at stall_current_a
  double stall_current_a;  // approximated as the 2.5A default current limit (sourced, Purdue
                            // SIGBots wiki, per SIM_FIDELITY.md) -- treating the limit as a stand-in
                            // for true stall current, not independently sourced as the same number.
  double free_current_a;   // small no-load current draw; not sourced, a conventional small value.

  // Signed torque at a given output-shaft angular velocity (rad/s, signed in the wheel's own
  // rotation frame) and commanded duty (-1..1, fraction of the 12V nominal supply -- see
  // step_physics()'s applied_v): the standard linear DC-motor torque-speed line, extended
  // through the origin into all four quadrants instead of clamped to the forward-motoring one.
  // tau = tau_stall * (duty - omega/omega_free), clamped to +/-tau_stall.
  //
  // For 0 <= omega <= duty*omega_free (the only region the sim's very first version -- see this
  // struct's git history -- ever covered) this is algebraically identical to that version's
  // duty*(1-frac) form: tau_stall*duty*(1-omega/(duty*omega_free)) reduces to exactly
  // tau_stall*(duty-omega/omega_free). So the forward-motoring case (a motor accelerating a
  // wheel toward its commanded speed, including "low PID output near a target must produce low
  // torque even at rest" -- still true here at omega=0) is unchanged.
  //
  // What's different is everywhere that old formula clamped its "frac" ratio to 1 and returned
  // exactly 0: a wheel spinning faster than the commanded duty's implied speed (a PID easing off
  // while the chassis still coasts from momentum), or spinning opposite the commanded duty (a
  // PID reversing to brake), now gets genuine negative (regenerative/braking) torque instead of
  // silently freewheeling. Found auditing a false "Turn: Stuck" that turned out to be present
  // even in a from-scratch, zero-injected-fault control run: with no braking authority once
  // torque hit that clamp, a light, fast, low-friction archetype (archetype_light_fast) spinning
  // up during a saturated turn had nothing to slow it back down until the PID commanded a hard
  // full reversal, so it coasted well past where it should have started braking -- overshooting
  // the target and swinging back, or drifting to a stop short of it, depending on exact timing --
  // and got falsely flagged stuck either way. A genuine sim-physics gap, not evidence of anything
  // about sensor faults or real turn/swing behavior.
  double torque_at(double angular_velocity_rad_s, double duty) const {
    double free_speed_rad_s = free_speed_rpm * 2.0 * M_PI / 60.0;
    if (free_speed_rad_s <= 0.0) return 0.0;
    double duty_c = ez::util::clamp(duty, 1.0, -1.0);
    double torque = stall_torque_nm * (duty_c - angular_velocity_rad_s / free_speed_rad_s);
    return ez::util::clamp(torque, stall_torque_nm, -stall_torque_nm);
  }
};

// green (200rpm) is the sourced baseline; red/blue derived, see MotorCurve's own comment.
inline MotorCurve motor_curve_for_cartridge(double cartridge_rpm) {
  constexpr double kGreenRpm = 200.0;
  constexpr double kGreenStallNm = 2.1;
  double stall = kGreenStallNm * (kGreenRpm / cartridge_rpm);
  return MotorCurve{cartridge_rpm, stall, /*stall_current_a=*/2.5, /*free_current_a=*/0.1};
}

// Firmware current-limit throttling. SIM_FIDELITY.md found two reported temperature-throttle
// tier systems in the same source (VEX Forum thread on current limit vs temperature) and could
// not confirm whether they're the same mechanism described two ways or genuinely distinct.
// This models the *temperature-threshold* version (halved at 55C, quartered at 60C, 1/8 at 65C,
// disabled at 70C) rather than the *numbered-level* version, because it composes more naturally
// with a simple thermal integrator (below) than an unspecified level-transition rule would.
// This is a documented choice, not a claim that the other system is wrong.
inline double current_scale_for_temperature(double temp_c) {
  if (temp_c >= 70.0) return 0.0;
  if (temp_c >= 65.0) return 1.0 / 8.0;
  if (temp_c >= 60.0) return 1.0 / 4.0;
  if (temp_c >= 55.0) return 1.0 / 2.0;
  return 1.0;
}

// A robot archetype: everything the sim needs to turn "commanded voltage per motor" into
// "encoder ticks, actual velocity, current draw, IMU heading" over time. Three required by the
// audit's hard constraints: light/fast, heavy/slow, sticky/high-friction.
struct SimArchetype {
  const char* name;
  int motors_per_side;
  double cartridge_rpm;       // 100 (red), 200 (green), or 600 (blue)
  double wheel_diameter_in;
  double track_width_in;      // center-to-center distance between left and right wheel contact
  double mass_kg;
  double moment_of_inertia_kg_m2;  // about the vertical (yaw) axis
  double rolling_resistance_nm;    // constant opposing torque per side, referred to the wheel
  // Turn-scrub knob (UNVALIDATED PLACEHOLDER, see file header): extra resistive torque per side,
  // proportional to this coefficient times track width times commanded yaw rate. Higher =
  // harder to pivot, more current drawn while turning, models traction wheels / high-friction
  // tile better than a plain omni-like drive. Not measured against a real robot.
  double scrub_coefficient;
  bool has_tracking_wheels;   // if true, drive_sensor_left/right read a tracking wheel (never
                               // slips with the drive base) rather than the drive motors directly
  double encoder_noise_stddev_in;      // Gaussian, applied to reported position (inches)
  double velocity_noise_stddev_in_s;   // Gaussian, applied to reported actual_velocity
  double imu_noise_stddev_deg;         // Gaussian, applied to reported IMU heading
};

// moment_of_inertia_kg_m2 for all three: modeled as a uniform 18in x 18in square plate (VEX's
// starting-size footprint, independent of track_width_in which is just wheel spacing and is
// usually narrower than the full chassis) rotating about its center -- I = (1/6) * mass * side^2,
// side = 18in = 0.4572m, side^2 ~= 0.20903 m^2. This replaced values that turned out to be inert
// (see step_physics()'s header comment) rather than calibrated against anything, so these are
// still a simplification, not sourced hardware data -- a real chassis with mechanisms extending
// away from center would have a higher figure than this plate estimate.
inline SimArchetype archetype_light_fast() {
  return SimArchetype{
      "light_fast", 4, 600.0 /*blue*/, 3.25, 12.0, 3.5, 0.122,
      /*rolling_resistance_nm=*/0.02, /*scrub_coefficient=*/0.15,
      /*has_tracking_wheels=*/false,
      /*encoder_noise_stddev_in=*/0.01, /*velocity_noise_stddev_in_s=*/0.05, /*imu_noise_stddev_deg=*/0.05};
}
inline SimArchetype archetype_heavy_slow() {
  return SimArchetype{
      "heavy_slow", 6, 100.0 /*red*/, 4.125, 15.0, 9.0, 0.314,
      /*rolling_resistance_nm=*/0.08, /*scrub_coefficient=*/0.35,
      /*has_tracking_wheels=*/true,
      /*encoder_noise_stddev_in=*/0.015, /*velocity_noise_stddev_in_s=*/0.08, /*imu_noise_stddev_deg=*/0.08};
}
inline SimArchetype archetype_sticky_high_friction() {
  return SimArchetype{
      "sticky_high_friction", 6, 200.0 /*green*/, 4.0, 13.5, 6.0, 0.209,
      /*rolling_resistance_nm=*/0.18, /*scrub_coefficient=*/0.65,
      /*has_tracking_wheels=*/false,
      /*encoder_noise_stddev_in=*/0.01, /*velocity_noise_stddev_in_s=*/0.06, /*imu_noise_stddev_deg=*/0.06};
}

struct NoiseConfig {
  bool enabled = true;
  std::uint32_t seed = 12345;
};

// One side (left or right) of the simulated drive: true (noise-free) state plus a running
// thermal estimate. Position/velocity are wheel-linear (inches / inches-per-second) -- converted
// to encoder ticks only when written into the fake motor registry, via the real Drive's own
// drive_tick_per_inch(), so this sim never has to re-derive PROS's tick-per-degree conventions.
struct SideState {
  double position_in = 0.0;
  double velocity_in_s = 0.0;
  double temp_c = 25.0;  // ambient start
};

// Heating/cooling is a simple first-order integrator (heats with I^2, cools toward ambient),
// NOT a sourced thermal model -- SIM_FIDELITY.md didn't find V5-specific thermal time-constant
// data. Tuned only so that sustained near-stall current visibly climbs through the tiers within
// a few seconds and idle current cools back down within tens of seconds -- a plausibility target,
// not a calibrated one.
inline void step_temperature(double& temp_c, double current_a, double dt_s) {
  constexpr double kAmbient = 25.0;
  constexpr double kHeatGain = 6.0;    // deg C per (A^2 * s), unvalidated
  constexpr double kCoolRate = 0.05;   // 1/s toward ambient, unvalidated
  temp_c += current_a * current_a * kHeatGain * dt_s;
  temp_c += (kAmbient - temp_c) * kCoolRate * dt_s;
}

// The whole simulated robot: two SideStates, an archetype, current heading, and the noise RNG.
// One instance is installed as the active sim via install(); ~SimRobot() uninstalls it. Owns the
// ordering decision documented at length below.
class SimRobot {
 public:
  SimRobot(ez::Drive& drive, SimArchetype archetype, NoiseConfig noise = {})
      : drive_(drive), archetype_(archetype), noise_(noise), rng_(noise.seed) {
    install(this);
  }
  ~SimRobot() {
    if (active_ == this) active_ = nullptr;
  }

  SimRobot(const SimRobot&) = delete;
  SimRobot& operator=(const SimRobot&) = delete;

  double heading_deg() const { return heading_deg_; }
  const SideState& left() const { return left_; }
  const SideState& right() const { return right_; }

 private:
  static void install(SimRobot* r) {
    active_ = r;
    test_stub::g_clock.on_delay = &SimRobot::on_delay_trampoline;
  }
  static void on_delay_trampoline() {
    if (active_ != nullptr) active_->tick();
  }
  inline static SimRobot* active_ = nullptr;

  // --- Ordering decision (read before changing anything below) ---
  //
  // Real hardware: a persistent auto task computes PID output from the LAST sensor reading and
  // commands motor voltage; the physical robot then moves under that voltage for one tick; the
  // NEXT auto task pass reads the sensors that movement produced. There is no single "instant"
  // where a test can observe motion and control in perfect lockstep -- control always lags
  // physics by one tick.
  //
  // This sim reproduces that lag deliberately rather than trying to make everything atomic. Each
  // call to tick() (invoked from pros::delay(), i.e. once per DELAY_TIME the wait loop or a
  // manual test loop asks for):
  //   1. Runs one auto-task-equivalent pass FIRST (check_imu_task, ez_tracking_task, then the
  //      current mode's per-mode task function) against whatever sensor state already exists.
  //      This computes a fresh PID output and writes fresh motor voltage.
  //   2. THEN advances physics by one DELAY_TIME using that just-written voltage, producing the
  //      sensor state the NEXT tick's pass will read.
  //
  // Doing (1) before (2) on the very first tick matters: pid_wait()'s own first line is
  // `pros::delay(DELAY_TIME)` specifically so "the PID runs at least 1 iteration" before the
  // exit-condition loop starts checking (see exit_conditions.cpp). With this ordering, that
  // first delay call performs exactly one real PID computation before the loop's first
  // exit_condition() check -- matching what the comment in the real code says should happen.
  // Doing physics-then-task instead would mean the first exit_condition() check runs at t=0
  // against a target/output pair that was never used to move anything, which doesn't match that
  // comment's intent.
  //
  // No recursion risk: this function calls ez_tracking_task/mode-task-functions (which do not
  // themselves call pros::delay()) and then a plain physics update (also no pros::delay()) -- it
  // never calls ez_auto_task() itself, which is the one function that both contains this logic
  // AND ends with its own pros::delay(), which would re-enter on_delay from inside on_delay.
  void tick() {
    run_auto_task_pass();
    step_physics(ez::util::DELAY_TIME / 1000.0);
  }

  void run_auto_task_pass() {
    // Mirrors ez_auto_task()'s own bookkeeping (pid_tasks.cpp:22): the real task increments this
    // every pass, and StuckWatch/SingleStuckWatch's stuck() (exit_conditions.cpp) cross-checks the
    // wall-clock window against an EXPECTED pass count derived from it, to tell a task genuinely
    // starved of time from one that's simply dead. This sim calls the task bodies directly instead
    // of running the real ez_auto_task(), so without this line the counter never moves, the pass
    // check can never trip, and every sim-backed stuck detection silently falls back to the
    // wall-clock-only STUCK_STARVED_WINDOWS path (4x the configured window) instead of firing at the
    // window it's actually configured for. Found auditing a stuck-detection timing test against this
    // harness -- fixes the harness to match what it's simulating, not a change to the library.
    ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
    ez::DriveTestAccess::check_imu_task(drive_);
    drive_.ez_tracking_task();  // public -- no DriveTestAccess wrapper needed
    switch (drive_.drive_mode_get()) {
      case ez::DRIVE:
        ez::DriveTestAccess::drive_pid_task(drive_);
        break;
      case ez::TURN:
      case ez::TURN_TO_POINT:
        ez::DriveTestAccess::turn_pid_task(drive_);
        break;
      case ez::SWING:
        ez::DriveTestAccess::swing_pid_task(drive_);
        break;
      case ez::POINT_TO_POINT:
        ez::DriveTestAccess::ptp_task(drive_);
        break;
      case ez::PURE_PURSUIT:
        ez::DriveTestAccess::pp_task(drive_);
        break;
      case ez::DISABLE:
      default:
        break;
    }
  }

  // --- Why this function looks the way it does (read before changing it) ---
  //
  // An earlier version integrated each wheel's velocity independently, using the SAME
  // translational effective mass (archetype_.mass_kg / 2) whether that wheel's acceleration was
  // contributing to the robot driving straight or to it turning, and derived heading purely
  // kinematically from the resulting velocity difference every tick. moment_of_inertia_kg_m2 was
  // declared on SimArchetype and set per archetype, but never read anywhere in this file --
  // nothing played the role a chassis's actual rotational inertia plays in resisting how fast its
  // spin rate can change. Against the library's real, hardware-tuned angle-PID gains
  // (pid_odom_angular_constants_set(6.5, 0, 52.5) etc., drive.cpp), that let simulated heading
  // swing by tens of degrees in a single 10ms tick, which those gains are nowhere near tuned to
  // handle -- full-power sign flips every tick instead of settling (see STEP2_REPRO_RESULTS.md
  // section 2 for the trace that caught this).
  //
  // For this archetype's own numbers, the rotational effective mass implied by a proper moment
  // of inertia (I / track_radius^2) is roughly 3x the translational per-side mass the old model
  // was using for BOTH roles -- i.e. the old model let the chassis turn as if it were about a
  // third as rotationally resistant as it should be.
  //
  // Fixed by tracking two robot-level degrees of freedom instead of two independent wheel
  // velocities: common_velocity_in_s_ (translation, resisted by the full archetype mass) and
  // yaw_rate_deg_s_ (rotation, resisted by moment_of_inertia_kg_m2, now genuinely integrated from
  // an angular acceleration instead of derived instantaneously from wheel speeds). Each wheel's
  // own velocity -- needed for its own torque-speed curve and for what its encoder reports -- is
  // derived from those two via the standard rigid, no-slip differential-drive relation:
  // v_wheel = v_common +/- yaw_rate * track_radius. This is the standard decomposition for a
  // rigid two-wheel-driven chassis, not a novel model.
  void step_physics(double dt_s) {
    if (dt_s <= 0.0) return;
    MotorCurve curve = motor_curve_for_cartridge(archetype_.cartridge_rpm);
    double wheel_radius_in = archetype_.wheel_diameter_in / 2.0;
    double wheel_radius_m = wheel_radius_in * 0.0254;
    double track_radius_m = (archetype_.track_width_in / 2.0) * 0.0254;

    // Per-side commanded voltage: EZ-Template's private_drive_set() (drive.cpp) actually calls
    // move_voltage(power * (12000.0 / 127.0)) -- i.e. fake().voltage is real millivolts (-12000
    // to 12000), not the library's own internal -127..127 power units. Verified by reading
    // private_drive_set() directly rather than assuming -- confirming a wrong first guess made
    // while writing this file, kept here as the reason the /12000.0 conversion below exists.
    double left_v = first_motor_voltage(drive_.left_motors);
    double right_v = first_motor_voltage(drive_.right_motors);

    double common_v_m_s = common_velocity_in_s_ * 0.0254;
    double yaw_rate_rad_s = yaw_rate_deg_s_ * M_PI / 180.0;
    double left_wheel_v_m_s = common_v_m_s - yaw_rate_rad_s * track_radius_m;
    double right_wheel_v_m_s = common_v_m_s + yaw_rate_rad_s * track_radius_m;

    // Torque/resistance for one side at its own (derived) wheel speed -- same motor-curve +
    // rolling-resistance + scrub-placeholder model as before, but now returns a ground-contact
    // force instead of directly integrating that side's own motion.
    auto side_force = [&](double commanded_mv, double this_wheel_v_m_s, double other_wheel_v_m_s) {
      double supply_mv = 12000.0;  // no battery sag model, per SIM_FIDELITY.md's recommendation
      double duty = ez::util::clamp(commanded_mv, supply_mv, -supply_mv) / supply_mv;  // signed, -1..1

      double wheel_angular_v = this_wheel_v_m_s / wheel_radius_m;  // rad/s at the wheel
      // Back-EMF (including braking/regen in all four quadrants) is implicit in torque_at()
      // itself now, already signed -- see its own comment. No copysign(duty) here any more: that
      // used to be needed because the old torque_at() only ever returned a magnitude; doing it
      // again here would double up the sign torque_at() now already applies.
      double torque_available = curve.torque_at(wheel_angular_v, duty);

      // Turn-scrub resistive torque (placeholder model, see file header): opposes whichever side
      // is moving faster than the other, scaled by how much the two sides disagree (a proxy for
      // commanded yaw rate) -- a differential-speed robot pays a scrub penalty on both sides
      // while turning, worse for the archetypes with a higher scrub_coefficient.
      double diff_in_s = (this_wheel_v_m_s - other_wheel_v_m_s) / 0.0254;
      double scrub_torque = archetype_.scrub_coefficient * (archetype_.track_width_in * 0.0254) * std::fabs(diff_in_s) * 0.1;
      double resistive = archetype_.rolling_resistance_nm + scrub_torque;
      double net_torque = torque_available - std::copysign(std::fmin(std::fabs(torque_available), resistive), torque_available == 0 ? 1.0 : torque_available);

      double net_force_n = (net_torque * archetype_.motors_per_side) / wheel_radius_m;
      double current_a = std::fabs(torque_available) / curve.stall_torque_nm * curve.stall_current_a + curve.free_current_a;
      return std::pair<double, double>{net_force_n, current_a};
    };

    auto [left_force_n, left_current] = side_force(left_v, left_wheel_v_m_s, right_wheel_v_m_s);
    auto [right_force_n, right_current] = side_force(right_v, right_wheel_v_m_s, left_wheel_v_m_s);

    // Translation: resisted by the whole robot's mass. Rotation: resisted by
    // moment_of_inertia_kg_m2 about the yaw axis -- the piece that was missing entirely before.
    double common_accel_m_s2 = (left_force_n + right_force_n) / archetype_.mass_kg;
    double yaw_torque_nm = (right_force_n - left_force_n) * track_radius_m;
    double yaw_accel_rad_s2 = yaw_torque_nm / archetype_.moment_of_inertia_kg_m2;

    common_v_m_s += common_accel_m_s2 * dt_s;
    yaw_rate_rad_s += yaw_accel_rad_s2 * dt_s;
    heading_deg_ += (yaw_rate_rad_s * 180.0 / M_PI) * dt_s;

    common_velocity_in_s_ = common_v_m_s / 0.0254;
    yaw_rate_deg_s_ = yaw_rate_rad_s * 180.0 / M_PI;

    // Recompute each wheel's velocity from the just-updated common/yaw state -- for encoder
    // reporting and so next tick's torque calc uses post-update speed, same no-slip relation.
    left_.velocity_in_s = (common_v_m_s - yaw_rate_rad_s * track_radius_m) / 0.0254;
    right_.velocity_in_s = (common_v_m_s + yaw_rate_rad_s * track_radius_m) / 0.0254;
    left_.position_in += left_.velocity_in_s * dt_s;
    right_.position_in += right_.velocity_in_s * dt_s;

    left_current *= current_scale_for_temperature(left_.temp_c);
    right_current *= current_scale_for_temperature(right_.temp_c);
    step_temperature(left_.temp_c, left_current, dt_s);
    step_temperature(right_.temp_c, right_current, dt_s);

    write_back(left_current, right_current);
  }

  static double first_motor_voltage(std::vector<pros::Motor>& motors) {
    if (motors.empty()) return 0.0;
    return motors.front().fake().voltage;
  }

  double gaussian(double stddev) {
    if (!noise_.enabled || stddev <= 0.0) return 0.0;
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(rng_);
  }

  void write_back(double left_current_a, double right_current_a) {
    double tick_per_inch = drive_.drive_tick_per_inch();

    auto write_side = [&](std::vector<pros::Motor>& motors, SideState& side, double current_a) {
      double reported_pos_in = side.position_in + gaussian(archetype_.encoder_noise_stddev_in);
      double reported_vel_in_s = side.velocity_in_s + gaussian(archetype_.velocity_noise_stddev_in_s);
      for (auto& m : motors) {
        auto& fake = m.fake();
        fake.position = reported_pos_in * tick_per_inch;
        fake.actual_velocity = reported_vel_in_s * tick_per_inch;
        fake.current_draw = current_a * 1000.0;  // PROS reports current in mA
        // 2.5A: the default current limit (sourced, Purdue SIGBots wiki, per SIM_FIDELITY.md),
        // which MotorCurve::stall_current_a is deliberately set equal to (see its own comment).
        fake.over_current = current_a >= 2.5;
      }
    };
    write_side(drive_.left_motors, left_, left_current_a);
    write_side(drive_.right_motors, right_, right_current_a);

    // Tracking-wheel archetypes: a dedicated tracker never slips with the drive base and isn't
    // modeled separately here (its reading would be identical to the noise-free position in this
    // sim, since neither side ever actually slips against the ground in this model) -- flagging
    // as a known simplification, not a claim that tracking wheels behave identically to motor
    // encoders in general (they don't on real hardware, e.g. under wheel slip, which this sim
    // does not model as a distinct phenomenon from the scrub-torque approximation above).

    auto& imus = ez::DriveTestAccess::all_imus(drive_);
    if (imus.size() > 0) {
      // heading_deg_ is integrated from yaw_rate_deg_s_, itself derived from (right_force - left_force)
      // -- positive when the right side pushes harder, which is a COUNTERclockwise rotation as this
      // sim's forces are signed. The real V5 IMU (and this library's own turn_pid_task, which drives
      // left=+gyro_out/right=-gyro_out for a positive/increasing-heading error, i.e. commands a
      // CLOCKWISE turn to INCREASE heading) is clockwise-positive. Reporting heading_deg_ unchanged
      // hands the turn/swing PIDs a mirrored sensor: a correction in the commanded direction reads
      // back as moving the wrong way, which is unconditional positive feedback, not merely "an
      // inaccurate sim" -- caught by a turn/swing config-fuzz test whose heading diverged into the
      // hundreds of degrees within ~1.5s regardless of the commanded target. Straight-line odom tests
      // are unaffected: heading stays near 0 there regardless of sign convention. Negating here, not
      // the yaw formula itself, keeps step_physics()'s internal force/torque bookkeeping consistent
      // and only fixes what's handed to the one consumer that has an external sign convention to match.
      double reported_heading = -heading_deg_ + gaussian(archetype_.imu_noise_stddev_deg);
      imus[0]->fake_rotation = reported_heading;
    }
  }

  ez::Drive& drive_;
  SimArchetype archetype_;
  NoiseConfig noise_;
  std::mt19937 rng_;
  SideState left_, right_;
  double heading_deg_ = 0.0;
  // Robot-level translational/rotational state -- see step_physics()'s header comment for why
  // these exist instead of deriving everything from left_/right_.velocity_in_s directly.
  double common_velocity_in_s_ = 0.0;
  double yaw_rate_deg_s_ = 0.0;
};

}  // namespace sim
