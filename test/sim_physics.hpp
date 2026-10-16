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
#include <functional>
#include <random>
#include <vector>

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
  // torque hit that clamp, a chassis spinning up during a saturated turn had nothing to slow it
  // back down until the PID commanded a hard full reversal, so it coasted well past where it
  // should have started braking -- overshooting the target and swinging back, or running away
  // and never recovering, depending on the archetype -- and got falsely flagged stuck either way.
  // This broke all three sim archetypes at the library's shipped default turn constants (see
  // test_turn_control_no_fault.cpp's own header comment for the measured pre-fix numbers), not
  // just the lightest one -- a genuine sim-physics gap, not evidence of anything about sensor
  // faults or real turn/swing behavior, and not specific to any one archetype's numbers being
  // unrealistic. See side_force()'s own comment for a related, independently demonstrated
  // friction-sign gap fixed alongside this one (not required for this scenario by itself, but
  // closing the same class of "nothing opposes a coasting wheel" problem more completely).
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
  double cartridge_rpm;  // 100 (red), 200 (green), or 600 (blue)
  double wheel_diameter_in;
  double track_width_in;  // center-to-center distance between left and right wheel contact
  double mass_kg;
  double moment_of_inertia_kg_m2;  // about the vertical (yaw) axis
  double rolling_resistance_nm;    // constant opposing torque per side, referred to the wheel
  // Turn-scrub knob (UNVALIDATED PLACEHOLDER, see file header): extra resistive torque per side,
  // proportional to this coefficient times track width times commanded yaw rate. Higher =
  // harder to pivot, more current drawn while turning, models traction wheels / high-friction
  // tile better than a plain omni-like drive. Not measured against a real robot.
  double scrub_coefficient;
  bool has_tracking_wheels;           // if true, drive_sensor_left/right read a tracking wheel (never
                                      // slips with the drive base) rather than the drive motors directly
  double encoder_noise_stddev_in;     // Gaussian, applied to reported position (inches)
  double velocity_noise_stddev_in_s;  // Gaussian, applied to reported actual_velocity
  double imu_noise_stddev_deg;        // Gaussian, applied to reported IMU heading
};

// moment_of_inertia_kg_m2 for all three: modeled as a uniform 18in x 18in square plate (VEX's
// starting-size footprint, independent of track_width_in which is just wheel spacing and is
// usually narrower than the full chassis) rotating about its center -- I = (1/6) * mass * side^2,
// side = 18in = 0.4572m, side^2 ~= 0.20903 m^2. This replaced values that turned out to be inert
// (see step_physics()'s header comment) rather than calibrated against anything, so these are
// still a simplification, not sourced hardware data -- a real chassis with mechanisms extending
// away from center would have a higher figure than this plate estimate.
inline SimArchetype archetype_light_fast() {
  return SimArchetype{"light_fast",
                      4,
                      600.0 /*blue*/,
                      3.25,
                      12.0,
                      3.5,
                      0.122,
                      /*rolling_resistance_nm=*/0.02,
                      /*scrub_coefficient=*/0.15,
                      /*has_tracking_wheels=*/false,
                      /*encoder_noise_stddev_in=*/0.01,
                      /*velocity_noise_stddev_in_s=*/0.05,
                      /*imu_noise_stddev_deg=*/0.05};
}
inline SimArchetype archetype_heavy_slow() {
  return SimArchetype{"heavy_slow",
                      6,
                      100.0 /*red*/,
                      4.125,
                      15.0,
                      9.0,
                      0.314,
                      /*rolling_resistance_nm=*/0.08,
                      /*scrub_coefficient=*/0.35,
                      /*has_tracking_wheels=*/true,
                      /*encoder_noise_stddev_in=*/0.015,
                      /*velocity_noise_stddev_in_s=*/0.08,
                      /*imu_noise_stddev_deg=*/0.08};
}
inline SimArchetype archetype_sticky_high_friction() {
  return SimArchetype{"sticky_high_friction",
                      6,
                      200.0 /*green*/,
                      4.0,
                      13.5,
                      6.0,
                      0.209,
                      /*rolling_resistance_nm=*/0.18,
                      /*scrub_coefficient=*/0.65,
                      /*has_tracking_wheels=*/false,
                      /*encoder_noise_stddev_in=*/0.01,
                      /*velocity_noise_stddev_in_s=*/0.06,
                      /*imu_noise_stddev_deg=*/0.06};
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
  constexpr double kHeatGain = 6.0;   // deg C per (A^2 * s), unvalidated
  constexpr double kCoolRate = 0.05;  // 1/s toward ambient, unvalidated
  temp_c += current_a * current_a * kHeatGain * dt_s;
  temp_c += (kAmbient - temp_c) * kCoolRate * dt_s;
}

// The whole simulated robot: two SideStates, an archetype, current heading, and the noise RNG.
// One instance is installed as the active sim via install(); ~SimRobot() uninstalls it. Owns the
// ordering decision documented at length below.
class SimRobot {
public:
  SimRobot(ez::Drive& drive, SimArchetype archetype, NoiseConfig noise = {}) : drive_(drive), archetype_(archetype), noise_(noise), rng_(noise.seed) {
    // Whatever the fake IMU holds right now is the sim's own baseline, replaced by the physical heading on
    // the first tick as it always was. Only a write made after this point is adopted as an offset (see
    // write_back()'s IMU comment).
    auto& imus = ez::DriveTestAccess::all_imus(drive_);
    if (imus.size() > 0) {
      imu_last_written_ = imus[0]->fake_rotation;
      imu_written_ = true;
    }
    install(this);
  }
  ~SimRobot() {
    if (active_ == this) active_ = nullptr;
  }

  SimRobot(const SimRobot&) = delete;
  SimRobot& operator=(const SimRobot&) = delete;

  // Off by default: the sim runs the per-mode task bodies itself (run_auto_task_pass()). On: it runs the real
  // ez_auto_task() once per tick instead.
  void use_real_auto_task(bool on) { use_real_auto_task_ = on; }
  // How many auto task passes run per tick (default 1). Two or three is a task that catches up after being late, and a
  // heavy robot is only stable in the sim with more than one.
  void passes_per_tick(int n) { passes_per_tick_ = n; }
  // Run on every auto task pass, just before and just after it, with the pass's number (counting from 0 across the
  // sim's life). Before is where a test relocalizes or shoves the robot the way another thread would between passes;
  // after is where it reads what the pass computed. Both are empty by default.
  std::function<void(int)> before_pass;
  std::function<void(int)> after_pass;
  // Moves both wheels `inches` along the robot's heading at once, without giving the robot any velocity: what being
  // pushed a short way and let go looks like to the sensors. The motors read the new position right away, not only
  // after the next tick.
  void displace(double inches) {
    left_.position_in += inches;
    right_.position_in += inches;
    double tick_per_inch = encoder_tick_per_inch();
    for (auto* side : {&drive_.left_motors, &drive_.right_motors}) {
      bool is_left = side == &drive_.left_motors;
      double pos = (is_left ? left_.position_in : right_.position_in) * tick_per_inch + (is_left ? enc_off_left_ : enc_off_right_);
      for (auto& m : *side)
        if (!drive_.pto_check(m)) m.fake().position = pos;
    }
  }

  // A transmission shifts: the wheel's real speed becomes `new_wheel_rpm` (the same number drive_rpm_set() takes), so the
  // motors' free speed and torque change with it and each encoder tick now stands for a different distance. The encoders
  // keep counting from where they were: the raw count does not jump, only how many ticks a wheel inch is worth does. A
  // test that wants the library told calls drive_rpm_set() itself, at the same instant, AFTER this: the sim
  // reads what the Drive believes right now to work out the physical scale it is leaving, so telling the library first makes
  // it take the new number for the old one.
  void shift_gearing(double new_wheel_rpm) {
    double old_tpi = encoder_tick_per_inch();
    double new_tpi = old_tpi * archetype_.cartridge_rpm / new_wheel_rpm;
    enc_off_left_ += left_.position_in * (old_tpi - new_tpi);
    enc_off_right_ += right_.position_in * (old_tpi - new_tpi);
    phys_tpi_ = new_tpi;
    archetype_.cartridge_rpm = new_wheel_rpm;
  }

  // Zeroes both wheels' encoder readings without moving the robot: what drive_sensor_reset()'s tare_position() does on a real
  // motor. The sim keeps its own wheel positions and writes them to the motors every tick, so a tare made only on the motor
  // is undone one tick later; a test that resets sensors mid motion calls this right after.
  void tare_encoders() {
    left_.position_in = 0.0;
    right_.position_in = 0.0;
    enc_off_left_ = 0.0;
    enc_off_right_ = 0.0;
    for (auto* side : {&drive_.left_motors, &drive_.right_motors})
      for (auto& m : *side)
        if (!drive_.pto_check(m)) m.fake().position = 0.0;
  }

  // --- Physical interference, all off by default. Times are sim milliseconds (now_ms(), one DELAY_TIME per tick). ---

  double now_ms() const { return sim_ms_; }
  // An external force along the robot's heading, in newtons (positive pushes it forward), from `start_ms` for
  // `duration_ms`. Windows add up; a robot being shoved by another robot is a window of 40 to 150 N for 100 to 600 ms.
  void push(double newtons, double start_ms, double duration_ms) { forces_.push_back({newtons, start_ms, start_ms + duration_ms}); }
  // The robot is held still, in every direction, from `start_ms` for `duration_ms` (a robot pinned against a defender).
  void pin(double start_ms, double duration_ms) { pins_.push_back({0.0, start_ms, start_ms + duration_ms}); }
  // The robot is held (pinned) and carried at v in/s along its heading and w deg/s (library sign, clockwise positive) for the
  // window: wheels, heading and encoders all move, nothing the motors do changes it. A robot dragged along by something else.
  void carry(double v_in_s, double w_deg_s, double start_ms, double duration_ms) {
    pins_.push_back({0.0, start_ms, start_ms + duration_ms});
    carries_.push_back({v_in_s, w_deg_s, start_ms, start_ms + duration_ms});
  }
  // A wall `position_in` inches ahead of where the wheels are zero: the average wheel travel cannot pass it, and
  // whatever velocity it hits it at is lost. The motors keep pushing, so they stall against it.
  void wall(double position_in) {
    wall_in_ = position_in;
    wall_set_ = true;
  }
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
    for (int i = 0; i < passes_per_tick_; i++) {
      if (before_pass) before_pass(pass_count_);
      if (use_real_auto_task_)
        run_real_auto_task_pass();
      else
        run_auto_task_pass();
      if (after_pass) after_pass(pass_count_);
      pass_count_++;
    }
    step_physics(ez::util::DELAY_TIME / 1000.0);
  }

  // One pass of the real ez_auto_task(), for tests that need what run_auto_task_pass() leaves out: the
  // competition-status handling and everything else that function does around the per-mode task bodies.
  // ez_auto_task() ends in its own pros::delay(), which would re-enter this hook and advance the fake clock a
  // second time, so the hook is parked and the loop is unwound with StopLoop, and the extra DELAY_TIME the
  // inner delay added is taken back out.
  void run_real_auto_task_pass() {
    auto saved_hook = test_stub::g_clock.on_delay;
    int saved_stop = test_stub::g_clock.delay_calls_until_stop;
    test_stub::g_clock.on_delay = nullptr;
    test_stub::g_clock.delay_calls_until_stop = 0;
    try {
      ez::DriveTestAccess::ez_auto_task(drive_);
    } catch (test_stub::StopLoop&) {}
    test_stub::g_clock.now_ms -= ez::util::DELAY_TIME;
    test_stub::g_clock.on_delay = saved_hook;
    test_stub::g_clock.delay_calls_until_stop = saved_stop;
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
  // side_force()'s return, incl. what its own zero-crossing guard (in step_physics(), below)
  // needs to tell a friction-induced crossing apart from a genuine commanded-reversal one.
  struct SideForceResult {
    double net_force_n;
    double current_a;
    double duty;       // signed commanded duty, -1..1 (NOT torque_available -- see the guard's own comment)
    bool was_kinetic;  // true if this side used the moving-wheel friction branch this tick
  };

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
      double supply_mv = 12000.0;                                                      // no battery sag model, per SIM_FIDELITY.md's recommendation
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
      // Friction has to oppose the wheel's actual motion, not torque_available's sign -- the two
      // are the same thing only while a motor is still accelerating toward its commanded speed.
      // The old code subtracted resistive opposing torque_available's sign, which (now that
      // torque_at() can itself go negative to brake/coast) meant friction dropped toward 0 right
      // when torque_available did, instead of continuing to fight the wheel's own momentum --
      // exactly the gap that let archetype_light_fast coast through a turn's overshoot instead of
      // slowing down. Below kStaticVelEpsilonMS the wheel is treated as at rest: friction can
      // only oppose whatever torque IS being applied, up to its own magnitude (the sim's original
      // clamp), so it can never spin a resting wheel backward on its own -- needed by
      // rolling_resistance_nm-dwarfs-everything "never moves" scenarios (see
      // test_sim_harness_pass_counter_and_imu_sign.cpp).
      constexpr double kStaticVelEpsilonMS = 1.0e-4;  // true (noise-free) state; exact rest stays exact
      double net_torque;
      if (std::fabs(this_wheel_v_m_s) > kStaticVelEpsilonMS) {
        net_torque = torque_available - std::copysign(resistive, this_wheel_v_m_s);
      } else {
        net_torque = torque_available - std::copysign(std::fmin(std::fabs(torque_available), resistive), torque_available == 0.0 ? 1.0 : torque_available);
      }

      double net_force_n = (net_torque * archetype_.motors_per_side) / wheel_radius_m;
      double current_a = std::fabs(torque_available) / curve.stall_torque_nm * curve.stall_current_a + curve.free_current_a;
      bool was_kinetic = std::fabs(this_wheel_v_m_s) > kStaticVelEpsilonMS;
      return SideForceResult{net_force_n, current_a, duty, was_kinetic};
    };

    SideForceResult left_r = side_force(left_v, left_wheel_v_m_s, right_wheel_v_m_s);
    SideForceResult right_r = side_force(right_v, right_wheel_v_m_s, left_wheel_v_m_s);

    // Translation: resisted by the whole robot's mass. Rotation: resisted by
    // moment_of_inertia_kg_m2 about the yaw axis -- the piece that was missing entirely before.
    double external_n = 0.0;
    for (const auto& f : forces_)
      if (sim_ms_ >= f.start_ms && sim_ms_ < f.end_ms) external_n += f.value;
    double common_accel_m_s2 = (left_r.net_force_n + right_r.net_force_n + external_n) / archetype_.mass_kg;
    double yaw_torque_nm = (right_r.net_force_n - left_r.net_force_n) * track_radius_m;
    double yaw_accel_rad_s2 = yaw_torque_nm / archetype_.moment_of_inertia_kg_m2;

    common_v_m_s += common_accel_m_s2 * dt_s;
    yaw_rate_rad_s += yaw_accel_rad_s2 * dt_s;

    // Each wheel's candidate new velocity from the just-integrated common/yaw state, same
    // no-slip relation used everywhere else in this function.
    double left_wheel_v_new = common_v_m_s - yaw_rate_rad_s * track_radius_m;
    double right_wheel_v_new = common_v_m_s + yaw_rate_rad_s * track_radius_m;

    // Zero-crossing guard. Kinetic friction (side_force's "was_kinetic" branch, above) is a
    // constant opposing torque applied with plain forward-Euler integration -- nothing stops one
    // dt's worth of it from overshooting straight through zero and out the other side. Without
    // this guard that shows up as a coasting wheel's velocity (and so yaw rate) flipping sign
    // every single tick once it's near rest, forever, instead of settling -- real kinetic
    // friction can bring a wheel to rest but can't reverse it on its own. Only clamp a side to
    // exactly 0 when BOTH: it was in the kinetic branch this tick, AND the COMMANDED duty (not
    // torque_available, which already has back-EMF braking baked in -- see torque_at()'s own
    // comment -- so it can legitimately oppose the old velocity on pure coast/ease-off too, at
    // duty==0, with no PID reversal involved at all) isn't itself pointing opposite the wheel's
    // OLD velocity. duty at 0 or still matching the old direction means friction (and/or passive
    // back-EMF) is what pushed the crossing, not an active command; a duty that already opposed
    // the old velocity is a real commanded reversal, which this sim's motor curve can
    // legitimately produce a large enough swing from in one 10ms tick and must be left alone.
    // common_v_m_s/yaw_rate_rad_s are rebuilt from the (possibly one-side-clamped) pair of wheel
    // velocities via the exact inverse of the no-slip relation above, which leaves an unclamped
    // side's own velocity completely undisturbed.
    auto friction_caused_crossing = [](const SideForceResult& r, double old_v, double new_v) {
      bool crossed = old_v != 0.0 && new_v * old_v < 0.0;
      bool duty_still_old_direction = r.duty == 0.0 || r.duty * old_v > 0.0;
      return r.was_kinetic && duty_still_old_direction && crossed;
    };
    bool left_crossed = friction_caused_crossing(left_r, left_wheel_v_m_s, left_wheel_v_new);
    bool right_crossed = friction_caused_crossing(right_r, right_wheel_v_m_s, right_wheel_v_new);
    if (left_crossed) left_wheel_v_new = 0.0;
    if (right_crossed) right_wheel_v_new = 0.0;
    if (left_crossed || right_crossed) {
      common_v_m_s = (left_wheel_v_new + right_wheel_v_new) / 2.0;
      yaw_rate_rad_s = (right_wheel_v_new - left_wheel_v_new) / (2.0 * track_radius_m);
    }

    heading_deg_ += (yaw_rate_rad_s * 180.0 / M_PI) * dt_s;

    common_velocity_in_s_ = common_v_m_s / 0.0254;
    yaw_rate_deg_s_ = yaw_rate_rad_s * 180.0 / M_PI;

    // Interference that holds the robot rather than pushing it (a push is external_n above). A pin freezes it
    // where it stands for as long as its window lasts; a wall stops forward travel below.
    bool pinned = false;
    for (const auto& p : pins_)
      if (sim_ms_ >= p.start_ms && sim_ms_ < p.end_ms) pinned = true;
    if (pinned) {
      if (!was_pinned_) {
        pin_left_in_ = left_.position_in;
        pin_right_in_ = right_.position_in;
        pin_heading_deg_ = heading_deg_;
      }
      common_velocity_in_s_ = 0.0;
      yaw_rate_deg_s_ = 0.0;
      left_wheel_v_new = 0.0;
      right_wheel_v_new = 0.0;
      heading_deg_ = pin_heading_deg_;
      double cv = 0.0, cw = 0.0;
      for (const auto& c : carries_)
        if (sim_ms_ >= c.start_ms && sim_ms_ < c.end_ms) {
          cv += c.v;
          cw += c.w;
        }
      if (cv != 0.0 || cw != 0.0) {
        yaw_rate_deg_s_ = -cw;
        common_velocity_in_s_ = cv;
        double yaw_rad = yaw_rate_deg_s_ * M_PI / 180.0;
        left_wheel_v_new = cv * 0.0254 - yaw_rad * track_radius_m;
        right_wheel_v_new = cv * 0.0254 + yaw_rad * track_radius_m;
        pin_heading_deg_ += yaw_rate_deg_s_ * dt_s;
        pin_left_in_ += left_wheel_v_new / 0.0254 * dt_s;
        pin_right_in_ += right_wheel_v_new / 0.0254 * dt_s;
      }
    }
    was_pinned_ = pinned;

    // For encoder reporting; next tick's torque calc re-derives from common_velocity_in_s_/
    // yaw_rate_deg_s_ (above) rather than from these, so the zero-crossing guard above already
    // reached the value that matters for the sim's own recurrence.
    left_.velocity_in_s = left_wheel_v_new / 0.0254;
    right_.velocity_in_s = right_wheel_v_new / 0.0254;
    if (pinned) {
      left_.position_in = pin_left_in_;
      right_.position_in = pin_right_in_;
    } else {
      left_.position_in += left_.velocity_in_s * dt_s;
      right_.position_in += right_.velocity_in_s * dt_s;
    }

    // A wall: average wheel travel cannot pass it, and the forward velocity it hits it at is lost.
    if (wall_set_) {
      double avg = (left_.position_in + right_.position_in) / 2.0;
      if (avg > wall_in_) {
        left_.position_in -= avg - wall_in_;
        right_.position_in -= avg - wall_in_;
        double v_common = (left_.velocity_in_s + right_.velocity_in_s) / 2.0;
        if (v_common > 0.0) {
          left_.velocity_in_s -= v_common;
          right_.velocity_in_s -= v_common;
        }
        if (common_velocity_in_s_ > 0.0) common_velocity_in_s_ = 0.0;
      }
    }
    sim_ms_ += dt_s * 1000.0;

    double left_current = left_r.current_a;
    double right_current = right_r.current_a;

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
    double tick_per_inch = encoder_tick_per_inch();

    auto write_side = [&](std::vector<pros::Motor>& motors, SideState& side, double current_a, double offset_ticks) {
      double reported_pos_in = side.position_in + gaussian(archetype_.encoder_noise_stddev_in);
      double reported_vel_in_s = side.velocity_in_s + gaussian(archetype_.velocity_noise_stddev_in_s);
      for (auto& m : motors) {
        // A motor handed to the PTO isn't the drive's: whatever a test scripts on it (say, an intake that
        // stalls) must survive the tick instead of being overwritten with the drive's own reading.
        if (drive_.pto_check(m)) continue;
        auto& fake = m.fake();
        fake.position = reported_pos_in * tick_per_inch + offset_ticks;
        fake.actual_velocity = reported_vel_in_s * tick_per_inch;
        fake.current_draw = current_a * 1000.0;  // PROS reports current in mA
        // 2.5A: the default current limit (sourced, Purdue SIGBots wiki, per SIM_FIDELITY.md),
        // which MotorCurve::stall_current_a is deliberately set equal to (see its own comment).
        fake.over_current = current_a >= 2.5;
      }
    };
    write_side(drive_.left_motors, left_, left_current_a, enc_off_left_);
    write_side(drive_.right_motors, right_, right_current_a, enc_off_right_);

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
      //
      // The sim owns this register, but the library (drive_angle_set(), odom_xyt_set(), odom_pose_set()) and
      // tests also write it between ticks to say "the heading is now X". Overwriting it from the physical
      // heading every tick silently undid those, so a robot set to 180 read 0 again one tick later. A write
      // the sim did not make is instead adopted as an offset the sim adds to its own physical heading, the
      // way a real IMU's set_rotation() shifts its reading without moving the robot. The constructor
      // records the register's starting value, so one a test left there before the sim existed is still
      // replaced by the first tick, as it always was.
      if (imu_written_ && imus[0]->fake_rotation != imu_last_written_) {
        imu_offset_deg_ = imus[0]->fake_rotation + imu_heading_at_write_deg_;
      }
      double reported_heading = -heading_deg_ + imu_offset_deg_ + gaussian(archetype_.imu_noise_stddev_deg);
      imus[0]->fake_rotation = reported_heading;
      imu_last_written_ = reported_heading;
      imu_heading_at_write_deg_ = heading_deg_;
      imu_written_ = true;
    }
  }

  ez::Drive& drive_;
  SimArchetype archetype_;
  NoiseConfig noise_;
  std::mt19937 rng_;
  SideState left_, right_;
  double heading_deg_ = 0.0;
  // See write_back()'s IMU comment: what the sim last wrote to the fake IMU, the physical heading at that
  // moment, and the offset a foreign write to the register (set_rotation) has added since.
  // Interference (push / pin / wall), see the public block.
  struct Window {
    double value, start_ms, end_ms;
  };
  std::vector<Window> forces_, pins_;
  struct Carry {
    double v, w, start_ms, end_ms;
  };
  std::vector<Carry> carries_;
  double sim_ms_ = 0.0;
  bool wall_set_ = false, was_pinned_ = false;
  double wall_in_ = 0.0, pin_left_in_ = 0.0, pin_right_in_ = 0.0, pin_heading_deg_ = 0.0;
  bool use_real_auto_task_ = false;

  // Ticks per wheel inch the encoders really count at. Until shift_gearing() is called this is whatever the Drive believes
  // (so every existing test is unchanged); after, it is the physical value, which the Drive only learns about if the test
  // tells it. enc_off_* keep the raw count continuous across a shift.
  double encoder_tick_per_inch() { return phys_tpi_ > 0.0 ? phys_tpi_ : drive_.drive_tick_per_inch(); }
  double phys_tpi_ = 0.0;
  double enc_off_left_ = 0.0;
  double enc_off_right_ = 0.0;
  int pass_count_ = 0;
  int passes_per_tick_ = 1;
  bool imu_written_ = false;
  double imu_last_written_ = 0.0;
  double imu_heading_at_write_deg_ = 0.0;
  double imu_offset_deg_ = 0.0;
  // Robot-level translational/rotational state -- see step_physics()'s header comment for why
  // these exist instead of deriving everything from left_/right_.velocity_in_s directly.
  double common_velocity_in_s_ = 0.0;
  double yaw_rate_deg_s_ = 0.0;
};

}  // namespace sim
