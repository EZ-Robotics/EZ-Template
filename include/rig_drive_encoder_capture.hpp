/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

// Test-only capture rig for the redundant-drive-encoders investigation (issue 372, Phase 1).
// Not part of the EZ-Template library: excluded from EXCLUDE_SRC_FROM_LIB in the Makefile, so it
// never ships in the template zip or gets linked into a user's own build. It owns its own
// pros::Motor/pros::Imu handles rather than touching a Drive object, so it never touches
// drive_mutex and can run safely alongside a live chassis without taking the EZ lock.
//
// Records every drive motor's raw state every 10ms from its own task, buffers everything in RAM
// (no SD/serial I/O during capture, so the write-up doesn't disturb the timing being measured),
// then writes one CSV after the run ends.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace rig {

// Per-motor fields captured every pass. Mirrors what Phase 1's run sheet asks for.
struct MotorSample {
  double position = 0;           // get_position(), in whatever units the port is set to (counts, see .cpp)
  int32_t raw_position = 0;      // get_raw_position() -- native encoder ticks, unit-independent
  uint32_t raw_timestamp = 0;    // device timestamp handed back by get_raw_position(); stalls if the sample is stale
  double velocity = 0;           // get_actual_velocity()
  int32_t current_draw_ma = 0;   // get_current_draw()
  int32_t voltage_mv = 0;        // get_voltage()
  uint32_t faults = 0;           // get_faults()
  uint32_t flags = 0;            // get_flags()
  float temperature_c = 0;       // get_temperature(), refreshed once/sec (held between refreshes)
  bool port_has_motor = false;   // registry_get_plugged_type(port) == E_DEVICE_MOTOR
};

// Fields captured once per pass, shared across all motors on this side.
struct PassSample {
  uint64_t micros = 0;            // pros::micros() at the top of this pass
  uint32_t elapsed_us = 0;        // measured time since the previous pass (not an assumed 10ms)
  int32_t left_voltage_cmd = 0;   // proxy for "what EZ commanded": left primary motor's get_voltage()
  int32_t right_voltage_cmd = 0;  // same proxy, right side -- see the .cpp comment on why this is a proxy
  double imu_rotation = 0;        // NAN if this run was constructed with imu_port == 0
  uint8_t competition_state = 0;  // raw competition_get_status() bitmask
  std::vector<MotorSample> left;
  std::vector<MotorSample> right;
};

class DriveEncoderCapture {
 public:
  // left_ports/right_ports use the same signed-port convention as Drive's constructor (negative
  // = reversed; sign doesn't affect what this rig reads, it's just accepted for symmetry with how
  // you already have your ports written down). imu_port == 0 skips IMU capture.
  // max_seconds bounds the pre-reserved RAM buffer; see the .cpp for the size math.
  DriveEncoderCapture(std::vector<int> left_ports, std::vector<int> right_ports, int imu_port = 0, int max_seconds = 90);

  // Starts the 10ms capture task. run_label becomes part of the output filename -- keep it
  // short and filesystem-safe (letters, digits, underscore).
  void start(const std::string& run_label);

  // Stops the task and writes the buffered run to /usd/rig_<run_label>.csv. If no SD card is
  // installed, dumps the same CSV over serial (printf) instead. Blocks until the write finishes.
  // Safe to call start() again afterwards for a new run.
  void stop_and_dump();

  // Passes recorded so far in the current (or just-finished) run.
  size_t passes_recorded() const;

 private:
  void capture_loop();

  std::vector<pros::Motor> left_motors_;
  std::vector<pros::Motor> right_motors_;
  pros::Imu* imu_ = nullptr;

  size_t max_passes_;
  std::vector<PassSample> passes_;
  size_t pass_count_ = 0;

  std::string run_label_;
  pros::Task* task_ = nullptr;
  volatile bool running_ = false;
};

}  // namespace rig
