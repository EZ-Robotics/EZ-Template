/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "rig_drive_encoder_capture.hpp"

#include <cinttypes>
#include <cmath>
#include <cstdio>

#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/rtos.h"

// Buffer size math (see DriveEncoderCapture ctor): each MotorSample is ~40 bytes of typed fields
// (rounds to ~48 with alignment); each PassSample adds ~29 bytes of shared fields (micros,
// elapsed_us, two voltages, imu_rotation, competition_state) on top of its two motor vectors.
// For 8 drive motors (4 left + 4 right) at 100 passes/sec for 60s = 6000 passes:
//   6000 * (29 + 8 * 48) = 6000 * 413 = ~2.48 MB
// The V5's linker script reserves a ~48MB heap for user code (pros 4.2.2, firmware/v5.ld,
// _HEAP_SIZE = 0x02E00000), so this comfortably fits with enormous headroom -- confirmed from
// source, not assumed.

namespace rig {

namespace {
constexpr int CAPTURE_PERIOD_MS = 10;

bool port_has_motor(int signed_port) {
  int zero_indexed = std::abs(signed_port) - 1;
  return pros::c::registry_get_plugged_type(static_cast<std::uint8_t>(zero_indexed)) == pros::c::E_DEVICE_MOTOR;
}
}  // namespace

DriveEncoderCapture::DriveEncoderCapture(std::vector<int> left_ports, std::vector<int> right_ports, int imu_port, int max_seconds) {
  for (int port : left_ports) {
    pros::Motor m(port);
    m.set_encoder_units(pros::MotorUnits::counts);
    left_motors_.push_back(m);
  }
  for (int port : right_ports) {
    pros::Motor m(port);
    m.set_encoder_units(pros::MotorUnits::counts);
    right_motors_.push_back(m);
  }
  if (imu_port != 0) imu_ = new pros::Imu(imu_port);

  max_passes_ = static_cast<size_t>(max_seconds) * 1000 / CAPTURE_PERIOD_MS;
}

void DriveEncoderCapture::start(const std::string& run_label) {
  run_label_ = run_label;
  pass_count_ = 0;

  // Pre-reserve everything up front so the hot loop below never allocates -- allocation timing
  // jitter would corrupt the exact thing this rig exists to measure.
  passes_.clear();
  passes_.resize(max_passes_);
  for (auto& pass : passes_) {
    pass.left.resize(left_motors_.size());
    pass.right.resize(right_motors_.size());
  }

  printf("rig: reserved %u passes (%u bytes) for run '%s'\n",
         static_cast<unsigned>(max_passes_),
         static_cast<unsigned>(max_passes_ * sizeof(PassSample)),
         run_label_.c_str());

  running_ = true;
  task_ = new pros::Task([this] { capture_loop(); }, "rig_drive_encoder_capture");
}

void DriveEncoderCapture::capture_loop() {
  uint64_t last_micros = pros::micros();
  uint32_t last_temp_second = 0;

  while (running_ && pass_count_ < max_passes_) {
    uint32_t pass_start_ms = pros::millis();
    PassSample& pass = passes_[pass_count_];

    uint64_t now = pros::micros();
    pass.micros = now;
    pass.elapsed_us = static_cast<uint32_t>(now - last_micros);
    last_micros = now;

    pass.competition_state = pros::c::competition_get_status();
    pass.imu_rotation = imu_ ? imu_->get_rotation() : NAN;

    // Proxy for "what EZ commanded" -- this rig deliberately never touches the Drive object (no
    // lock, no coupling), so it reads back the actual applied voltage on each side's first motor
    // instead of the literal value private_drive_set() sent. On a healthy drive these are the
    // same number; flagged here and in the run report since it's an approximation, not a fact.
    pass.left_voltage_cmd = left_motors_.empty() ? 0 : left_motors_[0].get_voltage();
    pass.right_voltage_cmd = right_motors_.empty() ? 0 : right_motors_[0].get_voltage();

    bool refresh_temp = (pass_start_ms / 1000) != last_temp_second;
    if (refresh_temp) last_temp_second = pass_start_ms / 1000;

    auto sample_side = [&](std::vector<pros::Motor>& motors, std::vector<int>& /*unused*/) {};
    (void)sample_side;

    for (size_t i = 0; i < left_motors_.size(); i++) {
      MotorSample& s = pass.left[i];
      uint32_t raw_ts = 0;
      s.position = left_motors_[i].get_position();
      s.raw_position = left_motors_[i].get_raw_position(&raw_ts);
      s.raw_timestamp = raw_ts;
      s.velocity = left_motors_[i].get_actual_velocity();
      s.current_draw_ma = left_motors_[i].get_current_draw();
      s.voltage_mv = left_motors_[i].get_voltage();
      s.faults = left_motors_[i].get_faults();
      s.flags = left_motors_[i].get_flags();
      s.port_has_motor = port_has_motor(left_motors_[i].get_port());
      if (refresh_temp || pass_count_ == 0) s.temperature_c = static_cast<float>(left_motors_[i].get_temperature());
      else s.temperature_c = passes_[pass_count_ - 1].left[i].temperature_c;
    }
    for (size_t i = 0; i < right_motors_.size(); i++) {
      MotorSample& s = pass.right[i];
      uint32_t raw_ts = 0;
      s.position = right_motors_[i].get_position();
      s.raw_position = right_motors_[i].get_raw_position(&raw_ts);
      s.raw_timestamp = raw_ts;
      s.velocity = right_motors_[i].get_actual_velocity();
      s.current_draw_ma = right_motors_[i].get_current_draw();
      s.voltage_mv = right_motors_[i].get_voltage();
      s.faults = right_motors_[i].get_faults();
      s.flags = right_motors_[i].get_flags();
      s.port_has_motor = port_has_motor(right_motors_[i].get_port());
      if (refresh_temp || pass_count_ == 0) s.temperature_c = static_cast<float>(right_motors_[i].get_temperature());
      else s.temperature_c = passes_[pass_count_ - 1].right[i].temperature_c;
    }

    pass_count_++;
    pros::Task::delay_until(&pass_start_ms, CAPTURE_PERIOD_MS);
  }
  running_ = false;
}

size_t DriveEncoderCapture::passes_recorded() const { return pass_count_; }

void DriveEncoderCapture::stop_and_dump() {
  running_ = false;
  if (task_ != nullptr) {
    task_->join();
    delete task_;
    task_ = nullptr;
  }

  // Build the header row once.
  std::string header = "pass,micros,elapsed_us,competition_state,imu_rotation,left_voltage_cmd,right_voltage_cmd";
  for (size_t i = 0; i < left_motors_.size(); i++) {
    std::string p = "L" + std::to_string(i) + "_";
    header += "," + p + "port," + p + "installed," + p + "position," + p + "raw_position," + p + "raw_timestamp," +
              p + "velocity," + p + "current_ma," + p + "voltage_mv," + p + "faults," + p + "flags," + p + "temp_c";
  }
  for (size_t i = 0; i < right_motors_.size(); i++) {
    std::string p = "R" + std::to_string(i) + "_";
    header += "," + p + "port," + p + "installed," + p + "position," + p + "raw_position," + p + "raw_timestamp," +
              p + "velocity," + p + "current_ma," + p + "voltage_mv," + p + "faults," + p + "flags," + p + "temp_c";
  }
  header += "\n";

  auto write_row = [&](FILE* f, size_t i) {
    const PassSample& pass = passes_[i];
    fprintf(f, "%u,%" PRIu64 ",%" PRIu32 ",%u,%f,%" PRId32 ",%" PRId32, static_cast<unsigned>(i), pass.micros,
            pass.elapsed_us, static_cast<unsigned>(pass.competition_state), pass.imu_rotation, pass.left_voltage_cmd,
            pass.right_voltage_cmd);
    for (size_t m = 0; m < left_motors_.size(); m++) {
      const MotorSample& s = pass.left[m];
      fprintf(f, ",%d,%d,%f,%" PRId32 ",%" PRIu32 ",%f,%" PRId32 ",%" PRId32 ",%" PRIu32 ",%" PRIu32 ",%f",
              left_motors_[m].get_port(), s.port_has_motor ? 1 : 0, s.position, s.raw_position, s.raw_timestamp,
              s.velocity, s.current_draw_ma, s.voltage_mv, s.faults, s.flags, s.temperature_c);
    }
    for (size_t m = 0; m < right_motors_.size(); m++) {
      const MotorSample& s = pass.right[m];
      fprintf(f, ",%d,%d,%f,%" PRId32 ",%" PRIu32 ",%f,%" PRId32 ",%" PRId32 ",%" PRIu32 ",%" PRIu32 ",%f",
              right_motors_[m].get_port(), s.port_has_motor ? 1 : 0, s.position, s.raw_position, s.raw_timestamp,
              s.velocity, s.current_draw_ma, s.voltage_mv, s.faults, s.flags, s.temperature_c);
    }
    fprintf(f, "\n");
  };

  bool have_sd = pros::usd::is_installed();
  if (have_sd) {
    std::string path = "/usd/rig_" + run_label_ + ".csv";
    FILE* f = fopen(path.c_str(), "w");
    if (f != nullptr) {
      fputs(header.c_str(), f);
      for (size_t i = 0; i < pass_count_; i++) write_row(f, i);
      fclose(f);
      printf("rig: wrote %u passes to %s\n", static_cast<unsigned>(pass_count_), path.c_str());
      return;
    }
    printf("rig: SD card present but %s could not be opened for write, falling back to serial dump\n", path.c_str());
  }

  printf("rig: dumping %u passes over serial (run '%s')\n", static_cast<unsigned>(pass_count_), run_label_.c_str());
  printf("%s", header.c_str());
  for (size_t i = 0; i < pass_count_; i++) write_row(stdout, i);
}

}  // namespace rig
