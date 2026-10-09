// Minimal stand-in for PROS's pros::Imu. Fake state is public so tests can
// drive it directly (e.g. chassis.imu->fake_rotation = ...).
#pragma once

#include <cstdint>

namespace pros {

// Test-only: when set, every Imu::get_rotation() calls it once the value is read and before it returns, so a test can land another task's call inside a
// sensor read. Null by default and cleared by test_stub::reset_all().
inline void (*imu_read_hook)() = nullptr;

class Imu {
 public:
  struct accel_s_t {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  Imu() = default;
  explicit Imu(std::int32_t port) : port_(port) {}

  double fake_rotation = 0.0;
  accel_s_t fake_accel{};
  bool fake_calibrating = false;
  bool fake_installed = true;

  std::int32_t get_port() const { return port_; }
  double get_rotation() const {
    double rotation = fake_rotation;
    if (imu_read_hook != nullptr) imu_read_hook();  // test-only: runs after the value is read, before it is returned
    return rotation;
  }
  std::int32_t set_rotation(double target) {
    fake_rotation = target;
    return 1;
  }
  bool is_calibrating() const { return fake_calibrating; }
  std::int32_t reset(bool blocking = true) {
    fake_calibrating = false;
    return 1;
  }
  bool is_installed() const { return fake_installed; }
  accel_s_t get_accel() const { return fake_accel; }
  std::int32_t set_data_rate(std::uint32_t rate) const { return 1; }

 private:
  std::int32_t port_ = 0;
};

}  // namespace pros
