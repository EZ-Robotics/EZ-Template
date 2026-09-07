// Minimal stand-in for PROS's pros::MotorGroup -- just enough for
// PID::exit_condition(const pros::MotorGroup&), which only calls size() and
// get_port().
#pragma once

#include <cstdint>
#include <vector>

#include "pros/motors.h"

namespace pros {

class MotorGroup {
 public:
  MotorGroup() = default;
  explicit MotorGroup(std::vector<std::int8_t> ports) : ports_(std::move(ports)) {}

  std::int32_t size() const { return (std::int32_t)ports_.size(); }
  std::int32_t get_port(std::int32_t index) const {
    return index >= 0 && (std::size_t)index < ports_.size() ? std::abs((int)ports_[index]) : 0;
  }

 private:
  std::vector<std::int8_t> ports_ = {1, 2};
};

}  // namespace pros
