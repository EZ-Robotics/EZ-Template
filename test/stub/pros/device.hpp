// Minimal stand-in for PROS's pros::Device -- only used as a pointer type in
// EZ-Template/health.hpp's device_add() declaration, which nothing compiled
// into the test binary calls (health.cpp is excluded).
#pragma once

namespace pros {

class Device {
 public:
  virtual ~Device() = default;
  virtual bool is_installed() { return true; }
};

}  // namespace pros
