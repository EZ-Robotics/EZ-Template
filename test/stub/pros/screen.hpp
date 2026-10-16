// Minimal stand-in for PROS's pros/screen.hpp -- no-op drawing calls, since
// the test binary never has a real screen. Only reached (from drive.cpp's
// drive_imu_display_loading()) when pros::lcd::is_initialized() is false and
// the loading animation is on; tests should pass run_loading_animation=false
// to drive_imu_calibrate() to avoid depending on these being more than no-ops.
#pragma once

#include <cstdint>

namespace pros {

namespace c {
constexpr std::uint32_t COLOR_WHITE = 0xFFFFFF;
constexpr std::uint32_t COLOR_RED = 0xFF0000;
}  // namespace c

namespace screen {
inline void set_pen(std::uint32_t color) {}
inline void draw_rect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1) {}
inline void fill_rect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1) {}
}  // namespace screen

struct screen_touch_status_s_t {
  int touch_status = 0;
  std::int16_t x = 0;
  std::int16_t y = 0;
};
using screen_touch_status = screen_touch_status_s_t;

}  // namespace pros
