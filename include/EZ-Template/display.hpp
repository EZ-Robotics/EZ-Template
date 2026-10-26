#pragma once

#include <string>

namespace ez {

/// Rotates the brain screen in 90-degree steps, for a brain mounted sideways or upside down.  Call it after the screen is initialized
/// (ez::as::initialize() or pros::lcd::initialize()).
///
/// At 90 and 270 the stock LLEMU screen is swapped for EZ's portrait screen, which re-flows the same print lines and button bar to the rotated
/// size.  At 0 and 180 LLEMU stays.  Touch rotates with the screen.  Experimental, unverified on hardware.
///
/// \param degrees
///        0, 90, 180, or 270, measured clockwise.  Any other value prints a message and does nothing
///
/// \code{.cpp}
/// void initialize() {
///   ez::as::initialize();
///
///   // Brain is mounted sideways
///   ez::screen_rotation_set(90);
/// }
/// \endcode
void screen_rotation_set(int degrees);

/// True while the portrait screen is the loaded screen (rotation 90 or 270).
///
/// \return
///        true while the portrait screen is loaded, false otherwise
///
/// \code{.cpp}
/// void initialize() {
///   ez::as::initialize();
///   ez::screen_rotation_set(90);
///   printf("%i\n", ez::screen_portrait_enabled());  // Prints 1
/// }
/// \endcode
bool screen_portrait_enabled();

/// Writes one of the 8 print lines, to the portrait screen when it is loaded
/// and to LLEMU otherwise. ez::screen_print routes through here.
///
/// \param line
///        0 through 7
/// \param text
///        the text to print
///
/// \code{.cpp}
/// void initialize() {
///   ez::screen_line_set(0, "Hello!");
/// }
/// \endcode
void screen_line_set(int line, std::string text);

/// Clears one of the 8 print lines through the same routing as
/// ez::screen_line_set.
///
/// \param line
///        0 through 7
///
/// \code{.cpp}
/// void initialize() {
///   ez::as::initialize();
///
///   ez::screen_print("Hello!", 0);
///
///   // Leave it up long enough to see
///   pros::delay(2000);
///
///   ez::screen_line_clear(0);
/// }
/// \endcode
void screen_line_clear(int line);

/// Clears all 8 print lines through the same routing as ez::screen_line_set.
///
/// \code{.cpp}
/// void initialize() {
///   ez::as::initialize();
///
///   ez::screen_print("Hello!", 0);
///   ez::screen_print("Goodbye!", 1);
///
///   // Leave them up long enough to see
///   pros::delay(2000);
///
///   ez::screen_lines_clear();
/// }
/// \endcode
void screen_lines_clear();

/// A point in screen coordinates.
struct screen_point {
  int x;
  int y;
};

// Convention read off the vendored liblvgl.a (LVGL 8.3.4), whose DWARF maps to
// lv_refr.c / lv_indev.c / lv_hal_disp.c:
//   lv_refr.c:1109-1114, 1127-1132 map a rendered area logical -> physical as
//     90:  px = ly,               py = panel_h - 1 - lx
//     180: px = panel_w - 1 - lx, py = panel_h - 1 - ly
//     270: px = panel_w - 1 - ly, py = lx
//   lv_indev.c:347-354 applies exactly the inverse below to pointer input.
//   lv_hal_disp.c:349-354 and 372-377 swap hor_res/ver_res on 90 and 270,
//   leaving the driver's own hor_res/ver_res physical.

/// Maps a raw panel touch into the coordinate space of a screen rotated by
/// `degrees`. panel_w/panel_h are the physical, unrotated panel size (480x272
/// on the V5). Unsupported angles pass the point through unchanged.
///
/// LVGL already applies this to its own pointer input, so widgets need no help;
/// this is for code that reads the panel directly (pros::screen::touch_status)
/// and so bypasses LVGL. Do not apply it to lv_indev data or the point is
/// transformed twice.
///
/// \param degrees
///        0, 90, 180, or 270
/// \param panel_w
///        the physical, unrotated panel width (480 on the V5)
/// \param panel_h
///        the physical, unrotated panel height (272 on the V5)
/// \param raw_x
///        raw touch x from the panel
/// \param raw_y
///        raw touch y from the panel
/// \return
///        the touch point in the rotated screen's coordinates, as an ez::screen_point
///
/// \code{.cpp}
/// void opcontrol() {
///   while (true) {
///     pros::screen_touch_status_s_t status = pros::screen::touch_status();
///     ez::screen_point p =
///         ez::screen_touch_rotate(ez::screen_rotation_get(), 480, 272, status.x, status.y);
///     printf("x: %i, y: %i\n", p.x, p.y);
///
///     pros::delay(ez::util::DELAY_TIME);
///   }
/// }
/// \endcode
inline screen_point screen_touch_rotate(int degrees, int panel_w, int panel_h, int raw_x, int raw_y) {
  switch (degrees) {
    case 90:
      return {panel_h - 1 - raw_y, raw_x};
    case 180:
      return {panel_w - 1 - raw_x, panel_h - 1 - raw_y};
    case 270:
      return {raw_y, panel_w - 1 - raw_x};
    default:
      return {raw_x, raw_y};
  }
}

/// Current rotation in degrees (0, 90, 180, or 270).
///
/// \return
///        the current rotation in degrees, measured clockwise
///
/// \code{.cpp}
/// void initialize() {
///   ez::as::initialize();
///   ez::screen_rotation_set(90);
///   printf("%i\n", ez::screen_rotation_get());  // Prints 90
/// }
/// \endcode
int screen_rotation_get();

}  // namespace ez
