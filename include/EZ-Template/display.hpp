#pragma once

#include <string>

namespace ez {

/// Rotates the brain screen in 90-degree steps for brains mounted sideways or
/// upside down. Accepts 0, 90, 180, or 270 (measured clockwise); other values
/// are rejected with a printed message. Call after the screen is initialized
/// (ez::as::initialize or pros::lcd::initialize).
///
/// At 90 and 270 the stock LLEMU screen is swapped for EZ's portrait screen:
/// LLEMU builds its widgets at the resolution live when pros::lcd::initialize
/// ran (480x272), so once the canvas becomes 272x480 its text runs off the
/// right edge. The portrait screen re-flows the same LLEMU theme, font, print
/// lines and three-button bar to the rotated resolution, wrapping long lines
/// rather than clipping them, and replays its button presses onto LLEMU's own
/// buttons so pros::lcd::register_btn*_cb callbacks behave identically.
/// 0 and 180 restore LLEMU untouched.
///
/// Experimental, unverified on hardware: uses LVGL's software rotation on the
/// live display driver. Touch rotates with the pixels, so on-screen buttons
/// respond where they are drawn; LVGL applies that transform to its own
/// pointer input. 90/270 swap the screen's width and height. The physical
/// LLEMU buttons are unaffected.
void screen_rotation_set(int degrees);

/// True while the portrait screen is the loaded screen (rotation 90 or 270).
bool screen_portrait_enabled();

/// Writes one of the 8 print lines, to the portrait screen when it is loaded
/// and to LLEMU otherwise. ez::screen_print routes through here.
void screen_line_set(int line, std::string text);

/// Clears one of the 8 print lines through the same routing as
/// ez::screen_line_set.
void screen_line_clear(int line);

/// Clears all 8 print lines through the same routing as ez::screen_line_set.
void screen_lines_clear();

/// A point in screen coordinates.
struct screen_point {
  int x;
  int y;
};

/// Maps a raw panel touch into the coordinate space of a screen rotated by
/// `degrees`. panel_w/panel_h are the physical, unrotated panel size (480x272
/// on the V5). Unsupported angles pass the point through unchanged.
///
/// LVGL already applies this to its own pointer input, so widgets need no help;
/// this is for code that reads the panel directly (pros::screen_touch_status)
/// and so bypasses LVGL. Do not apply it to lv_indev data or the point is
/// transformed twice.
///
/// Convention read off the vendored liblvgl.a (LVGL 8.3.4), whose DWARF maps to
/// lv_refr.c / lv_indev.c / lv_hal_disp.c:
///   lv_refr.c:1109-1114, 1127-1132 map a rendered area logical -> physical as
///     90:  px = ly,               py = panel_h - 1 - lx
///     180: px = panel_w - 1 - lx, py = panel_h - 1 - ly
///     270: px = panel_w - 1 - ly, py = lx
///   lv_indev.c:347-354 applies exactly the inverse below to pointer input.
///   lv_hal_disp.c:349-354 and 372-377 swap hor_res/ver_res on 90 and 270,
///   leaving the driver's own hor_res/ver_res physical.
inline screen_point screen_touch_rotate(int degrees, int panel_w, int panel_h, int raw_x, int raw_y) {
  switch (degrees) {
    case 90: return {panel_h - 1 - raw_y, raw_x};
    case 180: return {panel_w - 1 - raw_x, panel_h - 1 - raw_y};
    case 270: return {raw_y, panel_w - 1 - raw_x};
    default: return {raw_x, raw_y};
  }
}

/// Current rotation in degrees (0, 90, 180, or 270).
int screen_rotation_get();

}  // namespace ez
