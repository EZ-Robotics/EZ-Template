// Minimal stand-in for PROS's pros/llemu.hpp -- pros::lcd::*. None of these
// are called from anything compiled into the test binary (display.cpp and
// sdcard.cpp, the only callers, are excluded); this exists so declarations
// that mention pros::lcd::is_initialized() (drive.cpp) still link.
#pragma once

namespace pros {
namespace lcd {
inline bool is_initialized() { return false; }
inline bool initialize() { return true; }
inline bool shutdown() { return true; }
}  // namespace lcd
}  // namespace pros
