// Minimal stand-in for PROS's pros/usd.hpp -- only pros::usd::is_installed(),
// which util.hpp calls once at static-init time to set SD_CARD_ACTIVE.
#pragma once

namespace pros {
namespace usd {
inline bool is_installed() { return false; }
}  // namespace usd
}  // namespace pros
