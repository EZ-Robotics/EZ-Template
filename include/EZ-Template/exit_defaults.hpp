/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

namespace ez {
namespace detail {

// The big errors drive_defaults_set() gives every motion, and the ones the waits fall back on for an axis whose small error and big error a
// team has both set to 0 (its window exits are then off, and "inside the big error" would never be true, so a robot at rest on its target
// could never be called settled). One place, read by both, so the fallback is always the band the library ships.
//   distance (drive and odom xy): 3 inches
//   angle (turn, swing and odom heading): 7 degrees
// Internal, not a setting: a team changes its own big error with the exit condition setters.
constexpr double DEFAULT_BIG_ERROR_DISTANCE = 3.0;
constexpr double DEFAULT_BIG_ERROR_ANGLE = 7.0;

}  // namespace detail
}  // namespace ez
