/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/auton_selector.hpp"

namespace ez {
namespace as {
extern AutonSelector auton_selector;
/**
 * Sets sd card to current page.
 */
void init_auton_selector();
/**
 * Sets the sd card to current page.
 */
void update_auto_sd();
/**
 * Increases the page by 1.
 */
void page_up();
/**
 * Decreases the page by 1.
 */
void page_down();
/**
 * Initializes LLEMU and sets up callbacks for auton selector.
 */
void initialize();
/**
 * Wrapper for pros::lcd::shutdown.
 */
void shutdown();
}  // namespace as
}  // namespace ez
