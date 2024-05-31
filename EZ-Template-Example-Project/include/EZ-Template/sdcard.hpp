/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/auton_selector.hpp"
#include "api.h"

namespace ez {
namespace as {
extern AutonSelector auton_selector;
/**
 * Sets sd card to current page.
 */
void auton_selector_initialize();

/**
 * Sets the sd card to current page.
 */
void auto_sd_update();

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

/**
 * Returns true if the auton selector is running
 */
bool enabled();

inline bool auton_selector_running;

extern bool turn_off;

extern pros::adi::DigitalIn* limit_switch_left;
extern pros::adi::DigitalIn* limit_switch_right;
/**
 * Initialize two limitswithces to change pages on the lcd
 *
 * @param left_limit_port
 *        port for the left limit switch
 * @param right_limit_port
 *        port for the right limit switch
 */
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit = nullptr);

/**
 * pre_auto_task
 */
void limitSwitchTask();
}  // namespace as
}  // namespace ez
