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

extern bool turn_off;

extern pros::ADIDigitalIn* left_limit_switch;
extern pros::ADIDigitalIn* right_limit_switch;
/**
 * Initialize two limitswithces to change pages on the lcd
 *
 * @param left_limit_port
 *        port for the left limit switch
 * @param right_limit_port
 *        port for the right limit switch
 */
void limit_switch_lcd_initialize(pros::ADIDigitalIn* right_limit, pros::ADIDigitalIn* left_limit = nullptr);

/**
 * pre_auto_task
 */
void limitSwitchTask();
}  // namespace as
}  // namespace ez
