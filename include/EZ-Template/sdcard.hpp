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
 * Returns true if the auton selector is running.
 */
bool enabled();

inline bool auton_selector_running;

extern bool turn_off;

extern pros::adi::DigitalIn* limit_switch_left;
extern pros::adi::DigitalIn* limit_switch_right;

/**
 * Initialize two limit switches to change pages on the lcd.
 *
 * The library never takes ownership of the pointers passed in and will not
 * delete them, including when called with both pointers null to disable this
 * feature.
 *
 * @param right_limit
 *        limit switch that goes forward a page
 * @param left_limit
 *        limit switch that goes back a page, defaults to nullptr
 */
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit = nullptr);

/**
 * pre_auto_task
 */
void limitSwitchTask();

/**
 * Returns the current blank page that is on.  Negative value means the current page isn't blank.
 */
int page_blank_current();

/**
 * Checks if this blank page is open.  If this page doesn't exist, this will create it.
 *
 * @param page
 *        blank page to check, starting at 0
 */
bool page_blank_is_on(int page);

/**
 * Removes the blank page if it exists, and previous ones.
 *
 * @param page
 *        blank page to remove, starting at 0.  The blank pages before it are removed too
 */
void page_blank_remove(int page);

/**
 * Removes all blank pages.
 */
void page_blank_remove_all();

/**
 * Removes the current amount of blank pages.
 */
int page_blank_amount();

/**
 * Current amount of blank pages.
 */
extern int amount_of_blank_pages;
}  // namespace as
}  // namespace ez
