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
 * Sets the current page to what's saved on the SD card.  Creates the save file if it isn't there,
 * and goes to page 0 if the saved page doesn't exist.
 */
void auton_selector_initialize();

/**
 * Sets the sd card to current page.
 */
void auto_sd_update();

/**
 * Increases the page number by 1.  Best used with the lcd callback functions.
 */
void page_up();

/**
 * Decreases the page number by 1.  Best used with the lcd callback functions.
 */
void page_down();

/**
 * Initializes the autonomous selector.  If an SD card is plugged in, the current page will set to what's on the SD card.
 *
 * Also initializes LLEMU and sets up the callbacks for the auton selector.
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
 * Sets external buttons to increase/decrease the current autonomous page.
 *
 * The library never takes ownership of the pointers passed in and will not
 * delete them.  The switches must outlive this call, the library keeps polling the pointers.  Make them global, not locals inside initialize().
 *
 * Calling this with both pointers null disables the limit switches, this works any time it is called.
 * The library stops polling the old pointers.
 *
 * @param right_limit
 *        a button to go forward a page
 * @param left_limit
 *        a button to go backwards a page, defaults to nullptr
 */
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit = nullptr);

/**
 * Task that polls the limit switches and changes pages on a new press.  Runs as an internal task,
 * it is not meant to be called.
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
 *
 * The next page_blank_is_on() call creates its page again, so stop calling it once you have removed the pages (for example, only call it while
 * pros::competition::is_connected() is false).
 */
void page_blank_remove_all();

/**
 * Returns the current amount of blank pages.
 */
int page_blank_amount();

/**
 * Current amount of blank pages.
 */
extern int amount_of_blank_pages;
}  // namespace as
}  // namespace ez
