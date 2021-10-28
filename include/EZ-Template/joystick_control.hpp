/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

void init_curve_sd();
/**
 * Sets the chassis motors speeds to the controller joystick throttle amounts (settings in setup.hpp)
*/
void chassis_joystick_control();
