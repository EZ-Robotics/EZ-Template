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
enum e_type{ k_single=0, k_split=1 };
void chassis_tank();
void chassis_arcade_standard(e_type t);
void chassis_arcade_flipped(e_type t);
