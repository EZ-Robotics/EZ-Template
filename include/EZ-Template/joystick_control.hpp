/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

void init_curve_sd();

enum e_type{ k_single=0, k_split=1 };

/**
 * Sets the chassis to controller joysticks, using tank control.
*/
void chassis_tank();

/**
 * Sets the chassis to controller joysticks, using standard arcade control.
 * \param t
 *        enum e_type, k_single or k_split control
*/
void chassis_arcade_standard(e_type t);

/**
 * Sets the chassis to controller joysticks, using flipped arcade control.
 * \param t
 *        enum e_type, k_single or k_split control
*/
void chassis_arcade_flipped(e_type t);
