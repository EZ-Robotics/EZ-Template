/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include <string>

namespace okapi {
struct OdomState {
  QLength x{0_m};
  QLength y{0_m};
  QAngle theta{0_deg};

  /**
   * @return A string representing the state.
   */
  std::string str() const;

  bool operator==(const OdomState &rhs) const;

  bool operator!=(const OdomState &rhs) const;
};
} // namespace okapi
