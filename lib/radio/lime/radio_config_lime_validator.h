/*
 *
 * Copyright 2021-2024 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#pragma once

#include "radio_lime_device.h"
#include "radio_lime_sdrdevice.h"
#include "srsran/radio/radio_configuration.h"

namespace srsran {

/// \brief Radio configuration validator for lime based radios.
///
/// This validator validates that parameters values are generally valid, for example frequencies are valid numbers and
/// positive or stream arguments follow a certain pattern. However, it does not validate whether the actual values are
/// supported by the LIME device.
///
/// It is up to the user to select parameters within ranges. These can be consulted using the LIME application \e
/// lime_usrp_probe.
class radio_config_lime_config_validator : public radio_configuration::validator
{
public:
  // See interface for documentation.
  bool is_configuration_valid(const radio_configuration::radio& config) const override;
};

} // namespace srsran
