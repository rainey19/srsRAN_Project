/*
 *
 * Copyright 2021-2025 Software Radio Systems Limited
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

#include "srsran/cu_cp/cu_cp_configuration.h"
#include "srsran/cu_cp/cu_cp_types.h"
#include "srsran/srslog/srslog.h"
#include <string>

namespace srsran {
namespace srs_cu_cp {

struct cu_up_processor_config_t {
  std::string                name        = "srs_cu_cp";
  cu_up_index_t              cu_up_index = cu_up_index_t::invalid;
  const cu_cp_configuration& cu_cp_cfg;
  srslog::basic_logger&      logger = srslog::fetch_basic_logger("CU-CP");
};

} // namespace srs_cu_cp
} // namespace srsran
