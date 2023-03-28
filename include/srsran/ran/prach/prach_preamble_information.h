/*
 *
 * Copyright 2021-2023 Software Radio Systems Limited
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

#include "srsran/ran/phy_time_unit.h"
#include "srsran/ran/prach/prach_preamble_format.h"
#include "srsran/ran/prach/prach_subcarrier_spacing.h"

namespace srsran {

struct prach_configuration;

/// \brief Collects PRACH preamble information parameters.
///
/// The parameters are used and described in TS38.211 Section 6.3.3.1.
struct prach_preamble_information {
  /// Sequence length in frequency domain, parameter \f$L_{RA}\f$.
  unsigned sequence_length;
  /// Parameter \f$\Delta f^{RA}\f$.
  prach_subcarrier_spacing scs;
  /// Parameter \f$N_u\f$. Expressed in units of the reference symbol time \f$\kappa\f$.
  phy_time_unit symbol_length;
  /// Parameter \f$N_{CP}^{RA}\f$. Expressed in units of the reference symbol time \f$\kappa\f$.
  phy_time_unit cp_length;
  /// Flag: true if the preamble supports the restricted sets A and B.
  bool support_restricted_sets;
};

/// \brief Get long PRACH preamble information as per TS38.211 Table 6.3.3.1-1.
///
/// An assertion is triggered if the PRACH preamble format is not long.
///
/// \param[in] format PRACH preamble format.
/// \return PRACH preamble information.
prach_preamble_information get_prach_preamble_long_info(preamble_format format);

/// \brief Collects PRACH preamble duration information.
struct prach_symbols_slots_duration {
  /// Duration of the PRACH Preamble in slots, with reference to the PUSCH SCS.
  unsigned prach_length_slots;
  /// PRACH starting slot within the subframe, with reference to the PUSCH SCS.
  unsigned start_slot_pusch_scs;
  /// PRACH duration in symbols, with reference to the PUSCH SCS.
  unsigned nof_symbols;
  /// PRACH starting symbol within the slot, with reference to the PUSCH SCS.
  unsigned start_symbol_pusch_scs;
};

/// \brief Compute PRACH preamble duration information.
///
/// \param[in] prach_cfg PRACH preamble configuration.
/// \param[in] pusch_scs PUSCH SCS that is used as a reference for symbol and slot unit.
/// \return PRACH preamble duration information.
prach_symbols_slots_duration get_prach_duration_info(const prach_configuration& prach_cfg,
                                                     subcarrier_spacing         pusch_scs);

} // namespace srsran
