#
# Copyright 2021-2025 Software Radio Systems Limited
#
# This file is part of srsRAN
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

add_library(srsran_pusch_processor STATIC
        factories.cpp
        pusch_codeblock_decoder.cpp
        pusch_decoder_empty_impl.cpp
        pusch_decoder_hw_impl.cpp
        pusch_decoder_impl.cpp
        pusch_demodulator_impl.cpp
        pusch_processor_impl.cpp
        pusch_processor_phy_capabilities.cpp
        pusch_processor_validator_impl.cpp
        ulsch_demultiplex_impl.cpp)
set(SRSRAN_PUSCH_LIBS srsran_upper_phy_support srsran_ran)

target_link_libraries(srsran_pusch_processor ${SRSRAN_PUSCH_LIBS})
