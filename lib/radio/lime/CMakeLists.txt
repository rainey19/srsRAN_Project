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

set(SOURCES_LIME
        radio_config_lime_validator.cpp
        radio_lime_handle.cpp
        radio_lime_impl.cpp
        radio_lime_rx_stream.cpp
        radio_lime_tx_stream.cpp)
add_library(srsran_radio_lime STATIC ${SOURCES_LIME})

target_link_libraries(srsran_radio_lime ${LimeSuiteNG_LIBRARIES})

install(TARGETS srsran_radio_lime EXPORT srsran_export)