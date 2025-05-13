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

#pragma GCC diagnostic push
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wall"
#else // __clang__
#pragma GCC diagnostic ignored "-Wsuggest-override"
#endif // __clang__
#include <limesuiteng/limesuiteng.hpp>
#include "radio_lime_handle.h"
#pragma GCC diagnostic pop


// TODO: remove
struct RxGainRow {
    int lna;
    int pga;
};

// TODO: remove
struct TxGainRow {
    int main;
    int lin;
};

namespace lime
{
    static inline int64_t ts_to_time(int64_t fs, int64_t ts)
    {
        int n, r;
        n = (ts / fs);
        r = (ts % fs);
        return (int64_t)n * 1000000 + (((int64_t)r * 1000000) / fs);
    }

    typedef struct {
        unsigned stream_id;
        void* state_fsm;
        void* notifier;
    } callback_info_t;
}
