/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

// Response type for biquad lowpass filters
typedef enum {
    BIQUAD_LPF_BUTTERWORTH = 0,
    BIQUAD_LPF_BESSEL,
} biquadLpfResponse_e;

typedef struct filterConfig_s {
    uint8_t biquad_lpf_response; // butterworth or bessel
} filterConfig_t;

PG_DECLARE(filterConfig_t, filterConfig);

