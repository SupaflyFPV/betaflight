/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#ifdef USE_DTERM_CHEBY3_FILTER

// Fixed 3rd-order Chebyshev Type II D-term filter for 8 kHz loop.
// Notch at ~260 Hz with selectable stopband attenuation.
// Implemented as biquad + first-order cascade.

typedef enum {
    DTERM_CHEBY3_RS23 = 0, // ~2.0 ms latency, lightest suppression
    DTERM_CHEBY3_RS25 = 1, // ~2.2 ms latency, mid option
    DTERM_CHEBY3_RS27 = 2, // ~2.5 ms latency, strongest suppression
} dtermCheby3Stopband_e;

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float d1, d2;
} chebyBiquad_t;

typedef struct {
    float b0, b1;
    float a1;
    float d1;
} chebyFirstOrder_t;

typedef struct {
    chebyBiquad_t     biquad;
    chebyFirstOrder_t first;
} dtermCheby3Filter_t;

void dtermCheby3InitVariant(dtermCheby3Filter_t *f, dtermCheby3Stopband_e stopband);
void dtermCheby3Init(dtermCheby3Filter_t *f, dtermCheby3Stopband_e stopband);
float dtermCheby3Apply(dtermCheby3Filter_t *f, float input);

#endif // USE_DTERM_CHEBY3_FILTER
