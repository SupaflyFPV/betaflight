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

#include "platform.h"

#ifdef USE_DTERM_CHEBY3_FILTER

#include "flight/dterm_cheby3_filter.h"

static inline float chebyBiquadApply(chebyBiquad_t *f, float in)
{
    const float out = f->b0 * in + f->d1;

    f->d1 = f->b1 * in - f->a1 * out + f->d2;
    f->d2 = f->b2 * in - f->a2 * out;

    return out;
}

static inline float chebyFirstOrderApply(chebyFirstOrder_t *f, float in)
{
    const float out = f->b0 * in + f->d1;
    f->d1 = f->b1 * in - f->a1 * out;
    return out;
}

void dtermCheby3InitVariant(dtermCheby3Filter_t *f, dtermCheby3Stopband_e stopband)
{
    switch (stopband) {
    case DTERM_CHEBY3_RS27:
        // Rs = 27 dB
        f->biquad.b0 =  0.01091985f;
        f->biquad.b1 = -0.02138593f;
        f->biquad.b2 =  0.01091985f;
        f->biquad.a1 = -1.91003668f;
        f->biquad.a2 =  0.91884836f;

        f->first.b0 =  1.0f;
        f->first.b1 =  1.0f;
        f->first.a1 = -0.89700867f;
        break;
    case DTERM_CHEBY3_RS25:
        // Rs = 25 dB
        f->biquad.b0 =  0.01365603f;
        f->biquad.b1 = -0.02674459f;
        f->biquad.b2 =  0.01365603f;
        f->biquad.a1 = -1.90492303f;
        f->biquad.a2 =  0.91503092f;

        f->first.b0 =  1.0f;
        f->first.b1 =  1.0f;
        f->first.a1 = -0.88771886f;
        break;
    case DTERM_CHEBY3_RS23:
    default:
        // Rs = 23 dB
        f->biquad.b0 =  0.01707484f;
        f->biquad.b1 = -0.03344016f;
        f->biquad.b2 =  0.01707484f;
        f->biquad.a1 = -1.89991231f;
        f->biquad.a2 =  0.91148046f;

        f->first.b0 =  1.0f;
        f->first.b1 =  1.0f;
        f->first.a1 = -0.87733077f;
        break;
    }

    f->biquad.d1 = 0.0f;
    f->biquad.d2 = 0.0f;
    f->first.d1 = 0.0f;
}

void dtermCheby3Init(dtermCheby3Filter_t *f, dtermCheby3Stopband_e stopband)
{
    dtermCheby3InitVariant(f, stopband);
}

float dtermCheby3Apply(dtermCheby3Filter_t *f, float input)
{
    const float x = chebyBiquadApply(&f->biquad, input);
    return chebyFirstOrderApply(&f->first, x);
}

#endif // USE_DTERM_CHEBY3_FILTER
