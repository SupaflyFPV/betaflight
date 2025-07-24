#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct biquadConfig_s {
    uint8_t biquad_response; // 0=Butterworth, 1=Bessel
} biquadConfig_t;

PG_DECLARE(biquadConfig_t, biquadConfig);
