#pragma once

#include "pg/pg.h"

typedef struct tlConfig_s {
    uint8_t gain;   // 0-100 user scale
    uint8_t shape;  // 0-100 user scale (inverted)
    float maxGain;  // 1.0 - 5.0 (0.1 step)
    float shapeBoost; // 0.5 - 3.0 exponent for gain shaping
} tlConfig_t;

PG_DECLARE(tlConfig_t, tlConfig);
