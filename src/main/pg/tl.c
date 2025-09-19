/*
 * Thrust Linearization configuration
 */
#include "platform.h"

#ifdef USE_THRUST_LINEARIZATION

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "tl.h"

PG_REGISTER_WITH_RESET_TEMPLATE(tlConfig_t, tlConfig, PG_THRUST_LINEARIZATION_CONFIG, 4);

PG_RESET_TEMPLATE(tlConfig_t, tlConfig,
    .gain = 46,
    .shape = 58,
    .maxGain = 2.5f,
    .shapeBoost = 1.0f,
);

#endif // USE_THRUST_LINEARIZATION
