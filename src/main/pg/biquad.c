#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "biquad.h"

PG_REGISTER_WITH_RESET_TEMPLATE(biquadConfig_t, biquadConfig, PG_BIQUAD_CONFIG, 1);

PG_RESET_TEMPLATE(biquadConfig_t, biquadConfig,
    .biquad_response = 0,
);
