#ifndef _SG_FILTER_H_
#define _SG_FILTER_H_

#include <stdbool.h>
#include <stdint.h>

#define SG_FILTER_MAX_WINDOW 15
#define SG_FILTER_MAX_ORDER 5

typedef struct sgFilter_s {
    int windowSize;
    int order;
    float sampleRateHz;
    float buf[SG_FILTER_MAX_WINDOW];
    float coeffs[SG_FILTER_MAX_WINDOW];
    int index;
    bool primed;
    float prev;
} sgFilter_t;

void sgFilterInitDiff(sgFilter_t *filter, float sampleRateHz);
void sgFilterInit(sgFilter_t *filter, int windowSize, int order, float sampleRateHz);
float sgFilterApplyDiff(sgFilter_t *filter, float input);
float sgFilterApply(sgFilter_t *filter, float input);

#endif
