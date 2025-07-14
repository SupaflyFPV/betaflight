#include "platform.h"
#include "sg_filter.h"
#include <math.h>

static void invertMatrix(float *a, float *inv, int n)
{
    float temp[SG_FILTER_MAX_ORDER + 1][2*(SG_FILTER_MAX_ORDER + 1)];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            temp[i][j] = a[i*n + j];
            temp[i][j+n] = (i == j) ? 1.0f : 0.0f;
        }
    }
    for (int i = 0; i < n; i++) {
        float pivot = temp[i][i];
        if (pivot == 0.0f) {
            pivot = 1e-12f;
        }
        float invPivot = 1.0f / pivot;
        for (int j = 0; j < 2*n; j++) {
            temp[i][j] *= invPivot;
        }
        for (int k = 0; k < n; k++) {
            if (k == i) continue;
            float factor = temp[k][i];
            for (int j = 0; j < 2*n; j++) {
                temp[k][j] -= factor * temp[i][j];
            }
        }
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            inv[i*n + j] = temp[i][j+n];
        }
    }
}

void sgFilterInitDiff(sgFilter_t *filter, float sampleRateHz)
{
    filter->windowSize = 2;
    filter->order = 1;
    filter->sampleRateHz = sampleRateHz;
    filter->index = 0;
    filter->primed = false;
    filter->prev = 0.0f;
}

void sgFilterInit(sgFilter_t *filter, int windowSize, int order, float sampleRateHz)
{
    if (windowSize > SG_FILTER_MAX_WINDOW) {
        windowSize = SG_FILTER_MAX_WINDOW;
    }
    if (order > SG_FILTER_MAX_ORDER) {
        order = SG_FILTER_MAX_ORDER;
    }
    filter->windowSize = windowSize;
    filter->order = order;
    filter->sampleRateHz = sampleRateHz;
    filter->index = 0;
    filter->primed = false;
    for (int i = 0; i < windowSize; i++) {
        filter->buf[i] = 0.0f;
        filter->coeffs[i] = 0.0f;
    }

    const int m = windowSize;
    const int n = order;
    const int half = m / 2;

    float A[(SG_FILTER_MAX_ORDER + 1)*(SG_FILTER_MAX_ORDER + 1)] = {0};
    float ATA[(SG_FILTER_MAX_ORDER + 1)*(SG_FILTER_MAX_ORDER + 1)] = {0};

    float matA[SG_FILTER_MAX_WINDOW][SG_FILTER_MAX_ORDER + 1];
    for (int k = -half; k <= half; k++) {
        float p = 1.0f;
        for (int j = 0; j <= n; j++) {
            matA[k + half][j] = p;
            p *= k;
        }
    }
    // compute ATA
    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= n; j++) {
            float sum = 0.0f;
            for (int k = 0; k < m; k++) {
                sum += matA[k][i] * matA[k][j];
            }
            ATA[i*(n+1) + j] = sum;
        }
    }

    float ATAinv[(SG_FILTER_MAX_ORDER + 1)*(SG_FILTER_MAX_ORDER + 1)];
    invertMatrix(ATA, ATAinv, n+1);

    for (int k = 0; k < m; k++) {
        float sum = 0.0f;
        for (int j = 0; j <= n; j++) {
            sum += ATAinv[1*(n+1) + j] * matA[k][j];
        }
        filter->coeffs[k] = sum * filter->sampleRateHz;
    }
}

float sgFilterApplyDiff(sgFilter_t *filter, float input)
{
    float out = -(input - filter->prev) * filter->sampleRateHz;
    filter->prev = input;
    return out;
}

float sgFilterApply(sgFilter_t *filter, float input)
{
    filter->buf[filter->index] = input;
    filter->index++;
    if (filter->index >= filter->windowSize) {
        filter->index = 0;
        filter->primed = true;
    }
    if (!filter->primed) {
        return 0.0f;
    }
    float result = 0.0f;
    int idx = filter->index;
    for (int i = 0; i < filter->windowSize; i++) {
        result += filter->coeffs[i] * filter->buf[idx];
        idx++;
        if (idx >= filter->windowSize) idx = 0;
    }
    return result;
}

