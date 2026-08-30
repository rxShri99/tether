#include "tracking/signal_sweep.h"
#include "config/device_config.h"

#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace tether {

constexpr int BINS = 12;
constexpr float BIN_DEG = 360.0f / BINS;
constexpr int SAMPLES_PER_BIN = 8;

struct Sample {
    int8_t rssi;
    uint32_t ms;
};

static Sample s_bins[BINS][SAMPLES_PER_BIN];
static uint8_t s_binHead[BINS];
static SemaphoreHandle_t s_lock;

static void ensureLock()
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

void sweepAddSample(float yawDeg, int8_t rssi, uint32_t nowMs)
{
    ensureLock();
    float wrapped = fmodf(yawDeg, 360.0f);
    if (wrapped < 0) wrapped += 360.0f;
    int bin = (int)(wrapped / BIN_DEG) % BINS;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_bins[bin][s_binHead[bin]] = {rssi, nowMs};
    s_binHead[bin] = (s_binHead[bin] + 1) % SAMPLES_PER_BIN;
    xSemaphoreGive(s_lock);
}

/* trimmed mean of a bin's fresh samples; NAN if under-populated */
static float binValue(int bin, uint32_t nowMs)
{
    float vals[SAMPLES_PER_BIN];
    int n = 0;
    for (int i = 0; i < SAMPLES_PER_BIN; i++) {
        const Sample &s = s_bins[bin][i];
        if (s.ms != 0 && nowMs - s.ms < SWEEP_SAMPLE_TTL_MS) vals[n++] = s.rssi;
    }
    if (n < 2) return NAN;
    if (n >= 4) { /* drop min and max */
        int lo = 0, hi = 0;
        for (int i = 1; i < n; i++) {
            if (vals[i] < vals[lo]) lo = i;
            if (vals[i] > vals[hi]) hi = i;
        }
        float sum = 0;
        for (int i = 0; i < n; i++) {
            if (i != lo && i != hi) sum += vals[i];
        }
        return sum / (n - 2);
    }
    float sum = 0;
    for (int i = 0; i < n; i++) sum += vals[i];
    return sum / n;
}

SweepEstimate sweepGetEstimate(uint32_t nowMs)
{
    ensureLock();
    SweepEstimate est;

    float raw[BINS];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < BINS; i++) raw[i] = binValue(i, nowMs);
    xSemaphoreGive(s_lock);

    int covered = 0;
    for (int i = 0; i < BINS; i++) {
        if (!std::isnan(raw[i])) covered++;
    }
    est.binsCovered = covered;
    if (covered == 0) return est;

    /* circular smoothing [0.25 0.5 0.25], skipping empty neighbors */
    float smooth[BINS];
    for (int i = 0; i < BINS; i++) {
        if (std::isnan(raw[i])) { smooth[i] = NAN; continue; }
        float sum = 0.5f * raw[i], w = 0.5f;
        float l = raw[(i + BINS - 1) % BINS], r = raw[(i + 1) % BINS];
        if (!std::isnan(l)) { sum += 0.25f * l; w += 0.25f; }
        if (!std::isnan(r)) { sum += 0.25f * r; w += 0.25f; }
        smooth[i] = sum / w;
    }

    int best = -1;
    for (int i = 0; i < BINS; i++) {
        if (!std::isnan(smooth[i]) && (best < 0 || smooth[i] > smooth[best])) best = i;
    }
    /* runner-up must not be adjacent to the winning sector */
    float second = -1000.0f;
    for (int i = 0; i < BINS; i++) {
        if (std::isnan(smooth[i])) continue;
        int d = abs(i - best);
        if (d <= 1 || d >= BINS - 1) continue;
        if (smooth[i] > second) second = smooth[i];
    }

    /* refine within the winning sector: weight best bin and neighbors */
    float floor = (second > -999.0f ? second : smooth[best] - 6.0f) - 1.0f;
    float cx = 0, cy = 0;
    for (int off = -1; off <= 1; off++) {
        int i = (best + off + BINS) % BINS;
        if (std::isnan(smooth[i])) continue;
        float wgt = smooth[i] - floor;
        if (wgt <= 0) continue;
        float ang = (i + 0.5f) * BIN_DEG * (float)M_PI / 180.0f;
        cx += wgt * cosf(ang);
        cy += wgt * sinf(ang);
    }
    est.bearingDeg = atan2f(cy, cx) * 180.0f / (float)M_PI;
    if (est.bearingDeg < 0) est.bearingDeg += 360.0f;
    est.guess = true;

    est.marginDb = second > -999.0f ? smooth[best] - second : 0.0f;
    est.valid = covered >= SWEEP_MIN_BINS && second > -999.0f && est.marginDb >= SWEEP_MARGIN_DB;
    return est;
}

} // namespace tether
