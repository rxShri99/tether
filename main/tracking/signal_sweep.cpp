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

static Sample s_bins[SWEEP_TRACK_COUNT][BINS][SAMPLES_PER_BIN];
static uint8_t s_binHead[SWEEP_TRACK_COUNT][BINS];
static SemaphoreHandle_t s_lock;

/* friend bearing relative to the hub, captured when both tracks were valid */
static bool s_relValid = false;
static float s_relDeg = 0.0f;
static uint32_t s_relMs = 0;

static uint32_t trackTtl(SweepTrack t)
{
    return t == SWEEP_TRACK_HUB ? SWEEP_HUB_TTL_MS : SWEEP_SAMPLE_TTL_MS;
}

static void ensureLock()
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

void sweepAddSample(SweepTrack track, float yawDeg, int8_t rssi, uint32_t nowMs)
{
    if (track >= SWEEP_TRACK_COUNT) return;
    ensureLock();
    float wrapped = fmodf(yawDeg, 360.0f);
    if (wrapped < 0) wrapped += 360.0f;
    int bin = (int)(wrapped / BIN_DEG) % BINS;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_bins[track][bin][s_binHead[track][bin]] = {rssi, nowMs};
    s_binHead[track][bin] = (s_binHead[track][bin] + 1) % SAMPLES_PER_BIN;
    xSemaphoreGive(s_lock);
}

/* trimmed mean of a bin's fresh samples; NAN if under-populated */
static float binValue(SweepTrack track, int bin, uint32_t nowMs)
{
    float vals[SAMPLES_PER_BIN];
    int n = 0;
    for (int i = 0; i < SAMPLES_PER_BIN; i++) {
        const Sample &s = s_bins[track][bin][i];
        if (s.ms != 0 && nowMs - s.ms < trackTtl(track)) vals[n++] = s.rssi;
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

SweepEstimate sweepGetEstimate(SweepTrack track, uint32_t nowMs)
{
    ensureLock();
    SweepEstimate est;
    if (track >= SWEEP_TRACK_COUNT) return est;

    float raw[BINS];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < BINS; i++) raw[i] = binValue(track, i, nowMs);
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

FriendBearing sweepGetFriendBearing(uint32_t nowMs)
{
    FriendBearing fb;
    SweepEstimate fr = sweepGetEstimate(SWEEP_TRACK_FRIEND, nowMs);
    SweepEstimate hub = sweepGetEstimate(SWEEP_TRACK_HUB, nowMs);

    if (fr.valid) {
        fb.valid = true;
        fb.bearingDeg = fr.bearingDeg;
        if (hub.valid) { /* both fresh from the same spin: (re)learn the anchor */
            s_relDeg = fmodf(fr.bearingDeg - hub.bearingDeg + 360.0f, 360.0f);
            s_relMs = nowMs;
            s_relValid = true;
        }
        return fb;
    }

    /* friend track stale: serve hubBearing + rel while the anchor is young.
       The hub never moves, so its long-TTL bearing is still trustworthy;
       drift affects both bearings equally and cancels in the difference. */
    if (hub.valid && s_relValid && nowMs - s_relMs < SWEEP_REL_TTL_MS) {
        fb.valid = true;
        fb.hubAnchored = true;
        fb.bearingDeg = fmodf(hub.bearingDeg + s_relDeg, 360.0f);
    }
    return fb;
}

} // namespace tether
