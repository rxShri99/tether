#include "tracking/proximity.h"
#include "config/device_config.h"

namespace tether {

/* Minimum smoothed RSSI to qualify for each level (FOUND..FAR). */
static float levelFloor(ProximityLevel l)
{
    switch (l) {
    case PROX_FOUND:      return TH_FOUND;
    case PROX_VERY_CLOSE: return TH_VERY_CLOSE;
    case PROX_CLOSE:      return TH_CLOSE;
    case PROX_NEAR:       return TH_NEAR;
    case PROX_FAR:        return TH_FAR;
    default:              return -127.0f;
    }
}

static ProximityLevel classify(float rssi)
{
    if (rssi >= TH_FOUND) return PROX_FOUND;
    if (rssi >= TH_VERY_CLOSE) return PROX_VERY_CLOSE;
    if (rssi >= TH_CLOSE) return PROX_CLOSE;
    if (rssi >= TH_NEAR) return PROX_NEAR;
    if (rssi >= TH_FAR) return PROX_FAR;
    return PROX_OUT_OF_RANGE;
}

ProximityLevel proximityUpdate(ProximityLevel current, float rssi, bool online)
{
    if (!online) return PROX_OUT_OF_RANGE;

    ProximityLevel candidate = classify(rssi);
    if (candidate == current) return current;

    if (candidate < current) {
        /* moving closer: must clear the candidate's floor by the hysteresis margin */
        return (rssi >= levelFloor(candidate) + PROX_HYSTERESIS_DB) ? candidate : current;
    }
    /* moving away: must fall below the current level's floor by the margin */
    return (rssi < levelFloor(current) - PROX_HYSTERESIS_DB) ? candidate : current;
}

const char *proximityLabel(ProximityLevel level)
{
    switch (level) {
    case PROX_FOUND:      return "FOUND";
    case PROX_VERY_CLOSE: return "VERY CLOSE";
    case PROX_CLOSE:      return "CLOSE";
    case PROX_NEAR:       return "NEAR";
    case PROX_FAR:        return "FAR";
    default:              return "OUT OF RANGE";
    }
}

} // namespace tether
