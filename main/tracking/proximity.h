#pragma once
#include <cstdint>

namespace tether {

/* Semantic proximity only — never shown as physical distance. */
enum ProximityLevel : uint8_t {
    PROX_FOUND = 0,
    PROX_VERY_CLOSE,
    PROX_CLOSE,
    PROX_NEAR,
    PROX_FAR,
    PROX_OUT_OF_RANGE,
};

/* Applies thresholds from device_config.h with hysteresis. */
ProximityLevel proximityUpdate(ProximityLevel current, float smoothedRssi, bool online);

const char *proximityLabel(ProximityLevel level);

} // namespace tether
