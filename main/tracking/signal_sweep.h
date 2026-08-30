#pragma once
#include <cstdint>

/*
 * Passive 360° signal sweep (stretch feature — must never break core).
 *
 * RSSI samples are tagged with the gyro heading they were heard at and
 * dropped into 12 angular bins. Once enough of the circle is covered, the
 * strongest broad sector gives an approximate RELATIVE bearing. This is not
 * RF direction finding; the arrow is only valid relative to the gyro frame
 * and we refuse to show one without sufficient confidence.
 */
namespace tether {

struct SweepEstimate {
    bool valid = false;      /* confident: coverage + margin thresholds met */
    bool guess = false;      /* at least some data: bearingDeg is best-effort */
    float bearingDeg = 0.0f; /* in the gyro yaw frame */
    float marginDb = 0.0f;
    int binsCovered = 0;
};

void sweepAddSample(float yawDeg, int8_t rssi, uint32_t nowMs);
SweepEstimate sweepGetEstimate(uint32_t nowMs);

} // namespace tether
