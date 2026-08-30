#pragma once
#include <cstdint>

/*
 * Passive 360° signal sweep (stretch feature — must never break core).
 *
 * RSSI samples are tagged with the gyro heading they were heard at and
 * dropped into 12 angular bins, one track per beacon:
 *
 *   FRIEND — the other wearable. Short TTL: the friend walks, so old samples
 *            stop describing where they are.
 *   HUB    — the stationary event hub. Long TTL: it never moves, so its
 *            bearing stays valid until gyro drift erodes it.
 *
 * Whenever both tracks are confidently estimated from the same spin, the
 * friend's bearing RELATIVE to the hub is remembered. When the friend track
 * later goes stale, the arrow is re-anchored as hubBearing + relative — so a
 * single spin keeps a usable arrow for minutes, hub permitting. With the hub
 * off, everything degrades to exactly the old friend-only behavior.
 *
 * This is not RF direction finding; bearings live in the gyro frame and we
 * refuse to claim confidence we don't have.
 */
namespace tether {

enum SweepTrack : uint8_t {
    SWEEP_TRACK_FRIEND = 0,
    SWEEP_TRACK_HUB = 1,
    SWEEP_TRACK_COUNT,
};

struct SweepEstimate {
    bool valid = false;      /* confident: coverage + margin thresholds met */
    bool guess = false;      /* at least some data: bearingDeg is best-effort */
    float bearingDeg = 0.0f; /* in the gyro yaw frame */
    float marginDb = 0.0f;
    int binsCovered = 0;
};

/* Composite friend direction with the hub-anchor fallback applied. */
struct FriendBearing {
    bool valid = false;       /* confident (fresh sweep or hub-anchored) */
    bool hubAnchored = false; /* true when served via the hub anchor */
    float bearingDeg = 0.0f;
};

void sweepAddSample(SweepTrack track, float yawDeg, int8_t rssi, uint32_t nowMs);
SweepEstimate sweepGetEstimate(SweepTrack track, uint32_t nowMs);
FriendBearing sweepGetFriendBearing(uint32_t nowMs);

} // namespace tether
