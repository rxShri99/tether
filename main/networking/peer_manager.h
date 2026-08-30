#pragma once
#include <cstdint>
#include "tracking/proximity.h"

namespace tether {

struct PeerState {
    uint32_t id = 0;
    uint8_t mac[6] = {};
    int8_t rawRssi = -127;
    float smoothedRssi = -90.0f;
    uint32_t lastSeenMs = 0;
    uint16_t lastSeq = 0;
    uint32_t received = 0;
    uint32_t missed = 0;
    uint8_t staleStreak = 0;
    bool online = false;
    ProximityLevel level = PROX_OUT_OF_RANGE;

    float packetLossPct() const
    {
        uint32_t total = received + missed;
        return total ? (100.0f * missed) / total : 0.0f;
    }
};

void peersInit();

/* Returns false for duplicates / stale sequence numbers (drop the packet). */
bool peersOnPacket(uint32_t id, const uint8_t mac[6], uint16_t seq, int8_t rssi, uint32_t nowMs);

/* Offline detection + proximity level decay. Call periodically. */
void peersTick(uint32_t nowMs);

bool peersGet(uint32_t id, PeerState &out);
int peersList(PeerState *out, int maxCount);

} // namespace tether
