#pragma once
#include <cstdint>
#include "networking/protocol.h"

namespace tether {

/* Hub role: AMOLED dashboard (Phase 7) + relay (Phase 8). */
bool hubInit();

/*
 * Called from the app loop for every packet that survived version, self and
 * duplicate filtering. Handles the SOS overlay and the relay hop.
 * No-op in a wearable build.
 */
void hubOnPacket(const TetherPacket &pkt, int8_t rssi);

} // namespace tether
