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

/*
 * RESET on the dashboard: forget all onboarded devices and broadcast
 * MSG_UNPAIR so the wearables fall back to "TAP THE HUB".
 * (Tile visuals are reset by the caller, which runs in the LVGL task.)
 */
void hubResetConnections();

} // namespace tether
