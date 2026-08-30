#pragma once
#include <cstdint>
#include "networking/protocol.h"

/*
 * Hub onboarding (wearable side). Until onboarded, the home screen shows
 * "TAP THE HUB". Holding the wearable against the hub makes the hub's RSSI
 * exceed ONBOARD_RSSI_DB; the wearable then sends PAIR_REQUEST to the hub
 * until the hub answers PAIR_CONFIRM. RAM-only: re-tap after a reboot.
 */
namespace tether {

void onboardTick(uint32_t nowMs);
void onboardOnPacket(const TetherPacket &pkt, uint32_t nowMs);
bool onboardDone();

} // namespace tether
