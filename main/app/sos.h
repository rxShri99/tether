#pragma once
#include <cstdint>
#include "networking/protocol.h"

/*
 * Minimal SOS: hold the BOOT button ~1.5s -> play the SOS tone, broadcast a
 * burst of MSG_SOS packets, and enter SOS mode for ~20s. A wearable receiving
 * SOS also beeps and enters SOS mode. SOS mode is shown as a static red
 * home-screen background (no animation — one redraw in, one redraw out).
 * The hub shows its own SOS view and auto-clears.
 */
namespace tether {

void sosInit();
void sosTick(uint32_t nowMs);
void sosOnPacket(const TetherPacket &pkt, uint32_t nowMs);

/* true while this device is in SOS mode (sent or received, ~20s window) */
bool sosActive();

} // namespace tether
