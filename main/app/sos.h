#pragma once
#include <cstdint>

/*
 * Minimal SOS: hold the BOOT button ~1.5s -> play the SOS tone locally and
 * broadcast a short burst of MSG_SOS packets. The hub shows its SOS overlay
 * and auto-clears ~20s after the last packet. No wearable UI is involved.
 */
namespace tether {

void sosInit();
void sosTick(uint32_t nowMs);

} // namespace tether
