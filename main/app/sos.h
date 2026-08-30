#pragma once
#include <cstdint>
#include "networking/protocol.h"

/*
 * SOS state machine (wearable). Trigger: hold the BOOT button ~1.5s.
 * Hold again to cancel. Short-press dismisses an incoming alert (the home
 * screen underneath is Find mode). Outgoing SOS re-broadcasts every second —
 * that is also what keeps the hub's overlay refreshed and gets relayed for
 * reach. The receiver ACKs so the sender can show "delivered".
 */
namespace tether {

enum SosState : uint8_t {
    SOS_IDLE = 0,
    SOS_OUTGOING,
    SOS_INCOMING,
};

struct SosStatus {
    SosState state;
    uint32_t peerId;  /* who we alerted / who is calling */
    bool delivered;   /* outgoing: friend ACKed */
};

void sosInit();
void sosOnPacket(const TetherPacket &pkt, uint32_t nowMs);
void sosTick(uint32_t nowMs);
SosStatus sosGet();

} // namespace tether
