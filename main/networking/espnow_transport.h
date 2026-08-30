#pragma once
#include <cstdint>
#include "networking/protocol.h"

/*
 * Transport layer: all ESP-NOW calls live here. UI and app logic never touch
 * esp_now directly. RX happens on the Wi-Fi task; packets are copied into a
 * queue and consumed from the app task via transportReceive().
 */
namespace tether {

struct RxEvent {
    TetherPacket pkt;
    uint8_t mac[6];
    int8_t rssi;
    uint8_t len;
};

bool transportInit();

/* Fills version/sourceId/sequence/uptimeMs. mac=nullptr -> broadcast. */
bool transportSend(TetherPacket &pkt, const uint8_t *mac = nullptr);

/* Forward a packet unmodified except caller-adjusted fields (used by relay). */
bool transportForward(const TetherPacket &pkt);

/* Blocks up to waitMs. Returns true if an event was dequeued. */
bool transportReceive(RxEvent &ev, uint32_t waitMs);

/* Register a unicast peer MAC (needed before esp_now unicast send). */
bool transportAddPeer(const uint8_t mac[6]);

} // namespace tether
