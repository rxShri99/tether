#pragma once
#include <cstdint>
#include <cstddef>

namespace tether {

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint32_t BROADCAST_ID = 0;

enum MessageType : uint8_t {
    MSG_DISCOVERY = 0,
    MSG_HEARTBEAT,
    MSG_PAIR_REQUEST,
    MSG_PAIR_CONFIRM,
    MSG_PROXIMITY,
    MSG_SOS,
    MSG_SOS_CLEAR,
    MSG_RELAY,
    MSG_SWEEP_START,
    MSG_SWEEP_SAMPLE,
    MSG_PING,
    MSG_ACK,
    MSG_UNPAIR, /* hub reset: wearables drop their onboarded state */
    MSG_TYPE_COUNT,
};

#pragma pack(push, 1)
struct TetherPacket {
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint32_t sourceId;
    uint32_t destinationId; /* BROADCAST_ID = everyone */
    uint8_t ttl;
    uint32_t uptimeMs;
    union {
        struct { char name[16]; uint8_t role; } discovery;
        struct { uint32_t bumpAgeMs; } pair;
        struct { int8_t rssi; uint8_t level; } proximity;
        struct { uint8_t active; } sos;
        uint8_t raw[20];
    } payload;
};
#pragma pack(pop)

static_assert(sizeof(TetherPacket) <= 250, "must fit an ESP-NOW frame");

inline const char *messageTypeName(uint8_t t)
{
    static const char *names[] = {
        "DISCOVERY", "HEARTBEAT", "PAIR_REQUEST", "PAIR_CONFIRM", "PROXIMITY",
        "SOS", "SOS_CLEAR", "RELAY", "SWEEP_START", "SWEEP_SAMPLE", "PING", "ACK",
        "UNPAIR",
    };
    return t < MSG_TYPE_COUNT ? names[t] : "?";
}

} // namespace tether
