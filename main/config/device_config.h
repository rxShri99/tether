#pragma once
#include <cstdint>

/*
 * Compile-time device identity. Set from the build system:
 *   idf.py -DTETHER_DEVICE_ID=2 -DTETHER_ROLE=0 -B build_b build
 * (see tools/flash.sh)
 */
#define TETHER_ROLE_WEARABLE 0
#define TETHER_ROLE_HUB      1

#ifndef TETHER_DEVICE_ID
#define TETHER_DEVICE_ID 1
#endif
#ifndef TETHER_ROLE
#define TETHER_ROLE TETHER_ROLE_WEARABLE
#endif

namespace tether {

struct DeviceInfo {
    uint32_t id;
    const char *name;
};

constexpr DeviceInfo KNOWN_DEVICES[] = {
    {1, "TANVEER"},
    {2, "SHRI"},
    {100, "HUB"},
};

constexpr uint32_t DEVICE_ID = TETHER_DEVICE_ID;
constexpr uint32_t HUB_ID = 100;
/* Each wearable has exactly one friend in this two-device demo */
constexpr uint32_t FRIEND_ID = (TETHER_ROLE == TETHER_ROLE_HUB) ? 0 : (DEVICE_ID == 1 ? 2u : 1u);

inline const char *deviceName(uint32_t id)
{
    for (const auto &d : KNOWN_DEVICES) {
        if (d.id == id) return d.name;
    }
    return "?";
}

/* ---- Timing ---- */
constexpr uint32_t HEARTBEAT_PERIOD_MS = 200;
constexpr uint32_t DISCOVERY_EVERY_N_HEARTBEATS = 10; /* -> every ~2s */
constexpr uint32_t PEER_OFFLINE_TIMEOUT_MS = 1200;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint8_t DEFAULT_TTL = 2;

/* ---- RSSI / proximity (calibrate these in the venue!) ---- */
constexpr float RSSI_EMA_ALPHA = 0.2f;    /* smoothed = a*raw + (1-a)*prev */
constexpr float PROX_HYSTERESIS_DB = 3.0f;
constexpr float TH_FOUND = -40.0f;
constexpr float TH_VERY_CLOSE = -45.0f;
constexpr float TH_CLOSE = -55.0f;
constexpr float TH_NEAR = -67.0f;
constexpr float TH_FAR = -78.0f;

/* ---- SOS ---- */
constexpr uint32_t SOS_LONGPRESS_MS = 1500; /* BOOT hold to raise/cancel */
constexpr uint32_t SOS_RESEND_MS = 1000;    /* keeps hub overlay refreshed too */
constexpr uint32_t SOS_TIMEOUT_MS = 20000;  /* incoming expires if sender silent */

/* ---- 360° signal sweep (stretch; direction is RELATIVE to gyro frame) ---- */
constexpr uint32_t SWEEP_SAMPLE_TTL_MS = 20000; /* friend samples expire; they walk */
constexpr uint32_t SWEEP_HUB_TTL_MS = 90000;    /* hub is stationary: bearings age well */
constexpr uint32_t SWEEP_REL_TTL_MS = 180000;   /* friend-vs-hub anchor validity */
constexpr int SWEEP_MIN_BINS = 8;               /* of 12 — coverage before estimating */
constexpr float SWEEP_MARGIN_DB = 4.0f;         /* best sector must beat runner-up by this */
/*
 * Screen arrow angle is clockwise-from-up; gyro yaw is counterclockwise-
 * positive. -1 makes the arrow counter-rotate against device rotation so it
 * holds a world-fixed direction (flip to +1 if it rotates backwards).
 */
constexpr float SWEEP_YAW_SIGN = -1.0f;

} // namespace tether
