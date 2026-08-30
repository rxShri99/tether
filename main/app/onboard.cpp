#include "app/onboard.h"
#include "config/device_config.h"
#include "networking/espnow_transport.h"
#include "networking/peer_manager.h"
#include "audio/audio.h"

#include "esp_log.h"

namespace tether {

static const char *TAG = "onboard";

static bool s_done = false;
static bool s_armed = true; /* after an unpair, require leaving the hub first */
static uint32_t s_lastReqMs = 0;

void onboardTick(uint32_t nowMs)
{
    if (s_done) return;

    PeerState hub;
    if (!peersGet(HUB_ID, hub) || !hub.online) return;
    if (!s_armed) {
        /* hub pressed RESET while we were still lying on it: re-arm only
           once the device has actually been taken away */
        if (hub.smoothedRssi < ONBOARD_RSSI_DB - 8.0f) s_armed = true;
        return;
    }
    if (hub.smoothedRssi < ONBOARD_RSSI_DB) return;
    if (nowMs - s_lastReqMs < ONBOARD_RESEND_MS) return;

    s_lastReqMs = nowMs;
    TetherPacket pkt = {};
    pkt.type = MSG_PAIR_REQUEST;
    pkt.destinationId = HUB_ID;
    pkt.ttl = DEFAULT_TTL;
    transportSend(pkt);
    ESP_LOGI(TAG, "near hub (%.0f dBm) -> PAIR_REQUEST", hub.smoothedRssi);
}

void onboardOnPacket(const TetherPacket &pkt, uint32_t)
{
    if (pkt.type == MSG_UNPAIR && pkt.sourceId == HUB_ID) {
        if (s_done) {
            s_done = false;
            s_armed = false;
            audioPlay(TONE_LOST);
            ESP_LOGW(TAG, "hub reset: onboarding cleared");
        }
        return;
    }

    if (s_done) return;
    if (pkt.type != MSG_PAIR_CONFIRM) return;
    if (pkt.sourceId != HUB_ID || pkt.destinationId != DEVICE_ID) return;

    s_done = true;
    audioPlay(TONE_PAIRED);
    ESP_LOGI(TAG, "onboarded with hub");
}

bool onboardDone() { return s_done; }

} // namespace tether
