#include "app/onboard.h"
#include "config/device_config.h"
#include "networking/espnow_transport.h"
#include "networking/peer_manager.h"
#include "audio/audio.h"

#include "esp_log.h"

namespace tether {

static const char *TAG = "onboard";

static bool s_done = false;
static uint32_t s_lastReqMs = 0;

void onboardTick(uint32_t nowMs)
{
    if (s_done) return;

    PeerState hub;
    if (!peersGet(HUB_ID, hub) || !hub.online) return;
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
    if (s_done) return;
    if (pkt.type != MSG_PAIR_CONFIRM) return;
    if (pkt.sourceId != HUB_ID || pkt.destinationId != DEVICE_ID) return;

    s_done = true;
    audioPlay(TONE_PAIRED);
    ESP_LOGI(TAG, "onboarded with hub");
}

bool onboardDone() { return s_done; }

} // namespace tether
