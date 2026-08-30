#include "networking/peer_manager.h"
#include "config/device_config.h"

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace tether {

constexpr int MAX_PEERS = 8;

static PeerState s_peers[MAX_PEERS];
static int s_peerCount = 0;
static SemaphoreHandle_t s_lock = nullptr;

void peersInit()
{
    s_lock = xSemaphoreCreateMutex();
}

static PeerState *findOrCreate(uint32_t id)
{
    for (int i = 0; i < s_peerCount; i++) {
        if (s_peers[i].id == id) return &s_peers[i];
    }
    if (s_peerCount >= MAX_PEERS) return nullptr;
    PeerState *p = &s_peers[s_peerCount++];
    *p = PeerState{};
    p->id = id;
    return p;
}

bool peersOnPacket(uint32_t id, const uint8_t mac[6], uint16_t seq, int8_t rssi, uint32_t nowMs)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    PeerState *p = findOrCreate(id);
    if (!p) {
        xSemaphoreGive(s_lock);
        return false;
    }

    bool fresh = true;
    if (p->received > 0) {
        uint16_t delta = (uint16_t)(seq - p->lastSeq);
        if (delta == 0 || delta > 0x8000) {
            fresh = false; /* duplicate or stale (e.g. hub-relayed copy) */
        } else {
            p->missed += delta - 1;
        }
    }

    if (fresh) {
        memcpy(p->mac, mac, 6);
        p->lastSeq = seq;
        p->received++;
        p->rawRssi = rssi;
        p->smoothedRssi = p->online
            ? RSSI_EMA_ALPHA * rssi + (1.0f - RSSI_EMA_ALPHA) * p->smoothedRssi
            : (float)rssi; /* snap on (re)connect instead of averaging from stale value */
        p->lastSeenMs = nowMs;
        p->online = true;
        p->level = proximityUpdate(p->level, p->smoothedRssi, true);
        /* keep loss stats from growing unbounded */
        if (p->received + p->missed > 2000) {
            p->received /= 2;
            p->missed /= 2;
        }
    }
    xSemaphoreGive(s_lock);
    return fresh;
}

void peersTick(uint32_t nowMs)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_peerCount; i++) {
        PeerState *p = &s_peers[i];
        if (p->online && nowMs - p->lastSeenMs > PEER_OFFLINE_TIMEOUT_MS) {
            p->online = false;
            p->level = PROX_OUT_OF_RANGE;
        }
    }
    xSemaphoreGive(s_lock);
}

bool peersGet(uint32_t id, PeerState &out)
{
    bool found = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_peerCount; i++) {
        if (s_peers[i].id == id) {
            out = s_peers[i];
            found = true;
            break;
        }
    }
    xSemaphoreGive(s_lock);
    return found;
}

int peersList(PeerState *out, int maxCount)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = s_peerCount < maxCount ? s_peerCount : maxCount;
    for (int i = 0; i < n; i++) out[i] = s_peers[i];
    xSemaphoreGive(s_lock);
    return n;
}

} // namespace tether
