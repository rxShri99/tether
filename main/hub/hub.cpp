#include "hub/hub.h"
#include "hub/hub_ui.h"
#include "config/device_config.h"
#include "networking/espnow_transport.h"
#include "networking/peer_manager.h"
#include "sensors/imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#if TETHER_ROLE == TETHER_ROLE_HUB
#include "bsp/esp-bsp.h"
#endif

namespace tether {

#if TETHER_ROLE == TETHER_ROLE_HUB

namespace {

const char *TAG = "hub";

/* An SOS that stops being refreshed should not pin the overlay on forever. */
constexpr uint32_t SOS_HOLD_MS = 20000;

uint32_t s_rxTotal = 0;
uint32_t s_relayed = 0;
uint32_t s_connectedMask = 0; /* onboarded device ids (bit = id) */
bool s_sosActive = false;
uint32_t s_sosFrom = 0;
uint32_t s_sosLastMs = 0;

uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

/*
 * Relay policy: forward only messages whose *delivery* matters and whose RSSI
 * does not.
 *
 * Heartbeats and discovery are deliberately never relayed. A wearable derives
 * proximity from the RSSI of the packet it receives, so a relayed copy landing
 * first would have it measuring the hub's link instead of its friend's -- and
 * duplicate suppression would then discard the genuine packet. Proximity would
 * read wrong whenever the hub is powered and right when it is off, which is
 * precisely backwards for a demo built on "the hub is optional".
 *
 * SOS and pairing carry no RSSI meaning, and for those extra reach is the
 * whole point of having a hub.
 */
bool shouldRelay(uint8_t type)
{
    switch (type) {
        case MSG_SOS:
        case MSG_SOS_CLEAR:
        case MSG_PAIR_REQUEST:
        case MSG_PAIR_CONFIRM:
        case MSG_RELAY:
            return true;
        default:
            return false;
    }
}

void dashboardTask(void *)
{
    while (true) {
        PeerState peers[8];
        const int n = peersList(peers, 8);

        hubUiSetPeers(peers, n);
        hubUiSetStats(s_rxTotal, s_relayed);
        hubUiSetImu(imuGetYawDeg(), imuGetTiltDeg(), imuReady());

        /* Expire a stale SOS so the overlay cannot stick after the wearable
           stops transmitting (e.g. it walked out of range mid-alert). */
        if (s_sosActive && (nowMs() - s_sosLastMs) > SOS_HOLD_MS) {
            s_sosActive = false;
            hubUiSetSos(false, 0);
            ESP_LOGW(TAG, "SOS from %lu expired after %lums with no refresh",
                     static_cast<unsigned long>(s_sosFrom),
                     static_cast<unsigned long>(SOS_HOLD_MS));
        }

        /* Keep the serial view -- useful when the panel is not in front of you. */
        printf("\n== TETHER HUB ==  devices: %d  rx=%lu relayed=%lu\n", n,
               static_cast<unsigned long>(s_rxTotal),
               static_cast<unsigned long>(s_relayed));
        for (int i = 0; i < n; i++) {
            printf("  %-10s %-8s rssi=%.1f loss=%.0f%% %s\n",
                   deviceName(peers[i].id),
                   peers[i].online ? "ONLINE" : "OFFLINE",
                   peers[i].smoothedRssi, peers[i].packetLossPct(),
                   proximityLabel(peers[i].level));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

} // namespace

void hubOnPacket(const TetherPacket &pkt, int8_t rssi)
{
    (void)rssi;
    s_rxTotal++;

    switch (pkt.type) {
        case MSG_SOS: {
            const bool active = pkt.payload.sos.active != 0;
            s_sosLastMs = nowMs();
            if (active != s_sosActive || pkt.sourceId != s_sosFrom) {
                s_sosActive = active;
                s_sosFrom = pkt.sourceId;
                hubUiSetSos(active, pkt.sourceId);
                ESP_LOGW(TAG, "SOS %s from %lu (%s)", active ? "RAISED" : "cleared",
                         static_cast<unsigned long>(pkt.sourceId),
                         deviceName(pkt.sourceId));
            }
            break;
        }
        case MSG_SOS_CLEAR:
            if (s_sosActive) {
                s_sosActive = false;
                hubUiSetSos(false, 0);
                ESP_LOGI(TAG, "SOS cleared by %lu",
                         static_cast<unsigned long>(pkt.sourceId));
            }
            break;
        case MSG_PAIR_REQUEST: {
            /* tap-to-connect onboarding: the wearable is pressed against the
               hub. Mark connected and confirm (it retries until we answer). */
            const uint32_t bit = 1u << (pkt.sourceId & 31);
            if (!(s_connectedMask & bit)) {
                s_connectedMask |= bit;
                hubUiSetConnected(pkt.sourceId, true);
                ESP_LOGI(TAG, "%s connected (onboarded)", deviceName(pkt.sourceId));
            }
            TetherPacket confirm = {};
            confirm.type = MSG_PAIR_CONFIRM;
            confirm.destinationId = pkt.sourceId;
            confirm.ttl = DEFAULT_TTL;
            transportSend(confirm);
            break;
        }
        default:
            break;
    }

    /*
     * Phase 8 relay. The caller has already dropped our own packets and
     * anything peer_manager flagged as a duplicate, so forwarding here cannot
     * loop: TTL strictly decreases each hop and wearables never re-forward.
     * Packets addressed to the hub itself terminate here.
     */
    if (shouldRelay(pkt.type) && pkt.ttl > 0 && pkt.destinationId != DEVICE_ID) {
        TetherPacket fwd = pkt;
        fwd.ttl = static_cast<uint8_t>(pkt.ttl - 1);
        if (transportForward(fwd)) {
            s_relayed++;
        }
    }
}

void hubResetConnections()
{
    s_connectedMask = 0;
    for (int i = 0; i < 3; i++) { /* small burst for reliability */
        TetherPacket pkt = {};
        pkt.type = MSG_UNPAIR;
        pkt.destinationId = BROADCAST_ID;
        pkt.ttl = DEFAULT_TTL;
        transportSend(pkt);
    }
    ESP_LOGW(TAG, "connections reset: all devices unpaired");
}

bool hubInit()
{
    if (!hubUiInit()) {
        ESP_LOGE(TAG, "dashboard init failed -- falling back to serial only");
    }
    /* The hub carries the same QMI8658 as the wearables, so it doubles as a
       bench rig for the tilt-compensated heading: flip the board and watch
       yaw keep counting the same way. */
    imuInit(bsp_i2c_get_handle());

    xTaskCreate(dashboardTask, "hub_dash", 4096, nullptr, 3, nullptr);
    ESP_LOGI(TAG, "hub running: AMOLED dashboard + relay");
    return true;
}

#else

bool hubInit() { return true; }
void hubOnPacket(const TetherPacket &, int8_t) {}
void hubResetConnections() {}

#endif

} // namespace tether
