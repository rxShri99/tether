#pragma once
#include <cstdint>
#include "networking/peer_manager.h"

/*
 * Phase 7 -- hub dashboard on the ESP32-S3-Touch-AMOLED-1.8 V2 (CO5300 368x448,
 * CST820 touch). Hub-only; every entry point is a no-op in a wearable build so
 * the shared sources still compile for role 0.
 */
namespace tether {

/* Brings up the AMOLED via the Waveshare BSP, LVGL, touch, and the dashboard. */
bool hubUiInit();

/* Refresh the peer tiles. Safe to call from the app task; takes the LVGL lock. */
void hubUiSetPeers(const PeerState *peers, int count);

/* Counters shown in the footer. */
void hubUiSetStats(uint32_t rxTotal, uint32_t relayed);

/* Heading/tilt readout -- also the flip test rig for the tilt-compensated IMU. */
void hubUiSetImu(float yawDeg, float tiltDeg, bool ready);

/* Full-screen SOS overlay -- the hub must show this the moment a wearable fires. */
void hubUiSetSos(bool active, uint32_t fromId);

} // namespace tether
