# Tether — Build Plan

Offline friend-finding network for crowded events. Granola Hardware Hack Day.

Two wearables that always work peer-to-peer; an optional hub that observes and
relays. **The demo must survive: no internet, hub off, sweep disabled, FTM
disabled.**

## Hardware

| Device | Board | Identity |
|---|---|---|
| Wearable A | Waveshare ESP32-S3-Touch-LCD-1.46 (SPD2010 412×412 round, QMI8658, PCM5101) | id 1 · TANVEER |
| Wearable B | Waveshare ESP32-S3-Touch-LCD-1.46 | id 2 · SHRI |
| Event hub | Waveshare ESP32-S3-Touch-AMOLED-1.8 V2 (CO5300 368×448, CST820, ES8311) | id 100 · HUB |

## Architecture

```
Wearable A <---- ESP-NOW (ch 1, broadcast) ----> Wearable B
        \                                       /
         `--------------- EVENT HUB ----------'
                (observe + optional relay)
```

- **ESP-IDF v5.5.2** with official registry drivers (`esp_lcd_spd2010`,
  `esp_lcd_touch_spd2010`, `esp_lvgl_port`, LVGL 9). Chosen over
  Arduino/PlatformIO because the SPD2010 QSPI panel + TCA9554 reset path is
  proven here and ESP-NOW RX metadata (RSSI) is first-class.
- One firmware, three build configs (`tools/flash.sh a|b|hub`) via
  `TETHER_DEVICE_ID` / `TETHER_ROLE` compile definitions.
- Layering: UI never touches `esp_now` — `networking/espnow_transport` isolates
  radio calls, RX is queued to the app task; `peer_manager` owns per-peer state
  (EMA RSSI, dup suppression, loss, online/offline); `tracking/` turns signal
  into semantics (proximity categories, sweep bearing).
- Packet: packed 37-byte `TetherPacket` — version, type, seq, sourceId,
  destId, TTL, uptime, payload union. Heartbeats @200ms, discovery @2s.

## Phase plan

Build strictly in order. Tag every working phase (`phaseN-working`); never
demo from an untagged state.

| Phase | Deliverable | Acceptance | Status |
|---|---|---|---|
| 1 | Both 1.46 displays via official drivers | screens reliable | ✅ done (`phase1-working`) |
| 2 | ESP-NOW comms | A↔B continuous; log peer/seq/RSSI | ✅ done, verified both directions (`phase2-working`) |
| 3 | RSSI smoothing + proximity categories | walking apart changes FAR/NEAR/CLOSE | ✅ done; observed FOUND→CLOSE→NEAR live (`phase3-working`) |
| 4 | Consumer-quality proximity UI | understandable with zero explanation | 🔶 in progress — ring/labels/arrow shipped; touch fix + Find mode (WARMER/COLDER trend) + polish remain |
| 5 | SOS | long-press → instant full-screen alert + tone on peer | ⬜ next |
| 6 | Bump pairing | physical bump pairs devices (touch fallback) | ⬜ |
| 7 | Hub dashboard (AMOLED) | hub shows both devices online/offline + telemetry | ⬜ serial-only stub exists |
| 8 | Hub relay | forward TTL>0 packets, dup-safe, no loops | ⬜ dup cache already in peer_manager |
| 9 | 360° signal sweep | gyro bins RSSI over one rotation; arrow only when confident | 🔶 shipped early (passive mode) — venue validation + `SWEEP_YAW_SIGN`/margin tuning pending |
| 10 | Polish | animations, tones, transitions, branding, hide debug | ⬜ |
| 11 | Wi-Fi FTM (optional) | isolated experiment only | ⬜ never in core demo |

## Current state (2026-08-30)

Verified on both physical wearables:

- Displays + LVGL UI live; ESP-NOW bidirectional at ~5 pkt/s with 0–7% loss;
  proximity categories tracking real movement with hysteresis.
- QMI8658 up on both (0x6B); passive direction arrow implemented — awaiting
  physical spin test at distance.
- Debug overlay on-screen: raw/smoothed RSSI, loss, yaw, accel magnitude.

Known issues / risks:

- **SPD2010 touch** fails fw-version read on the new I2C master API — touch
  disabled. Needed for FIND/SOS buttons (Phase 4/5). Fallback: minimal custom
  I2C touch reader (protocol is simple). Bump pairing (Phase 6) reduces
  touch dependence.
- Gyro bias read exactly 0.000 on both boards — confirm real data via the
  on-screen `yaw` readout before trusting the sweep.
- Direction arrow depends on body-shadowing RSSI contrast — needs several
  meters of separation; correctly refuses (`SIGNAL UNCLEAR`) when flat.
- RSSI thresholds (`config/device_config.h`) must be calibrated in the venue.

## Demo script (target)

1. Bump both wearables → pairing animation + tone.
2. Walk apart → proximity label degrades automatically.
3. Find mode → WARMER/COLDER while walking; FOUND celebration + sound.
4. Spin in place → direction arrow points toward friend.
5. Trigger SOS → instant full-screen alert on the other wearable; hub
   dashboard shows the SOS simultaneously.
6. Switch the hub off → everything above still works.
