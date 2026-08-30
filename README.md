# Tether

Offline friend-finding network for crowded events. Built for Granola Hardware Hack Day.

## Hardware

| Device | Board | Role |
|---|---|---|
| Wearable A "TANVEER" (id 1) | Waveshare ESP32-S3-Touch-LCD-1.46 | user device |
| Wearable B "ALEX" (id 2) | Waveshare ESP32-S3-Touch-LCD-1.46 | user device |
| Hub (id 100) | Waveshare ESP32-S3-Touch-AMOLED-1.8 V2 | event dashboard / relay |

Wearables talk directly over ESP-NOW (channel 1). The hub is optional — the
demo works with it switched off.

## Build & flash

ESP-IDF v5.5.2 (checkout lives at `../esp-idf`). One firmware, three build
configs via compile-time identity:

```sh
tools/flash.sh a          # wearable TANVEER
tools/flash.sh b          # wearable ALEX
tools/flash.sh hub        # event hub
tools/flash.sh a /dev/cu.usbmodemXXXX   # explicit port
```

Monitor: `idf.py -B build_a -p <port> monitor` (exit with Ctrl+]).

## Structure

- `main/config/` — device identity, timing, RSSI thresholds (calibrate in venue!)
- `main/networking/` — protocol, ESP-NOW transport (all esp_now calls isolated here), peer manager (RSSI EMA, dup suppression, online/offline)
- `main/tracking/` — proximity categories with hysteresis; signal sweep (Phase 9)
- `main/ui/` — SPD2010 display + LVGL; screens under `ui/screens/`
- `main/sensors/`, `main/audio/` — QMI8658 IMU and PCM5101 (later phases)
- `main/hub/` — hub dashboard + relay

## Phase status

- [x] Phase 1 — wearable displays via official drivers (esp_lcd_spd2010 + LVGL)
- [x] Phase 2 — ESP-NOW heartbeats with peer/seq/RSSI logging *(needs 2 boards to verify)*
- [x] Phase 3 — RSSI smoothing + proximity categories (code in place, venue calibration pending)
- [ ] Phase 4 — consumer-quality proximity UI (basic ring/labels done; polish pending)
- [ ] Phase 5 — SOS
- [ ] Phase 6 — bump pairing
- [ ] Phase 7 — hub dashboard (serial-only stub today)
- [ ] Phase 8 — hub relay
- [ ] Phase 9 — 360° signal sweep (stretch)
- [ ] Phase 10 — polish
- [ ] Phase 11 — Wi-Fi FTM (optional)

## Hackathon rule

Tag every working phase (`git tag phaseN-working`). Never demo from an
untagged state.
