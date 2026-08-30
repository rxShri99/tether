#include "hub/hub_ui.h"
#include "config/device_config.h"

#if TETHER_ROLE == TETHER_ROLE_HUB

#include "tracking/signal_sweep.h"
#include "sensors/imu.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "bsp/esp-bsp.h"
#include "lvgl.h"

namespace tether {
namespace {

const char *TAG = "hub_ui";

/*
 * Touch alignment.
 *
 * The Waveshare BSP shifts the *display* by x_gap = 16 px on this panel
 * revision but configures the CST820 with no matching offset, so touch lands
 * slightly off from the pixels you see. The correction goes in through
 * esp_lcd_touch's process_coordinates hook -- inside the driver, before LVGL
 * sees a sample.
 *
 * The offset is read from NVS under the same namespace/keys the bring-up
 * firmware used, so an alignment already dialled in on this board carries over
 * (the nvs partition sits at the same offset in both partition tables, so it
 * survives reflashing). A stored value beyond +/-40 px is a bad measurement
 * rather than a calibration and is refused -- an earlier automated five-point
 * routine once wrote -67,-73 and made the panel unusable.
 */
constexpr const char *NVS_NAMESPACE = "granola";
constexpr const char *NVS_KEY_DX = "touch_dx";
constexpr const char *NVS_KEY_DY = "touch_dy";
constexpr int TOUCH_OFFSET_LIMIT = 40;

int s_touchDx = 0;
int s_touchDy = 0;

// Widgets
lv_obj_t *s_uptimeLabel = nullptr;
lv_obj_t *s_statsLabel = nullptr;
lv_obj_t *s_imuLabel = nullptr;
lv_obj_t *s_sosOverlay = nullptr;
lv_obj_t *s_sosLabel = nullptr;

struct PeerTile {
    uint32_t id = 0;
    bool connected = false; /* tap-to-connect onboarding done */
    lv_obj_t *card = nullptr;
    lv_obj_t *name = nullptr;
    lv_obj_t *status = nullptr;
    lv_obj_t *detail = nullptr;
    lv_obj_t *bar = nullptr;
};

/* The hub watches every known device that is not itself. */
constexpr int MAX_TILES = 4;
PeerTile s_tiles[MAX_TILES];
int s_tileCount = 0;

void processCoordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y,
                        uint16_t *strength, uint8_t *point_num,
                        uint8_t max_point_num)
{
    (void)tp;
    (void)strength;
    (void)max_point_num;

    if (point_num == nullptr || x == nullptr || y == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < *point_num; i++) {
        int nx = static_cast<int>(x[i]) + s_touchDx;
        int ny = static_cast<int>(y[i]) + s_touchDy;

        if (nx < 0) { nx = 0; }
        if (ny < 0) { ny = 0; }
        if (nx > BSP_LCD_H_RES - 1) { nx = BSP_LCD_H_RES - 1; }
        if (ny > BSP_LCD_V_RES - 1) { ny = BSP_LCD_V_RES - 1; }

        x[i] = static_cast<uint16_t>(nx);
        y[i] = static_cast<uint16_t>(ny);
    }
}

/*
 * esp_lvgl_port stores lvgl_port_touch_ctx_t as the indev driver data, and its
 * first member is the esp_lcd_touch handle. Reading it this way avoids
 * vendoring a private struct definition.
 */
esp_lcd_touch_handle_t touchHandleFromIndev(lv_indev_t *indev)
{
    if (indev == nullptr) {
        return nullptr;
    }
    void *ctx = lv_indev_get_driver_data(indev);
    if (ctx == nullptr) {
        return nullptr;
    }
    return *static_cast<esp_lcd_touch_handle_t *>(ctx);
}

void loadTouchOffset()
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored touch offset; using 0,0");
        return;
    }

    int32_t dx = 0, dy = 0;
    const bool ok = (nvs_get_i32(h, NVS_KEY_DX, &dx) == ESP_OK) &&
                    (nvs_get_i32(h, NVS_KEY_DY, &dy) == ESP_OK);
    nvs_close(h);

    if (!ok) {
        ESP_LOGI(TAG, "no stored touch offset; using 0,0");
        return;
    }
    if (abs(static_cast<int>(dx)) > TOUCH_OFFSET_LIMIT ||
        abs(static_cast<int>(dy)) > TOUCH_OFFSET_LIMIT) {
        ESP_LOGW(TAG, "stored offset (%d,%d) exceeds +/-%d -- ignoring",
                 static_cast<int>(dx), static_cast<int>(dy), TOUCH_OFFSET_LIMIT);
        return;
    }

    s_touchDx = static_cast<int>(dx);
    s_touchDy = static_cast<int>(dy);
    ESP_LOGI(TAG, "loaded touch offset %+d,%+d", s_touchDx, s_touchDy);
}

// ------------------------------------------------------------- find screen

/*
 * Tap a connected tile -> full-screen sweep arrow for that wearable, driven by
 * the hub's own QMI8658. Pick the hub up and rotate it like a wearable. The
 * hub reuses the two sweep tracks: track 0 = first wearable, track 1 = second.
 */
lv_obj_t *s_findScreen = nullptr;
lv_obj_t *s_findName = nullptr;
lv_obj_t *s_findDetail = nullptr;
lv_obj_t *s_findLines[3] = {};
uint32_t s_findId = 0;

SweepTrack trackFor(uint32_t id)
{
    return id == KNOWN_DEVICES[0].id ? SWEEP_TRACK_FRIEND : SWEEP_TRACK_HUB;
}

void findArrowDraw(float angleDeg)
{
    static lv_point_precise_t pts[3][2];
    const float cx = 184.0f, cy = 230.0f;
    const float rad = angleDeg * (float)M_PI / 180.0f;
    const float dx = sinf(rad), dy = -cosf(rad);
    const float tipX = cx + 92.0f * dx, tipY = cy + 92.0f * dy;

    pts[0][0] = {(lv_value_precise_t)(cx - 50.0f * dx), (lv_value_precise_t)(cy - 50.0f * dy)};
    pts[0][1] = {(lv_value_precise_t)tipX, (lv_value_precise_t)tipY};
    const float headLen = 44.0f, spread = 32.0f * (float)M_PI / 180.0f;
    for (int side = 0; side < 2; side++) {
        float a = rad + (float)M_PI + (side ? spread : -spread);
        pts[1 + side][0] = {(lv_value_precise_t)tipX, (lv_value_precise_t)tipY};
        pts[1 + side][1] = {(lv_value_precise_t)(tipX + headLen * sinf(a)),
                            (lv_value_precise_t)(tipY - headLen * cosf(a))};
    }
    for (int i = 0; i < 3; i++) lv_line_set_points(s_findLines[i], pts[i], 2);
}

void findTimerCb(lv_timer_t *)
{
    if (s_findScreen == nullptr || lv_obj_has_flag(s_findScreen, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }
    static float dispAngle = 0.0f, drawnAngle = 1e9f, lastBearing = 0.0f;

    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    SweepEstimate est = sweepGetEstimate(trackFor(s_findId), now);
    if (est.valid || est.guess) lastBearing = est.bearingDeg;

    uint32_t colour = est.valid ? 0xf2f4f8 : (est.guess ? 0x8a8f9c : 0x3a3f4d);
    for (auto *l : s_findLines) lv_obj_set_style_line_color(l, lv_color_hex(colour), 0);

    float target = lastBearing - SWEEP_YAW_SIGN * imuGetYawDeg();
    float delta = fmodf(target - dispAngle, 360.0f);
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    dispAngle += delta * 0.25f;
    if (fabsf(dispAngle - drawnAngle) > 0.7f) {
        drawnAngle = dispAngle;
        findArrowDraw(dispAngle);
    }
}

void findBackCb(lv_event_t *)
{
    lv_obj_add_flag(s_findScreen, LV_OBJ_FLAG_HIDDEN);
}

void buildFindScreen()
{
    s_findScreen = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_findScreen, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_pos(s_findScreen, 0, 0);
    lv_obj_set_style_radius(s_findScreen, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_findScreen, lv_color_hex(0x0b0e13), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_findScreen, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_findScreen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_findScreen, LV_OBJ_FLAG_HIDDEN);

    s_findName = lv_label_create(s_findScreen);
    lv_obj_set_style_text_color(s_findName, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_findName, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(s_findName, LV_ALIGN_TOP_MID, 0, 24);

    s_findDetail = lv_label_create(s_findScreen);
    lv_label_set_text(s_findDetail, "rotate the hub to locate");
    lv_obj_set_style_text_color(s_findDetail, lv_color_hex(0x8b93a7), LV_PART_MAIN);
    lv_obj_align(s_findDetail, LV_ALIGN_TOP_MID, 0, 62);

    for (auto &l : s_findLines) {
        l = lv_line_create(s_findScreen);
        lv_obj_set_pos(l, 0, 0);
        lv_obj_set_size(l, BSP_LCD_H_RES, BSP_LCD_V_RES);
        lv_obj_set_style_line_width(l, 14, 0);
        lv_obj_set_style_line_rounded(l, true, 0);
        lv_obj_set_style_line_color(l, lv_color_hex(0x3a3f4d), 0);
    }
    findArrowDraw(0.0f);

    lv_obj_t *back = lv_button_create(s_findScreen);
    lv_obj_set_size(back, 160, 52);
    lv_obj_align(back, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_bg_color(back, lv_color_hex(0x2a3040), LV_PART_MAIN);
    lv_obj_add_event_cb(back, findBackCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *backLabel = lv_label_create(back);
    lv_label_set_text(backLabel, "BACK");
    lv_obj_center(backLabel);

    lv_timer_create(findTimerCb, 60, nullptr);
}

void tileClickCb(lv_event_t *e)
{
    const uint32_t id = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
    for (int i = 0; i < s_tileCount; i++) {
        if (s_tiles[i].id == id && !s_tiles[i].connected) {
            return; /* find is available once the device has onboarded */
        }
    }
    if (s_findScreen == nullptr) buildFindScreen();
    s_findId = id;
    char text[48];
    snprintf(text, sizeof(text), "FIND %s", deviceName(id));
    lv_label_set_text(s_findName, text);
    lv_obj_clear_flag(s_findScreen, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------- widgets

lv_obj_t *makeCard(lv_obj_t *parent, int y, int h)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, 336, h);
    lv_obj_align(card, LV_ALIGN_TOP_MID, 0, y);
    lv_obj_set_style_radius(card, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x171a21), LV_PART_MAIN);
    lv_obj_set_style_border_color(card, lv_color_hex(0x2a3040), LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 12, LV_PART_MAIN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

void buildPeerTile(lv_obj_t *screen, int index, uint32_t id, int y)
{
    PeerTile &t = s_tiles[index];
    t.id = id;
    t.card = makeCard(screen, y, 116);

    t.name = lv_label_create(t.card);
    lv_label_set_text(t.name, deviceName(id));
    lv_obj_set_style_text_color(t.name, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(t.name, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(t.name, LV_ALIGN_TOP_LEFT, 0, 0);

    t.status = lv_label_create(t.card);
    lv_label_set_text(t.status, "OFFLINE");
    lv_obj_set_style_text_color(t.status, lv_color_hex(0x8b93a7), LV_PART_MAIN);
    lv_obj_set_style_text_font(t.status, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(t.status, LV_ALIGN_TOP_RIGHT, 0, 2);

    t.detail = lv_label_create(t.card);
    lv_label_set_text(t.detail, "no signal");
    lv_obj_set_style_text_color(t.detail, lv_color_hex(0xc7cedb), LV_PART_MAIN);
    lv_obj_align(t.detail, LV_ALIGN_TOP_LEFT, 0, 32);

    lv_obj_add_flag(t.card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(t.card, tileClickCb, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(static_cast<uintptr_t>(id)));

    /* Signal strength bar: -90 dBm (empty) .. -35 dBm (full). */
    t.bar = lv_bar_create(t.card);
    lv_obj_set_size(t.bar, 312, 10);
    lv_obj_align(t.bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_bar_set_range(t.bar, -90, -35);
    lv_bar_set_value(t.bar, -90, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(t.bar, lv_color_hex(0x2a3040), LV_PART_MAIN);
    lv_obj_set_style_bg_color(t.bar, lv_color_hex(0x2f9bff), LV_PART_INDICATOR);
}

void buildSosOverlay()
{
    s_sosOverlay = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_sosOverlay, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_pos(s_sosOverlay, 0, 0);
    lv_obj_set_style_radius(s_sosOverlay, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_sosOverlay, lv_color_hex(0xc0130b), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_sosOverlay, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_sosOverlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_sosOverlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *title = lv_label_create(s_sosOverlay);
    lv_label_set_text(title, "SOS");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -40);

    s_sosLabel = lv_label_create(s_sosOverlay);
    lv_label_set_text(s_sosLabel, "");
    lv_obj_set_style_text_color(s_sosLabel, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_sosLabel, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(s_sosLabel, LV_ALIGN_CENTER, 0, 20);
}

void buildDashboard()
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0b0e13), LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "TETHER HUB");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    char sub[64];
    snprintf(sub, sizeof(sub), "id %lu  ch %u  espnow",
             static_cast<unsigned long>(DEVICE_ID),
             static_cast<unsigned>(WIFI_CHANNEL));
    lv_obj_t *subtitle = lv_label_create(screen);
    lv_label_set_text(subtitle, sub);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x8b93a7), LV_PART_MAIN);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 46);

    /* One tile per known device other than the hub itself. */
    int y = 76;
    for (const auto &d : KNOWN_DEVICES) {
        if (d.id == DEVICE_ID || s_tileCount >= MAX_TILES) {
            continue;
        }
        buildPeerTile(screen, s_tileCount, d.id, y);
        s_tileCount++;
        y += 126;
    }

    lv_obj_t *footer = makeCard(screen, y + 6, 100);
    s_statsLabel = lv_label_create(footer);
    lv_label_set_text(s_statsLabel, "rx 0   relayed 0");
    lv_obj_set_style_text_color(s_statsLabel, lv_color_hex(0xc7cedb), LV_PART_MAIN);
    lv_obj_align(s_statsLabel, LV_ALIGN_TOP_LEFT, 0, 0);

    s_imuLabel = lv_label_create(footer);
    lv_label_set_text(s_imuLabel, "imu --");
    lv_obj_set_style_text_color(s_imuLabel, lv_color_hex(0x8bd3ff), LV_PART_MAIN);
    lv_obj_align(s_imuLabel, LV_ALIGN_TOP_LEFT, 0, 26);

    s_uptimeLabel = lv_label_create(footer);
    lv_label_set_text(s_uptimeLabel, "uptime 0s");
    lv_obj_set_style_text_color(s_uptimeLabel, lv_color_hex(0x8b93a7), LV_PART_MAIN);
    lv_obj_align(s_uptimeLabel, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    buildSosOverlay();
}

void uptimeTimerCb(lv_timer_t *timer)
{
    (void)timer;
    if (s_uptimeLabel == nullptr) {
        return;
    }
    const uint32_t s = static_cast<uint32_t>(esp_timer_get_time() / 1000000);
    char text[48];
    snprintf(text, sizeof(text), "uptime %luh %02lum %02lus",
             static_cast<unsigned long>(s / 3600),
             static_cast<unsigned long>((s / 60) % 60),
             static_cast<unsigned long>(s % 60));
    lv_label_set_text(s_uptimeLabel, text);
}

} // namespace

// ---------------------------------------------------------------- api

bool hubUiInit()
{
    lv_display_t *disp = bsp_display_start();
    if (disp == nullptr) {
        ESP_LOGE(TAG, "display init failed");
        return false;
    }
    bsp_display_brightness_set(80);

    loadTouchOffset();

    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "could not take the LVGL lock");
        return false;
    }

    lv_indev_t *indev = bsp_display_get_input_dev();
    esp_lcd_touch_handle_t tp = touchHandleFromIndev(indev);
    if (tp != nullptr) {
        tp->config.process_coordinates = processCoordinates;
        ESP_LOGI(TAG, "touch coordinate hook installed");
    } else {
        ESP_LOGW(TAG, "touch handle unreachable; running uncorrected");
    }

    buildDashboard();
    lv_timer_create(uptimeTimerCb, 1000, nullptr);
    bsp_display_unlock();

    ESP_LOGI(TAG, "dashboard up (%d peer tiles)", s_tileCount);
    return true;
}

void hubUiSetPeers(const PeerState *peers, int count)
{
    if (s_tileCount == 0 || !bsp_display_lock(200)) {
        return;
    }

    for (int i = 0; i < s_tileCount; i++) {
        PeerTile &t = s_tiles[i];

        const PeerState *found = nullptr;
        for (int j = 0; j < count; j++) {
            if (peers[j].id == t.id) {
                found = &peers[j];
                break;
            }
        }

        if (found == nullptr || !found->online) {
            lv_label_set_text(t.status, "OFFLINE");
            lv_obj_set_style_text_color(t.status, lv_color_hex(0x8b93a7), LV_PART_MAIN);
            lv_label_set_text(t.detail, found ? "last seen: signal lost" : "never seen");
            lv_bar_set_value(t.bar, -90, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(t.bar, lv_color_hex(0x4a5163), LV_PART_INDICATOR);
            continue;
        }

        if (!t.connected) {
            lv_label_set_text(t.status, "NOT CONNECTED");
            lv_obj_set_style_text_color(t.status, lv_color_hex(0xff9f0a), LV_PART_MAIN);
            lv_label_set_text(t.detail, "hold the device on the hub\nto connect");
        } else {
            lv_label_set_text(t.status, "CONNECTED");
            lv_obj_set_style_text_color(t.status, lv_color_hex(0x34c759), LV_PART_MAIN);

            char detail[96];
            snprintf(detail, sizeof(detail), "%s   %.0f dBm   loss %.0f%%\ntap to find",
                     proximityLabel(found->level),
                     found->smoothedRssi, found->packetLossPct());
            lv_label_set_text(t.detail, detail);
        }

        int rssi = static_cast<int>(found->smoothedRssi);
        if (rssi < -90) { rssi = -90; }
        if (rssi > -35) { rssi = -35; }
        lv_bar_set_value(t.bar, rssi, LV_ANIM_ON);

        /* Green when comfortably in range, amber mid, red at the edge. */
        uint32_t colour = 0x34c759;
        if (found->smoothedRssi < TH_NEAR) { colour = 0xff9f0a; }
        if (found->smoothedRssi < TH_FAR)  { colour = 0xff3b30; }
        lv_obj_set_style_bg_color(t.bar, lv_color_hex(colour), LV_PART_INDICATOR);
    }

    bsp_display_unlock();
}

void hubUiSetStats(uint32_t rxTotal, uint32_t relayed)
{
    if (s_statsLabel == nullptr || !bsp_display_lock(200)) {
        return;
    }
    char text[64];
    snprintf(text, sizeof(text), "rx %lu   relayed %lu",
             static_cast<unsigned long>(rxTotal),
             static_cast<unsigned long>(relayed));
    lv_label_set_text(s_statsLabel, text);
    bsp_display_unlock();
}

void hubUiSetImu(float yawDeg, float tiltDeg, bool ready)
{
    if (s_imuLabel == nullptr || !bsp_display_lock(200)) {
        return;
    }
    char text[80];
    if (!ready) {
        snprintf(text, sizeof(text), "imu calibrating...");
    } else {
        /* tilt > 90 means the board is face down -- the case that used to
           make the bearing count backwards. */
        snprintf(text, sizeof(text), "yaw %+.0f  tilt %.0f  %s",
                 static_cast<double>(yawDeg), static_cast<double>(tiltDeg),
                 tiltDeg > 90.0f ? "FACE DOWN" : "face up");
    }
    lv_label_set_text(s_imuLabel, text);
    bsp_display_unlock();
}

void hubUiSetSos(bool active, uint32_t fromId)
{
    if (s_sosOverlay == nullptr || !bsp_display_lock(200)) {
        return;
    }
    if (active) {
        char text[48];
        snprintf(text, sizeof(text), "%s needs help", deviceName(fromId));
        lv_label_set_text(s_sosLabel, text);
        lv_obj_move_foreground(s_sosOverlay); /* above the find screen */
        lv_obj_clear_flag(s_sosOverlay, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_sosOverlay, LV_OBJ_FLAG_HIDDEN);
    }
    bsp_display_unlock();
}

void hubUiSetConnected(uint32_t id, bool connected)
{
    if (s_tileCount == 0 || !bsp_display_lock(200)) {
        return;
    }
    for (int i = 0; i < s_tileCount; i++) {
        if (s_tiles[i].id == id) {
            s_tiles[i].connected = connected;
            break;
        }
    }
    bsp_display_unlock();
}

} // namespace tether

#else // wearable build -- keep the shared sources compiling

namespace tether {
bool hubUiInit() { return true; }
void hubUiSetPeers(const PeerState *, int) {}
void hubUiSetStats(uint32_t, uint32_t) {}
void hubUiSetImu(float, float, bool) {}
void hubUiSetSos(bool, uint32_t) {}
void hubUiSetConnected(uint32_t, bool) {}
} // namespace tether

#endif
