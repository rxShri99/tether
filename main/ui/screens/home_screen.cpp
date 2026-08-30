#include "ui/screens/home_screen.h"
#include "config/device_config.h"
#include "networking/peer_manager.h"
#include "tracking/signal_sweep.h"
#include "sensors/imu.h"

#include <cstdio>
#include <initializer_list>
#include <cmath>
#include "esp_timer.h"

namespace tether {

static lv_obj_t *s_dot;
static lv_obj_t *s_ring;
static lv_obj_t *s_nameLabel;
static lv_obj_t *s_proxLabel;
static lv_obj_t *s_debugLabel;
static lv_obj_t *s_arrowShaft;
static lv_obj_t *s_arrowHeadL;
static lv_obj_t *s_arrowHeadR;

static lv_color_t levelColor(ProximityLevel level, bool online)
{
    if (!online) return lv_color_hex(0x555a66);
    switch (level) {
    case PROX_FOUND:      return lv_color_hex(0x35e08a);
    case PROX_VERY_CLOSE: return lv_color_hex(0x35e08a);
    case PROX_CLOSE:      return lv_color_hex(0x4cc9f0);
    case PROX_NEAR:       return lv_color_hex(0xffc53d);
    case PROX_FAR:        return lv_color_hex(0xff8c42);
    default:              return lv_color_hex(0x555a66);
    }
}

static void ringAnimCb(void *var, int32_t v)
{
    lv_obj_t *ring = (lv_obj_t *)var;
    lv_obj_set_size(ring, v, v);
    lv_obj_center(ring);
    /* fade out as the ring expands (v: 160 -> 320); kept subtle behind the arrow */
    int32_t opa = (255 - ((v - 160) * 255) / 160) / 4;
    lv_obj_set_style_border_opa(ring, (lv_opa_t)(opa < 0 ? 0 : opa), 0);
}

/* ---- AirPods-style direction arrow (relative, gyro+RSSI sweep) ---- */

static void arrowSetColor(lv_color_t color, lv_opa_t opa)
{
    for (lv_obj_t *o : {s_arrowShaft, s_arrowHeadL, s_arrowHeadR}) {
        lv_obj_set_style_line_color(o, color, 0);
        lv_obj_set_style_line_opa(o, opa, 0);
    }
}

static void arrowDraw(float angleDeg)
{
    static lv_point_precise_t shaft[2], headL[2], headR[2];
    const float cx = 206.0f, cy = 206.0f;
    const float rad = angleDeg * (float)M_PI / 180.0f;
    /* screen coords: 0° = up */
    const float dx = sinf(rad), dy = -cosf(rad);
    const float tipX = cx + 110.0f * dx, tipY = cy + 110.0f * dy;

    shaft[0] = {(lv_value_precise_t)(cx - 60.0f * dx), (lv_value_precise_t)(cy - 60.0f * dy)};
    shaft[1] = {(lv_value_precise_t)tipX, (lv_value_precise_t)tipY};

    const float headLen = 55.0f, spread = 32.0f * (float)M_PI / 180.0f;
    for (int side = 0; side < 2; side++) {
        float a = rad + (float)M_PI + (side ? spread : -spread);
        lv_point_precise_t *line = side ? headR : headL;
        line[0] = {(lv_value_precise_t)tipX, (lv_value_precise_t)tipY};
        line[1] = {(lv_value_precise_t)(tipX + headLen * sinf(a)),
                   (lv_value_precise_t)(tipY - headLen * cosf(a))};
    }
    lv_line_set_points(s_arrowShaft, shaft, 2);
    lv_line_set_points(s_arrowHeadL, headL, 2);
    lv_line_set_points(s_arrowHeadR, headR, 2);
}

static void arrowTimerCb(lv_timer_t *)
{
    static float dispAngle = 0.0f;
    static float lastBearing = 0.0f; /* remembered across estimate dropouts */

    PeerState peer;
    bool online = peersGet(FRIEND_ID, peer) && peer.online;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    FriendBearing fb = sweepGetFriendBearing(now);
    SweepEstimate est = sweepGetEstimate(SWEEP_TRACK_FRIEND, now);

    /* arrow is always on: fresh sweep -> white, hub-anchored -> warm white,
       best-guess -> dimmed, no data / offline -> dark, holding last bearing */
    if (fb.valid) lastBearing = fb.bearingDeg;
    else if (est.guess) lastBearing = est.bearingDeg;

    if (!online) {
        arrowSetColor(lv_color_hex(0x3a3f4d), LV_OPA_60);
    } else if (fb.valid && !fb.hubAnchored) {
        arrowSetColor(lv_color_hex(0xf2f4f8), LV_OPA_COVER);
    } else if (fb.valid) {
        arrowSetColor(lv_color_hex(0xffe9c2), LV_OPA_90); /* hub-anchored */
    } else {
        arrowSetColor(lv_color_hex(0x8a8f9c), LV_OPA_70);
    }

    /* point at the bearing relative to where the user faces right now */
    static float drawnAngle = 1e9f;
    float target = lastBearing - SWEEP_YAW_SIGN * imuGetYawDeg();
    float delta = fmodf(target - dispAngle, 360.0f);
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    dispAngle += delta * 0.25f; /* smooth pursuit */
    /* redrawing the arrow invalidates a large center region — skip when the
       change would be invisible, it is the main source of screen churn */
    if (fabsf(dispAngle - drawnAngle) > 0.7f) {
        drawnAngle = dispAngle;
        arrowDraw(dispAngle);
    }
}

static void updateTimerCb(lv_timer_t *)
{
    PeerState peer;
    bool have = peersGet(FRIEND_ID, peer);
    bool online = have && peer.online;

    lv_obj_set_style_bg_color(s_dot, online ? lv_color_hex(0x35e08a) : lv_color_hex(0x555a66), 0);
    lv_label_set_text(s_proxLabel, online ? proximityLabel(peer.level) : "SEARCHING...");
    lv_obj_set_style_border_color(s_ring, levelColor(online ? peer.level : PROX_OUT_OF_RANGE, online), 0);
    lv_obj_set_style_text_color(s_proxLabel, levelColor(online ? peer.level : PROX_OUT_OF_RANGE, online), 0);

    char buf[128];
    if (have) {
        snprintf(buf, sizeof(buf), "rssi %d avg %.1f loss %.0f%%  yaw %.0f  g %.2f",
                 peer.rawRssi, peer.smoothedRssi, peer.packetLossPct(),
                 imuGetYawDeg(), imuGetAccelMagG());
    } else {
        snprintf(buf, sizeof(buf), "no packets yet  yaw %.0f  g %.2f",
                 imuGetYawDeg(), imuGetAccelMagG());
    }
    lv_label_set_text(s_debugLabel, buf);
}

void homeScreenCreate(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, lv_color_hex(0x0b0e1a), 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    /* pulsing proximity ring (behind everything) */
    s_ring = lv_obj_create(parent);
    lv_obj_remove_style_all(s_ring);
    lv_obj_set_size(s_ring, 160, 160);
    lv_obj_set_style_radius(s_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_ring, 5, 0);
    lv_obj_set_style_border_color(s_ring, lv_color_hex(0x555a66), 0);
    lv_obj_center(s_ring);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_ring);
    lv_anim_set_exec_cb(&a, ringAnimCb);
    lv_anim_set_values(&a, 160, 320);
    lv_anim_set_duration(&a, 1400);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    /* online/offline dot */
    s_dot = lv_obj_create(parent);
    lv_obj_remove_style_all(s_dot);
    lv_obj_set_size(s_dot, 14, 14);
    lv_obj_set_style_radius(s_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_dot, lv_color_hex(0x555a66), 0);
    lv_obj_align(s_dot, LV_ALIGN_TOP_MID, 0, 56);

    /* friend name */
    s_nameLabel = lv_label_create(parent);
    lv_label_set_text(s_nameLabel, deviceName(FRIEND_ID));
    lv_obj_set_style_text_font(s_nameLabel, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(s_nameLabel, lv_color_hex(0xf2f4f8), 0);
    lv_obj_align(s_nameLabel, LV_ALIGN_TOP_MID, 0, 84);

    /* proximity label */
    s_proxLabel = lv_label_create(parent);
    lv_label_set_text(s_proxLabel, "SEARCHING...");
    lv_obj_set_style_text_font(s_proxLabel, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_proxLabel, lv_color_hex(0x555a66), 0);
    lv_obj_align(s_proxLabel, LV_ALIGN_BOTTOM_MID, 0, -96);

    /* debug overlay (Phase 2 acceptance; hidden in later polish) */
    s_debugLabel = lv_label_create(parent);
    lv_label_set_text(s_debugLabel, "no packets yet");
    lv_obj_set_style_text_color(s_debugLabel, lv_color_hex(0x8a8f9c), 0);
    lv_obj_align(s_debugLabel, LV_ALIGN_BOTTOM_MID, 0, -58);

    /* direction arrow: shaft + two head strokes, redrawn per frame, always on */
    for (lv_obj_t **line : {&s_arrowShaft, &s_arrowHeadL, &s_arrowHeadR}) {
        *line = lv_line_create(parent);
        lv_obj_set_pos(*line, 0, 0);
        lv_obj_set_size(*line, 412, 412);
        lv_obj_set_style_line_width(*line, 16, 0);
        lv_obj_set_style_line_rounded(*line, true, 0);
        lv_obj_set_style_line_color(*line, lv_color_hex(0x3a3f4d), 0);
    }
    arrowDraw(0.0f);

    lv_timer_create(updateTimerCb, 250, nullptr);
    lv_timer_create(arrowTimerCb, 60, nullptr);
}

} // namespace tether
