#include "ui/screens/sos_screen.h"
#include "config/device_config.h"
#include "app/sos.h"

namespace tether {

static lv_obj_t *s_overlay;
static lv_obj_t *s_sosLabel;
static lv_obj_t *s_nameLabel;
static lv_obj_t *s_subLabel;

static void sosBlinkCb(void *var, int32_t v)
{
    lv_obj_set_style_text_opa((lv_obj_t *)var, (lv_opa_t)v, 0);
}

static void updateCb(lv_timer_t *)
{
    SosStatus st = sosGet();

    if (st.state == SOS_IDLE) {
        lv_obj_add_flag(s_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(s_overlay, LV_OBJ_FLAG_HIDDEN);

    if (st.state == SOS_INCOMING) {
        lv_obj_set_style_bg_color(s_overlay, lv_color_hex(0xc1121f), 0);
        lv_obj_set_style_text_color(s_sosLabel, lv_color_hex(0xffffff), 0);
        lv_label_set_text(s_nameLabel, deviceName(st.peerId));
        lv_label_set_text(s_subLabel, "NEEDS YOU\nPRESS BUTTON TO FIND");
    } else { /* SOS_OUTGOING */
        lv_obj_set_style_bg_color(s_overlay, lv_color_hex(0x1a0b10), 0);
        lv_obj_set_style_text_color(s_sosLabel, lv_color_hex(0xff2e3f), 0);
        lv_label_set_text(s_nameLabel, "SOS SENT");
        lv_label_set_text(s_subLabel, st.delivered ? "FRIEND ALERTED\nHOLD BUTTON TO CANCEL"
                                                   : "ALERTING...\nHOLD BUTTON TO CANCEL");
    }
}

void sosScreenCreate(lv_obj_t *parent)
{
    s_overlay = lv_obj_create(parent);
    lv_obj_remove_style_all(s_overlay);
    lv_obj_set_size(s_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(s_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_overlay, lv_color_hex(0xc1121f), 0);
    lv_obj_clear_flag(s_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_overlay, LV_OBJ_FLAG_HIDDEN);

    s_sosLabel = lv_label_create(s_overlay);
    lv_label_set_text(s_sosLabel, "SOS");
    lv_obj_set_style_text_font(s_sosLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(s_sosLabel, LV_ALIGN_TOP_MID, 0, 96);

    s_nameLabel = lv_label_create(s_overlay);
    lv_label_set_text(s_nameLabel, "");
    lv_obj_set_style_text_font(s_nameLabel, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(s_nameLabel, lv_color_hex(0xffffff), 0);
    lv_obj_align(s_nameLabel, LV_ALIGN_CENTER, 0, 10);

    s_subLabel = lv_label_create(s_overlay);
    lv_label_set_text(s_subLabel, "");
    lv_obj_set_style_text_font(s_subLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_subLabel, lv_color_hex(0xffd6d6), 0);
    lv_obj_set_style_text_align(s_subLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_subLabel, LV_ALIGN_BOTTOM_MID, 0, -84);

    /* urgency: blink the SOS wordmark */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_sosLabel);
    lv_anim_set_exec_cb(&a, sosBlinkCb);
    lv_anim_set_values(&a, 255, 60);
    lv_anim_set_duration(&a, 450);
    lv_anim_set_playback_duration(&a, 450);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    lv_timer_create(updateCb, 150, nullptr);
}

} // namespace tether
