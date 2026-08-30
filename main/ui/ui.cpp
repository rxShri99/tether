#include "ui/ui.h"
#include "config/device_config.h"

#if TETHER_ROLE == TETHER_ROLE_WEARABLE

#include "ui/screens/home_screen.h"
#include "ui/screens/sos_screen.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_spd2010.h"
#include "esp_lcd_touch_spd2010.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

namespace tether {

static const char *TAG = "ui";

/* Waveshare ESP32-S3-Touch-LCD-1.46 pin map */
#define PIN_QSPI_SCK  40
#define PIN_QSPI_D0   46
#define PIN_QSPI_D1   45
#define PIN_QSPI_D2   42
#define PIN_QSPI_D3   41
#define PIN_LCD_CS    21
#define PIN_LCD_BL    5
#define PIN_I2C_SCL   10
#define PIN_I2C_SDA   11

#define TCA9554_REG_OUTPUT 0x01
#define TCA9554_REG_CONFIG 0x03
#define EXIO_TP_RST_BIT    (1 << 1)
#define EXIO_LCD_RST_BIT   (1 << 2)

#define LCD_H_RES 412
#define LCD_V_RES 412
#define LCD_HOST  SPI2_HOST

static i2c_master_bus_handle_t s_i2cBus = nullptr;

static void backlightInit()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t chan = {};
    chan.gpio_num = PIN_LCD_BL;
    chan.speed_mode = LEDC_LOW_SPEED_MODE;
    chan.channel = LEDC_CHANNEL_0;
    chan.timer_sel = LEDC_TIMER_0;
    chan.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&chan));
}

static void backlightSet(int percent)
{
    /* duty 1024 (= 2^resolution) is LEDC's true always-on level */
    uint32_t duty = percent >= 100 ? 1024 : (1023u * percent) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

/* LCD reset is EXIO2 on the TCA9554 expander; keep other EXIOs high like the
 * Waveshare demo does. */
static void lcdResetViaExpander()
{
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port = -1;
    busCfg.scl_io_num = (gpio_num_t)PIN_I2C_SCL;
    busCfg.sda_io_num = (gpio_num_t)PIN_I2C_SDA;
    busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt = 7;
    busCfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &s_i2cBus));

    uint8_t addr = 0;
    for (uint8_t a = 0x20; a <= 0x27; a++) {
        if (i2c_master_probe(s_i2cBus, a, 100) == ESP_OK) { addr = a; break; }
    }
    if (!addr) {
        ESP_LOGE(TAG, "TCA9554 expander not found");
        return;
    }

    i2c_device_config_t devCfg = {};
    devCfg.device_address = addr;
    devCfg.scl_speed_hz = 400000;
    i2c_master_dev_handle_t dev = nullptr;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2cBus, &devCfg, &dev));

    /* SPD2010 is a TDDI chip (touch+display in one): hard-reset both its
     * lines together — EXIO1 = TP_RST, EXIO2 = LCD_RST — with long holds */
    uint8_t allOut[2] = {TCA9554_REG_CONFIG, 0x00};
    uint8_t hi[2] = {TCA9554_REG_OUTPUT, 0xFF};
    uint8_t rstLo[2] = {TCA9554_REG_OUTPUT, (uint8_t)~(EXIO_TP_RST_BIT | EXIO_LCD_RST_BIT)};
    ESP_ERROR_CHECK(i2c_master_transmit(dev, hi, 2, 100));
    ESP_ERROR_CHECK(i2c_master_transmit(dev, allOut, 2, 100));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(i2c_master_transmit(dev, rstLo, 2, 100));
    vTaskDelay(pdMS_TO_TICKS(80));
    ESP_ERROR_CHECK(i2c_master_transmit(dev, hi, 2, 100));
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev));

    if (i2c_master_probe(s_i2cBus, 0x53, 200) != ESP_OK) {
        ESP_LOGE(TAG, "SPD2010 not ACKing at 0x53 — display module flex/power problem likely");
    }
    ESP_LOGI(TAG, "LCD+TP reset pulsed via TCA9554@0x%02X", addr);
}

/* SPD2010 wants draw areas aligned to 4 pixels. */
static void rounderCb(lv_event_t *e)
{
    lv_area_t *a = (lv_area_t *)lv_event_get_param(e);
    a->x1 &= ~3;
    a->y1 &= ~3;
    a->x2 |= 3;
    a->y2 |= 3;
}

bool uiInit()
{
    backlightInit();
    lcdResetViaExpander();

    /* hand-expanded SPD2010_PANEL_BUS_QSPI_CONFIG (macros are C-only) */
    spi_bus_config_t busCfg = {};
    busCfg.sclk_io_num = PIN_QSPI_SCK;
    busCfg.data0_io_num = PIN_QSPI_D0;
    busCfg.data1_io_num = PIN_QSPI_D1;
    busCfg.data2_io_num = PIN_QSPI_D2;
    busCfg.data3_io_num = PIN_QSPI_D3;
    busCfg.max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &busCfg, SPI_DMA_CH_AUTO));

    /* hand-expanded SPD2010_PANEL_IO_QSPI_CONFIG */
    esp_lcd_panel_io_handle_t io = nullptr;
    esp_lcd_panel_io_spi_config_t ioCfg = {};
    ioCfg.cs_gpio_num = PIN_LCD_CS;
    ioCfg.dc_gpio_num = -1;
    ioCfg.spi_mode = 3;
    ioCfg.pclk_hz = 20 * 1000 * 1000;
    ioCfg.trans_queue_depth = 10;
    ioCfg.lcd_cmd_bits = 32;
    ioCfg.lcd_param_bits = 8;
    ioCfg.flags.quad_mode = true;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &ioCfg, &io));

    spd2010_vendor_config_t vendorCfg = {};
    vendorCfg.flags.use_qspi_interface = 1;
    esp_lcd_panel_handle_t panel = nullptr;
    esp_lcd_panel_dev_config_t panelCfg = {};
    panelCfg.reset_gpio_num = -1;
    panelCfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panelCfg.bits_per_pixel = 16;
    panelCfg.vendor_config = &vendorCfg;
    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(io, &panelCfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    /* touch (same I2C bus as the expander); non-fatal if it fails */
    esp_lcd_touch_handle_t touch = nullptr;
    {
        esp_lcd_panel_io_i2c_config_t tpIoCfg = {};
        tpIoCfg.dev_addr = ESP_LCD_TOUCH_IO_I2C_SPD2010_ADDRESS;
        tpIoCfg.control_phase_bytes = 1;
        tpIoCfg.flags.disable_control_phase = 1;
        tpIoCfg.scl_speed_hz = 400000;
        esp_lcd_panel_io_handle_t tpIo = nullptr;
        if (esp_lcd_new_panel_io_i2c(s_i2cBus, &tpIoCfg, &tpIo) == ESP_OK) {
            esp_lcd_touch_config_t tpCfg = {};
            tpCfg.x_max = LCD_H_RES;
            tpCfg.y_max = LCD_V_RES;
            tpCfg.rst_gpio_num = GPIO_NUM_NC;
            tpCfg.int_gpio_num = GPIO_NUM_NC;
            if (esp_lcd_touch_new_i2c_spd2010(tpIo, &tpCfg, &touch) != ESP_OK) {
                ESP_LOGW(TAG, "touch init failed, continuing without touch");
                touch = nullptr;
            }
        }
    }

    lvgl_port_cfg_t portCfg = ESP_LVGL_PORT_INIT_CONFIG();
    portCfg.task_stack = 8192;
    ESP_ERROR_CHECK(lvgl_port_init(&portCfg));

    lvgl_port_display_cfg_t dispCfg = {};
    dispCfg.io_handle = io;
    dispCfg.panel_handle = panel;
    dispCfg.buffer_size = LCD_H_RES * 60;
    dispCfg.double_buffer = true;
    dispCfg.hres = LCD_H_RES;
    dispCfg.vres = LCD_V_RES;
    dispCfg.color_format = LV_COLOR_FORMAT_RGB565;
    dispCfg.flags.buff_dma = true;
    dispCfg.flags.swap_bytes = true;
    lv_display_t *disp = lvgl_port_add_disp(&dispCfg);
    if (!disp) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return false;
    }
    lv_display_add_event_cb(disp, rounderCb, LV_EVENT_INVALIDATE_AREA, nullptr);

    if (touch) {
        lvgl_port_touch_cfg_t touchCfg = {};
        touchCfg.disp = disp;
        touchCfg.handle = touch;
        lvgl_port_add_touch(&touchCfg);
    }

    lvgl_port_lock(0);
    homeScreenCreate(lv_scr_act());
    sosScreenCreate(lv_scr_act()); /* created last: overlays everything */
    lvgl_port_unlock();

    vTaskDelay(pdMS_TO_TICKS(100)); /* first frame renders before light-up */
    backlightSet(100);
    ESP_LOGI(TAG, "UI up (touch %s)", touch ? "ok" : "disabled");
    return true;
}

i2c_master_bus_handle_t uiI2CBus() { return s_i2cBus; }

} // namespace tether

#else

namespace tether {
bool uiInit() { return true; } /* hub UI arrives in Phase 7 */
i2c_master_bus_handle_t uiI2CBus() { return nullptr; }
}

#endif
