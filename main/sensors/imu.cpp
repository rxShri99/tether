#include "sensors/imu.h"

#include <atomic>
#include <cmath>
#include <initializer_list>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace tether {

static const char *TAG = "imu";

/* QMI8658 registers */
#define REG_WHO_AM_I 0x00 /* = 0x05 */
#define REG_CTRL1    0x02
#define REG_CTRL2    0x03
#define REG_CTRL3    0x04
#define REG_CTRL7    0x08
#define REG_RESET    0x60
#define REG_AX_L     0x35 /* 12 bytes: ax..gz, little endian */

static i2c_master_dev_handle_t s_dev = nullptr;
static std::atomic<float> s_yawDeg{0.0f};
static std::atomic<float> s_accelMagG{1.0f};

static bool writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100) == ESP_OK;
}

static bool readRegs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100) == ESP_OK;
}

static void imuTask(void *)
{
    /* gyro bias calibration: device is assumed still for the first second */
    float bias = 0.0f;
    const float GYRO_LSB_TO_DPS = 512.0f / 32768.0f;
    const float ACCEL_LSB_TO_G = 8.0f / 32768.0f;

    int calSamples = 0;
    float calSum = 0.0f;
    int64_t lastUs = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10)); /* ~100 Hz */
        uint8_t raw[12];
        if (!readRegs(REG_AX_L, raw, sizeof(raw))) continue;

        int16_t ax = (int16_t)(raw[0] | raw[1] << 8);
        int16_t ay = (int16_t)(raw[2] | raw[3] << 8);
        int16_t az = (int16_t)(raw[4] | raw[5] << 8);
        int16_t gz = (int16_t)(raw[10] | raw[11] << 8);

        float axg = ax * ACCEL_LSB_TO_G, ayg = ay * ACCEL_LSB_TO_G, azg = az * ACCEL_LSB_TO_G;
        s_accelMagG = sqrtf(axg * axg + ayg * ayg + azg * azg);

        float gzDps = gz * GYRO_LSB_TO_DPS;
        int64_t nowUs = esp_timer_get_time();
        float dt = (nowUs - lastUs) / 1e6f;
        lastUs = nowUs;

        if (calSamples < 100) {
            calSum += gzDps;
            if (++calSamples == 100) {
                bias = calSum / 100.0f;
                ESP_LOGI(TAG, "gyro Z bias: %.3f dps", bias);
            }
            continue;
        }

        float rate = gzDps - bias;
        if (fabsf(rate) > 0.6f) { /* deadband kills stationary drift */
            s_yawDeg = s_yawDeg + rate * dt;
        }
    }
}

bool imuInit(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        ESP_LOGW(TAG, "no I2C bus, IMU disabled");
        return false;
    }
    uint8_t addr = 0;
    for (uint8_t a : {0x6B, 0x6A}) {
        if (i2c_master_probe(bus, a, 100) == ESP_OK) { addr = a; break; }
    }
    if (!addr) {
        ESP_LOGW(TAG, "QMI8658 not found on I2C bus");
        return false;
    }

    i2c_device_config_t devCfg = {};
    devCfg.device_address = addr;
    devCfg.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devCfg, &s_dev));

    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1) || who != 0x05) {
        ESP_LOGW(TAG, "unexpected WHO_AM_I 0x%02X", who);
        return false;
    }

    writeReg(REG_RESET, 0xB0);
    vTaskDelay(pdMS_TO_TICKS(15));
    writeReg(REG_CTRL1, 0x40); /* auto address increment */
    writeReg(REG_CTRL2, 0x25); /* accel ±8g @235Hz */
    writeReg(REG_CTRL3, 0x55); /* gyro ±512dps @235Hz */
    writeReg(REG_CTRL7, 0x03); /* enable accel + gyro */
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreate(imuTask, "imu", 4096, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "QMI8658 up at 0x%02X", addr);
    return true;
}

float imuGetYawDeg() { return s_yawDeg; }
float imuGetAccelMagG() { return s_accelMagG; }

} // namespace tether
