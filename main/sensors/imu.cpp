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
#define REG_AX_L     0x35 /* 12 bytes: ax, ay, az, gx, gy, gz — little endian */

static i2c_master_dev_handle_t s_dev = nullptr;
static std::atomic<float> s_yawDeg{0.0f};
static std::atomic<float> s_accelMagG{1.0f};
static std::atomic<float> s_tiltDeg{0.0f};
static std::atomic<bool> s_ready{false};

static bool writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100) == ESP_OK;
}

static bool readRegs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100) == ESP_OK;
}

/*
 * Heading is integrated from the component of rotation about TRUE VERTICAL,
 * not about the device's own Z axis.
 *
 * Integrating raw gyro-Z only works while the board lies flat and face up.
 * Turn it over and Z points down, so the same physical turn yields the
 * opposite sign and the bearing counts backwards; stand it on edge and
 * rotation about vertical lands on X/Y, which Z never sees, so heading
 * freezes. Both were visible as a direction arrow that broke whenever a
 * device was flipped.
 *
 * The accelerometer gives the up direction in the device frame, so the
 * rotation rate about vertical is simply the gyro vector projected onto it:
 *
 *     yawRate = gyro · up̂
 *
 * Flipping the board flips up̂ as well, so the sign tracks the orientation by
 * itself — no manual SWEEP_YAW_SIGN needed, and no flat-device assumption.
 */
static void imuTask(void *)
{
    const float GYRO_LSB_TO_DPS = 512.0f / 32768.0f;
    const float ACCEL_LSB_TO_G = 8.0f / 32768.0f;

    /* Gyro bias, per axis — the device is assumed still for the first second. */
    float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int calSamples = 0;
    int calRestartLog = 0;
    int stillSamples = 0;
    constexpr int CAL_COUNT = 100;

    /* Peak-to-peak spread over the window below which the gyro counts as
       resting. Noise on a still QMI8658 is well under a dps; deliberate
       handling is many. Independent of however large the offset happens to be. */
    constexpr float STILL_SPREAD_DPS = 2.0f;
    constexpr int STILL_WINDOW = 32;   /* ~320 ms at 100 Hz */
    /* ~0.5 s of continuous stillness before trusting a reading as pure bias. */
    constexpr int STILL_SAMPLES_BEFORE_TRIM = 50;

    float win[3][STILL_WINDOW] = {};
    int winHead = 0;
    int winFill = 0;

    /* Calibration must never deadlock. A wearable being carried may not offer a
       single genuinely still window, and waiting forever leaves imuReady() false
       and the bearing frozen. Track the calmest window seen and adopt it if no
       properly still one turns up in time; the continuous refinement below then
       corrects it the moment the device does come to rest. */
    float bestSpread = 1e9f;
    float bestBias[3] = {0.0f, 0.0f, 0.0f};
    int calElapsed = 0;
    constexpr int CAL_TIMEOUT_SAMPLES = 800;  /* ~8 s at 100 Hz */

    /* Low-passed up vector in the device frame. Seeded from the first sample
       so it does not have to converge away from a wrong assumed orientation. */
    float upX = 0.0f, upY = 0.0f, upZ = 1.0f;
    bool upSeeded = false;

    int64_t lastUs = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10)); /* ~100 Hz */
        uint8_t raw[12];
        if (!readRegs(REG_AX_L, raw, sizeof(raw))) continue;

        int16_t ax = (int16_t)(raw[0] | raw[1] << 8);
        int16_t ay = (int16_t)(raw[2] | raw[3] << 8);
        int16_t az = (int16_t)(raw[4] | raw[5] << 8);
        int16_t gx = (int16_t)(raw[6] | raw[7] << 8);
        int16_t gy = (int16_t)(raw[8] | raw[9] << 8);
        int16_t gz = (int16_t)(raw[10] | raw[11] << 8);

        const float axg = ax * ACCEL_LSB_TO_G;
        const float ayg = ay * ACCEL_LSB_TO_G;
        const float azg = az * ACCEL_LSB_TO_G;

        const float mag = sqrtf(axg * axg + ayg * ayg + azg * azg);
        s_accelMagG = mag;

        /* Only trust the accelerometer as a gravity reference when it is close
           to 1g. Under a bump or a hard swing it is measuring motion, not
           gravity, and would drag the up vector off. */
        if (mag > 0.75f && mag < 1.25f) {
            const float nx = axg / mag, ny = ayg / mag, nz = azg / mag;
            if (!upSeeded) {
                upX = nx; upY = ny; upZ = nz;
                upSeeded = true;
            } else {
                /* ~0.5 s time constant at 100 Hz: steady against hand shake,
                   still quick enough to follow a deliberate flip. */
                const float A = 0.02f;
                upX += A * (nx - upX);
                upY += A * (ny - upY);
                upZ += A * (nz - upZ);
                const float m = sqrtf(upX * upX + upY * upY + upZ * upZ);
                if (m > 1e-6f) { upX /= m; upY /= m; upZ /= m; }
            }
        }

        /* Angle between the screen normal (device +Z) and up: 0 = face up,
           180 = face down. Purely diagnostic, but it makes "is it flipped?"
           answerable from the debug overlay instead of by guessing. */
        float upzClamped = upZ;
        if (upzClamped > 1.0f) upzClamped = 1.0f;
        if (upzClamped < -1.0f) upzClamped = -1.0f;
        s_tiltDeg = acosf(upzClamped) * 180.0f / (float)M_PI;

        const float gxDps = gx * GYRO_LSB_TO_DPS;
        const float gyDps = gy * GYRO_LSB_TO_DPS;
        const float gzDps = gz * GYRO_LSB_TO_DPS;

        const int64_t nowUs = esp_timer_get_time();
        const float dt = (nowUs - lastUs) / 1e6f;
        lastUs = nowUs;

        /*
         * Bias must be measured while the device is genuinely still. Averaging
         * blindly over the first second captures whatever handling was going on
         * at boot -- a board dangling off a USB cable measured -6.7 dps, which
         * integrates to roughly 400 deg of drift per minute and sends the
         * bearing wandering no matter how well the tilt is compensated.
         *
         * So: gate on stillness and restart the window whenever motion appears.
         */
        /*
         * Stillness is judged by how much the gyro READING VARIES, never by how
         * large it is. Testing the magnitude is circular: it includes the very
         * bias being measured, and this QMI8658 sits at ~8 dps at rest, so a
         * magnitude gate can never be satisfied and calibration deadlocks.
         * A resting gyro is steady whatever its offset; a handled one is not.
         */
        win[0][winHead] = gxDps;
        win[1][winHead] = gyDps;
        win[2][winHead] = gzDps;
        winHead = (winHead + 1) % STILL_WINDOW;
        if (winFill < STILL_WINDOW) winFill++;

        float maxSpread = 0.0f;
        if (winFill == STILL_WINDOW) {
            for (int axis = 0; axis < 3; axis++) {
                float lo = win[axis][0], hi = win[axis][0];
                for (int i = 1; i < STILL_WINDOW; i++) {
                    if (win[axis][i] < lo) lo = win[axis][i];
                    if (win[axis][i] > hi) hi = win[axis][i];
                }
                if (hi - lo > maxSpread) maxSpread = hi - lo;
            }
        }
        const bool still = winFill == STILL_WINDOW &&
                           maxSpread <= STILL_SPREAD_DPS &&
                           fabsf(mag - 1.0f) < 0.12f;

        if (calSamples < CAL_COUNT) {
            calElapsed++;

            /* Remember the calmest window so far, in case we have to fall back. */
            if (winFill == STILL_WINDOW && maxSpread < bestSpread) {
                bestSpread = maxSpread;
                for (int axis = 0; axis < 3; axis++) {
                    float acc = 0.0f;
                    for (int i = 0; i < STILL_WINDOW; i++) acc += win[axis][i];
                    bestBias[axis] = acc / STILL_WINDOW;
                }
            }

            if (!still && calElapsed > CAL_TIMEOUT_SAMPLES && bestSpread < 1e8f) {
                biasX = bestBias[0];
                biasY = bestBias[1];
                biasZ = bestBias[2];
                calSamples = CAL_COUNT;
                s_ready = true;
                ESP_LOGW(TAG, "no still window in %ds -- provisional bias "
                              "x=%.3f y=%.3f z=%.3f dps from calmest sample "
                              "(spread %.2f); will refine when the device rests",
                         CAL_TIMEOUT_SAMPLES / 100, biasX, biasY, biasZ,
                         static_cast<double>(bestSpread));
                continue;
            }

            if (!still) {
                if (calSamples > 0) {
                    calSamples = 0;
                    sumX = sumY = sumZ = 0.0f;
                }
                if (++calRestartLog % 200 == 0) {
                    ESP_LOGW(TAG, "hold still to calibrate the gyro "
                                  "(gyro spread %.2f dps, |a| %.2f g)",
                             static_cast<double>(maxSpread),
                             static_cast<double>(mag));
                }
                continue;
            }
            sumX += gxDps; sumY += gyDps; sumZ += gzDps;
            if (++calSamples == CAL_COUNT) {
                biasX = sumX / CAL_COUNT;
                biasY = sumY / CAL_COUNT;
                biasZ = sumZ / CAL_COUNT;
                s_ready = true;
                ESP_LOGI(TAG, "gyro bias x=%.3f y=%.3f z=%.3f dps", biasX, biasY, biasZ);
                if (biasX == 0.0f && biasY == 0.0f && biasZ == 0.0f) {
                    ESP_LOGW(TAG, "all three bias terms are exactly zero -- the gyro "
                                  "is almost certainly not producing data; check CTRL3/CTRL7");
                }
                ESP_LOGI(TAG, "up vector (device frame) %.2f %.2f %.2f, tilt %.0f deg",
                         upX, upY, upZ, s_tiltDeg.load());
            }
            continue;
        }

        /*
         * Keep refining the bias whenever the device sits still. Gyro bias
         * drifts with temperature, and a wearable spends most of its life
         * stationary between movements -- when it is still, whatever the gyro
         * reads IS the bias, so it can be tracked for free instead of trusting
         * one measurement taken at boot.
         */
        if (still) {
            if (++stillSamples > STILL_SAMPLES_BEFORE_TRIM) {
                constexpr float B = 0.01f;
                biasX += B * (gxDps - biasX);
                biasY += B * (gyDps - biasY);
                biasZ += B * (gzDps - biasZ);
            }
        } else {
            stillSamples = 0;
        }

        /* Rotation rate about true vertical. */
        const float rate = (gxDps - biasX) * upX +
                           (gyDps - biasY) * upY +
                           (gzDps - biasZ) * upZ;

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

    /* Read CTRL7 back: a silent I2C write leaves the gyro off, which shows up
       downstream as a gyro bias of exactly zero and a heading that never moves. */
    uint8_t ctrl7 = 0;
    if (readRegs(REG_CTRL7, &ctrl7, 1) && (ctrl7 & 0x03) != 0x03) {
        ESP_LOGW(TAG, "CTRL7 reads 0x%02X -- accel/gyro not both enabled", ctrl7);
    }

    xTaskCreate(imuTask, "imu", 4096, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "QMI8658 up at 0x%02X (tilt-compensated heading)", addr);
    return true;
}

float imuGetYawDeg() { return s_yawDeg; }
float imuGetAccelMagG() { return s_accelMagG; }
float imuGetTiltDeg() { return s_tiltDeg; }
bool imuReady() { return s_ready; }

} // namespace tether
