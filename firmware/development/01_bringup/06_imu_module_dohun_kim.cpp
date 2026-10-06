/* ============================================================================
 *  IMU MODULE (BNO055) - standalone bring-up
 *  ----------------------------------------------------------------------------
 *  Written by Dohun Kim: BNO055 I2C driver, high-g interrupt configuration,
 *  and the spasm/collision detection heuristics (sliding-window spike and
 *  direction-change counters, gravity-angle-from-pitch mapping).
 *
 *  This standalone module was integrated into the wider dual-core firmware
 *  (task partitioning, safety hierarchy, torque control loop) by Calvin Yap.
 *  See firmware/development/02_integration/ for the integration steps and
 *  firmware/final/main_demoday.cpp for the final combined system.
 * ==========================================================================*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdatomic.h>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_PORT I2C_NUM_0

// J6 IMU pins - 실제 테스트 결과 SDA/SCL 반대
#define SDA_PIN 10
#define SCL_PIN 9
#define INT_PIN 11

#define I2C_FREQ_HZ 100000

#define BNO055_ADDR 0x28

#define BNO055_CHIP_ID_REG 0x00
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_EULER_H_LSB 0x1A
#define BNO055_PAGE_ID_REG 0x07
#define BNO055_OPR_MODE_REG 0x3D
#define BNO055_PWR_MODE_REG 0x3E

#define BNO055_ACC_CONFIG_REG 0x08
#define BNO055_INT_MSK_REG 0x0F
#define BNO055_INT_EN_REG 0x10
#define BNO055_ACC_INT_SETTINGS_REG 0x12
#define BNO055_ACC_HIGH_G_THRESHOLD_REG 0x14
#define BNO055_ACC_HIGH_G_DURATION_REG 0x15

#define OPR_CONFIG 0x00
#define OPR_NDOF 0x0C

#define IMU_TASK_PERIOD_MS 20
#define WINDOW_SAMPLES 50

// Spasm 
#define SPASM_CHANGE_THRESHOLD 4.5f
#define SPASM_SPIKE_THRESHOLD 6
#define SPASM_DIRECTION_CHANGE_THRESHOLD 4

#define COLLISION_MAG_THRESHOLD 11.0f

static const char *TAG = "IMU";

static atomic_bool atomic_spasm_flag = false;
static atomic_bool atomic_collision_flag = false;
static atomic_int atomic_gravity_angle_deg = 0;

typedef enum {
    AXIS_NONE = 0,
    AXIS_X,
    AXIS_Y,
    AXIS_Z
} axis_t;

typedef struct {
    float yaw;
    float roll;
    float pitch;

    float gravity_angle;

    float ax;
    float ay;
    float az;
    float acc_mag;
    float acc_change;

    int spike_count;
    int direction_change_count;
    axis_t dominant_axis;

    bool spasm_flag;
    bool collision_flag;
} imu_shared_data_t;

static imu_shared_data_t imu_data;
static SemaphoreHandle_t imu_mutex;

static void IRAM_ATTR imu_int_isr(void *arg)
{
    atomic_store(&atomic_collision_flag, true);
}

static float clampf_custom(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static esp_err_t bno_write8(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_write_to_device(
        I2C_PORT,
        BNO055_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t bno_read_len(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(
        I2C_PORT,
        BNO055_ADDR,
        &reg,
        1,
        buf,
        len,
        pdMS_TO_TICKS(100)
    );
}

static void imu_shared_update(const imu_shared_data_t *new_data)
{
    if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        imu_data = *new_data;
        xSemaphoreGive(imu_mutex);
    }
}

bool imu_get_data(imu_shared_data_t *out)
{
    if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *out = imu_data;
        xSemaphoreGive(imu_mutex);
        return true;
    }

    return false;
}

void imu_clear_flags(void)
{
    if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        imu_data.spasm_flag = false;
        imu_data.collision_flag = false;
        xSemaphoreGive(imu_mutex);
    }

    atomic_store(&atomic_spasm_flag, false);
    atomic_store(&atomic_collision_flag, false);
}

static axis_t get_dominant_axis(float dx, float dy, float dz)
{
    float adx = fabsf(dx);
    float ady = fabsf(dy);
    float adz = fabsf(dz);

    if (adx > ady && adx > adz) return AXIS_X;
    if (ady > adx && ady > adz) return AXIS_Y;
    if (adz > adx && adz > ady) return AXIS_Z;

    return AXIS_NONE;
}

static int sign_of(float value)
{
    if (value > 0.2f) return 1;
    if (value < -0.2f) return -1;
    return 0;
}

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    esp_err_t err;

    err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2C ready: PORT=%d, SDA=GPIO%d, SCL=GPIO%d", I2C_PORT, SDA_PIN, SCL_PIN);
}

static void gpio_interrupt_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    esp_err_t err;

    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return;
    }

    err = gpio_isr_handler_add(INT_PIN, imu_int_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "IMU interrupt ready: INT=GPIO%d", INT_PIN);
}

static bool bno055_init_with_high_g_interrupt(void)
{
    uint8_t chip_id = 0;
    esp_err_t err;

    err = bno_read_len(BNO055_CHIP_ID_REG, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BNO055 chip ID: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Check J6 wiring: 3V3, GND, SDA=GPIO10, SCL=GPIO9");
        return false;
    }

    ESP_LOGI(TAG, "BNO055 Chip ID = 0x%02X", chip_id);

    if (chip_id != 0xA0) {
        ESP_LOGE(TAG, "BNO055 not detected. Expected 0xA0");
        return false;
    }

    err = bno_write8(BNO055_OPR_MODE_REG, OPR_CONFIG);
    if (err != ESP_OK) return false;

    vTaskDelay(pdMS_TO_TICKS(25));

    bno_write8(BNO055_PAGE_ID_REG, 0x00);
    bno_write8(BNO055_PWR_MODE_REG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    bno_write8(BNO055_PAGE_ID_REG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    bno_write8(BNO055_ACC_CONFIG_REG, 0x0D);
    bno_write8(BNO055_ACC_HIGH_G_THRESHOLD_REG, 0x08);
    bno_write8(BNO055_ACC_HIGH_G_DURATION_REG, 0x04);
    bno_write8(BNO055_ACC_INT_SETTINGS_REG, 0xE0);
    bno_write8(BNO055_INT_EN_REG, 0x20);
    bno_write8(BNO055_INT_MSK_REG, 0x20);

    bno_write8(BNO055_PAGE_ID_REG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    err = bno_write8(BNO055_OPR_MODE_REG, OPR_NDOF);
    if (err != ESP_OK) return false;

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "BNO055 initialized: NDOF + High-G interrupt");
    return true;
}

static float calculate_gravity_angle_from_pitch(float pitch)
{
    if (pitch >= 17.0f)
    {
        return clampf_custom(
            90.0f * (92.8f - pitch) / (92.8f - 17.0f),
            0.0f,
            90.0f
        );
    }
    else
    {
        return clampf_custom(
            90.0f + 90.0f * (17.0f - pitch) / (17.0f - (-87.4f)),
            90.0f,
            180.0f
        );
    }
}

static void imu_task(void *pvParameters)
{
    float prev_mag = 0.0f;
    float prev_ax = 0.0f;
    float prev_ay = 0.0f;
    float prev_az = 0.0f;

    int spike_window[WINDOW_SAMPLES] = {0};
    int dir_window[WINDOW_SAMPLES] = {0};
    int window_index = 0;

    int spike_count = 0;
    int direction_change_count = 0;
    int prev_dominant_sign = 0;

    while (1) {
        uint8_t acc_buf[6];
        uint8_t euler_buf[6];
        imu_shared_data_t local = {0};

        if (bno_read_len(BNO055_ACC_DATA_X_LSB, acc_buf, 6) == ESP_OK &&
            bno_read_len(BNO055_EULER_H_LSB, euler_buf, 6) == ESP_OK) {

            int16_t ax_raw = (int16_t)((acc_buf[1] << 8) | acc_buf[0]);
            int16_t ay_raw = (int16_t)((acc_buf[3] << 8) | acc_buf[2]);
            int16_t az_raw = (int16_t)((acc_buf[5] << 8) | acc_buf[4]);

            local.ax = ax_raw / 100.0f;
            local.ay = ay_raw / 100.0f;
            local.az = az_raw / 100.0f;

            local.acc_mag = sqrtf(
                local.ax * local.ax +
                local.ay * local.ay +
                local.az * local.az
            );

            local.acc_change = fabsf(local.acc_mag - prev_mag);

            int16_t yaw_raw = (int16_t)((euler_buf[1] << 8) | euler_buf[0]);
            int16_t roll_raw = (int16_t)((euler_buf[3] << 8) | euler_buf[2]);
            int16_t pitch_raw = (int16_t)((euler_buf[5] << 8) | euler_buf[4]);

            local.yaw = yaw_raw / 16.0f;
            local.roll = roll_raw / 16.0f;
            local.pitch = pitch_raw / 16.0f;

            local.gravity_angle =
                calculate_gravity_angle_from_pitch(local.pitch);

            atomic_store(&atomic_gravity_angle_deg, (int)local.gravity_angle);

            float dx = local.ax - prev_ax;
            float dy = local.ay - prev_ay;
            float dz = local.az - prev_az;

            local.dominant_axis = get_dominant_axis(dx, dy, dz);

            int current_sign = 0;

            if (local.dominant_axis == AXIS_X) {
                current_sign = sign_of(dx);
            } else if (local.dominant_axis == AXIS_Y) {
                current_sign = sign_of(dy);
            } else if (local.dominant_axis == AXIS_Z) {
                current_sign = sign_of(dz);
            }

            int spike_now = (local.acc_change > SPASM_CHANGE_THRESHOLD) ? 1 : 0;
            int direction_change_now = 0;

            if (current_sign != 0 &&
                prev_dominant_sign != 0 &&
                current_sign != prev_dominant_sign) {
                direction_change_now = 1;
            }

            spike_count -= spike_window[window_index];
            direction_change_count -= dir_window[window_index];

            spike_window[window_index] = spike_now;
            dir_window[window_index] = direction_change_now;

            spike_count += spike_now;
            direction_change_count += direction_change_now;

            window_index = (window_index + 1) % WINDOW_SAMPLES;

            local.spike_count = spike_count;
            local.direction_change_count = direction_change_count;

            local.collision_flag = false;

            if (local.acc_mag > COLLISION_MAG_THRESHOLD && spike_count <= 1) {
                local.collision_flag = true;
                atomic_store(&atomic_collision_flag, true);
            }

            if (atomic_load(&atomic_collision_flag)) {
                local.collision_flag = true;
            }

            if (local.collision_flag) {
                spike_count = 0;
                direction_change_count = 0;

                for (int i = 0; i < WINDOW_SAMPLES; i++) {
                    spike_window[i] = 0;
                    dir_window[i] = 0;
                }

                local.spasm_flag = false;
                local.spike_count = 0;
                local.direction_change_count = 0;
            } else {
                local.spasm_flag =
                    (spike_count >= SPASM_SPIKE_THRESHOLD) &&
                    (direction_change_count >= SPASM_DIRECTION_CHANGE_THRESHOLD);
            }

            atomic_store(&atomic_spasm_flag, local.spasm_flag);
            imu_shared_update(&local);

            ESP_LOGI(
                TAG,
                "GravityAngle=%.1f Pitch=%.1f | spasm=%d collision=%d",
                local.gravity_angle,
                local.pitch,
                local.spasm_flag,
                local.collision_flag
            );

            if (local.spasm_flag) {
                ESP_LOGW(TAG, "SPASM DETECTED");
            }

            if (local.collision_flag) {
                ESP_LOGE(TAG, "COLLISION DETECTED");
                atomic_store(&atomic_collision_flag, false);
            }

            prev_mag = local.acc_mag;
            prev_ax = local.ax;
            prev_ay = local.ay;
            prev_az = local.az;

            if (current_sign != 0) {
                prev_dominant_sign = current_sign;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read BNO055 data");
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting IMU module on J6");

    imu_mutex = xSemaphoreCreateMutex();

    if (imu_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    i2c_init();
    gpio_interrupt_init();

    bool imu_ok = bno055_init_with_high_g_interrupt();

    if (!imu_ok) {
        ESP_LOGE(TAG, "IMU init failed. System will stay alive for debugging.");

        while (1) {
            ESP_LOGI(TAG, "alive but IMU not initialized");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        NULL,
        5,
        NULL,
        0
    );
}