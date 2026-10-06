/* ============================================================================
 *  EMG ORTHOSIS - MOTOR BENCH TEST  (PATH 2: pure ESP-IDF + esp_simplefoc)
 *  ----------------------------------------------------------------------------
 *  Serial input handled by a DEDICATED input task using blocking getchar().
 *  Because that task does nothing else, blocking is safe - it never starves
 *  the FOC loop (core 0) or the telemetry task (core 1).
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"          // console VFS config
#include "driver/uart_vfs.h"      // uart_vfs_dev_* (IDF 5.x)

#include "esp_simplefoc.h"

// ============================================================================
//  MODE
// ============================================================================
#define MODE_CALIBRATE 0
#define MODE_RUN       1
#define MODE  MODE_RUN // <-- change this to switch modes

// ----------------------------------------------------------------------------
//  PIN MAP
// ----------------------------------------------------------------------------
#define PIN_IN1      41
#define PIN_IN2      40
#define PIN_IN3      39
#define PIN_ENABLE   35
#define PIN_FAULT    48

#define PIN_SPI_MOSI GPIO_NUM_21
#define PIN_SPI_MISO GPIO_NUM_38
#define PIN_SPI_SCK  GPIO_NUM_37
#define PIN_SPI_CS   GPIO_NUM_47

// ----------------------------------------------------------------------------
//  MOTOR CONSTANTS
// ----------------------------------------------------------------------------
#define MOTOR_POLE_PAIRS  11
#define MOTOR_KV          141.0f
#define SUPPLY_VOLTAGE    11.1f
#define MOTOR_PHASE_R     5.6f

#define MAX_TARGET_VOLTAGE  2.0f
#define HARD_VOLTAGE_CEIL   3.0f

// ----------------------------------------------------------------------------
//  SimpleFOC objects
// ----------------------------------------------------------------------------
BLDCMotor       motor  = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_R, MOTOR_KV);
BLDCDriver3PWM  driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);
AS5048a         sensor = AS5048a(SPI2_HOST, PIN_SPI_SCK, PIN_SPI_MISO,
                                 PIN_SPI_MOSI, PIN_SPI_CS);

// ----------------------------------------------------------------------------
//  SHARED STATE
// ----------------------------------------------------------------------------
volatile float g_target_voltage = 0.0f;
volatile float g_shaft_angle    = 0.0f;
volatile float g_shaft_velocity = 0.0f;
volatile bool  g_fault_latched  = false;

// ----------------------------------------------------------------------------
//  FAULT pin
// ----------------------------------------------------------------------------
static void fault_pin_init() {
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_FAULT);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);
}
static inline bool fault_active() { return gpio_get_level((gpio_num_t)PIN_FAULT) == 0; }

// ----------------------------------------------------------------------------
//  Console setup so blocking line reads work reliably.
//  We do NOT install our own UART driver on UART0; we configure the existing
//  console VFS for blocking reads and line endings, then use fgets/getchar.
// ----------------------------------------------------------------------------
static void console_init() {
    // Make stdout unbuffered so prints appear immediately.
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);
    // Tell the UART VFS to treat CR and LF sensibly on input/output.
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}


// ============================================================================
//  MODE 1 : CALIBRATION
// ============================================================================
#if MODE == MODE_CALIBRATE

extern "C" void app_main(void) {
    printf("\n=== MODE: CALIBRATION ===\n");
    printf("Motor will move during alignment - keep the shaft FREE.\n\n");

    fault_pin_init();

    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = HARD_VOLTAGE_CEIL;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_limit     = HARD_VOLTAGE_CEIL;
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    SimpleFOCDebug::enable();

    motor.init();
    motor.initFOC();

    printf("\n========== CALIBRATION RESULTS ==========\n");
    printf("zero_electric_angle : %.4f\n", motor.zero_electric_angle);
    printf("sensor_direction    : %d\n", (int)motor.sensor_direction);
    printf("pole_pairs          : %d\n", motor.pole_pairs);
    printf("phase_resistance    : %.6f\n", motor.phase_resistance);
    printf("KV_rating           : %.2f\n", motor.KV_rating);
    printf("=========================================\n\n");

    while (1) {
        sensor.update();
        printf("angle(rad): %.3f\n", sensor.getAngle());
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

#endif // MODE_CALIBRATE


// ============================================================================
//  MODE 2 : RUN
// ============================================================================
#if MODE == MODE_RUN

// ----- CORE 0 : FOC loop, free-running -----
static void foc_task(void *arg) {
    printf(">>> FOC task started on core %d\n", xPortGetCoreID());
    printf(">>> about to call loopFOC for the first time\n");
    motor.loopFOC();
    printf(">>> loopFOC #1 OK\n");
    motor.move(0.0f);
    printf(">>> move #1 OK\n");
    static uint32_t c = 0;
    for (;;) {
        motor.loopFOC();
        motor.move(g_target_voltage);
        g_shaft_angle    = motor.shaft_angle;
        g_shaft_velocity = motor.shaft_velocity;
        if (++c >= 100) { c = 0; vTaskDelay(1); }   // let SPI/driver tasks run
    }
}


// ----- CORE 1 : telemetry only (no input here) -----
static void telemetry_task(void *arg) {
    printf(">>> TELEM task started on core %d\n", xPortGetCoreID());
    for (;;) {
        if (g_fault_latched) {
            printf("*** DRIVER FAULT - outputs disabled. Reset to clear. ***\n");
        } else {
            printf("angle(rad): %.3f   vel(rad/s): %.3f   target(V): %.3f\n",
                   g_shaft_angle, g_shaft_velocity, g_target_voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(200));   // 5 Hz
    }
}

// ----- CORE 1 : DEDICATED input task (blocking getchar is safe here) -----
static void input_task(void *arg) {
    printf(">>> INPUT task started on core %d\n", xPortGetCoreID());
    char line[32];
    size_t idx = 0;

    for (;;) {
        int c = getchar();              // BLOCKS until a char arrives - fine,
                                        // this task does nothing else.
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        char ch = (char)c;

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                line[idx] = '\0';
                idx = 0;

                // ---- parse the command ----
                float pct = atof(line);
                if (pct >  100.0f) pct =  100.0f;
                if (pct < -100.0f) pct = -100.0f;
                float volts = (pct / 100.0f) * MAX_TARGET_VOLTAGE;
                g_target_voltage = volts;
                printf("[cmd] %.1f%%  ->  %.3f V\n", pct, volts);
            }
        } else if (idx < sizeof(line) - 1) {
            line[idx++] = ch;
        }
    }
}

extern "C" void app_main(void) {
    console_init();

    printf("\n=== MODE: RUN ===\n");
    printf("Type a number -100..100 then Enter. 100 = MAX_TARGET_VOLTAGE.\n");
    printf("Negative = opposite direction. 0 = stop.\n\n");

    fault_pin_init();

    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = HARD_VOLTAGE_CEIL;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_limit     = HARD_VOLTAGE_CEIL;
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    motor.zero_electric_angle = 1.81673f;
    motor.sensor_direction    = Direction::CW;

    motor.init();
    motor.initFOC();
    motor.enable();

    printf("FOC ready. Spawning tasks.\n\n");

    // FOC on Core 0 (free-running, highest priority, alone)
    xTaskCreatePinnedToCore(foc_task, "FOC", 8192, NULL,
                            configMAX_PRIORITIES - 1, NULL, 0);

    // Telemetry on Core 1
    xTaskCreatePinnedToCore(telemetry_task, "TELEM", 4096, NULL, 5, NULL, 1);

    // Dedicated input reader on Core 1 (blocking getchar is safe here)
    xTaskCreatePinnedToCore(input_task, "INPUT", 4096, NULL, 6, NULL, 1);
}

#endif // MODE_RUN