/* ============================================================================
 *  EMG ORTHOSIS - MOTOR BENCH TEST  (PATH 2: pure ESP-IDF + esp_simplefoc)
 *  ----------------------------------------------------------------------------
 *  NO Arduino. NO pioarduino. Pure framework = espidf, official platform.
 *  esp_simplefoc is an ESP-IDF managed component (see idf_component.yml).
 *
 *  Hardware:
 *    MCU     : ESP32-S3 (Freenove WROOM N16R8, octal PSRAM)
 *    Driver  : SimpleFOC Mini (DRV8313, 3-PWM, NO current sense -> voltage mode)
 *    Motor   : iPower GM3506 (24N/22P => 11 pole pairs, Y-wound)
 *    Encoder : AS5048A magnetic absolute, SPI, on the MOTOR shaft
 *    Power   : 3S LiPo (~11.1V nominal)
 *
 *  Two modes via the MODE #define:
 *    MODE_CALIBRATE : print sensor angle + run alignment, read off constants
 *    MODE_RUN       : Core 0 free-runs FOC; Core 1 reads -100..+100 from serial
 *                     and writes shared target voltage; prints angle/velocity.
 *
 *  NOTE: esp_simplefoc uses the SAME SimpleFOC API as Arduino-FOC. The only
 *  differences from the Arduino sketch are: app_main() instead of setup()/loop(),
 *  ESP-IDF UART for serial I/O, and ESP-IDF GPIO for the FAULT pin.
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

// esp_simplefoc umbrella header (pulls in BLDCMotor, drivers, sensors)
#include "esp_simplefoc.h"

#include <fcntl.h>
#include <unistd.h>

static const char *TAG = "FOC_BENCH";

// ============================================================================
//  >>>>>>>>>>>>>>>>>>>>  SET THE MODE HERE  <<<<<<<<<<<<<<<<<<<<<<
// ============================================================================
#define MODE_CALIBRATE 0
#define MODE_RUN       1

#define MODE  MODE_CALIBRATE    // <-- change to MODE_RUN after calibration
// ============================================================================


// ----------------------------------------------------------------------------
//  PIN MAP (validated against Freenove ESP32-S3 pinout)
// ----------------------------------------------------------------------------
#define PIN_IN1      15          // SimpleFOC Mini IN1 (PWM)
#define PIN_IN2      16          // SimpleFOC Mini IN2 (PWM)
#define PIN_IN3      17          // SimpleFOC Mini IN3 (PWM)
#define PIN_ENABLE   7          // SimpleFOC Mini EN
#define PIN_FAULT    6          // SimpleFOC Mini FAULT (input, ACTIVE-LOW)

#define PIN_SPI_MOSI GPIO_NUM_13        // AS5048A MOSI
#define PIN_SPI_MISO GPIO_NUM_12         // AS5048A MISO
#define PIN_SPI_SCK  GPIO_NUM_11         // AS5048A SCK/SCL
#define PIN_SPI_CS   GPIO_NUM_14        // AS5048A CS/SDA

// ----------------------------------------------------------------------------
//  MOTOR CONSTANTS
// ----------------------------------------------------------------------------
#define MOTOR_POLE_PAIRS  11
#define MOTOR_KV          141.0f
#define SUPPLY_VOLTAGE    11.1f
// Replace with the value MODE_CALIBRATE reports. Datasheet 5.6 mOhm is suspect.
#define MOTOR_PHASE_R     5.6f

// ----------------------------------------------------------------------------
//  SAFETY: input "100" maps to MAX_TARGET_VOLTAGE. Start low; tune up while
//  watching FAULT and motor temperature. ~1.2A continuous budget, air-cooled.
// ----------------------------------------------------------------------------
#define MAX_TARGET_VOLTAGE  2.0f
#define HARD_VOLTAGE_CEIL   3.0f

// ----------------------------------------------------------------------------
//  SimpleFOC objects (same classes as Arduino-FOC)
// ----------------------------------------------------------------------------
BLDCMotor       motor  = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_R, MOTOR_KV);
BLDCDriver3PWM  driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);
// esp_simplefoc AS5048A SPI sensor: (host, sck, miso, mosi, cs)
AS5048a         sensor = AS5048a(SPI2_HOST, PIN_SPI_SCK, PIN_SPI_MISO,
                                 PIN_SPI_MOSI, PIN_SPI_CS);

// ----------------------------------------------------------------------------
//  SHARED STATE (inter-core, single 4-byte aligned floats => lock-free)
// ----------------------------------------------------------------------------
volatile float g_target_voltage = 0.0f;   // Core 1 -> Core 0
volatile float g_shaft_angle    = 0.0f;   // Core 0 -> Core 1
volatile float g_shaft_velocity = 0.0f;   // Core 0 -> Core 1
volatile bool  g_fault_latched  = false;

// ----------------------------------------------------------------------------
//  FAULT pin (active LOW)
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
//  Minimal UART input (reads a line from the default console UART0)
// ----------------------------------------------------------------------------
static void stdin_set_nonblocking() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// reads available chars into buf; returns true when a full line is ready
static bool stdin_read_line(char *buf, size_t bufsz, size_t *idx) {
    int c;
    while ((c = getchar()) != EOF) {
        char ch = (char)c;
        if (ch == '\n' || ch == '\r') {
            if (*idx > 0) { buf[*idx] = '\0'; *idx = 0; return true; }
        } else if (*idx < bufsz - 1) {
            buf[(*idx)++] = ch;
        }
    }
    return false;
}


// ============================================================================
//  MODE 1 : CALIBRATION
// ============================================================================
#if MODE == MODE_CALIBRATE

extern "C" void app_main(void) {
    printf("\n=== MODE: CALIBRATION ===\n");
    printf("Motor will move during alignment - keep the shaft FREE.\n\n");

    fault_pin_init();

    // sensor
    sensor.init();
    motor.linkSensor(&sensor);

    // driver
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = HARD_VOLTAGE_CEIL;
    driver.init();
    motor.linkDriver(&driver);

    // motor: voltage torque mode (no current sense on the Mini)
    motor.voltage_limit     = HARD_VOLTAGE_CEIL;
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    // enable verbose monitoring so alignment / detection prints
    uart_console_init();
    SimpleFOCDebug::enable();

    motor.init();
    motor.initFOC();   // aligns sensor + detects pole pairs/zero electric angle

    printf("\n========== CALIBRATION RESULTS ==========\n");
    printf("zero_electric_angle : %.4f\n", motor.zero_electric_angle);
    printf("sensor_direction    : %d\n", (int)motor.sensor_direction);
    printf("pole_pairs          : %d\n", motor.pole_pairs);
    printf("phase_resistance    : %.6f\n", motor.phase_resistance);
    printf("KV_rating           : %.2f\n", motor.KV_rating);
    printf("=========================================\n");
    printf("Copy these into the constants, then set MODE = MODE_RUN.\n\n");

    // idle: print live angle so you can sanity-check by turning the shaft
    while (1) {
        sensor.update();
        printf("angle(rad): %.3f\n", sensor.getAngle());
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

#endif // MODE_CALIBRATE


// ============================================================================
//  MODE 2 : RUN  (dual-core, mirrors the real EMG architecture)
// ============================================================================
#if MODE == MODE_RUN

// ----- CORE 0 : FOC loop, free-running, owns the motor -----
static void foc_task(void *arg) {
    for (;;) {
        if (fault_active()) {
            g_fault_latched = true;
            motor.move(0.0f);
            motor.disable();
        } else {
            motor.loopFOC();
            float cmd = g_target_voltage;            // atomic read
            if (cmd >  HARD_VOLTAGE_CEIL) cmd =  HARD_VOLTAGE_CEIL;
            if (cmd < -HARD_VOLTAGE_CEIL) cmd = -HARD_VOLTAGE_CEIL;
            motor.move(cmd);
        }
        // publish kinematics to Core 1 (atomic writes)
        g_shaft_angle    = motor.shaft_angle;
        g_shaft_velocity = motor.shaft_velocity;
        // no delay: owns Core 0
    }
}

// ----- CORE 1 : serial writer (stands in for the future EMG task) -----
static void writer_task(void *arg) {
    char buf[16];
    size_t idx = 0;
    int64_t last_print = 0;

    for (;;) {
        if (stdin_read_line(buf, sizeof(buf), &idx)) {
            float pct = atof(buf);                  // -100..+100
            if (pct >  100.0f) pct =  100.0f;
            if (pct < -100.0f) pct = -100.0f;
            float volts = (pct / 100.0f) * MAX_TARGET_VOLTAGE;
            g_target_voltage = volts;               // atomic write to Core 0
            printf("[cmd] %.1f%%  ->  %.3f V\n", pct, volts);
        }

        int64_t now = esp_timer_get_time();         // microseconds
        if (now - last_print >= 200000) {           // 5 Hz
            last_print = now;
            if (g_fault_latched) {
                printf("*** DRIVER FAULT - outputs disabled. Reset to clear. ***\n");
            } else {
                printf("angle(rad): %.3f   vel(rad/s): %.3f   target(V): %.3f\n",
                       g_shaft_angle, g_shaft_velocity, g_target_voltage);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));               // yield
    }
}

extern "C" void app_main(void) {
    printf("\n=== MODE: RUN ===\n");
    printf("Type a number -100..100 then Enter. 100 = MAX_TARGET_VOLTAGE.\n");
    printf("Negative = opposite direction. 0 = stop.\n\n");

    fault_pin_init();
    stdin_set_nonblocking();      // was: uart_console_init();

    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = HARD_VOLTAGE_CEIL;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_limit     = HARD_VOLTAGE_CEIL;
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    // After calibration, paste these to skip re-alignment each boot:
    motor.zero_electric_angle = 1.1598f;
    motor.sensor_direction    = Direction::CW;   // sensor_direction = 1 -> CW

    motor.init();
    motor.initFOC();
    motor.enable();            // <-- ensure outputs are active

    printf("FOC ready. Spawning tasks.\n\n");

    // FOC on Core 0 (free-running, high priority, alone)
    xTaskCreatePinnedToCore(foc_task, "FOC", 8192, NULL,
                            configMAX_PRIORITIES - 1, NULL, 0);
    // Writer/serial on Core 1 (lower priority)
    xTaskCreatePinnedToCore(writer_task, "WRITER", 4096, NULL, 5, NULL, 1);

    // app_main can return; tasks keep running.
}

#endif // MODE_RUN