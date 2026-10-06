/* ============================================================================
 *  EMG ORTHOSIS - INTEGRATED FIRMWARE (REAL FOC + EMG CONTROL)
 *  ----------------------------------------------------------------------------
 *  Real now: FOC motor control (esp_simplefoc, Core 0), full EMG pipeline,
 *  feature extraction, torque equation, mode LERP, limits, two-tier safety.
 *  Still mocked: IMU (manual flags via serial), AI (mode from serial 1-4).
 *
 *  CORE 0:
 *    foc_task    - real FOC. Reads the CHOSEN target torque (A or B, compile
 *                  flag), converts torque->voltage, drives motor.move().
 *                  Publishes real shaft angle/velocity. Watches DRV8313 FAULT:
 *                  on fault -> LATCH -> freewheel -> stays latched (full reset
 *                  to recover). Prints only in DEBUG MODE 2.
 *
 *  CORE 1:
 *    emg_task    - dual-channel ADC -> filter -> 500 Hz envelope (x2 MVC norms)
 *                  -> 20 Hz features -> mode from AI -> LERP G/D/K
 *                  -> torque eq (x2: A,B) -> limits + safety -> two torques.
 *                  Reads REAL angle/velocity from foc_task. Prints DBG 1.
 *                  Runs noise-baseline then MVC AFTER motor alignment.
 *    imu_task    - MOCK. collision/spasm flags from serial. Silent.
 *    ai_task     - MOCK. mode 1-4 from serial -> emg_task. Prints DBG 3.
 *    serial_task - READ ONLY. Routes input to mocks; sets debug mode.
 *
 *  STARTUP SEQUENCE:
 *    1) motor align (fresh initFOC, or stored calib via compile flag)
 *    2) EMG noise baseline (relax)  3) EMG MVC (contract)  4) RUN (mode REST)
 *
 *  SERIAL COMMANDS (runtime):
 *    1..4  mode (->AI)   collision / spasm / clear (->IMU)   dbg 1|2|3
 *    NOTE: 'clear' clears collision/spasm only. A latched DRIVER FAULT
 *          requires a full reset (power cycle / reset button).
 *
 *  COMPILE FLAGS (below): DRIVE_NORM_B, USE_STORED_CALIB
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
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"

#include "esp_simplefoc.h"

// ============================================================================
//  COMPILE FLAGS
// ============================================================================
// Which MVC normalization drives the REAL motor. Default = A.
// Define DRIVE_NORM_B to drive with method B instead. The other is still
// computed and shown in debug mode 2 for comparison, but does NOT move the motor.
//#define DRIVE_NORM_B

// Motor alignment: default runs a FRESH initFOC() each boot (shaft must be
// free to move). Define USE_STORED_CALIB to skip alignment and apply the
// hardcoded constants below instead (instant boot; assumes wiring unchanged).
//#define USE_STORED_CALIB

// Stored calibration values (only used if USE_STORED_CALIB is defined).
// Update these from a fresh alignment run's printout.
#define STORED_ZERO_ELEC_ANGLE   0.03f
#define STORED_SENSOR_DIR        Direction::CW


// ============================================================================
//  PIN MAP  (from working bench code; confirmed no overlap with EMG ADC)
// ============================================================================
#define PIN_IN1       41
#define PIN_IN2       40
#define PIN_IN3       39
#define PIN_ENABLE    35
#define PIN_FAULT     48          // DRV8313 FAULT, active-low input

#define PIN_SPI_MOSI  GPIO_NUM_21
#define PIN_SPI_MISO  GPIO_NUM_38
#define PIN_SPI_SCK   GPIO_NUM_37
#define PIN_SPI_CS    GPIO_NUM_47

// ============================================================================
//  CONFIG - EMG
// ============================================================================
#define ADC_CHANNEL_A        ADC_CHANNEL_3   // bicep
#define ADC_CHANNEL_B        ADC_CHANNEL_4   // tricep
#define SAMPLE_RATE_HZ       2000
#define READ_LEN             512
#define INTERNAL_BUF_SIZE    2048

#define BASELINE_TIME_SEC    5
#define MVC_TIME_SEC         5

#define FEATURE_WINDOW_LEN   400              // 200 ms @ 2 kHz
#define FEATURE_HOP_SAMPLES  100              // 20 Hz

// ============================================================================
//  CONFIG - MOTOR  (datasheet-confirmed)
// ============================================================================
#define MOTOR_POLE_PAIRS   11
#define MOTOR_KV           141.0f
#define SUPPLY_VOLTAGE     11.1f
#define MOTOR_KT           0.0677f            // N.m/A (from load-test on datasheet)
#define MOTOR_PHASE_R      5.6f               // ohm (datasheet, confirmed)

// 3 V continuous ceiling = ~0.5 A stall = air-cooled-safe at low speed.
#define HARD_VOLTAGE_CEIL  3.0f

// ============================================================================
//  CONFIG - CONTROL / MODES
//  Constants derived for V_CEIL = 3 V:
//    TORQUE_MAX = (V_CEIL/R)*Kt = (3.0/5.6)*0.0677 ~= 0.036 N.m
//    G_max (full contraction diff=1 -> TORQUE_MAX) = 0.036
//  G/D scaled per mode; D values are order-of-magnitude (tune with real omega).
// ============================================================================
typedef enum { MODE_REST = 1, MODE_REACH = 2, MODE_LIFT = 3, MODE_HOLD = 4 } RehabMode;

typedef struct { float G; float D; float K; } ModeCoeffs;

//                          G        D        K
static const ModeCoeffs MODE_TABLE[5] = {
    { 0.000f, 0.000f, 0.0f },   // [0] unused
    { 0.000f, 0.00f, 0.0f },   // [1] REST  : no drive, light hold-damping
    { 0.029f, 0.00f, 0.0f },   // [2] REACH : strong drive, light damping
/**   { 0.029f, 0.004f, 0.0f },   // [2] REACH : strong drive, light damping*/ 
    { 0.036f, 0.008f, 0.0f },   // [3] LIFT  : full drive, medium damping
    { 0.011f, 0.015f, 0.0f },   // [4] HOLD  : gentle drive, highest damping
};
// K (gravity comp) = 0 everywhere for now; structure kept for later.

#define LERP_TIME_SEC        0.30f

// Global limits (same for all modes)
#define EMG_DEADBAND         0.05f            // on normalized differential
#define TORQUE_MAX           0.036f           // N.m ceiling (= 3 V worth)
#define TORQUE_SLEW_PER_SEC  0.20f            // max d|T|/dt (N.m per second)
#define D_SAFE               0.015f           // spasm damping (> any mode D)
#define OMEGA_DEADBAND  0.10f    // rad/s; below this, treat shaft as still

// Range-of-motion soft limits (rad, joint)
#define ROM_MIN_RAD         -100000000.0f
#define ROM_MAX_RAD          100000000.0f
#define ROM_BRAKE_BAND       0.30f

#define CONTROL_PERIOD_MS    2                // ~500 Hz fallback dt

static const char *TAG = "EMG_FW";

// Pick the torque that actually drives the motor (compile flag).
#ifdef DRIVE_NORM_B
  #define DRIVE_TORQUE  g_target_torque_B
#else
  #define DRIVE_TORQUE  g_target_torque_A
#endif

// ============================================================================
//  DEBUG MODE
// ============================================================================
typedef enum { DBG_EMG = 1, DBG_FOC = 2, DBG_AI = 3 } DebugMode;
volatile int g_debug_mode = DBG_EMG;

// ============================================================================
//  SHARED STATE
// ============================================================================
// emg_task -> foc_task : two target torques (N.m)
volatile float g_target_torque_A = 0.0f;
volatile float g_target_torque_B = 0.0f;

// foc_task -> emg_task : REAL kinematics
volatile float g_shaft_angle    = 0.0f;
volatile float g_shaft_velocity = 0.0f;
volatile float g_angle_offset = 0.0f;   // captured at boot; makes start = 0

// foc_task -> everyone : latched driver fault (full reset to clear)
volatile bool  g_fault_latched  = false;

// startup handshake: EMG waits until FOC has finished motor alignment
volatile bool  g_motor_ready    = false;

// imu_task -> emg_task : safety flags
volatile bool g_flag_collision = false;
volatile bool g_flag_spasm     = false;

// serial_task -> imu_task
volatile bool g_req_collision = false;
volatile bool g_req_spasm     = false;
volatile bool g_req_clear     = false;

// serial_task -> ai_task
volatile int  g_req_mode = MODE_REST;
// ai_task -> emg_task
volatile int  g_active_mode = MODE_REST;

// emg_task -> ai_task : feature vector (debug display)
typedef struct {
    float rms_a, rms_b;
    float mav_a, mav_b;
    int   zc_a,  zc_b;
    float wl_a,  wl_b;
} FeatureVector;
volatile FeatureVector g_features = {};

// foc_task published voltages (debug mode 2)
volatile float g_volt_A = 0.0f, g_volt_B = 0.0f;

// ============================================================================
//  SimpleFOC objects
// ============================================================================
BLDCMotor       motor  = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_R, MOTOR_KV);
BLDCDriver3PWM  driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);
AS5048a         sensor = AS5048a(SPI2_HOST, PIN_SPI_SCK, PIN_SPI_MISO,
                                 PIN_SPI_MOSI, PIN_SPI_CS);

// ============================================================================
//  FAULT pin
// ============================================================================
static void fault_pin_init() {
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_FAULT);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);
}
static inline bool fault_active() { return gpio_get_level((gpio_num_t)PIN_FAULT) == 0; }

// ============================================================================
//  EMG filter + calibration state
// ============================================================================
static adc_continuous_handle_t adc_handle = NULL;
float coeffs_notch[5], coeffs_hpf[5], coeffs_lpf[5];

typedef struct {
    float delay_notch[2];
    float delay_hpf[2];
    float delay_lpf[2];
} ChannelFilterState;

typedef struct {
    bool     base_done;
    double   base_sum;
    uint32_t base_count;
    float    baseline;
    float    mvc_a_peak;     // method A: fixed-window peak
    float    mvc_b_peak;     // method B: adaptive running peak
} ChannelCalib;

static ChannelFilterState filt[2] = {};
static ChannelCalib       cal[2]  = {};

static float win[2][FEATURE_WINDOW_LEN] = {{0}};
static int   win_pos[2] = {0};
static int   win_filled[2] = {0};
static int   hop_counter = 0;

static void init_filters() {
    dsps_biquad_gen_notch_f32(coeffs_notch, 50.0f / SAMPLE_RATE_HZ, 0.0f, 0.707f);
    dsps_biquad_gen_hpf_f32(coeffs_hpf,    20.0f / SAMPLE_RATE_HZ, 0.707f);
    dsps_biquad_gen_lpf_f32(coeffs_lpf,     5.0f / SAMPLE_RATE_HZ, 0.707f);
}

static inline float mvc_norm(float env, float peak) {
    if (peak < 1e-6f) return 0.0f;
    float n = env / peak;
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

static void compute_features(int ch) {
    int n = win_filled[ch] ? FEATURE_WINDOW_LEN : win_pos[ch];
    if (n < 2) return;
    float sumsq = 0.0f, summav = 0.0f, wl = 0.0f;
    int   zc = 0;
    float prev = win[ch][0];
    for (int i = 0; i < n; i++) {
        float x = win[ch][i];
        sumsq  += x * x;
        summav += fabsf(x);
        if (i > 0) {
            wl += fabsf(x - prev);
            if ((x > 0.0f && prev < 0.0f) || (x < 0.0f && prev > 0.0f)) zc++;
        }
        prev = x;
    }
    float rms = sqrtf(sumsq / n);
    float mav = summav / n;
    if (ch == 0) { g_features.rms_a = rms; g_features.mav_a = mav; g_features.zc_a = zc; g_features.wl_a = wl; }
    else         { g_features.rms_b = rms; g_features.mav_b = mav; g_features.zc_b = zc; g_features.wl_b = wl; }
}

// ============================================================================
//  CONTROL: torque equation + limits + safety (runs inside emg_task)
// ============================================================================
static float cur_G = 0.0f, cur_D = 0.0f, cur_K = 0.0f;   // boot = REST
static float last_T_A = 0.0f, last_T_B = 0.0f;

static float compute_torque(float diff, float omega, float dt, float *last_T) {
    if (omega > -OMEGA_DEADBAND && omega < OMEGA_DEADBAND) omega = 0.0f;
    // --- safety override: latched fault and collision => freewheel ---
    if (g_fault_latched || g_flag_collision) {
        *last_T = 0.0f;
        return 0.0f;
    }
    // --- spasm => damping only ---
    if (g_flag_spasm) {
        float t = -D_SAFE * omega;
        if (t >  TORQUE_MAX) t =  TORQUE_MAX;
        if (t < -TORQUE_MAX) t = -TORQUE_MAX;
        *last_T = t;
        return t;
    }

    // --- deadband on EMG differential ---
    if (diff > -EMG_DEADBAND && diff < EMG_DEADBAND) diff = 0.0f;

    // --- T = K*sin(theta) + G*(b-t) - D*omega  (K=0 now) ---
    float theta = g_shaft_angle;
    float T = cur_K * sinf(theta) + cur_G * diff - cur_D * omega;

    // --- ROM soft-brake ---
    if (theta > ROM_MAX_RAD - ROM_BRAKE_BAND) {
        float over = (theta - (ROM_MAX_RAD - ROM_BRAKE_BAND)) / ROM_BRAKE_BAND;
        if (over > 1.0f) over = 1.0f;
        if (T > 0.0f) T *= (1.0f - over);
        T += -D_SAFE * omega * over;
    } else if (theta < ROM_MIN_RAD + ROM_BRAKE_BAND) {
        float over = ((ROM_MIN_RAD + ROM_BRAKE_BAND) - theta) / ROM_BRAKE_BAND;
        if (over > 1.0f) over = 1.0f;
        if (T < 0.0f) T *= (1.0f - over);
        T += -D_SAFE * omega * over;
    }

    // --- saturation ---
    if (T >  TORQUE_MAX) T =  TORQUE_MAX;
    if (T < -TORQUE_MAX) T = -TORQUE_MAX;

    // --- slew-rate limit (only limit RISING magnitude, snap toward zero) ---
    // T here is the post-equation, post-saturation target.
    if (T == 0.0f) {
        // target is zero -> release immediately, no slew hunting
        *last_T = 0.0f;
        return 0.0f;
    }
    float max_step = TORQUE_SLEW_PER_SEC * dt;
    float dT = T - *last_T;
    if (dT >  max_step) T = *last_T + max_step;
    if (dT < -max_step) T = *last_T - max_step;
    if (T > -0.002f && T < 0.002f) T = 0.0f;   // keep the snap too
    *last_T = T;
    return T;
}

// ============================================================================
//  CORE 1 : MAIN EMG TASK
// ============================================================================
static void emg_task(void *arg) {
    printf(">>> EMG task on core %d\n", xPortGetCoreID());
    init_filters();

    // --- ADC init (both channels) ---
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = INTERNAL_BUF_SIZE,
        .conv_frame_size    = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t pat[2] = {};
    pat[0].atten = ADC_ATTEN_DB_12; pat[0].channel = ADC_CHANNEL_A; pat[0].unit = ADC_UNIT_1; pat[0].bit_width = 12;
    pat[1].atten = ADC_ATTEN_DB_12; pat[1].channel = ADC_CHANNEL_B; pat[1].unit = ADC_UNIT_1; pat[1].bit_width = 12;

    adc_continuous_config_t dig = {
        .pattern_num    = 2,
        .adc_pattern    = pat,
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig));

    // --- WAIT for the motor to finish aligning before we calibrate EMG ---
    while (!g_motor_ready) vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    uint8_t  raw[READ_LEN] = {0};
    uint32_t ret_num = 0;
    const int max_results = READ_LEN / SOC_ADC_DIGI_RESULT_BYTES;
    float bufA[max_results], bufB[max_results];

    const uint32_t base_target = SAMPLE_RATE_HZ * BASELINE_TIME_SEC;
    const uint32_t mvc_target  = SAMPLE_RATE_HZ * MVC_TIME_SEC;
    uint32_t mvc_count = 0;
    enum { PH_BASELINE, PH_MVC, PH_RUN } phase = PH_BASELINE;

    printf("\n[CAL] PHASE 1/2: NOISE BASELINE - relax both muscles for %d s...\n", BASELINE_TIME_SEC);
    int64_t last_ctrl_us = esp_timer_get_time();

    while (1) {
        if (adc_continuous_read(adc_handle, raw, READ_LEN, &ret_num, portMAX_DELAY) != ESP_OK)
            continue;

        int total = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
        int ca = 0, cb = 0;
        for (int i = 0; i < total; i++) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw[i * SOC_ADC_DIGI_RESULT_BYTES];
            float v = (float)p->type2.data;
            if      (p->type2.channel == ADC_CHANNEL_A && ca < max_results) bufA[ca++] = v;
            else if (p->type2.channel == ADC_CHANNEL_B && cb < max_results) bufB[cb++] = v;
        }

        float *bufs[2] = { bufA, bufB };
        int    cnts[2] = { ca, cb };
        float  env_raw[2] = {0, 0};

        for (int ch = 0; ch < 2; ch++) {
            float *b = bufs[ch]; int n = cnts[ch];
            ChannelFilterState *fs = &filt[ch];

            dsps_biquad_f32(b, b, n, coeffs_notch, fs->delay_notch);
            dsps_biquad_f32(b, b, n, coeffs_hpf,   fs->delay_hpf);

            // feature window gets filtered (bipolar) samples, pre-rectify
            for (int i = 0; i < n; i++) {
                win[ch][win_pos[ch]] = b[i];
                win_pos[ch] = (win_pos[ch] + 1) % FEATURE_WINDOW_LEN;
                if (win_pos[ch] == 0) win_filled[ch] = 1;
            }

            // envelope: rectify + LPF
            for (int i = 0; i < n; i++) b[i] = fabsf(b[i]);
            dsps_biquad_f32(b, b, n, coeffs_lpf, fs->delay_lpf);
            env_raw[ch] = (n > 0) ? b[n - 1] : 0.0f;
        }

        hop_counter += (ca < cb ? ca : cb);
        if (hop_counter >= FEATURE_HOP_SAMPLES) {
            hop_counter = 0;
            compute_features(0);
            compute_features(1);
        }

        // ===== CALIBRATION =====
        if (phase == PH_BASELINE) {
            int navg = (ca < cb ? ca : cb);
            for (int ch = 0; ch < 2; ch++) {
                float *b = bufs[ch];
                for (int i = 0; i < navg; i++) cal[ch].base_sum += b[i];
                cal[ch].base_count += navg;
            }
            if (cal[0].base_count >= base_target) {
                for (int ch = 0; ch < 2; ch++) {
                    cal[ch].baseline  = (float)(cal[ch].base_sum / cal[ch].base_count);
                    cal[ch].base_done = true;
                }
                printf("[CAL] baseline done: A=%.2f  B=%.2f\n", cal[0].baseline, cal[1].baseline);
                printf("[CAL] PHASE 2/2: MVC - contract MAX for %d s...\n", MVC_TIME_SEC);
                phase = PH_MVC;
            }
            continue;
        }

        if (phase == PH_MVC) {
            int nmvc = (ca < cb ? ca : cb);
            for (int ch = 0; ch < 2; ch++) {
                float *b = bufs[ch];
                for (int i = 0; i < nmvc; i++) {
                    float e = b[i] - cal[ch].baseline;
                    if (e < 0) e = 0;
                    if (e > cal[ch].mvc_a_peak) cal[ch].mvc_a_peak = e;
                }
            }
            mvc_count += nmvc;
            if (mvc_count >= mvc_target) {
                printf("[CAL] MVC done (A peaks): A=%.2f  B=%.2f\n", cal[0].mvc_a_peak, cal[1].mvc_a_peak);
                cal[0].mvc_b_peak = cal[0].mvc_a_peak;
                cal[1].mvc_b_peak = cal[1].mvc_a_peak;
                printf("[CAL] complete. RUN (mode REST).\n\n");
                phase = PH_RUN;
            }
            continue;
        }

        // ===== RUN =====
        float eA = env_raw[0] - cal[0].baseline; if (eA < 0) eA = 0;
        float eB = env_raw[1] - cal[1].baseline; if (eB < 0) eB = 0;

        if (eA > cal[0].mvc_b_peak) cal[0].mvc_b_peak = eA;
        if (eB > cal[1].mvc_b_peak) cal[1].mvc_b_peak = eB;

        float a_normA = mvc_norm(eA, cal[0].mvc_a_peak);
        float b_normA = mvc_norm(eB, cal[1].mvc_a_peak);
        float a_normB = mvc_norm(eA, cal[0].mvc_b_peak);
        float b_normB = mvc_norm(eB, cal[1].mvc_b_peak);

        int m = g_active_mode;
        if (m < 1 || m > 4) m = MODE_REST;
        const ModeCoeffs *tgt = &MODE_TABLE[m];

        // dt from SAMPLES processed this block, not wall-clock -> stable.
        // Each channel got ~min(ca,cb) samples at SAMPLE_RATE_HZ.
        int nproc = (ca < cb ? ca : cb);
        float dt = (float)nproc / (float)SAMPLE_RATE_HZ;
        if (dt <= 0.0f || dt > 0.1f) dt = CONTROL_PERIOD_MS / 1000.0f;

        float a = dt / LERP_TIME_SEC; if (a > 1.0f) a = 1.0f;
        cur_G += (tgt->G - cur_G) * a;
        cur_D += (tgt->D - cur_D) * a;
        cur_K += (tgt->K - cur_K) * a;

        float omega = g_shaft_velocity;
        float diffA = a_normA - b_normA;
        float diffB = a_normB - b_normB;
        float T_A = compute_torque(diffA, omega, dt, &last_T_A);
        float T_B = compute_torque(diffB, omega, dt, &last_T_B);

        g_target_torque_A = T_A;
        g_target_torque_B = T_B;

       if (g_debug_mode == DBG_EMG) {
            printf("Min:0,Max:1000,A_normA:%.1f,B_normA:%.1f,A_normB:%.1f,B_normB:%.1f,mode:%d,G:%.4f,D:%.4f,Ttgt:%.4f\n",
                a_normA*1000.0f, b_normA*1000.0f, a_normB*1000.0f, b_normB*1000.0f,
                m, cur_G, cur_D, g_target_torque_A);
        }
    }
}

// ============================================================================
//  CORE 0 : REAL FOC TASK
// ============================================================================
static inline float torque_to_voltage(float T) {
    float V = (T / MOTOR_KT) * MOTOR_PHASE_R;   // low-speed approx: V ~= I*R
    if (V >  HARD_VOLTAGE_CEIL) V =  HARD_VOLTAGE_CEIL;
    if (V < -HARD_VOLTAGE_CEIL) V = -HARD_VOLTAGE_CEIL;
    return V;
}

static void foc_task(void *arg) {
    printf(">>> FOC task on core %d\n", xPortGetCoreID());

    // ---- motor / driver / sensor bring-up ----
    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = HARD_VOLTAGE_CEIL;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_limit     = HARD_VOLTAGE_CEIL;
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

#ifdef USE_STORED_CALIB
    motor.zero_electric_angle = STORED_ZERO_ELEC_ANGLE;
    motor.sensor_direction    = STORED_SENSOR_DIR;
    printf(">>> using STORED calibration (no motion).\n");
    motor.init();
    motor.initFOC();        // with known offset/dir, this skips the search
#else
    printf(">>> FRESH alignment - keep shaft FREE...\n");
    motor.init();
    motor.initFOC();        // moves the motor to find zero electric angle
    printf(">>> alignment done: zero_elec=%.4f dir=%d pp=%d\n",
           motor.zero_electric_angle, (int)motor.sensor_direction, motor.pole_pairs);
#endif

    motor.enable();
    // capture boot angle as the zero reference
    motor.loopFOC();                       // update sensor so shaft_angle is fresh
    g_angle_offset = motor.shaft_angle;    // this position becomes "0"
    printf(">>> angle offset captured: %.4f (this is now zero)\n", g_angle_offset);
    g_motor_ready = true;   // release EMG task to begin its calibration

    for (;;) {
        // latched fault: stop driving, freewheel, stay latched (reset to clear)
        if (g_fault_latched) {
            motor.move(0.0f);
            motor.loopFOC();
            g_shaft_angle = motor.shaft_angle - g_angle_offset;
            g_shaft_velocity = motor.shaft_velocity;
            continue;
        }
        if (fault_active()) {
            g_fault_latched = true;
            motor.move(0.0f);
            motor.disable();
            printf("*** DRIVER FAULT latched - freewheel. FULL RESET to recover. ***\n");
            continue;
        }

        motor.loopFOC();

        float T = DRIVE_TORQUE;                 // chosen normalization (compile flag)
        float V = torque_to_voltage(T);
        motor.move(V);

        // publish both voltages for debug 2 (only DRIVE_TORQUE actually moves motor)
        g_volt_A = torque_to_voltage(g_target_torque_A);
        g_volt_B = torque_to_voltage(g_target_torque_B);

        g_shaft_angle = motor.shaft_angle - g_angle_offset;
        g_shaft_velocity = motor.shaft_velocity;

        if (g_debug_mode == DBG_FOC) {
            static int64_t last_print_us = 0;
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_print_us >= 50000) {   // 50 ms = 20 Hz
                last_print_us = now_us;
                printf("Min:-3000,Max:3000,T_A_mNm:%.1f,V_A_mV:%.1f,T_B_mNm:%.1f,V_B_mV:%.1f,angle_mrad:%.1f,vel_mrad_s:%.1f\n",
                    g_target_torque_A * 1000.0f, g_volt_A * 1000.0f,
                    g_target_torque_B * 1000.0f, g_volt_B * 1000.0f,
                    g_shaft_angle, g_shaft_velocity);
            }
        }
    }
}

// ============================================================================
//  CORE 1 : IMU TASK (MOCK)
// ============================================================================
static void imu_task(void *arg) {
    printf(">>> IMU(mock) task on core %d\n", xPortGetCoreID());
    for (;;) {
        if (g_req_clear)     { g_flag_collision = false; g_flag_spasm = false; g_req_clear = false; }
        if (g_req_collision) { g_flag_collision = true;  g_req_collision = false; }
        if (g_req_spasm)     { g_flag_spasm     = true;  g_req_spasm = false; }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ============================================================================
//  CORE 1 : AI TASK (MOCK)
// ============================================================================
static void ai_task(void *arg) {
    printf(">>> AI(mock) task on core %d\n", xPortGetCoreID());
    for (;;) {
        g_active_mode = g_req_mode;
        if (g_debug_mode == DBG_AI) {
            FeatureVector f;
            f.rms_a = g_features.rms_a; f.rms_b = g_features.rms_b;
            f.mav_a = g_features.mav_a; f.mav_b = g_features.mav_b;
            f.zc_a  = g_features.zc_a;  f.zc_b  = g_features.zc_b;
            f.wl_a  = g_features.wl_a;  f.wl_b  = g_features.wl_b;
            printf("mode:%d | RMS a:%.1f b:%.1f | MAV a:%.1f b:%.1f | ZC a:%d b:%d | WL a:%.1f b:%.1f\n",
                   g_active_mode, f.rms_a, f.rms_b, f.mav_a, f.mav_b,
                   f.zc_a, f.zc_b, f.wl_a, f.wl_b);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
//  CORE 1 : SERIAL TASK (read-only router)
// ============================================================================
static void console_init() {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}

static void handle_command(const char *line) {
    if (strlen(line) == 1 && line[0] >= '1' && line[0] <= '4') {
        g_req_mode = line[0] - '0';
        return;
    }
    if      (strcmp(line, "collision") == 0) g_req_collision = true;
    else if (strcmp(line, "spasm")     == 0) g_req_spasm     = true;
    else if (strcmp(line, "clear")     == 0) g_req_clear     = true;
    else if (strncmp(line, "dbg", 3) == 0) {
        int d = atoi(line + 3);
        if (d >= 1 && d <= 3) g_debug_mode = d;
    }
}

static void serial_task(void *arg) {
    printf(">>> SERIAL task on core %d\n", xPortGetCoreID());
    char line[32];
    size_t idx = 0;
    for (;;) {
        int c = getchar();
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        char ch = (char)c;
        if (ch == '\n' || ch == '\r') {
            if (idx > 0) { line[idx] = '\0'; idx = 0; handle_command(line); }
        } else if (idx < sizeof(line) - 1) {
            line[idx++] = ch;
        }
    }
}

// ============================================================================
//  app_main
// ============================================================================
extern "C" void app_main(void) {
    console_init();
    fault_pin_init();

    printf("\n=== EMG ORTHOSIS - INTEGRATED FIRMWARE (REAL FOC) ===\n");
    printf("Cmds: 1-4 mode | collision | spasm | clear | dbg 1|2|3\n");
    printf("Debug 1=EMG envelope  2=FOC torque/volt  3=AI features\n");
#ifdef DRIVE_NORM_B
    printf("Motor driven by: normalization B\n");
#else
    printf("Motor driven by: normalization A\n");
#endif
    printf("Startup: motor aligns FIRST (shaft free), then EMG calibration.\n\n");

    // CORE 0 : real FOC (aligns motor, sets g_motor_ready when done)
    xTaskCreatePinnedToCore(foc_task,    "FOC",    8192, NULL, configMAX_PRIORITIES - 1, NULL, 0);

    // CORE 1 : EMG (waits for g_motor_ready), AI, IMU, Serial
    xTaskCreatePinnedToCore(emg_task,    "EMG",    8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(ai_task,     "AI",     4096, NULL, 6,  NULL, 1);
    xTaskCreatePinnedToCore(imu_task,    "IMU",    3072, NULL, 6,  NULL, 1);
    xTaskCreatePinnedToCore(serial_task, "SERIAL", 4096, NULL, 5,  NULL, 1);
}