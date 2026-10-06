/* ============================================================================
 *  EMG ORTHOSIS - DATA COLLECTION FIRMWARE (REAL FOC + EMG + REAL BNO055 IMU)
 *  ----------------------------------------------------------------------------
 *  Based on the integrated firmware, with AI training-data collection added.
 *
 *  DEBUG / COLLECTION MODES (set via "dbg N"):
 *    dbg 1  - EMG envelope plot
 *    dbg 2  - FOC torque/voltage plot
 *    dbg 3  - AI feature line
 *    dbg 4  - IMU stats
 *    dbg 5  - CSV DATA COLLECTION, MOTOR OFF (clean, no motor EMI)
 *    dbg 6  - CSV DATA COLLECTION, MOTOR ENERGIZED @ 0 TORQUE (idle motor EMI)
 *
 *  In dbg 5/6 the emg_task prints copy-pasteable CSV (header once, then rows
 *  at the 20 Hz feature rate). The selected MODE (1..5) is the LABEL:
 *    1 Rest  2 Reach  3 Lift  4 Hold  5 Spasm
 *  The motor never drives your limb during collection (dbg5 off, dbg6 zero
 *  torque), so labelled intent gestures are not contaminated by assist motion.
 *  The motor_emi column is 0 in dbg5, 1 in dbg6, so merged data is self-describing.
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
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
//#define DRIVE_NORM_B
#define USE_STORED_CALIB

#define STORED_ZERO_ELEC_ANGLE   3.6350f
#define STORED_SENSOR_DIR        Direction::CW

// ============================================================================
//  PIN MAP
// ============================================================================
#define PIN_IN1       41
#define PIN_IN2       40
#define PIN_IN3       39
#define PIN_ENABLE    35
#define PIN_FAULT     48

#define PIN_SPI_MOSI  GPIO_NUM_21
#define PIN_SPI_MISO  GPIO_NUM_38
#define PIN_SPI_SCK   GPIO_NUM_37
#define PIN_SPI_CS    GPIO_NUM_47

#define I2C_PORT      I2C_NUM_0
#define SDA_PIN       10
#define SCL_PIN       9
#define INT_PIN       11

// ============================================================================
//  CONFIG - IMU (BNO055)
// ============================================================================
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

#define SPASM_CHANGE_THRESHOLD 3.8f
#define SPASM_SPIKE_THRESHOLD 5
#define SPASM_DIRECTION_CHANGE_THRESHOLD 3
#define COLLISION_MAG_THRESHOLD 16.0f

typedef enum { AXIS_NONE = 0, AXIS_X, AXIS_Y, AXIS_Z } axis_t;

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
#define FEATURE_WINDOW_LEN   400
#define FEATURE_HOP_SAMPLES  100

// SSC needs a small threshold so noise-level slope flips are not counted.
#define SSC_THRESHOLD        2.0f

// ============================================================================
//  CONFIG - MOTOR
// ============================================================================
#define MOTOR_POLE_PAIRS   11
#define MOTOR_KV           141.0f
#define SUPPLY_VOLTAGE     11.1f
#define MOTOR_KT           0.0677f
#define MOTOR_PHASE_R      5.6f
#define HARD_VOLTAGE_CEIL  3.0f

// ============================================================================
//  CONFIG - CONTROL / MODES
// ============================================================================
typedef enum { MODE_REST = 1, MODE_REACH = 2, MODE_LIFT = 3, MODE_HOLD = 4, MODE_SPASM = 5 } RehabMode;
typedef struct { float G; float D; float K; } ModeCoeffs;

// MODE_TABLE only covers the 4 ASSIST modes (1..4). Mode 5 (SPASM) is NOT an
// assist mode - at runtime it routes to the spasm override (damping only), so
// it has no coefficients here. For data collection it is purely a LABEL.
//                          G        D        K
static const ModeCoeffs MODE_TABLE[5] = {
    { 0.000f, 0.000f, 0.00f },  // [0] unused
    { 0.000f, 0.000f, 0.00f },  // [1] REST
    { 0.029f, 0.000f, 0.00f },  // [2] REACH
    { 0.00f,  0.008f, 0.00f },  // [3] LIFT
    { 0.000f, 0.00f,  0.03f },  // [4] HOLD
};

#define LERP_TIME_SEC        0.30f
#define EMG_DEADBAND         0.05f
#define TORQUE_MAX           0.036f
#define TORQUE_SLEW_PER_SEC  0.20f
#define D_SAFE               0.015f
#define OMEGA_DEADBAND       0.10f
#define ROM_MIN_RAD         -100000000.0f
#define ROM_MAX_RAD          100000000.0f
#define ROM_BRAKE_BAND       0.30f
#define CONTROL_PERIOD_MS    2

static const char *TAG = "EMG_FW";

#ifdef DRIVE_NORM_B
  #define DRIVE_TORQUE  g_target_torque_B
#else
  #define DRIVE_TORQUE  g_target_torque_A
#endif

// ============================================================================
//  DEBUG / COLLECTION MODE
// ============================================================================
typedef enum {
    DBG_EMG = 1, DBG_FOC = 2, DBG_AI = 3, DBG_IMU = 4,
    DBG_CSV_MOTOR_OFF = 5,     // CSV, motor disabled
    DBG_CSV_MOTOR_EMI = 6      // CSV, motor energized @ 0 torque
} DebugMode;
volatile int g_debug_mode = DBG_EMG;

static inline bool is_csv_mode(int m) { return (m == DBG_CSV_MOTOR_OFF || m == DBG_CSV_MOTOR_EMI); }

// ============================================================================
//  SHARED STATE
// ============================================================================
volatile float g_target_torque_A = 0.0f;
volatile float g_target_torque_B = 0.0f;

volatile float g_shaft_angle    = 0.0f;
volatile float g_shaft_velocity = 0.0f;
volatile float g_angle_offset   = 0.0f;

volatile bool  g_fault_latched  = false;
volatile bool  g_motor_ready    = false;

volatile bool  g_flag_collision = false;
volatile bool  g_flag_spasm     = false;
volatile float g_imu_gravity_angle_rad = 0.0f;
volatile float g_imu_acc_mag    = 0.0f;          // bridged for CSV logging

volatile bool  g_req_clear      = false;
volatile int   g_req_mode       = MODE_REST;
volatile int   g_active_mode    = MODE_REST;

typedef struct {
    float rms_a, rms_b, mav_a, mav_b, wl_a, wl_b;
    int   zc_a, zc_b;
    int   ssc_a, ssc_b;          // slope sign changes
    float var_a, var_b;          // variance
} FeatureVector;
volatile FeatureVector g_features = {};

volatile float g_volt_A = 0.0f, g_volt_B = 0.0f;

// ============================================================================
//  SimpleFOC objects
// ============================================================================
BLDCMotor       motor  = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_R, MOTOR_KV);
BLDCDriver3PWM  driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);
AS5048a         sensor = AS5048a(SPI2_HOST, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
static i2c_master_dev_handle_t imu_i2c_dev = NULL;

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

typedef struct { float delay_notch[2]; float delay_hpf[2]; float delay_lpf[2]; } ChannelFilterState;
typedef struct { bool base_done; double base_sum; uint32_t base_count; float baseline; float mvc_a_peak; float mvc_b_peak; } ChannelCalib;

static ChannelFilterState filt[2] = {};
static ChannelCalib       cal[2]  = {};

static float win[2][FEATURE_WINDOW_LEN] = {{0}};
static int   win_pos[2] = {0}, win_filled[2] = {0}, hop_counter = 0;

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

    float sumsq = 0.0f, summav = 0.0f, wl = 0.0f, sum = 0.0f;
    int   zc = 0, ssc = 0;
    float prev = win[ch][0];
    float prevprev = win[ch][0];

    for (int i = 0; i < n; i++) {
        float x = win[ch][i];
        sumsq  += x * x;
        summav += fabsf(x);
        sum    += x;
        if (i > 0) {
            wl += fabsf(x - prev);
            // zero crossing
            if ((x > 0.0f && prev < 0.0f) || (x < 0.0f && prev > 0.0f)) zc++;
        }
        if (i > 1) {
            // slope sign change: the slope direction flips, with a noise threshold
            float d1 = x - prev;
            float d2 = prev - prevprev;
            if (((d1 > 0.0f && d2 < 0.0f) || (d1 < 0.0f && d2 > 0.0f)) &&
                (fabsf(d1) > SSC_THRESHOLD || fabsf(d2) > SSC_THRESHOLD)) {
                ssc++;
            }
        }
        prevprev = prev;
        prev = x;
    }

    float rms  = sqrtf(sumsq / n);
    float mav  = summav / n;
    float mean = sum / n;
    float var  = (sumsq / n) - (mean * mean);   // E[x^2] - E[x]^2
    if (var < 0.0f) var = 0.0f;

    if (ch == 0) {
        g_features.rms_a = rms; g_features.mav_a = mav; g_features.zc_a = zc; g_features.wl_a = wl;
        g_features.ssc_a = ssc; g_features.var_a = var;
    } else {
        g_features.rms_b = rms; g_features.mav_b = mav; g_features.zc_b = zc; g_features.wl_b = wl;
        g_features.ssc_b = ssc; g_features.var_b = var;
    }
}

// ============================================================================
//  CONTROL: torque equation + limits + safety
// ============================================================================
static float cur_G = 0.0f, cur_D = 0.0f, cur_K = 0.0f;
static float last_T_A = 0.0f, last_T_B = 0.0f;

static float compute_torque(float diff, float omega, float dt, float *last_T) {
    if (omega > -OMEGA_DEADBAND && omega < OMEGA_DEADBAND) omega = 0.0f;

    if (g_fault_latched || g_flag_collision) {
        *last_T = 0.0f;
        return 0.0f;
    }
    if (g_flag_spasm) {
        float t = -D_SAFE * omega;
        if (t >  TORQUE_MAX) t =  TORQUE_MAX;
        if (t < -TORQUE_MAX) t = -TORQUE_MAX;
        *last_T = t;
        return t;
    }

    if (diff > -EMG_DEADBAND && diff < EMG_DEADBAND) diff = 0.0f;

    float theta = g_shaft_angle;
    float T = cur_K * sinf(g_imu_gravity_angle_rad) + cur_G * diff - cur_D * omega;

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

    if (T >  TORQUE_MAX) T =  TORQUE_MAX;
    if (T < -TORQUE_MAX) T = -TORQUE_MAX;

    if (T == 0.0f) { *last_T = 0.0f; return 0.0f; }

    float max_step = TORQUE_SLEW_PER_SEC * dt;
    float dT = T - *last_T;
    if (dT >  max_step) T = *last_T + max_step;
    if (dT < -max_step) T = *last_T - max_step;
    if (T > -0.002f && T < 0.002f) T = 0.0f;
    *last_T = T;
    return T;
}

// ============================================================================
//  CSV HELPERS  (data collection)
// ============================================================================
static void print_csv_header(void) {
    // One header line. Columns chosen to be exactly what the AI receives,
    // plus context (IMU, kinematics) and bookkeeping (timestamp, label, emi).
    printf("timestamp_ms,mode,motor_emi,"
           "rms_a,rms_b,mav_a,mav_b,zc_a,zc_b,wl_a,wl_b,ssc_a,ssc_b,var_a,var_b,"
           "env_a,env_b,diff,"
           "grav_angle_rad,acc_mag,shaft_angle,shaft_vel\n");
}

// ============================================================================
//  CORE 1 : MAIN EMG TASK
// ============================================================================
static void emg_task(void *arg) {
    printf(">>> EMG task on core %d\n", xPortGetCoreID());
    init_filters();

    adc_continuous_handle_cfg_t adc_config = {};
    adc_config.max_store_buf_size = INTERNAL_BUF_SIZE;
    adc_config.conv_frame_size    = READ_LEN;
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

    // CSV state: detect entry into a CSV mode so the header prints once.
    int prev_dbg = g_debug_mode;
    bool csv_header_done = false;

    printf("\n[CAL] PHASE 1/2: NOISE BASELINE - relax both muscles for %d s...\n", BASELINE_TIME_SEC);

    while (1) {
        if (adc_continuous_read(adc_handle, raw, READ_LEN, &ret_num, portMAX_DELAY) != ESP_OK) continue;

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

            for (int i = 0; i < n; i++) {
                win[ch][win_pos[ch]] = b[i];
                win_pos[ch] = (win_pos[ch] + 1) % FEATURE_WINDOW_LEN;
                if (win_pos[ch] == 0) win_filled[ch] = 1;
            }

            for (int i = 0; i < n; i++) b[i] = fabsf(b[i]);
            dsps_biquad_f32(b, b, n, coeffs_lpf, fs->delay_lpf);
            env_raw[ch] = (n > 0) ? b[n - 1] : 0.0f;
        }

        // feature extraction at 20 Hz
        bool features_fresh = false;
        hop_counter += (ca < cb ? ca : cb);
        if (hop_counter >= FEATURE_HOP_SAMPLES) {
            hop_counter -= FEATURE_HOP_SAMPLES; // Keep the remainder
            compute_features(0); compute_features(1);
            features_fresh = true;
        }

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
                    float e = b[i] - cal[ch].baseline; if (e < 0) e = 0;
                    if (e > cal[ch].mvc_a_peak) cal[ch].mvc_a_peak = e;
                }
            }
            mvc_count += nmvc;
            if (mvc_count >= mvc_target) {
                printf("[CAL] MVC done (A peaks): A=%.2f  B=%.2f\n", cal[0].mvc_a_peak, cal[1].mvc_a_peak);
                cal[0].mvc_b_peak = cal[0].mvc_a_peak; cal[1].mvc_b_peak = cal[1].mvc_a_peak;
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

        float a_normA = mvc_norm(eA, cal[0].mvc_a_peak); float b_normA = mvc_norm(eB, cal[1].mvc_a_peak);
        float a_normB = mvc_norm(eA, cal[0].mvc_b_peak); float b_normB = mvc_norm(eB, cal[1].mvc_b_peak);

        int m = g_active_mode;
        if (m < 1 || m > 5) m = MODE_REST;
        // Mode 5 (spasm) is not an assist mode; clamp the coeff lookup to the
        // assist table (1..4). For collection the motor is off/zero anyway.
        const ModeCoeffs *tgt = &MODE_TABLE[(m >= 1 && m <= 4) ? m : MODE_REST];

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

        // ---- CSV header management: print once on entering a CSV mode ----
        int dbg = g_debug_mode;
        if (is_csv_mode(dbg) && !is_csv_mode(prev_dbg)) {
            csv_header_done = false;   // just entered a CSV mode
        }
        prev_dbg = dbg;

        // ---- DEBUG OUTPUTS ----
        if (dbg == DBG_EMG) {
            printf("Min:0,Max:1000,A_normA:%.1f,B_normA:%.1f,A_normB:%.1f,B_normB:%.1f,mode:%d,G:%.4f,D:%.4f,Ttgt:%.4f\n",
                a_normA*1000.0f, b_normA*1000.0f, a_normB*1000.0f, b_normB*1000.0f,
                m, cur_G, cur_D, g_target_torque_A);
        }
        else if (is_csv_mode(dbg) && features_fresh) {
            // print header once
            if (!csv_header_done) { print_csv_header(); csv_header_done = true; }

            int motor_emi = (dbg == DBG_CSV_MOTOR_EMI) ? 1 : 0;
            float diff_log = a_normA - b_normA;   // primary differential (method A)
            int64_t ts_ms  = esp_timer_get_time() / 1000;

            // snapshot features (volatile -> locals)
            float rms_a = g_features.rms_a, rms_b = g_features.rms_b;
            float mav_a = g_features.mav_a, mav_b = g_features.mav_b;
            int   zc_a  = g_features.zc_a,  zc_b  = g_features.zc_b;
            float wl_a  = g_features.wl_a,  wl_b  = g_features.wl_b;
            int   ssc_a = g_features.ssc_a, ssc_b = g_features.ssc_b;
            float var_a = g_features.var_a, var_b = g_features.var_b;

            printf("%lld,%d,%d,"
                   "%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%d,%d,%.3f,%.3f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.3f,%.4f,%.4f\n",
                   ts_ms, m, motor_emi,
                   rms_a, rms_b, mav_a, mav_b, zc_a, zc_b, wl_a, wl_b, ssc_a, ssc_b, var_a, var_b,
                   a_normA, b_normA, diff_log,
                   g_imu_gravity_angle_rad, g_imu_acc_mag, g_shaft_angle, g_shaft_velocity);
        }
    }
}

// ============================================================================
//  CORE 0 : REAL FOC TASK
// ============================================================================
static inline float torque_to_voltage(float T) {
    float V = (T / MOTOR_KT) * MOTOR_PHASE_R;
    if (V >  HARD_VOLTAGE_CEIL) V =  HARD_VOLTAGE_CEIL;
    if (V < -HARD_VOLTAGE_CEIL) V = -HARD_VOLTAGE_CEIL;
    return V;
}

static void foc_task(void *arg) {
    printf(">>> FOC task on core %d\n", xPortGetCoreID());

    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = HARD_VOLTAGE_CEIL;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_limit     = HARD_VOLTAGE_CEIL;
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.LPF_velocity.Tf = 0.01f;

#ifdef USE_STORED_CALIB
    motor.zero_electric_angle = STORED_ZERO_ELEC_ANGLE;
    motor.sensor_direction    = STORED_SENSOR_DIR;
    motor.init(); motor.initFOC();
#else
    printf(">>> FRESH alignment - keep shaft FREE...\n");
    motor.init(); motor.initFOC();
    printf(">>> alignment done: zero_elec=%.4f dir=%d pp=%d\n",
           motor.zero_electric_angle, (int)motor.sensor_direction, motor.pole_pairs);
#endif

    motor.enable();
    motor.loopFOC();
    g_angle_offset = motor.shaft_angle;
    printf(">>> angle offset captured: %.4f (this is now zero)\n", g_angle_offset);
    g_motor_ready = true;

    static uint32_t loop_count = 0;
    int  prev_dbg = g_debug_mode;
    bool motor_is_disabled = false;   // track dbg5 disable state

    for (;;) {
        if (++loop_count >= 100) { loop_count = 0; vTaskDelay(1); }

        int dbg = g_debug_mode;

        // ---- handle dbg5 (motor OFF) enter/leave transitions ----
        if (dbg == DBG_CSV_MOTOR_OFF && !motor_is_disabled && !g_fault_latched) {
            motor.move(0.0f);
            motor.disable();
            motor_is_disabled = true;
            // (encoder still read below for telemetry; driver is off)
        } else if (dbg != DBG_CSV_MOTOR_OFF && motor_is_disabled && !g_fault_latched) {
            motor.enable();
            motor_is_disabled = false;
        }
        prev_dbg = dbg;

        if (g_fault_latched) {
            motor.move(0.0f); motor.loopFOC();
            g_shaft_angle = motor.shaft_angle - g_angle_offset;
            g_shaft_velocity = motor.shaft_velocity;
            continue;
        }
        if (fault_active()) {
            g_fault_latched = true; motor.move(0.0f); motor.disable();
            printf("*** DRIVER FAULT latched - freewheel. FULL RESET to recover. ***\n");
            continue;
        }

        // ---- dbg5: motor disabled, just publish encoder, no FOC drive ----
        if (motor_is_disabled) {
            // keep encoder fresh without driving the windings
            sensor.update();
            g_shaft_angle = sensor.getAngle() - g_angle_offset;
            g_shaft_velocity = 0.0f;   // not estimating velocity while disabled
            continue;
        }

        motor.loopFOC();

        // ---- decide commanded torque ----
        float T;
        if (dbg == DBG_CSV_MOTOR_EMI) {
            T = 0.0f;          // energized (EMI present) but no assist torque
        } else {
            T = DRIVE_TORQUE;  // normal operation
        }
        float V = torque_to_voltage(T);
        motor.move(V);

        g_volt_A = torque_to_voltage(g_target_torque_A);
        g_volt_B = torque_to_voltage(g_target_torque_B);

        g_shaft_angle = motor.shaft_angle - g_angle_offset;
        g_shaft_velocity = motor.shaft_velocity;

        if (dbg == DBG_FOC) {
            static int64_t last_print_us = 0;
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_print_us >= 50000) {
                last_print_us = now_us;
                printf("Min:-3000,Max:3000,T_A_mNm:%.1f,V_A_mV:%.1f,T_B_mNm:%.1f,V_B_mV:%.1f,angle_mrad:%.1f,vel_mrad_s:%.1f\n",
                    g_target_torque_A * 1000.0f, g_volt_A * 1000.0f,
                    g_target_torque_B * 1000.0f, g_volt_B * 1000.0f,
                    g_shaft_angle * 1000.0f, g_shaft_velocity * 1000.0f);
            }
        }
    }
}

// ============================================================================
//  CORE 1 : IMU TASK (REAL BNO055)
// ============================================================================
static void IRAM_ATTR imu_int_isr(void *arg) {
    g_flag_collision = true;
}

static float clampf_custom(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static esp_err_t bno_write8(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(imu_i2c_dev, data, 2, 100);
}

static esp_err_t bno_read_len(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(imu_i2c_dev, &reg, 1, buf, len, 100);
}

static axis_t get_dominant_axis(float dx, float dy, float dz) {
    float adx = fabsf(dx), ady = fabsf(dy), adz = fabsf(dz);
    if (adx > ady && adx > adz) return AXIS_X;
    if (ady > adx && ady > adz) return AXIS_Y;
    if (adz > adx && adz > ady) return AXIS_Z;
    return AXIS_NONE;
}

static int sign_of(float value) {
    if (value > 0.2f) return 1;
    if (value < -0.2f) return -1;
    return 0;
}

static void i2c_init(void) {
    i2c_master_bus_config_t bus_conf = {};
    bus_conf.i2c_port = I2C_PORT;
    bus_conf.sda_io_num = (gpio_num_t)SDA_PIN;
    bus_conf.scl_io_num = (gpio_num_t)SCL_PIN;
    bus_conf.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_conf.glitch_ignore_cnt = 7;
    bus_conf.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));

    i2c_device_config_t dev_conf = {};
    dev_conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_conf.device_address = BNO055_ADDR;
    dev_conf.scl_speed_hz = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_conf, &imu_i2c_dev));

    printf(">>> I2C (V5 API) ready: PORT=%d, SDA=GPIO%d, SCL=GPIO%d\n", I2C_PORT, SDA_PIN, SCL_PIN);
}

static void gpio_interrupt_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INT_PIN), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "gpio_install_isr_service failed"); }
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)INT_PIN, imu_int_isr, NULL));
    printf(">>> IMU interrupt ready: INT=GPIO%d\n", INT_PIN);
}

static bool bno055_init_with_high_g_interrupt(void) {
    uint8_t chip_id = 0;
    if (bno_read_len(BNO055_CHIP_ID_REG, &chip_id, 1) != ESP_OK || chip_id != 0xA0) return false;
    bno_write8(BNO055_OPR_MODE_REG, OPR_CONFIG); vTaskDelay(pdMS_TO_TICKS(25));
    bno_write8(BNO055_PAGE_ID_REG, 0x00); bno_write8(BNO055_PWR_MODE_REG, 0x00); vTaskDelay(pdMS_TO_TICKS(10));
    bno_write8(BNO055_PAGE_ID_REG, 0x01); vTaskDelay(pdMS_TO_TICKS(10));
    bno_write8(BNO055_ACC_CONFIG_REG, 0x0D); bno_write8(BNO055_ACC_HIGH_G_THRESHOLD_REG, 0x08);
    bno_write8(BNO055_ACC_HIGH_G_DURATION_REG, 0x04); bno_write8(BNO055_ACC_INT_SETTINGS_REG, 0xE0);
    bno_write8(BNO055_INT_EN_REG, 0x20); bno_write8(BNO055_INT_MSK_REG, 0x20);
    bno_write8(BNO055_PAGE_ID_REG, 0x00); vTaskDelay(pdMS_TO_TICKS(10));
    bno_write8(BNO055_OPR_MODE_REG, OPR_NDOF); vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

static float calculate_gravity_angle_from_pitch(float pitch) {
    if (pitch >= 17.0f) return clampf_custom(90.0f * (92.8f - pitch) / (92.8f - 17.0f), 0.0f, 90.0f);
    else return clampf_custom(90.0f + 90.0f * (17.0f - pitch) / (17.0f - (-87.4f)), 90.0f, 180.0f);
}

static void imu_task(void *pvParameters) {
    printf(">>> IMU task on core %d\n", xPortGetCoreID());

    float prev_mag = 0.0f, prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;
    int spike_window[WINDOW_SAMPLES] = {0}, dir_window[WINDOW_SAMPLES] = {0};
    int window_index = 0, spike_count = 0, direction_change_count = 0, prev_dominant_sign = 0;
    bool local_collision = false, local_spasm = false;

    while (1) {
        if (g_req_clear) {
            g_flag_collision = false;
            g_flag_spasm = false;
            local_collision = false;
            local_spasm = false;
            spike_count = 0;
            direction_change_count = 0;
            memset(spike_window, 0, sizeof(spike_window));
            memset(dir_window, 0, sizeof(dir_window));
            g_req_clear = false;
            printf("[IMU] Safety Flags Cleared\n");
        }

        uint8_t acc_buf[6], euler_buf[6];
        if (bno_read_len(BNO055_ACC_DATA_X_LSB, acc_buf, 6) == ESP_OK &&
            bno_read_len(BNO055_EULER_H_LSB, euler_buf, 6) == ESP_OK) {

            float ax = ((int16_t)((acc_buf[1] << 8) | acc_buf[0])) / 100.0f;
            float ay = ((int16_t)((acc_buf[3] << 8) | acc_buf[2])) / 100.0f;
            float az = ((int16_t)((acc_buf[5] << 8) | acc_buf[4])) / 100.0f;
            float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
            float acc_change = fabsf(acc_mag - prev_mag);

            g_imu_acc_mag = acc_mag;   // bridge for CSV

            float pitch = ((int16_t)((euler_buf[5] << 8) | euler_buf[4])) / 16.0f;
            float gravity_deg = calculate_gravity_angle_from_pitch(pitch);
            g_imu_gravity_angle_rad = gravity_deg * (M_PI / 180.0f);

            float dx = ax - prev_ax, dy = ay - prev_ay, dz = az - prev_az;
            axis_t dom_axis = get_dominant_axis(dx, dy, dz);

            int current_sign = 0;
            if (dom_axis == AXIS_X) current_sign = sign_of(dx);
            else if (dom_axis == AXIS_Y) current_sign = sign_of(dy);
            else if (dom_axis == AXIS_Z) current_sign = sign_of(dz);

            int spike_now = (acc_change > SPASM_CHANGE_THRESHOLD) ? 1 : 0;
            int dir_change_now = (current_sign != 0 && prev_dominant_sign != 0 && current_sign != prev_dominant_sign) ? 1 : 0;

            spike_count -= spike_window[window_index];
            direction_change_count -= dir_window[window_index];
            spike_window[window_index] = spike_now;
            dir_window[window_index] = dir_change_now;
            spike_count += spike_now;
            direction_change_count += dir_change_now;
            window_index = (window_index + 1) % WINDOW_SAMPLES;

            if (acc_mag > COLLISION_MAG_THRESHOLD && spike_count <= 1) {
                g_flag_collision = true;
            }

            if (g_flag_collision) {
                spike_count = 0;
                direction_change_count = 0;
                memset(spike_window, 0, sizeof(spike_window));
                memset(dir_window, 0, sizeof(dir_window));
                g_flag_spasm = false;
            } else {
                g_flag_spasm = (spike_count >= SPASM_SPIKE_THRESHOLD) && (direction_change_count >= SPASM_DIRECTION_CHANGE_THRESHOLD);
            }

            if (g_flag_spasm && !local_spasm) printf("\n*** SPASM DETECTED - APPLYING DAMPING ***\n");
            if (g_flag_collision && !local_collision) printf("\n*** COLLISION DETECTED - FREEWHEELING ***\n");
            local_spasm = g_flag_spasm;
            local_collision = g_flag_collision;

            if (g_debug_mode == DBG_IMU) {
                printf("GravityDeg:%.1f | Pitch:%.1f | Spasm:%d | Collision:%d\n",
                       gravity_deg, pitch, local_spasm, local_collision);
            }

            prev_mag = acc_mag; prev_ax = ax; prev_ay = ay; prev_az = az;
            if (current_sign != 0) prev_dominant_sign = current_sign;
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
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
            printf("mode:%d | RMS a:%.1f b:%.1f | MAV a:%.1f b:%.1f | ZC a:%d b:%d | WL a:%.1f b:%.1f | SSC a:%d b:%d | VAR a:%.1f b:%.1f\n",
                   g_active_mode, g_features.rms_a, g_features.rms_b, g_features.mav_a, g_features.mav_b,
                   g_features.zc_a, g_features.zc_b, g_features.wl_a, g_features.wl_b,
                   g_features.ssc_a, g_features.ssc_b, g_features.var_a, g_features.var_b);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
//  CORE 1 : SERIAL TASK
// ============================================================================
static void console_init() {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}

static void handle_command(const char *line) {
    // mode select 1..5 (5 = spasm label)
    if (strlen(line) == 1 && line[0] >= '1' && line[0] <= '5') {
        g_req_mode = line[0] - '0';
        return;
    }
    if      (strcmp(line, "clear") == 0) g_req_clear = true;
    else if (strncmp(line, "dbg", 3) == 0) {
        int d = atoi(line + 3);
        if (d >= 1 && d <= 6) g_debug_mode = d;
    }
}

static void serial_task(void *arg) {
    printf(">>> SERIAL task on core %d\n", xPortGetCoreID());
    char line[32]; size_t idx = 0;
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

    printf("\n=== EMG ORTHOSIS - DATA COLLECTION FIRMWARE ===\n");
    printf("Cmds: 1-5 mode/label | clear | dbg 1|2|3|4|5|6\n");
    printf("Labels: 1 Rest  2 Reach  3 Lift  4 Hold  5 Spasm\n");
    printf("Debug 1=EMG  2=FOC  3=AI feat  4=IMU\n");
    printf("Collect 5=CSV MOTOR OFF   6=CSV MOTOR EMI (energized,0 torque)\n");
    printf("Workflow: select label (1-5), enter dbg5 or dbg6, copy CSV.\n\n");

    i2c_init();
    gpio_interrupt_init();
    if (!bno055_init_with_high_g_interrupt()) {
        printf("!!! IMU INIT FAILED - Check I2C Wiring !!!\n");
    }

    xTaskCreatePinnedToCore(foc_task,    "FOC",    8192, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(emg_task,    "EMG",    8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(imu_task,    "IMU",    4096, NULL, 6,  NULL, 1);
    xTaskCreatePinnedToCore(ai_task,     "AI",     4096, NULL, 5,  NULL, 1);
    xTaskCreatePinnedToCore(serial_task, "SERIAL", 4096, NULL, 4,  NULL, 1);
}