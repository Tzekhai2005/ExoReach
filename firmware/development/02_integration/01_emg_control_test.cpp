/* ============================================================================
 *  EMG ORTHOSIS - FULL CONTROL FIRMWARE (TEST BUILD)
 *  ----------------------------------------------------------------------------
 *  Same task/core layout as the real system, with the not-yet-tested pieces
 *  mocked at their boundaries. Real: EMG pipeline, feature extraction, torque
 *  equation, mode LERP, limits, two-tier safety. Mocked: FOC (consumes torque,
 *  static angle/vel), IMU (manual flags), AI (mode from serial, not classified).
 *
 *  CORE 0:
 *    foc_task    - reads BOTH target torques (A/B), converts each to voltage.
 *                  Static mock angle/velocity. Prints only in DEBUG MODE 2.
 *
 *  CORE 1:
 *    emg_task    - dual-channel ADC -> filter -> 500 Hz envelope (x2 MVC norms)
 *                  -> 20 Hz feature extraction -> receive mode from AI
 *                  -> LERP G/D/K -> torque eq (x2: A and B) -> limits + safety
 *                  -> write both target torques. Prints only in DEBUG MODE 1.
 *                  Runs noise-baseline then MVC calibration at startup.
 *    imu_task    - MOCK. Holds collision/spasm flags set via serial. Silent.
 *    ai_task     - MOCK. Receives feature vector (for display), receives mode
 *                  1-4 from serial, outputs selected mode to emg_task.
 *                  Prints features only in DEBUG MODE 3.
 *    serial_task - READ ONLY. Routes input to mocks; sets debug mode. Never
 *                  prints telemetry, never touches emg_task directly.
 *
 *  SERIAL COMMANDS (runtime):
 *    1..4        -> select mode (routed to AI mock): 1 Rest 2 Reach 3 Lift 4 Hold
 *    collision   -> IMU mock sets collision flag (freewheel)
 *    spasm       -> IMU mock sets spasm flag (damping)
 *    clear       -> IMU mock clears flags
 *    dbg 1|2|3   -> select which task prints (1 EMG, 2 FOC, 3 AI features)
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"

// ============================================================================
//  CONFIG - EMG
// ============================================================================
#define ADC_CHANNEL_A        ADC_CHANNEL_3   // bicep
#define ADC_CHANNEL_B        ADC_CHANNEL_4   // tricep
#define SAMPLE_RATE_HZ       2000
#define READ_LEN             512
#define INTERNAL_BUF_SIZE    2048

#define BASELINE_TIME_SEC    5               // noise/relax calibration
#define MVC_TIME_SEC         5               // max-contraction calibration

// Feature window: 400 samples @ 2 kHz = 200 ms; features extracted @ 20 Hz
#define FEATURE_WINDOW_LEN   400
#define FEATURE_HOP_SAMPLES  100             // 100/2000 = 20 Hz

// ============================================================================
//  CONFIG - CONTROL / MODES  (compile-time tuning; switch live, edit+reflash to tune)
// ============================================================================
typedef enum { MODE_REST = 1, MODE_REACH = 2, MODE_LIFT = 3, MODE_HOLD = 4 } RehabMode;

typedef struct { float G; float D; float K; } ModeCoeffs;

// Index 0 unused so MODE_REST(1)..MODE_HOLD(4) map directly.
//                          G      D      K
static const ModeCoeffs MODE_TABLE[5] = {
    { 0.0f,  0.0f,  0.0f },   // [0] unused
    { 0.0f,  0.0f, 0.0f },   // [1] REST    : no EMG drive, high damping
    { 0.029f, 0.004f, 0.0f },   // [2] REACH   : strong drive, light damping
    { 0.036f, 0.08f, 0.0f },   // [3] LIFT    : strongest drive, medium damping
    { 0.011f, 0.015f, 0.0f },   // [4] HOLD    : low drive, highest damping
};
// K (gravity comp) is 0 everywhere for now, by design. Structure kept for later.

// LERP: time to ramp coeffs to a newly-selected mode
#define LERP_TIME_SEC        0.30f

// Global limits (same for all modes)
#define EMG_DEADBAND         0.05f           // on normalized differential
#define TORQUE_MAX           0.30f           // N.m ceiling (|T|)
#define TORQUE_SLEW_PER_SEC  0.2f            // max d|T|/dt (N.m per second)
#define D_SAFE               0.015f           // spasm damping strength (> any mode D)

// Range-of-motion soft limits (rad, joint). Mock angle won't hit these, but
// the logic is present and exercised.
#define ROM_MIN_RAD         (-1.8f)
#define ROM_MAX_RAD         ( 1.8f)
#define ROM_BRAKE_BAND       0.30f           // start braking within this of limit

// ============================================================================
//  CONFIG - MOTOR (for FOC mock torque->voltage conversion)
// ============================================================================
#define MOTOR_KT             0.0677f          // N.m/A
#define MOTOR_PHASE_R        5.6f            // ohm (placeholder; from bench later)
#define HARD_VOLTAGE_CEIL    3.0f

// Control loop cadence (envelope/torque update). ~500 Hz.
#define CONTROL_PERIOD_MS    2

static const char *TAG = "EMG_FW";

// ============================================================================
//  DEBUG MODE (which task is allowed to print). Set live via "dbg N".
// ============================================================================
typedef enum { DBG_EMG = 1, DBG_FOC = 2, DBG_AI = 3 } DebugMode;
volatile int g_debug_mode = DBG_EMG;         // boot default = 1 (envelope)

// ============================================================================
//  SHARED STATE (inter-core / inter-task; single-writer scalars = lock-free)
// ============================================================================
// emg_task -> foc_task : two target torques (A and B normalization)
volatile float g_target_torque_A = 0.0f;
volatile float g_target_torque_B = 0.0f;

// foc_task -> emg_task : MOCK kinematics (static; from your working encoder)
volatile float g_mock_angle    = 0.0f;
volatile float g_mock_velocity = 0.0f;

// imu_task -> emg_task : safety flags
volatile bool g_flag_collision = false;
volatile bool g_flag_spasm     = false;

// serial_task -> imu_task : requested flag state (serial routes through IMU)
volatile bool g_req_collision = false;
volatile bool g_req_spasm     = false;
volatile bool g_req_clear     = false;

// serial_task -> ai_task : requested mode (1..4)
volatile int  g_req_mode = MODE_REST;

// ai_task -> emg_task : selected mode
volatile int  g_active_mode = MODE_REST;

// emg_task -> ai_task : latest feature vector (snapshot; debug display only)
typedef struct {
    float rms_a, rms_b;          // root mean square
    float mav_a, mav_b;          // mean absolute value
    int   zc_a,  zc_b;           // zero crossings
    float wl_a,  wl_b;           // waveform length
} FeatureVector;
volatile FeatureVector g_features = {0};

// emg_task published envelopes (normalized, for debug mode 1)
volatile float g_env_a_normA = 0.0f, g_env_b_normA = 0.0f;   // method A norm
volatile float g_env_a_normB = 0.0f, g_env_b_normB = 0.0f;   // method B norm

// foc_task published voltages (for debug mode 2)
volatile float g_volt_A = 0.0f, g_volt_B = 0.0f;

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
    // baseline (noise) calibration
    bool     base_done;
    double   base_sum;
    uint32_t base_count;
    float    baseline;
    // MVC method A: peak captured during the fixed MVC window
    float    mvc_a_peak;
    // MVC method B: adaptive running peak (updates forever)
    float    mvc_b_peak;
} ChannelCalib;

static ChannelFilterState filt[2] = {0};
static ChannelCalib       cal[2]  = {0};

// Feature sliding window (filtered, pre-envelope signal) per channel
static float win[2][FEATURE_WINDOW_LEN] = {{0}};
static int   win_pos[2] = {0};
static int   win_filled[2] = {0};
static int   hop_counter = 0;

static void init_filters() {
    dsps_biquad_gen_notch_f32(coeffs_notch, 50.0f / SAMPLE_RATE_HZ, 0.0f, 0.707f);
    dsps_biquad_gen_hpf_f32(coeffs_hpf,    20.0f / SAMPLE_RATE_HZ, 0.707f);
    dsps_biquad_gen_lpf_f32(coeffs_lpf,     5.0f / SAMPLE_RATE_HZ, 0.707f);
}

// MVC normalize, clamped 0..1. (env already baseline-subtracted, >=0)
static inline float mvc_norm(float env, float peak) {
    if (peak < 1e-6f) return 0.0f;
    float n = env / peak;
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

// Compute features for one channel from its window into the global g_features.
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

    if (ch == 0) {
        g_features.rms_a = rms; g_features.mav_a = mav;
        g_features.zc_a  = zc;  g_features.wl_a  = wl;
    } else {
        g_features.rms_b = rms; g_features.mav_b = mav;
        g_features.zc_b  = zc;  g_features.wl_b  = wl;
    }
}

// ============================================================================
//  CONTROL: torque equation + limits + safety (runs inside emg_task)
// ============================================================================
// LERP state for the active coefficients
static float cur_G = 0.0f, cur_D = 0.80f, cur_K = 0.0f;  // boot = REST
static float last_T_A = 0.0f, last_T_B = 0.0f;           // for slew limit

// One torque computation for a given normalized (b - t) differential.
// Applies deadband, the equation, ROM soft-brake, saturation, slew, and the
// two-tier safety override. dt in seconds. last_T updated by reference.
static float compute_torque(float diff, float omega, float dt, float *last_T) {
    // --- two-tier safety override takes precedence over everything ---
    if (g_flag_collision) {              // freewheel
        *last_T = 0.0f;
        return 0.0f;
    }
    if (g_flag_spasm) {                  // damping only (no EMG, no gravity)
        float t = -D_SAFE * omega;
        if (t >  TORQUE_MAX) t =  TORQUE_MAX;
        if (t < -TORQUE_MAX) t = -TORQUE_MAX;
        *last_T = t;
        return t;
    }

    // --- deadband on the EMG differential ---
    if (diff >  -EMG_DEADBAND && diff < EMG_DEADBAND) diff = 0.0f;

    // --- torque equation: T = K*sin(theta) + G*(b-t) - D*omega  (K=0 now) ---
    float theta = g_mock_angle;
    float T = cur_K * sinf(theta) + cur_G * diff - cur_D * omega;

    // --- ROM soft-brake near range limits ---
    if (theta > ROM_MAX_RAD - ROM_BRAKE_BAND) {
        float over = (theta - (ROM_MAX_RAD - ROM_BRAKE_BAND)) / ROM_BRAKE_BAND;
        if (over > 1.0f) over = 1.0f;
        if (T > 0.0f) T *= (1.0f - over);        // kill torque pushing further
        T += -D_SAFE * omega * over;             // add damping near the wall
    } else if (theta < ROM_MIN_RAD + ROM_BRAKE_BAND) {
        float over = ((ROM_MIN_RAD + ROM_BRAKE_BAND) - theta) / ROM_BRAKE_BAND;
        if (over > 1.0f) over = 1.0f;
        if (T < 0.0f) T *= (1.0f - over);
        T += -D_SAFE * omega * over;
    }

    // --- saturation ---
    if (T >  TORQUE_MAX) T =  TORQUE_MAX;
    if (T < -TORQUE_MAX) T = -TORQUE_MAX;

    // --- slew-rate limit ---
    float max_step = TORQUE_SLEW_PER_SEC * dt;
    float dT = T - *last_T;
    if (dT >  max_step) T = *last_T + max_step;
    if (dT < -max_step) T = *last_T - max_step;

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

    adc_digi_pattern_config_t pat[2] = {0};
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
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    uint8_t  raw[READ_LEN] = {0};
    uint32_t ret_num = 0;
    const int max_results = READ_LEN / SOC_ADC_DIGI_RESULT_BYTES;
    float bufA[max_results], bufB[max_results];

    // calibration phase tracking
    const uint32_t base_target = SAMPLE_RATE_HZ * BASELINE_TIME_SEC;
    const uint32_t mvc_target  = SAMPLE_RATE_HZ * MVC_TIME_SEC;
    uint32_t mvc_count = 0;
    enum { PH_BASELINE, PH_MVC, PH_RUN } phase = PH_BASELINE;

    printf("\n[CAL] PHASE 1/2: NOISE BASELINE - relax both muscles for %d s...\n",
           BASELINE_TIME_SEC);

    int64_t last_ctrl_us = esp_timer_get_time();

    while (1) {
        if (adc_continuous_read(adc_handle, raw, READ_LEN, &ret_num, portMAX_DELAY) != ESP_OK)
            continue;

        int total = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
        int ca = 0, cb = 0;
        for (int i = 0; i < total; i++) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw[i * SOC_ADC_DIGI_RESULT_BYTES];
            float v = (float)p->type2.data;
            if (p->type2.channel == ADC_CHANNEL_A && ca < max_results) bufA[ca++] = v;
            else if (p->type2.channel == ADC_CHANNEL_B && cb < max_results) bufB[cb++] = v;
        }

        // ---- filter both channels; keep filtered (pre-envelope) for features ----
        float *bufs[2] = { bufA, bufB };
        int    cnts[2] = { ca, cb };
        float  env_raw[2] = {0, 0};   // last-sample envelope per channel this block

        for (int ch = 0; ch < 2; ch++) {
            float *b = bufs[ch]; int n = cnts[ch];
            ChannelFilterState *fs = &filt[ch];

            // bandpass
            dsps_biquad_f32(b, b, n, coeffs_notch, fs->delay_notch);
            dsps_biquad_f32(b, b, n, coeffs_hpf,   fs->delay_hpf);

            // push filtered samples into the feature window (pre-rectify)
            for (int i = 0; i < n; i++) {
                win[ch][win_pos[ch]] = b[i];
                win_pos[ch] = (win_pos[ch] + 1) % FEATURE_WINDOW_LEN;
                if (win_pos[ch] == 0) win_filled[ch] = 1;
            }

            // envelope: rectify + LPF (in a temp copy so window keeps raw filtered)
            // reuse b for envelope now
            for (int i = 0; i < n; i++) b[i] = fabsf(b[i]);
            dsps_biquad_f32(b, b, n, coeffs_lpf, fs->delay_lpf);

            env_raw[ch] = (n > 0) ? b[n - 1] : 0.0f;   // latest envelope value
        }

        // ---- feature extraction at 20 Hz ----
        hop_counter += (ca < cb ? ca : cb);
        if (hop_counter >= FEATURE_HOP_SAMPLES) {
            hop_counter = 0;
            compute_features(0);
            compute_features(1);
        }

        // ============ CALIBRATION STATE MACHINE ============
        if (phase == PH_BASELINE) {
            // accumulate every envelope sample in the block so the 5 s target
            // (in SAMPLES) is reached in real 5 s, not 5 s * block_size.
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
                printf("[CAL] baseline done: A=%.2f  B=%.2f\n",
                       cal[0].baseline, cal[1].baseline);
                printf("[CAL] PHASE 2/2: MVC - contract/move through range MAX for %d s...\n",
                       MVC_TIME_SEC);
                phase = PH_MVC;
            }
            continue;   // no control output during calibration
        }

        if (phase == PH_MVC) {
            int nmvc = (ca < cb ? ca : cb);
            for (int ch = 0; ch < 2; ch++) {
                float *b = bufs[ch];
                for (int i = 0; i < nmvc; i++) {
                    float e = b[i] - cal[ch].baseline;
                    if (e < 0) e = 0;
                    if (e > cal[ch].mvc_a_peak) cal[ch].mvc_a_peak = e;   // method A peak
                }
            }
            mvc_count += nmvc;
            if (mvc_count >= mvc_target) {
                printf("[CAL] MVC done (method A peaks): A=%.2f  B=%.2f\n",
                       cal[0].mvc_a_peak, cal[1].mvc_a_peak);
                // seed adaptive method B with the A peaks so it starts sane
                cal[0].mvc_b_peak = cal[0].mvc_a_peak;
                cal[1].mvc_b_peak = cal[1].mvc_a_peak;
                printf("[CAL] complete. Entering RUN (mode REST).\n\n");
                phase = PH_RUN;
            }
            continue;
        }

        // ================= RUN =================
        // baseline-subtract envelopes
        float eA = env_raw[0] - cal[0].baseline; if (eA < 0) eA = 0;
        float eB = env_raw[1] - cal[1].baseline; if (eB < 0) eB = 0;

        // method B adaptive peak update
        if (eA > cal[0].mvc_b_peak) cal[0].mvc_b_peak = eA;
        if (eB > cal[1].mvc_b_peak) cal[1].mvc_b_peak = eB;

        // two normalizations
        float a_normA = mvc_norm(eA, cal[0].mvc_a_peak);
        float b_normA = mvc_norm(eB, cal[1].mvc_a_peak);
        float a_normB = mvc_norm(eA, cal[0].mvc_b_peak);
        float b_normB = mvc_norm(eB, cal[1].mvc_b_peak);

        g_env_a_normA = a_normA; g_env_b_normA = b_normA;
        g_env_a_normB = a_normB; g_env_b_normB = b_normB;

        // ---- receive mode from AI task; LERP coeffs toward it ----
        int m = g_active_mode;
        if (m < 1 || m > 4) m = MODE_REST;
        const ModeCoeffs *tgt = &MODE_TABLE[m];

        // dt for control step
        int64_t now = esp_timer_get_time();
        float dt = (now - last_ctrl_us) / 1e6f;
        last_ctrl_us = now;
        if (dt <= 0.0f || dt > 0.1f) dt = CONTROL_PERIOD_MS / 1000.0f;

        float a = dt / LERP_TIME_SEC; if (a > 1.0f) a = 1.0f;
        cur_G += (tgt->G - cur_G) * a;
        cur_D += (tgt->D - cur_D) * a;
        cur_K += (tgt->K - cur_K) * a;

        // ---- torque equation, computed for BOTH normalizations ----
        float omega = g_mock_velocity;
        float diffA = a_normA - b_normA;   // bicep - tricep (method A)
        float diffB = a_normB - b_normB;   // method B
        float T_A = compute_torque(diffA, omega, dt, &last_T_A);
        float T_B = compute_torque(diffB, omega, dt, &last_T_B);

        g_target_torque_A = T_A;
        g_target_torque_B = T_B;

        // ---- DEBUG MODE 1: this task prints normalized envelopes ----
        if (g_debug_mode == DBG_EMG) {
            printf("Min:0,Max:1000,A_normA:%.1f,B_normA:%.1f,A_normB:%.1f,B_normB:%.1f\n",
                   a_normA * 1000.0f, b_normA * 1000.0f,
                   a_normB * 1000.0f, b_normB * 1000.0f);
        }
    }
}

// ============================================================================
//  CORE 0 : FOC TASK (MOCK)  - torque -> voltage, static kinematics
// ============================================================================
static inline float torque_to_voltage(float T) {
    // voltage-mode FOC approximation: V = (T / Kt) * R  (current * resistance)
    float V = (T / MOTOR_KT) * MOTOR_PHASE_R;
    if (V >  HARD_VOLTAGE_CEIL) V =  HARD_VOLTAGE_CEIL;
    if (V < -HARD_VOLTAGE_CEIL) V = -HARD_VOLTAGE_CEIL;
    return V;
}

static void foc_task(void *arg) {
    printf(">>> FOC(mock) task on core %d\n", xPortGetCoreID());
    // static mock kinematics (your real encoder/motor already verified).
    g_mock_angle    = 0.0f;
    g_mock_velocity = 0.0f;

    for (;;) {
        float T_A = g_target_torque_A;
        float T_B = g_target_torque_B;
        g_volt_A = torque_to_voltage(T_A);
        g_volt_B = torque_to_voltage(T_B);

        // (real FOC would loopFOC()/move(g_volt) here)

        if (g_debug_mode == DBG_FOC) {
            printf("Min:-3000,Max:3000,T_A_mNm:%.1f,V_A_mV:%.1f,T_B_mNm:%.1f,V_B_mV:%.1f\n",
                   T_A * 1000.0f, g_volt_A * 1000.0f,
                   T_B * 1000.0f, g_volt_B * 1000.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(2));   // mock loop ~500 Hz
    }
}

// ============================================================================
//  CORE 1 : IMU TASK (MOCK) - applies serial-requested flags, silent
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
//  CORE 1 : AI TASK (MOCK) - mode from serial; holds features for display
// ============================================================================
static void ai_task(void *arg) {
    printf(">>> AI(mock) task on core %d\n", xPortGetCoreID());
    for (;;) {
        // "decision": pass the serial-requested mode through to emg_task.
        g_active_mode = g_req_mode;

        // DEBUG MODE 3: print the feature vector (snapshot read).
        if (g_debug_mode == DBG_AI) {
            // copy field-by-field; can't implicitly copy a volatile struct in C++
            FeatureVector f;
            f.rms_a = g_features.rms_a; f.rms_b = g_features.rms_b;
            f.mav_a = g_features.mav_a; f.mav_b = g_features.mav_b;
            f.zc_a  = g_features.zc_a;  f.zc_b  = g_features.zc_b;
            f.wl_a  = g_features.wl_a;  f.wl_b  = g_features.wl_b;
            printf("mode:%d | RMS a:%.1f b:%.1f | MAV a:%.1f b:%.1f | ZC a:%d b:%d | WL a:%.1f b:%.1f\n",
                   g_active_mode, f.rms_a, f.rms_b, f.mav_a, f.mav_b,
                   f.zc_a, f.zc_b, f.wl_a, f.wl_b);
        }
        vTaskDelay(pdMS_TO_TICKS(50));   // 20 Hz, matches feature rate
    }
}

// ============================================================================
//  CORE 1 : SERIAL TASK - READ ONLY router + debug-mode selector
// ============================================================================
static void console_init() {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}

static void handle_command(const char *line) {
    // mode select 1..4  -> route to AI mock
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
    // unknown -> ignore (serial task is read-only; no echo to avoid clutter)
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
    printf("\n=== EMG ORTHOSIS - FULL CONTROL FIRMWARE (TEST) ===\n");
    printf("Cmds: 1-4 mode | collision | spasm | clear | dbg 1|2|3\n");
    printf("Debug 1=EMG envelope  2=FOC torque/volt  3=AI features\n");
    printf("Boot debug mode = 1 (EMG). Calibration starts now.\n\n");

    // CORE 1 : EMG main (highest), then AI, IMU, Serial
    xTaskCreatePinnedToCore(emg_task,    "EMG",    8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(ai_task,     "AI",     4096, NULL, 6,  NULL, 1);
    xTaskCreatePinnedToCore(imu_task,    "IMU",    3072, NULL, 6,  NULL, 1);
    xTaskCreatePinnedToCore(serial_task, "SERIAL", 4096, NULL, 5,  NULL, 1);

    // CORE 0 : FOC mock
    xTaskCreatePinnedToCore(foc_task,    "FOC",    4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);
}