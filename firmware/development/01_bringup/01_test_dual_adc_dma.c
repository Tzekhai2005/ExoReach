#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"

// --- Configuration ---
#define ADC_CHANNEL_A       ADC_CHANNEL_3  // GPIO 4
#define ADC_CHANNEL_B       ADC_CHANNEL_4  // GPIO 5
#define SAMPLE_RATE_HZ      2000
// READ_LEN must hold 2 channels worth of samples
#define READ_LEN            512
#define INTERNAL_BUF_SIZE   2048
#define CALIBRATION_TIME_SEC 5

static const char *TAG = "EMG_DUAL";
static adc_continuous_handle_t adc_handle = NULL;

// --- Per-channel filter coefficients (shared, read-only after init) ---
float coeffs_notch[5];
float coeffs_hpf[5];
float coeffs_lpf[5];

// --- Per-channel filter DELAY LINES (must be separate — this is stateful) ---
typedef struct {
    float delay_notch[2];
    float delay_hpf[2];
    float delay_lpf[2];
} ChannelFilterState;

// --- Per-channel calibration state ---
typedef struct {
    bool     is_calibrated;
    double   calibration_sum;
    uint32_t calibration_sample_count;
    float    baseline_offset;
} ChannelCalibState;

static ChannelFilterState filter_state[2] = {0};
static ChannelCalibState  calib_state[2]  = {0};

const uint32_t calibration_target_samples = SAMPLE_RATE_HZ * CALIBRATION_TIME_SEC;

void init_filters() {
    dsps_biquad_gen_notch_f32(coeffs_notch, 50.0f / SAMPLE_RATE_HZ, 0.0f, 0.707f);
    dsps_biquad_gen_hpf_f32(coeffs_hpf,    20.0f / SAMPLE_RATE_HZ, 0.707f);
    dsps_biquad_gen_lpf_f32(coeffs_lpf,     5.0f / SAMPLE_RATE_HZ, 0.707f);
}

// Process one channel's buffer of samples in-place
void process_channel(float *buf, int num_samples, int ch_idx) {
    ChannelFilterState *fs = &filter_state[ch_idx];
    ChannelCalibState  *cs = &calib_state[ch_idx];

    // Filter
    dsps_biquad_f32(buf, buf, num_samples, coeffs_notch, fs->delay_notch);
    dsps_biquad_f32(buf, buf, num_samples, coeffs_hpf,   fs->delay_hpf);

    // Envelope
    for (int i = 0; i < num_samples; i++) buf[i] = fabsf(buf[i]);
    dsps_biquad_f32(buf, buf, num_samples, coeffs_lpf, fs->delay_lpf);

    // Calibration + output
    for (int i = 0; i < num_samples; i++) {
        if (!cs->is_calibrated) {
            cs->calibration_sum += buf[i];
            cs->calibration_sample_count++;
            if (cs->calibration_sample_count >= calibration_target_samples) {
                cs->baseline_offset = (float)(cs->calibration_sum / cs->calibration_sample_count);
                cs->is_calibrated   = true;
                ESP_LOGI(TAG, "Ch%d calibration DONE. Baseline: %f", ch_idx, cs->baseline_offset);
            }
        } else {
            // Store result back into buf so caller can print both channels together
            float val = buf[i] - cs->baseline_offset;
            buf[i] = val < 0.0f ? 0.0f : val;
        }
    }
}

void emg_task(void *pvParameters) {
    init_filters();

    // --- ADC init: register BOTH channels ---
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = INTERNAL_BUF_SIZE,
        .conv_frame_size    = READ_LEN
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t adc_pattern[2] = {0};
    adc_pattern[0].atten     = ADC_ATTEN_DB_12;
    adc_pattern[0].channel   = ADC_CHANNEL_A;
    adc_pattern[0].unit      = ADC_UNIT_1;
    adc_pattern[0].bit_width = 12;

    adc_pattern[1].atten     = ADC_ATTEN_DB_12;
    adc_pattern[1].channel   = ADC_CHANNEL_B;
    adc_pattern[1].unit      = ADC_UNIT_1;
    adc_pattern[1].bit_width = 12;

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .adc_pattern    = adc_pattern,
        .pattern_num    = 2   // <-- two channels now
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    uint8_t  raw[READ_LEN] = {0};
    uint32_t ret_num       = 0;
    const int max_results  = READ_LEN / SOC_ADC_DIGI_RESULT_BYTES;

    // Separate deinterleaved buffers for each channel
    float buf_a[max_results];
    float buf_b[max_results];

    ESP_LOGI(TAG, "Calibrating both channels — RELAX YOUR ARMS for %d seconds.", CALIBRATION_TIME_SEC);

    while (1) {
        esp_err_t ret = adc_continuous_read(adc_handle, raw, READ_LEN, &ret_num, portMAX_DELAY);
        if (ret != ESP_OK) continue;

        int total    = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
        int count_a  = 0, count_b = 0;

        // --- Deinterleave: the ADC interleaves ch A and ch B samples ---
        for (int i = 0; i < total; i++) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw[i * SOC_ADC_DIGI_RESULT_BYTES];
            float val = (float)p->type2.data;
            if (p->type2.channel == ADC_CHANNEL_A) {
                buf_a[count_a++] = val;
            } else if (p->type2.channel == ADC_CHANNEL_B) {
                buf_b[count_b++] = val;
            }
        }

        // --- Process each channel independently ---
        process_channel(buf_a, count_a, 0);
        process_channel(buf_b, count_b, 1);

        // --- Print both channels together so the plotter syncs them ---
        bool both_calibrated = calib_state[0].is_calibrated && calib_state[1].is_calibrated;
        if (both_calibrated) {
            int print_count = count_a < count_b ? count_a : count_b;
            for (int i = 0; i < print_count; i += 10) {
                printf("Min:0,Max:1000,Ch_A:%f,Ch_B:%f\n",
                       buf_a[i] * 2.0f,
                       buf_b[i] * 2.0f);
            }
        } else {
            // Show calibration progress for whichever channel is still settling
            printf("Min:0,Max:1000,Calibrating_A:%f,Calibrating_B:%f\n",
                   buf_a[0] * 2.0f, buf_b[0] * 2.0f);
        }
    }
}

void app_main() {
    xTaskCreatePinnedToCore(emg_task, "EMG_Core1", 8192, NULL, 10, NULL, 1);
}