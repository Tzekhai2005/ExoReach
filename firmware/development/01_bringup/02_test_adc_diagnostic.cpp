/* ============================================================================
 *  EMG ACQUISITION DIAGNOSTIC
 *  ----------------------------------------------------------------------------
 *  SAME continuous-ADC acquisition as the real firmware (identical handle cfg,
 *  pattern, sample rate, conv mode, format, deinterleave). NO filtering, NO
 *  envelope, NO normalization. It prints RAW 12-bit counts and per-channel
 *  statistics so you can prove the acquisition is correct and isolate any
 *  problem to the hardware (wiring / floating pin / front-end), not software.
 *
 *  THREE OUTPUT MODES (set OUT_MODE below, or switch live by typing 1/2/3):
 *    1 RAW    : streams raw counts for both channels (for the serial plotter)
 *    2 STATS  : per-block min/mean/max + peak-to-peak + sample counts (verify
 *               which channel actually carries signal vs floats)
 *    3 SYNC   : prints the channel TAG of the first N samples each block, so
 *               you can confirm deinterleave attribution is correct
 *
 *  HOW TO USE TO PROVE SOFTWARE-OK / HARDWARE-AT-FAULT:
 *    - Mode 2 with BOTH electrodes off, inputs floating: both channels show
 *      wandering mean, large p2p -> that's floating-pin noise (expected).
 *    - Ground the unused channel's pin: its mean snaps near 0, p2p tiny ->
 *      proves the channel reads exactly what's on the pin (software correct).
 *    - Electrode connected + relaxed: mean sits near front-end reference
 *      (~1900-2050 counts for AD8232 @ 12-bit/12dB), small p2p; contract ->
 *      p2p grows. That's correct acquisition.
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"

// --- SAME acquisition config as the real firmware ---
#define ADC_CHANNEL_A       ADC_CHANNEL_3   // bicep  (confirm GPIO on S3!)
#define ADC_CHANNEL_B       ADC_CHANNEL_4   // tricep
#define SAMPLE_RATE_HZ      2000
#define READ_LEN            512
#define INTERNAL_BUF_SIZE   2048

// output mode (boot default); change live with 1 / 2 / 3 over serial
typedef enum { OUT_RAW = 1, OUT_STATS = 2, OUT_SYNC = 3 } OutMode;
volatile int g_out = OUT_STATS;             // start in STATS - best for diagnosis

static const char *TAG = "ADC_DIAG";
static adc_continuous_handle_t adc_handle = NULL;

// ----------------------------------------------------------------------------
//  Console (so live 1/2/3 switching works) + a tiny input task
// ----------------------------------------------------------------------------
static void console_init() {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}

static void input_task(void *arg) {
    for (;;) {
        int c = getchar();
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        if (c >= '1' && c <= '3') {
            g_out = c - '0';
            printf("\n[diag] output mode -> %d  (1=RAW 2=STATS 3=SYNC)\n", g_out);
        }
    }
}

// ----------------------------------------------------------------------------
//  Acquisition task - IDENTICAL setup to the real firmware
// ----------------------------------------------------------------------------
static void diag_task(void *arg) {
    printf(">>> DIAG task on core %d\n", xPortGetCoreID());

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

    uint16_t a[max_results], b[max_results];   // raw counts per channel

    printf("[diag] acquisition started: 2 ch, %d Hz, 12-bit.\n", SAMPLE_RATE_HZ);
    printf("[diag] type 1=RAW 2=STATS 3=SYNC.  (boot=STATS)\n\n");

    while (1) {
        if (adc_continuous_read(adc_handle, raw, READ_LEN, &ret_num, portMAX_DELAY) != ESP_OK)
            continue;

        int total = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
        int ca = 0, cb = 0;
        int unknown = 0;                       // samples tagged with neither chan

        // first few raw tags for SYNC mode
        int   tag_seq[8];  int tag_n = 0;

        for (int i = 0; i < total; i++) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw[i * SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t chan = p->type2.channel;
            uint16_t val  = (uint16_t)p->type2.data;

            if (tag_n < 8) tag_seq[tag_n++] = (int)chan;

            if (chan == ADC_CHANNEL_A && ca < max_results)      a[ca++] = val;
            else if (chan == ADC_CHANNEL_B && cb < max_results) b[cb++] = val;
            else unknown++;
        }

        // ---------------- OUTPUT ----------------
        if (g_out == OUT_RAW) {
            // stream raw counts (decimated for readability) for the plotter
            int n = ca < cb ? ca : cb;
            for (int i = 0; i < n; i += 10) {
                printf("Min:0,Max:4095,RAW_A:%u,RAW_B:%u\n",
                       (unsigned)a[i], (unsigned)b[i]);
            }
        }
        else if (g_out == OUT_STATS) {
            // per-channel min/mean/max + peak-to-peak
            uint32_t mnA=4095, mxA=0, mnB=4095, mxB=0;
            double sumA=0, sumB=0;
            for (int i=0;i<ca;i++){ if(a[i]<mnA)mnA=a[i]; if(a[i]>mxA)mxA=a[i]; sumA+=a[i]; }
            for (int i=0;i<cb;i++){ if(b[i]<mnB)mnB=b[i]; if(b[i]>mxB)mxB=b[i]; sumB+=b[i]; }
            float meanA = ca? (float)(sumA/ca):0, meanB = cb? (float)(sumB/cb):0;
            printf("A[n=%3d min=%4u mean=%6.1f max=%4u p2p=%4u]  "
                   "B[n=%3d min=%4u mean=%6.1f max=%4u p2p=%4u]%s\n",
                   ca, (unsigned)mnA, meanA, (unsigned)mxA, (unsigned)(mxA-mnA),
                   cb, (unsigned)mnB, meanB, (unsigned)mxB, (unsigned)(mxB-mnB),
                   unknown? "  <UNKNOWN TAGS!>" : "");
            vTaskDelay(pdMS_TO_TICKS(200));    // ~5 Hz, readable
        }
        else { // OUT_SYNC
            printf("total=%d  ca=%d cb=%d unknown=%d | first tags: ",
                   total, ca, cb, unknown);
            for (int i=0;i<tag_n;i++) printf("%d ", tag_seq[i]);
            printf("(expect alternating %d/%d)\n",
                   (int)ADC_CHANNEL_A, (int)ADC_CHANNEL_B);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

extern "C" void app_main(void) {
    console_init();
    printf("\n=== EMG ACQUISITION DIAGNOSTIC ===\n");
    printf("Channels: A=%d  B=%d   (verify these map to your wired GPIOs!)\n",
           (int)ADC_CHANNEL_A, (int)ADC_CHANNEL_B);

    xTaskCreatePinnedToCore(diag_task,  "DIAG",  8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(input_task, "IN",    4096, NULL, 5,  NULL, 1);
}