/* ============================================================================
 *  SERIAL ECHO TEST  -  isolates the console input path, no motor/FOC/SPI.
 *  ----------------------------------------------------------------------------
 *  Uses the SAME pattern we want in the real firmware:
 *    - console_init() configures the UART VFS line endings + unbuffered IO
 *    - a dedicated input task does blocking getchar() and builds lines
 *    - a separate heartbeat task prints periodically (proves tasks run)
 *
 *  Expected behaviour:
 *    - "heartbeat" prints every second on its own
 *    - when you type a line + Enter, it echoes "[you typed]: <text>"
 *
 *  Build this as a standalone test: point src/CMakeLists.txt SRCS at this file.
 *  It is plain C-style but compiles as .cpp fine. No esp_simplefoc needed, so
 *  you can even drop the esp_simplefoc REQUIRES for this test if you want.
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Console VFS config (IDF 5.x). If these headers/functions are not found,
// see the NOTE at the bottom for the alternative older names.
#include "driver/uart_vfs.h"

// ----------------------------------------------------------------------------
//  Configure the console so blocking getchar() + printf both work cleanly.
//  We do NOT install a second UART driver on UART0 (that conflicts with the
//  console and crashes). We only adjust VFS line endings + buffering.
// ----------------------------------------------------------------------------
static void console_init() {
    setvbuf(stdout, NULL, _IONBF, 0);   // print immediately, no buffering
    setvbuf(stdin,  NULL, _IONBF, 0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}

// ----------------------------------------------------------------------------
//  Heartbeat task - proves the scheduler/tasks are alive independent of input.
// ----------------------------------------------------------------------------
static void heartbeat_task(void *arg) {
    int n = 0;
    for (;;) {
        printf("heartbeat %d\n", n++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----------------------------------------------------------------------------
//  Dedicated input task - blocking getchar() is safe; it does nothing else.
//  Builds a line until newline, then echoes it back.
// ----------------------------------------------------------------------------
static void input_task(void *arg) {
    char line[64];
    size_t idx = 0;

    printf("Input task ready. Type something and press Enter.\n");

    for (;;) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        char ch = (char)c;

        // Echo each raw character code so we can SEE what arrives (debug).
        // Comment this out once it works - it's noisy but very revealing.
        printf("  [rx byte: %d '%c']\n", c, (ch >= 32 && ch < 127) ? ch : '?');

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                line[idx] = '\0';
                idx = 0;
                printf("[you typed]: %s\n", line);
            }
        } else if (idx < sizeof(line) - 1) {
            line[idx++] = ch;
        }
    }
}

extern "C" void app_main(void) {
    console_init();

    printf("\n=== SERIAL ECHO TEST ===\n");
    printf("If you see 'heartbeat N' lines, tasks are running.\n");
    printf("Type a line + Enter; it should echo back.\n\n");

    xTaskCreatePinnedToCore(heartbeat_task, "HEARTBEAT", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(input_task,     "INPUT",     4096, NULL, 6, NULL, 1);
}

/* ============================================================================
 *  NOTE - if the build fails on "driver/uart_vfs.h" or the
 *  uart_vfs_dev_port_set_* functions:
 *
 *  Replace the include with:
 *      #include "esp_vfs_dev.h"
 *  and replace the two calls in console_init() with:
 *      esp_vfs_dev_uart_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
 *      esp_vfs_dev_uart_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
 *
 *  (Older IDF used the esp_vfs_dev_* names; IDF 5.x moved them to uart_vfs_dev_*.)
 * ==========================================================================*/