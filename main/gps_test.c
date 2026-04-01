/* Bare minimum GPS test — just raw UART read on GPIO5 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

#define TAG "gps_test"
#define GPS_RX_PIN  5
#define BUF_SIZE    512

void app_main(void)
{
    ESP_LOGI(TAG, "=== GPS Raw Test ===");
    ESP_LOGI(TAG, "Reading UART on GPIO%d at 9600 baud", GPS_RX_PIN);

    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &cfg);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t *buf = malloc(BUF_SIZE);
    int count = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM_1, buf, BUF_SIZE - 1, pdMS_TO_TICKS(2000));
        count++;
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "GOT %d bytes: %s", len, (char *)buf);
        } else {
            ESP_LOGW(TAG, "[%d] No data", count);
        }
    }
}
