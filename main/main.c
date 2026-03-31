#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "nmea_parser.h"

static const char *TAG = "tracker";

/* ── Pin Definitions ── */
#define GPS_UART_RX_PIN     5
#define GPS_UART_NUM        UART_NUM_1
#define GPS_UART_BAUD       9600

#define I2C_SDA_PIN         6
#define I2C_SCL_PIN         7
#define MPU6050_ADDR        0x68

#define LORA_SCK_PIN        18
#define LORA_MISO_PIN       19
#define LORA_MOSI_PIN       20
#define LORA_CS_PIN         21
#define LORA_RST_PIN        22
#define LORA_DIO0_PIN       23

/* ── Behavior States ── */
typedef enum {
    BEHAVIOR_RESTING,
    BEHAVIOR_GRAZING,
    BEHAVIOR_WALKING,
    BEHAVIOR_RUNNING,
} behavior_t;

static const char *behavior_str[] = {"RESTING", "GRAZING", "WALKING", "RUNNING"};

static const uint32_t gps_interval_ms[] = {
    [BEHAVIOR_RESTING]  = 60000,
    [BEHAVIOR_GRAZING]  = 20000,
    [BEHAVIOR_WALKING]  = 5000,
    [BEHAVIOR_RUNNING]  = 1000,
};

/* ── Global State ── */
static gps_t g_gps_data;
static bool g_gps_fix = false;
static behavior_t g_behavior = BEHAVIOR_RESTING;
static float g_accel_variance = 0.0f;
static i2c_master_dev_handle_t mpu6050_handle = NULL;
static spi_device_handle_t lora_spi = NULL;
static bool g_lora_ok = false;

/* ══════════════════════════════════════════
   LoRa (RFM9x / SX1276) SPI Driver
   ══════════════════════════════════════════ */

static esp_err_t lora_write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(reg | 0x80), val },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    return spi_device_transmit(lora_spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(reg & 0x7F), 0x00 },
        .rx_data = { 0 },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_transmit(lora_spi, &t);
    return t.rx_data[1];
}

static esp_err_t lora_init(void)
{
    /* Reset the module */
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << LORA_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_conf);
    gpio_set_level(LORA_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* SPI bus config */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LORA_MOSI_PIN,
        .miso_io_num = LORA_MISO_PIN,
        .sclk_io_num = LORA_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED), TAG, "SPI bus init failed");

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,  /* 1 MHz */
        .mode = 0,
        .spics_io_num = LORA_CS_PIN,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev_cfg, &lora_spi), TAG, "SPI device add failed");

    /* Check version register — SX1276 should return 0x12 */
    uint8_t version = lora_read_reg(0x42);
    ESP_LOGI(TAG, "LoRa version register: 0x%02X (expected 0x12 for SX1276)", version);

    if (version != 0x12) {
        ESP_LOGE(TAG, "LoRa NOT detected! Check wiring:");
        ESP_LOGE(TAG, "  SCK  -> GPIO%d", LORA_SCK_PIN);
        ESP_LOGE(TAG, "  MISO -> GPIO%d", LORA_MISO_PIN);
        ESP_LOGE(TAG, "  MOSI -> GPIO%d", LORA_MOSI_PIN);
        ESP_LOGE(TAG, "  CS   -> GPIO%d", LORA_CS_PIN);
        ESP_LOGE(TAG, "  RST  -> GPIO%d", LORA_RST_PIN);
        ESP_LOGE(TAG, "  VIN  -> 3.3V, GND -> GND");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LoRa SX1276 detected!");

    /* Set sleep mode */
    lora_write_reg(0x01, 0x00);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Set LoRa mode */
    lora_write_reg(0x01, 0x80);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Set frequency — 915 MHz (0xE4C000) */
    /* Change to 433 MHz (0x6C8000) if your module is 433 */
    lora_write_reg(0x06, 0xE4);
    lora_write_reg(0x07, 0xC0);
    lora_write_reg(0x08, 0x00);

    /* Set bandwidth 125kHz, coding rate 4/5, explicit header */
    lora_write_reg(0x1D, 0x72);
    /* Spreading factor 7, CRC on */
    lora_write_reg(0x1E, 0x74);
    /* Set TX power to 17 dBm */
    lora_write_reg(0x09, 0x8F);

    /* Standby mode */
    lora_write_reg(0x01, 0x81);

    ESP_LOGI(TAG, "LoRa configured: 915MHz, SF7, BW125kHz, 17dBm");
    return ESP_OK;
}

static esp_err_t lora_send(const uint8_t *data, size_t len)
{
    /* Set standby */
    lora_write_reg(0x01, 0x81);
    /* Set FIFO pointer to base */
    lora_write_reg(0x0D, 0x00);
    lora_write_reg(0x0E, 0x00);
    /* Write data to FIFO */
    for (size_t i = 0; i < len; i++) {
        lora_write_reg(0x00, data[i]);
    }
    /* Set payload length */
    lora_write_reg(0x22, (uint8_t)len);
    /* Set TX mode */
    lora_write_reg(0x01, 0x83);
    /* Wait for TX done (IRQ flag bit 3) */
    int timeout = 200;
    while (timeout-- > 0) {
        uint8_t irq = lora_read_reg(0x12);
        if (irq & 0x08) {
            /* Clear IRQ */
            lora_write_reg(0x12, 0xFF);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "LoRa TX timeout");
    return ESP_ERR_TIMEOUT;
}

/* ══════════════════════════════════════════
   MPU6050
   ══════════════════════════════════════════ */

static esp_err_t mpu6050_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus_handle), TAG, "I2C bus init failed");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_handle), TAG, "MPU6050 add failed");

    uint8_t wake_cmd[] = {0x6B, 0x00};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(mpu6050_handle, wake_cmd, sizeof(wake_cmd), 100), TAG, "MPU6050 wake failed");

    uint8_t accel_cfg[] = {0x1C, 0x00};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(mpu6050_handle, accel_cfg, sizeof(accel_cfg), 100), TAG, "MPU6050 accel config failed");

    ESP_LOGI(TAG, "MPU6050 initialized on SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
    return ESP_OK;
}

static esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t reg = 0x3B;
    uint8_t data[6];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(mpu6050_handle, &reg, 1, data, 6, 100), TAG, "MPU6050 read failed");

    *ax = (int16_t)((data[0] << 8) | data[1]) / 16384.0f;
    *ay = (int16_t)((data[2] << 8) | data[3]) / 16384.0f;
    *az = (int16_t)((data[4] << 8) | data[5]) / 16384.0f;
    return ESP_OK;
}

/* ── Behavior Classification ── */
static behavior_t classify_behavior(float variance)
{
    if (variance < 0.005f) return BEHAVIOR_RESTING;
    else if (variance < 0.05f) return BEHAVIOR_GRAZING;
    else if (variance < 0.3f) return BEHAVIOR_WALKING;
    else return BEHAVIOR_RUNNING;
}

/* ── Accelerometer Task ── */
#define ACCEL_WINDOW_SIZE  20
static float mag_history[ACCEL_WINDOW_SIZE];
static int mag_idx = 0;

static void accel_task(void *arg)
{
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 not found — defaulting to GRAZING");
        g_behavior = BEHAVIOR_GRAZING;
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        float ax, ay, az;
        if (mpu6050_read_accel(&ax, &ay, &az) == ESP_OK) {
            float mag = sqrtf(ax * ax + ay * ay + az * az);
            mag_history[mag_idx % ACCEL_WINDOW_SIZE] = mag;
            mag_idx++;

            int count = (mag_idx < ACCEL_WINDOW_SIZE) ? mag_idx : ACCEL_WINDOW_SIZE;
            float mean = 0;
            for (int i = 0; i < count; i++) mean += mag_history[i];
            mean /= count;

            float var = 0;
            for (int i = 0; i < count; i++) {
                float diff = mag_history[i] - mean;
                var += diff * diff;
            }
            var /= count;

            g_accel_variance = var;
            g_behavior = classify_behavior(var);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ── GPS Event Handler ── */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == GPS_UPDATE) {
        memcpy(&g_gps_data, event_data, sizeof(gps_t));
        g_gps_fix = (g_gps_data.fix != GPS_FIX_INVALID);
    }
}

/* ── Main Report Task ── */
static void report_task(void *arg)
{
    int64_t last_report_us = 0;

    while (1) {
        int64_t now_us = esp_timer_get_time();
        uint32_t interval_ms = gps_interval_ms[g_behavior];
        int64_t elapsed_ms = (now_us - last_report_us) / 1000;

        if (elapsed_ms >= interval_ms) {
            last_report_us = now_us;

            char json[256];
            if (g_gps_fix) {
                snprintf(json, sizeof(json),
                    "{\"cow_id\":1,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,"
                    "\"speed\":%.2f,\"sats\":%d,"
                    "\"behavior\":\"%s\",\"accel_var\":%.4f,"
                    "\"time\":\"%02d:%02d:%02d\",\"date\":\"%04d-%02d-%02d\"}",
                    g_gps_data.latitude, g_gps_data.longitude, g_gps_data.altitude,
                    g_gps_data.speed, g_gps_data.sats_in_use,
                    behavior_str[g_behavior], g_accel_variance,
                    g_gps_data.tim.hour, g_gps_data.tim.minute, g_gps_data.tim.second,
                    g_gps_data.date.year + 2000, g_gps_data.date.month, g_gps_data.date.day);
            } else {
                snprintf(json, sizeof(json),
                    "{\"cow_id\":1,\"lat\":0,\"lon\":0,\"alt\":0,"
                    "\"speed\":0,\"sats\":0,"
                    "\"behavior\":\"%s\",\"accel_var\":%.4f}",
                    behavior_str[g_behavior], g_accel_variance);
            }

            /* Print to serial */
            printf("%s\n", json);

            /* Send via LoRa if available */
            if (g_lora_ok) {
                if (lora_send((uint8_t *)json, strlen(json)) == ESP_OK) {
                    ESP_LOGI(TAG, "LoRa TX OK (%d bytes)", strlen(json));
                }
            }

            ESP_LOGI(TAG, "[%s] Lat:%.5f Lon:%.5f Sats:%d Interval:%lums",
                     behavior_str[g_behavior],
                     g_gps_data.latitude, g_gps_data.longitude,
                     g_gps_data.sats_in_use, (unsigned long)interval_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ── App Main ── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Cow GPS Tracker v1.1 ===");
    ESP_LOGI(TAG, "GPS + Accel + LoRa");

    /* Start GPS */
    nmea_parser_config_t gps_config = {
        .uart = {
            .uart_port = GPS_UART_NUM,
            .rx_pin = GPS_UART_RX_PIN,
            .baud_rate = GPS_UART_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .event_queue_size = 16,
        }
    };
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&gps_config);
    if (nmea_hdl) {
        nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
        ESP_LOGI(TAG, "GPS started on GPIO%d", GPS_UART_RX_PIN);
    }

    /* Start LoRa */
    g_lora_ok = (lora_init() == ESP_OK);

    /* Start accelerometer */
    xTaskCreate(accel_task, "accel", 4096, NULL, 5, NULL);

    /* Start reporting */
    xTaskCreate(report_task, "report", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "All systems started.");
}
