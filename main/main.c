#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "nmea_parser.h"

static const char *TAG = "tracker";

/* ── Wi-Fi Config ── */
#define WIFI_SSID       "GTother"
#define WIFI_PASS       "GeorgeP@1927"
#define DASHBOARD_URL   "http://143.215.190.84:5000/api/data"

/* ── Pin Definitions ── */
#define GPS_UART_RX_PIN     5
#define GPS_UART_NUM        UART_NUM_1
#define GPS_UART_BAUD       9600

#define I2C_SDA_PIN         6
#define I2C_SCL_PIN         7
#define MPU6050_ADDR        0x68

#define RF_SCK_PIN          18
#define RF_MISO_PIN         19
#define RF_MOSI_PIN         20
#define RF_CS_PIN           21
#define RF_RST_PIN          22
#define RF_DIO0_PIN         23

#define RF_NETWORK_ID       100
#define RF_NODE_ID          1
#define RF_GATEWAY_ID       0

/* ── Behavior States ── */
typedef enum {
    BEHAVIOR_RESTING, BEHAVIOR_GRAZING, BEHAVIOR_WALKING, BEHAVIOR_RUNNING,
} behavior_t;
static const char *behavior_str[] = {"RESTING", "GRAZING", "WALKING", "RUNNING"};
static const uint32_t gps_interval_ms[] = { 60000, 20000, 5000, 1000 };

/* ── Global State ── */
static gps_t g_gps_data;
static bool g_gps_fix = false;
static behavior_t g_behavior = BEHAVIOR_RESTING;
static float g_accel_variance = 0.0f;
static i2c_master_dev_handle_t mpu6050_handle = NULL;
static spi_device_handle_t rf_spi = NULL;
static bool g_rf_ok = false;

/* ══════════════════════════════════════════
   RFM69HCW Driver
   ══════════════════════════════════════════ */

#define REG_FIFO          0x00
#define REG_OPMODE        0x01
#define REG_DATAMODUL     0x02
#define REG_BITRATEMSB    0x03
#define REG_BITRATELSB    0x04
#define REG_FDEVMSB       0x05
#define REG_FDEVLSB       0x06
#define REG_FRFMSB        0x07
#define REG_FRFMID        0x08
#define REG_FRFLSB        0x09
#define REG_VERSION       0x10
#define REG_PALEVEL       0x11
#define REG_RXBW          0x19
#define REG_AFCBW         0x1A
#define REG_IRQFLAGS1     0x27
#define REG_IRQFLAGS2     0x28
#define REG_SYNCCONFIG    0x2E
#define REG_SYNCVALUE1    0x2F
#define REG_SYNCVALUE2    0x30
#define REG_PACKETCONFIG1 0x37
#define REG_PAYLOADLEN    0x38
#define REG_NODEADRS      0x39
#define REG_FIFOTHRESH    0x3C
#define REG_PACKETCONFIG2 0x3D
#define REG_TESTDAGC      0x6F

#define MODE_STANDBY  0x04
#define MODE_TX       0x0C
#define MODE_RX       0x10

static esp_err_t rf_write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = { .length = 16, .tx_data = { (uint8_t)(reg | 0x80), val }, .flags = SPI_TRANS_USE_TXDATA };
    return spi_device_transmit(rf_spi, &t);
}

static uint8_t rf_read_reg(uint8_t reg)
{
    spi_transaction_t t = { .length = 16, .tx_data = { (uint8_t)(reg & 0x7F), 0x00 }, .rx_data = {0}, .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA };
    spi_device_transmit(rf_spi, &t);
    return t.rx_data[1];
}

static void rf_set_mode(uint8_t mode)
{
    rf_write_reg(REG_OPMODE, (rf_read_reg(REG_OPMODE) & 0xE3) | mode);
    int timeout = 100;
    while (timeout-- > 0) { if (rf_read_reg(REG_IRQFLAGS1) & 0x80) return; vTaskDelay(pdMS_TO_TICKS(1)); }
}

static esp_err_t rf_init(void)
{
    gpio_config_t rst_conf = { .pin_bit_mask = (1ULL << RF_RST_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&rst_conf);
    gpio_set_level(RF_RST_PIN, 1); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(RF_RST_PIN, 0); vTaskDelay(pdMS_TO_TICKS(10));

    spi_bus_config_t bus = { .mosi_io_num = RF_MOSI_PIN, .miso_io_num = RF_MISO_PIN, .sclk_io_num = RF_SCK_PIN, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 256 };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED), TAG, "SPI init failed");

    spi_device_interface_config_t dev = { .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = RF_CS_PIN, .queue_size = 1 };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev, &rf_spi), TAG, "SPI device add failed");

    uint8_t version = rf_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "RFM69 version: 0x%02X (expected 0x24)", version);
    if (version != 0x24) { ESP_LOGE(TAG, "RFM69 not detected!"); return ESP_FAIL; }
    ESP_LOGI(TAG, "RFM69HCW detected!");

    rf_set_mode(MODE_STANDBY);
    rf_write_reg(REG_DATAMODUL, 0x00);
    rf_write_reg(REG_BITRATEMSB, 0x1A); rf_write_reg(REG_BITRATELSB, 0x0B);
    rf_write_reg(REG_FDEVMSB, 0x00); rf_write_reg(REG_FDEVLSB, 0x52);
    rf_write_reg(REG_FRFMSB, 0xE4); rf_write_reg(REG_FRFMID, 0xC0); rf_write_reg(REG_FRFLSB, 0x00);
    rf_write_reg(REG_RXBW, 0x55); rf_write_reg(REG_AFCBW, 0x55);
    rf_write_reg(REG_SYNCCONFIG, 0x88);
    rf_write_reg(REG_SYNCVALUE1, 0x2D); rf_write_reg(REG_SYNCVALUE2, RF_NETWORK_ID);
    rf_write_reg(REG_PACKETCONFIG1, 0x90);
    rf_write_reg(REG_PAYLOADLEN, 66);
    rf_write_reg(REG_NODEADRS, RF_NODE_ID);
    rf_write_reg(REG_FIFOTHRESH, 0x8F);
    rf_write_reg(REG_PACKETCONFIG2, 0x12);
    rf_write_reg(REG_TESTDAGC, 0x30);
    /* PA Level: PA1 only, +13dBm (safer for testing without antenna) */
    rf_write_reg(REG_PALEVEL, 0x5F);
    /* Normal power mode (not high power) */
    rf_write_reg(0x5A, 0x55); /* RegTestPa1: normal */
    rf_write_reg(0x5C, 0x70); /* RegTestPa2: normal */

    /* Verify IrqFlags after init */
    /* Verify writes actually stuck */
    ESP_LOGI(TAG, "Verify: OpMode=0x%02X BitRate=0x%02X%02X Freq=0x%02X%02X%02X Sync=0x%02X%02X",
             rf_read_reg(REG_OPMODE),
             rf_read_reg(REG_BITRATEMSB), rf_read_reg(REG_BITRATELSB),
             rf_read_reg(REG_FRFMSB), rf_read_reg(REG_FRFMID), rf_read_reg(REG_FRFLSB),
             rf_read_reg(REG_SYNCVALUE1), rf_read_reg(REG_SYNCVALUE2));
    /* Write a test value and read it back */
    rf_write_reg(REG_NODEADRS, 0xAA);
    uint8_t test = rf_read_reg(REG_NODEADRS);
    ESP_LOGI(TAG, "Write test: wrote 0xAA to NODEADRS, read back 0x%02X (%s)",
             test, test == 0xAA ? "MOSI OK" : "MOSI BROKEN");
    rf_set_mode(MODE_STANDBY);

    ESP_LOGI(TAG, "RFM69 configured: 915MHz, 4800bps, Network:%d, Node:%d", RF_NETWORK_ID, RF_NODE_ID);
    return ESP_OK;
}

static esp_err_t rf_send(const uint8_t *data, size_t len)
{
    if (len > 61) len = 61;
    rf_set_mode(MODE_STANDBY);

    /* Clear any pending FIFO data */
    rf_write_reg(REG_IRQFLAGS2, 0x10); /* Set FifoOverrun to clear FIFO */

    rf_write_reg(REG_FIFO, (uint8_t)(len + 3));
    rf_write_reg(REG_FIFO, RF_GATEWAY_ID);
    rf_write_reg(REG_FIFO, RF_NODE_ID);
    rf_write_reg(REG_FIFO, 0x00);
    for (size_t i = 0; i < len; i++) rf_write_reg(REG_FIFO, data[i]);

    ESP_LOGI(TAG, "TX: FIFO written, flags before TX: f1=0x%02X f2=0x%02X", rf_read_reg(REG_IRQFLAGS1), rf_read_reg(REG_IRQFLAGS2));

    rf_set_mode(MODE_TX);

    ESP_LOGI(TAG, "TX: mode set, flags: f1=0x%02X f2=0x%02X opmode=0x%02X", rf_read_reg(REG_IRQFLAGS1), rf_read_reg(REG_IRQFLAGS2), rf_read_reg(REG_OPMODE));

    int wait = 100;
    while (wait-- > 0) {
        if (rf_read_reg(REG_IRQFLAGS2) & 0x08) { rf_set_mode(MODE_STANDBY); return ESP_OK; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "RF TX timeout");
    rf_set_mode(MODE_STANDBY);
    return ESP_ERR_TIMEOUT;
}

/* ══════════════════════════════════════════
   Wi-Fi + HTTP
   ══════════════════════════════════════════ */

static EventGroupHandle_t wifi_events;
#define WIFI_CONNECTED_BIT BIT0
static bool wifi_connected = false;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) { wifi_connected = false; esp_wifi_connect(); }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Wi-Fi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Connecting to Wi-Fi '%s'...", WIFI_SSID);
    xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT, false, true, pdMS_TO_TICKS(10000));
}

static void wifi_post(const char *short_json)
{
    if (!wifi_connected) return;
    char expanded[512]; char *p = expanded; const char *s = short_json;
    while (*s && (p - expanded) < (int)sizeof(expanded) - 50) {
        if (strncmp(s, "\"id\"", 4) == 0) { memcpy(p, "\"cow_id\"", 8); p += 8; s += 4; }
        else if (strncmp(s, "\"la\"", 4) == 0) { memcpy(p, "\"lat\"", 5); p += 5; s += 4; }
        else if (strncmp(s, "\"lo\"", 4) == 0) { memcpy(p, "\"lon\"", 5); p += 5; s += 4; }
        else if (strncmp(s, "\"al\"", 4) == 0) { memcpy(p, "\"alt\"", 5); p += 5; s += 4; }
        else if (strncmp(s, "\"sp\"", 4) == 0) { memcpy(p, "\"speed\"", 7); p += 7; s += 4; }
        else if (strncmp(s, "\"sa\"", 4) == 0) { memcpy(p, "\"sats\"", 6); p += 6; s += 4; }
        else if (strncmp(s, "\"bh\"", 4) == 0) { memcpy(p, "\"behavior\"", 10); p += 10; s += 4; }
        else if (strncmp(s, "\"av\"", 4) == 0) { memcpy(p, "\"accel_var\"", 11); p += 11; s += 4; }
        else if (strncmp(s, "\"t\"", 3) == 0) { memcpy(p, "\"time\"", 6); p += 6; s += 3; }
        else { *p++ = *s++; }
    }
    *p = '\0';

    esp_http_client_config_t config = { .url = DASHBOARD_URL, .method = HTTP_METHOD_POST, .timeout_ms = 5000 };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, expanded, strlen(expanded));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) ESP_LOGI(TAG, "Wi-Fi POST OK (HTTP %d)", esp_http_client_get_status_code(client));
    else ESP_LOGW(TAG, "Wi-Fi POST failed: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
}

/* ══════════════════════════════════════════
   MPU6050
   ══════════════════════════════════════════ */

static esp_err_t mpu6050_init(void)
{
    i2c_master_bus_config_t bus_config = { .i2c_port = I2C_NUM_0, .sda_io_num = I2C_SDA_PIN, .scl_io_num = I2C_SCL_PIN, .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7 };
    i2c_master_bus_handle_t bus_handle;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus_handle), TAG, "I2C bus init failed");
    i2c_device_config_t dev_config = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = MPU6050_ADDR, .scl_speed_hz = 400000 };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_handle), TAG, "MPU6050 add failed");
    uint8_t wake[] = {0x6B, 0x00}; ESP_RETURN_ON_ERROR(i2c_master_transmit(mpu6050_handle, wake, 2, 100), TAG, "MPU6050 wake failed");
    uint8_t acfg[] = {0x1C, 0x00}; ESP_RETURN_ON_ERROR(i2c_master_transmit(mpu6050_handle, acfg, 2, 100), TAG, "MPU6050 config failed");
    ESP_LOGI(TAG, "MPU6050 initialized"); return ESP_OK;
}

static esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t reg = 0x3B; uint8_t data[6];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(mpu6050_handle, &reg, 1, data, 6, 100), TAG, "MPU6050 read failed");
    *ax = (int16_t)((data[0] << 8) | data[1]) / 16384.0f;
    *ay = (int16_t)((data[2] << 8) | data[3]) / 16384.0f;
    *az = (int16_t)((data[4] << 8) | data[5]) / 16384.0f;
    return ESP_OK;
}

static behavior_t classify_behavior(float variance)
{
    if (variance < 0.005f) return BEHAVIOR_RESTING;
    else if (variance < 0.05f) return BEHAVIOR_GRAZING;
    else if (variance < 0.3f) return BEHAVIOR_WALKING;
    else return BEHAVIOR_RUNNING;
}

#define ACCEL_WINDOW 20
static float mag_hist[ACCEL_WINDOW];
static int mag_idx = 0;

static void accel_task(void *arg)
{
    if (mpu6050_init() != ESP_OK) { g_behavior = BEHAVIOR_GRAZING; vTaskDelete(NULL); return; }
    while (1) {
        float ax, ay, az;
        if (mpu6050_read_accel(&ax, &ay, &az) == ESP_OK) {
            float mag = sqrtf(ax*ax + ay*ay + az*az);
            mag_hist[mag_idx % ACCEL_WINDOW] = mag; mag_idx++;
            int cnt = (mag_idx < ACCEL_WINDOW) ? mag_idx : ACCEL_WINDOW;
            float mean = 0; for (int i = 0; i < cnt; i++) mean += mag_hist[i]; mean /= cnt;
            float var = 0; for (int i = 0; i < cnt; i++) { float d = mag_hist[i] - mean; var += d*d; } var /= cnt;
            g_accel_variance = var; g_behavior = classify_behavior(var);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ── GPS ── */
static void gps_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == GPS_UPDATE) { memcpy(&g_gps_data, data, sizeof(gps_t)); g_gps_fix = (g_gps_data.fix != GPS_FIX_INVALID); }
}

/* ── Report Task ── */
static void report_task(void *arg)
{
    int64_t last = 0;
    while (1) {
        int64_t now = esp_timer_get_time();
        uint32_t interval = gps_interval_ms[g_behavior];
        if ((now - last) / 1000 >= interval) {
            last = now;
            char json[256];
            if (g_gps_fix)
                snprintf(json, sizeof(json), "{\"id\":%d,\"la\":%.6f,\"lo\":%.6f,\"al\":%.1f,\"sp\":%.2f,\"sa\":%d,\"bh\":\"%s\",\"av\":%.4f,\"t\":\"%02d:%02d:%02d\"}",
                    RF_NODE_ID, g_gps_data.latitude, g_gps_data.longitude, g_gps_data.altitude,
                    g_gps_data.speed, g_gps_data.sats_in_use, behavior_str[g_behavior], g_accel_variance,
                    g_gps_data.tim.hour, g_gps_data.tim.minute, g_gps_data.tim.second);
            else
                snprintf(json, sizeof(json), "{\"id\":%d,\"la\":0,\"lo\":0,\"al\":0,\"sp\":0,\"sa\":0,\"bh\":\"%s\",\"av\":%.4f}",
                    RF_NODE_ID, behavior_str[g_behavior], g_accel_variance);

            /* Print full JSON to serial for debugging */
            printf("%s\n", json);

            /* Send compact CSV over radio (fits in 61 byte FIFO limit) */
            /* Format: id,lat,lon,alt,speed,sats,behavior_char,accel_var */
            if (g_rf_ok) {
                char csv[60];
                char bh = behavior_str[g_behavior][0]; /* R, G, W, or R */
                if (g_gps_fix)
                    snprintf(csv, sizeof(csv), "%d,%.4f,%.4f,%.0f,%.1f,%d,%c,%.2f",
                        RF_NODE_ID, g_gps_data.latitude, g_gps_data.longitude,
                        g_gps_data.altitude, g_gps_data.speed,
                        g_gps_data.sats_in_use, bh, g_accel_variance);
                else
                    snprintf(csv, sizeof(csv), "%d,0,0,0,0,0,%c,%.2f",
                        RF_NODE_ID, bh, g_accel_variance);

                if (rf_send((uint8_t *)csv, strlen(csv)) == ESP_OK)
                    ESP_LOGI(TAG, "RF TX OK (%d bytes): %s", strlen(csv), csv);
                else
                    ESP_LOGW(TAG, "RF TX failed");
            }
            ESP_LOGI(TAG, "[%s] Lat:%.5f Lon:%.5f Sats:%d", behavior_str[g_behavior], g_gps_data.latitude, g_gps_data.longitude, g_gps_data.sats_in_use);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ── App Main ── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Cow GPS Tracker v2.0 ===");
    ESP_LOGI(TAG, "GPS + Accel + RFM69 + Wi-Fi");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }

    /* Init radio FIRST before Wi-Fi */
    g_rf_ok = (rf_init() == ESP_OK);

    wifi_init();

    nmea_parser_config_t gps_config = { .uart = { .uart_port = GPS_UART_NUM, .rx_pin = GPS_UART_RX_PIN, .baud_rate = GPS_UART_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .event_queue_size = 16 } };
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&gps_config);
    if (nmea_hdl) { nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL); ESP_LOGI(TAG, "GPS started on GPIO%d", GPS_UART_RX_PIN); }

    /* Re-verify radio after Wi-Fi */
    if (g_rf_ok) {
        uint8_t v = rf_read_reg(REG_VERSION);
        ESP_LOGI(TAG, "RFM69 post-WiFi check: version=0x%02X", v);
        if (v != 0x24) {
            ESP_LOGW(TAG, "RFM69 lost after Wi-Fi init! Re-initializing...");
            g_rf_ok = (rf_init() == ESP_OK);
        }
    }

    xTaskCreate(accel_task, "accel", 4096, NULL, 5, NULL);
    xTaskCreate(report_task, "report", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "All systems started.");
}
