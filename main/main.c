#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
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

#define RF_SCK_PIN          18
#define RF_MISO_PIN         19
#define RF_MOSI_PIN         20
#define RF_CS_PIN           21
#define RF_RST_PIN          22
#define RF_DIO0_PIN         23

/* Must match gateway */
#define RF_NETWORK_ID       100
#define RF_NODE_ID          1
#define RF_GATEWAY_ID       0

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
static spi_device_handle_t rf_spi = NULL;
static bool g_rf_ok = false;

/* ══════════════════════════════════════════
   SX1276 FSK Mode Driver
   Register map is DIFFERENT from RFM69!
   Configured to be compatible with RFM69 gateway.
   ══════════════════════════════════════════ */

/* SX1276 FSK register addresses */
#define SX_REG_FIFO          0x00
#define SX_REG_OPMODE        0x01
#define SX_REG_BITRATEMSB    0x02
#define SX_REG_BITRATELSB    0x03
#define SX_REG_FDEVMSB       0x04
#define SX_REG_FDEVLSB       0x05
#define SX_REG_FRFMSB        0x06
#define SX_REG_FRFMID        0x07
#define SX_REG_FRFLSB        0x08
#define SX_REG_PACONFIG      0x09
#define SX_REG_PARAMP        0x0A
#define SX_REG_OCP           0x0B
#define SX_REG_IRQFLAGS1     0x3E
#define SX_REG_IRQFLAGS2     0x3F
#define SX_REG_PREAMBLEMSB   0x25
#define SX_REG_PREAMBLELSB   0x26
#define SX_REG_SYNCCONFIG    0x27
#define SX_REG_SYNCVALUE1    0x28
#define SX_REG_SYNCVALUE2    0x29
#define SX_REG_PACKETCONFIG1 0x30
#define SX_REG_PACKETCONFIG2 0x31
#define SX_REG_PAYLOADLEN    0x32
#define SX_REG_NODEADRS      0x33
#define SX_REG_FIFOTHRESH    0x35
#define SX_REG_VERSION       0x42

/* SX1276 OpMode register bits:
 * bit7: LongRangeMode (0=FSK, 1=LoRa)
 * bit6:5: ModulationType (00=FSK)
 * bit4: reserved
 * bit3: LowFrequencyModeOn (0=high freq, for 915MHz)
 * bit2:0: Mode (000=sleep, 001=standby, 010=FS TX, 011=TX, 100=FS RX, 101=RX)
 */
#define SX_MODE_SLEEP     0x00
#define SX_MODE_STANDBY   0x01
#define SX_MODE_FSTX      0x02
#define SX_MODE_TX        0x03
#define SX_MODE_FSRX      0x04
#define SX_MODE_RX        0x05

static esp_err_t rf_write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(reg | 0x80), val },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    return spi_device_transmit(rf_spi, &t);
}

static uint8_t rf_read_reg(uint8_t reg)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(reg & 0x7F), 0x00 },
        .rx_data = { 0 },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_transmit(rf_spi, &t);
    return t.rx_data[1];
}

static void rf_set_mode(uint8_t mode)
{
    /* SX1276 OpMode register: bits [2:0] = mode, keep FSK (bit7=0, bits6:5=00) */
    rf_write_reg(SX_REG_OPMODE, mode);
    /* Wait for ModeReady (bit7 of IrqFlags1) */
    int timeout = 100;
    while (timeout-- > 0) {
        if (rf_read_reg(SX_REG_IRQFLAGS1) & 0x80) return;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ESP_LOGW(TAG, "Mode change timeout (mode=0x%02X)", mode);
}

static esp_err_t rf_init(void)
{
    /* Reset */
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << RF_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_conf);
    gpio_set_level(RF_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(RF_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(RF_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* SPI */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = RF_MOSI_PIN,
        .miso_io_num = RF_MISO_PIN,
        .sclk_io_num = RF_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED), TAG, "SPI init failed");

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = RF_CS_PIN,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev_cfg, &rf_spi), TAG, "SPI device add failed");

    /* Verify SX1276 */
    uint8_t version = rf_read_reg(SX_REG_VERSION);
    ESP_LOGI(TAG, "SX1276 version: 0x%02X (expected 0x12)", version);
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1276 not detected! Check wiring.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SX1276 detected!");

    /* Set Sleep mode first to switch to FSK */
    rf_write_reg(SX_REG_OPMODE, SX_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* FSK mode, Sleep — bit7=0 (FSK), bits6:5=00 (FSK modulation), bits2:0=000 (sleep) */
    rf_write_reg(SX_REG_OPMODE, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Standby */
    rf_set_mode(SX_MODE_STANDBY);

    /* Bitrate: 4800 bps — FXOSC(32MHz) / 4800 = 6667 = 0x1A0B */
    rf_write_reg(SX_REG_BITRATEMSB, 0x1A);
    rf_write_reg(SX_REG_BITRATELSB, 0x0B);

    /* Frequency deviation: 5 kHz — 5000 / FSTEP(61.035) = 82 = 0x0052 */
    rf_write_reg(SX_REG_FDEVMSB, 0x00);
    rf_write_reg(SX_REG_FDEVLSB, 0x52);

    /* Frequency: 915 MHz — 915000000 / FSTEP(61.035) = 14991360 = 0xE4C000 */
    rf_write_reg(SX_REG_FRFMSB, 0xE4);
    rf_write_reg(SX_REG_FRFMID, 0xC0);
    rf_write_reg(SX_REG_FRFLSB, 0x00);

    /* PA config: PA_BOOST pin (bit7=1), MaxPower=7 (bits6:4), OutputPower=15 (bits3:0) = +17dBm */
    rf_write_reg(SX_REG_PACONFIG, 0xFF);
    /* PA ramp time: 40us */
    rf_write_reg(SX_REG_PARAMP, 0x09);
    /* OCP: enable, 100mA */
    rf_write_reg(SX_REG_OCP, 0x2B);

    /* RegDioMapping1 (0x40): DIO0=PacketSent in TX mode */
    rf_write_reg(0x40, 0x00);
    /* RegDioMapping2 (0x41): default */
    rf_write_reg(0x41, 0x00);

    /* Preamble: 3 bytes (RFM69 default) */
    rf_write_reg(SX_REG_PREAMBLEMSB, 0x00);
    rf_write_reg(SX_REG_PREAMBLELSB, 0x03);

    /* Sync config: sync on, 2 bytes sync word — matches RFM69 */
    rf_write_reg(SX_REG_SYNCCONFIG, 0x91); /* AutoRestartRx=on, SyncOn=1, SyncSize=1 (2 bytes) */
    rf_write_reg(SX_REG_SYNCVALUE1, 0x2D); /* Must match RFM69 */
    rf_write_reg(SX_REG_SYNCVALUE2, RF_NETWORK_ID);

    /* Packet config1: variable length (bit7=1), CRC on (bit4=1) */
    rf_write_reg(SX_REG_PACKETCONFIG1, 0x90);
    /* Packet config2: DataMode=Packet (bit6=1), also try enabling sequencer */
    rf_write_reg(SX_REG_PACKETCONFIG2, 0x40);
    /* RegSeqConfig1 (0x36): enable sequencer for auto TX */
    /* RegImageCal (0x3B): auto image cal */
    /* Max payload length */
    rf_write_reg(SX_REG_PAYLOADLEN, 66);
    rf_write_reg(SX_REG_NODEADRS, RF_NODE_ID);

    /* FIFO threshold: TxStartCondition=FifoNotEmpty (bit7=1), threshold=15 */
    rf_write_reg(SX_REG_FIFOTHRESH, 0x8F);

    /* Verify key registers */
    ESP_LOGI(TAG, "  OpMode=0x%02X BitRate=0x%02X%02X Fdev=0x%02X%02X",
             rf_read_reg(SX_REG_OPMODE),
             rf_read_reg(SX_REG_BITRATEMSB), rf_read_reg(SX_REG_BITRATELSB),
             rf_read_reg(SX_REG_FDEVMSB), rf_read_reg(SX_REG_FDEVLSB));
    ESP_LOGI(TAG, "  Frf=0x%02X%02X%02X Sync=0x%02X%02X PktCfg1=0x%02X PktCfg2=0x%02X",
             rf_read_reg(SX_REG_FRFMSB), rf_read_reg(SX_REG_FRFMID), rf_read_reg(SX_REG_FRFLSB),
             rf_read_reg(SX_REG_SYNCVALUE1), rf_read_reg(SX_REG_SYNCVALUE2),
             rf_read_reg(SX_REG_PACKETCONFIG1), rf_read_reg(SX_REG_PACKETCONFIG2));

    /* Back to standby */
    rf_set_mode(SX_MODE_STANDBY);

    ESP_LOGI(TAG, "SX1276 FSK configured: 915MHz, 4800bps, Network:%d, Node:%d", RF_NETWORK_ID, RF_NODE_ID);
    return ESP_OK;
}

static esp_err_t rf_send(const uint8_t *data, size_t len)
{
    if (len > 61) len = 61;

    /* Sleep to reset state machine */
    rf_write_reg(SX_REG_OPMODE, SX_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* FSK standby */
    rf_write_reg(SX_REG_OPMODE, SX_MODE_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Clear FIFO: set FifoOverrun bit to flush */
    rf_write_reg(SX_REG_IRQFLAGS2, 0x10);

    /* Write packet to FIFO one byte at a time */
    rf_write_reg(SX_REG_FIFO, (uint8_t)(len + 3));  /* Length */
    rf_write_reg(SX_REG_FIFO, RF_GATEWAY_ID);        /* Target */
    rf_write_reg(SX_REG_FIFO, RF_NODE_ID);            /* Sender */
    rf_write_reg(SX_REG_FIFO, 0x00);                  /* Control */
    for (size_t i = 0; i < len; i++) {
        rf_write_reg(SX_REG_FIFO, data[i]);
    }

    /* Enter TX */
    rf_write_reg(SX_REG_OPMODE, SX_MODE_TX);

    /* Wait for PacketSent (bit3 of IrqFlags2 at 0x3F) */
    int timeout = 50;
    while (timeout-- > 0) {
        uint8_t f2 = rf_read_reg(SX_REG_IRQFLAGS2);
        if (f2 & 0x08) { /* PacketSent */
            rf_write_reg(SX_REG_OPMODE, SX_MODE_STANDBY);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    uint8_t om = rf_read_reg(SX_REG_OPMODE);
    ESP_LOGW(TAG, "TX timeout (f1=0x%02X f2=0x%02X opmode=0x%02X)",
             rf_read_reg(SX_REG_IRQFLAGS1), rf_read_reg(SX_REG_IRQFLAGS2), om);
    rf_write_reg(SX_REG_OPMODE, SX_MODE_STANDBY);
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
                    "{\"id\":%d,\"la\":%.6f,\"lo\":%.6f,\"al\":%.1f,"
                    "\"sp\":%.2f,\"sa\":%d,"
                    "\"bh\":\"%s\",\"av\":%.4f,"
                    "\"t\":\"%02d:%02d:%02d\"}",
                    RF_NODE_ID,
                    g_gps_data.latitude, g_gps_data.longitude, g_gps_data.altitude,
                    g_gps_data.speed, g_gps_data.sats_in_use,
                    behavior_str[g_behavior], g_accel_variance,
                    g_gps_data.tim.hour, g_gps_data.tim.minute, g_gps_data.tim.second);
            } else {
                snprintf(json, sizeof(json),
                    "{\"id\":%d,\"la\":0,\"lo\":0,\"al\":0,"
                    "\"sp\":0,\"sa\":0,"
                    "\"bh\":\"%s\",\"av\":%.4f}",
                    RF_NODE_ID,
                    behavior_str[g_behavior], g_accel_variance);
            }

            /* Print to serial */
            printf("%s\n", json);

            /* Send via radio */
            if (g_rf_ok) {
                if (rf_send((uint8_t *)json, strlen(json)) == ESP_OK) {
                    ESP_LOGI(TAG, "RF TX OK (%d bytes)", strlen(json));
                } else {
                    ESP_LOGW(TAG, "RF TX failed");
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
    ESP_LOGI(TAG, "=== Cow GPS Tracker v1.3 ===");
    ESP_LOGI(TAG, "GPS + Accel + SX1276-FSK");

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

    /* Start radio */
    g_rf_ok = (rf_init() == ESP_OK);

    /* Start accelerometer */
    xTaskCreate(accel_task, "accel", 4096, NULL, 5, NULL);

    /* Start reporting */
    xTaskCreate(report_task, "report", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "All systems started.");
}
