/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"

#define NMEA_PARSER_RUNTIME_BUFFER_SIZE (512)
#define NMEA_PARSER_RING_BUFFER_SIZE    (1024)
#define NMEA_MAX_STATEMENT_ITEM_LENGTH  (16)
#define NMEA_EVENT_LOOP_QUEUE_SIZE      (16)
#define NMEA_PARSER_TASK_STACK_SIZE     (3072)
#define NMEA_PARSER_TASK_PRIORITY       (2)

ESP_EVENT_DEFINE_BASE(ESP_NMEA_EVENT);

static const char *GPS_TAG = "nmea_parser";

typedef struct {
    uint8_t item_pos;
    uint8_t item_num;
    uint8_t asterisk;
    uint8_t crc;
    uint8_t parsed_statement;
    uint8_t sat_num;
    uint8_t sat_count;
    uint8_t cur_statement;
    uint32_t all_statements;
    char item_str[NMEA_MAX_STATEMENT_ITEM_LENGTH];
    gps_t parent;
    uart_port_t uart_port;
    uint8_t *buffer;
    esp_event_loop_handle_t event_loop_hdl;
    TaskHandle_t tsk_hdl;
    QueueHandle_t event_queue;
} esp_gps_t;

static float parse_lat_long(esp_gps_t *esp_gps)
{
    float ll = strtof(esp_gps->item_str, NULL);
    int deg = ((int)ll) / 100;
    float min = ll - (deg * 100);
    ll = deg + min / 60.0f;
    return ll;
}

static inline uint8_t convert_two_digit2number(const char *digit_char)
{
    return 10 * (digit_char[0] - '0') + (digit_char[1] - '0');
}

static void parse_utc_time(esp_gps_t *esp_gps)
{
    esp_gps->parent.tim.hour = convert_two_digit2number(esp_gps->item_str + 0);
    esp_gps->parent.tim.minute = convert_two_digit2number(esp_gps->item_str + 2);
    esp_gps->parent.tim.second = convert_two_digit2number(esp_gps->item_str + 4);
    if (esp_gps->item_str[6] == '.') {
        uint16_t tmp = 0;
        uint8_t i = 7;
        while (esp_gps->item_str[i]) {
            tmp = 10 * tmp + esp_gps->item_str[i] - '0';
            i++;
        }
        esp_gps->parent.tim.thousand = tmp;
    }
}

static void parse_gga(esp_gps_t *esp_gps)
{
    switch (esp_gps->item_num) {
    case 1: parse_utc_time(esp_gps); break;
    case 2: esp_gps->parent.latitude = parse_lat_long(esp_gps); break;
    case 3:
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
            esp_gps->parent.latitude *= -1;
        break;
    case 4: esp_gps->parent.longitude = parse_lat_long(esp_gps); break;
    case 5:
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
            esp_gps->parent.longitude *= -1;
        break;
    case 6: esp_gps->parent.fix = (gps_fix_t)strtol(esp_gps->item_str, NULL, 10); break;
    case 7: esp_gps->parent.sats_in_use = (uint8_t)strtol(esp_gps->item_str, NULL, 10); break;
    case 8: esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL); break;
    case 9: esp_gps->parent.altitude = strtof(esp_gps->item_str, NULL); break;
    case 11: esp_gps->parent.altitude += strtof(esp_gps->item_str, NULL); break;
    default: break;
    }
}

static void parse_gsa(esp_gps_t *esp_gps)
{
    switch (esp_gps->item_num) {
    case 2: esp_gps->parent.fix_mode = (gps_fix_mode_t)strtol(esp_gps->item_str, NULL, 10); break;
    case 15: esp_gps->parent.dop_p = strtof(esp_gps->item_str, NULL); break;
    case 16: esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL); break;
    case 17: esp_gps->parent.dop_v = strtof(esp_gps->item_str, NULL); break;
    default:
        if (esp_gps->item_num >= 3 && esp_gps->item_num <= 14)
            esp_gps->parent.sats_id_in_use[esp_gps->item_num - 3] = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    }
}

static void parse_gsv(esp_gps_t *esp_gps)
{
    switch (esp_gps->item_num) {
    case 1: esp_gps->sat_count = (uint8_t)strtol(esp_gps->item_str, NULL, 10); break;
    case 2: esp_gps->sat_num = (uint8_t)strtol(esp_gps->item_str, NULL, 10); break;
    case 3: esp_gps->parent.sats_in_view = (uint8_t)strtol(esp_gps->item_str, NULL, 10); break;
    default:
        if (esp_gps->item_num >= 4 && esp_gps->item_num <= 19) {
            uint8_t item_num = esp_gps->item_num - 4;
            uint8_t index = 4 * (esp_gps->sat_num - 1) + item_num / 4;
            if (index < GPS_MAX_SATELLITES_IN_VIEW) {
                uint32_t value = strtol(esp_gps->item_str, NULL, 10);
                switch (item_num % 4) {
                case 0: esp_gps->parent.sats_desc_in_view[index].num = (uint8_t)value; break;
                case 1: esp_gps->parent.sats_desc_in_view[index].elevation = (uint8_t)value; break;
                case 2: esp_gps->parent.sats_desc_in_view[index].azimuth = (uint16_t)value; break;
                case 3: esp_gps->parent.sats_desc_in_view[index].snr = (uint8_t)value; break;
                default: break;
                }
            }
        }
        break;
    }
}

static void parse_rmc(esp_gps_t *esp_gps)
{
    switch (esp_gps->item_num) {
    case 1: parse_utc_time(esp_gps); break;
    case 2: esp_gps->parent.valid = (esp_gps->item_str[0] == 'A'); break;
    case 3: esp_gps->parent.latitude = parse_lat_long(esp_gps); break;
    case 4:
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
            esp_gps->parent.latitude *= -1;
        break;
    case 5: esp_gps->parent.longitude = parse_lat_long(esp_gps); break;
    case 6:
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
            esp_gps->parent.longitude *= -1;
        break;
    case 7: esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 0.514444; break;
    case 8: esp_gps->parent.cog = strtof(esp_gps->item_str, NULL); break;
    case 9:
        esp_gps->parent.date.day = convert_two_digit2number(esp_gps->item_str + 0);
        esp_gps->parent.date.month = convert_two_digit2number(esp_gps->item_str + 2);
        esp_gps->parent.date.year = convert_two_digit2number(esp_gps->item_str + 4);
        break;
    case 10: esp_gps->parent.variation = strtof(esp_gps->item_str, NULL); break;
    default: break;
    }
}

static void parse_gll(esp_gps_t *esp_gps)
{
    switch (esp_gps->item_num) {
    case 1: esp_gps->parent.latitude = parse_lat_long(esp_gps); break;
    case 2:
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
            esp_gps->parent.latitude *= -1;
        break;
    case 3: esp_gps->parent.longitude = parse_lat_long(esp_gps); break;
    case 4:
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
            esp_gps->parent.longitude *= -1;
        break;
    case 5: parse_utc_time(esp_gps); break;
    case 6: esp_gps->parent.valid = (esp_gps->item_str[0] == 'A'); break;
    default: break;
    }
}

static void parse_vtg(esp_gps_t *esp_gps)
{
    switch (esp_gps->item_num) {
    case 1: esp_gps->parent.cog = strtof(esp_gps->item_str, NULL); break;
    case 3: esp_gps->parent.variation = strtof(esp_gps->item_str, NULL); break;
    case 5: esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 0.514444; break;
    case 7: esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) / 3.6; break;
    default: break;
    }
}

static esp_err_t parse_item(esp_gps_t *esp_gps)
{
    esp_err_t err = ESP_OK;
    if (esp_gps->item_num == 0 && esp_gps->item_str[0] == '$') {
        if (strstr(esp_gps->item_str, "GGA"))
            esp_gps->cur_statement = STATEMENT_GGA;
        else if (strstr(esp_gps->item_str, "GSA"))
            esp_gps->cur_statement = STATEMENT_GSA;
        else if (strstr(esp_gps->item_str, "RMC"))
            esp_gps->cur_statement = STATEMENT_RMC;
        else if (strstr(esp_gps->item_str, "GSV"))
            esp_gps->cur_statement = STATEMENT_GSV;
        else if (strstr(esp_gps->item_str, "GLL"))
            esp_gps->cur_statement = STATEMENT_GLL;
        else if (strstr(esp_gps->item_str, "VTG"))
            esp_gps->cur_statement = STATEMENT_VTG;
        else
            esp_gps->cur_statement = STATEMENT_UNKNOWN;
        goto out;
    }
    if (esp_gps->cur_statement == STATEMENT_UNKNOWN) goto out;
    else if (esp_gps->cur_statement == STATEMENT_GGA) parse_gga(esp_gps);
    else if (esp_gps->cur_statement == STATEMENT_GSA) parse_gsa(esp_gps);
    else if (esp_gps->cur_statement == STATEMENT_GSV) parse_gsv(esp_gps);
    else if (esp_gps->cur_statement == STATEMENT_RMC) parse_rmc(esp_gps);
    else if (esp_gps->cur_statement == STATEMENT_GLL) parse_gll(esp_gps);
    else if (esp_gps->cur_statement == STATEMENT_VTG) parse_vtg(esp_gps);
    else err = ESP_FAIL;
out:
    return err;
}

static esp_err_t gps_decode(esp_gps_t *esp_gps, size_t len)
{
    const uint8_t *d = esp_gps->buffer;
    while (*d) {
        if (*d == '$') {
            esp_gps->asterisk = 0;
            esp_gps->item_num = 0;
            esp_gps->item_pos = 0;
            esp_gps->cur_statement = 0;
            esp_gps->crc = 0;
            esp_gps->sat_count = 0;
            esp_gps->sat_num = 0;
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        } else if (*d == ',') {
            parse_item(esp_gps);
            esp_gps->crc ^= (uint8_t)(*d);
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        } else if (*d == '*') {
            parse_item(esp_gps);
            esp_gps->asterisk = 1;
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        } else if (*d == '\r') {
            uint8_t crc = (uint8_t)strtol(esp_gps->item_str, NULL, 16);
            if (esp_gps->crc == crc) {
                switch (esp_gps->cur_statement) {
                case STATEMENT_GGA: esp_gps->parsed_statement |= 1 << STATEMENT_GGA; break;
                case STATEMENT_GSA: esp_gps->parsed_statement |= 1 << STATEMENT_GSA; break;
                case STATEMENT_RMC: esp_gps->parsed_statement |= 1 << STATEMENT_RMC; break;
                case STATEMENT_GSV:
                    if (esp_gps->sat_num == esp_gps->sat_count)
                        esp_gps->parsed_statement |= 1 << STATEMENT_GSV;
                    break;
                case STATEMENT_GLL: esp_gps->parsed_statement |= 1 << STATEMENT_GLL; break;
                case STATEMENT_VTG: esp_gps->parsed_statement |= 1 << STATEMENT_VTG; break;
                default: break;
                }
                if (((esp_gps->parsed_statement) & esp_gps->all_statements) == esp_gps->all_statements) {
                    esp_gps->parsed_statement = 0;
                    esp_event_post_to(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, GPS_UPDATE,
                                      &(esp_gps->parent), sizeof(gps_t), 100 / portTICK_PERIOD_MS);
                }
            } else {
                ESP_LOGD(GPS_TAG, "CRC Error for statement:%s", esp_gps->buffer);
            }
            if (esp_gps->cur_statement == STATEMENT_UNKNOWN) {
                esp_event_post_to(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, GPS_UNKNOWN,
                                  esp_gps->buffer, len, 100 / portTICK_PERIOD_MS);
            }
        } else {
            if (!(esp_gps->asterisk))
                esp_gps->crc ^= (uint8_t)(*d);
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        d++;
    }
    return ESP_OK;
}

static void esp_handle_uart_pattern(esp_gps_t *esp_gps)
{
    int pos = uart_pattern_pop_pos(esp_gps->uart_port);
    if (pos != -1) {
        int read_len = uart_read_bytes(esp_gps->uart_port, esp_gps->buffer, pos + 1, 100 / portTICK_PERIOD_MS);
        esp_gps->buffer[read_len] = '\0';
        if (gps_decode(esp_gps, read_len + 1) != ESP_OK) {
            ESP_LOGW(GPS_TAG, "GPS decode line failed");
        }
    } else {
        ESP_LOGW(GPS_TAG, "Pattern Queue Size too small");
        uart_flush_input(esp_gps->uart_port);
    }
}

static void nmea_parser_task_entry(void *arg)
{
    esp_gps_t *esp_gps = (esp_gps_t *)arg;
    uart_event_t event;
    while (1) {
        if (xQueueReceive(esp_gps->event_queue, &event, pdMS_TO_TICKS(200))) {
            switch (event.type) {
            case UART_DATA: break;
            case UART_FIFO_OVF:
                ESP_LOGW(GPS_TAG, "HW FIFO Overflow");
                uart_flush(esp_gps->uart_port);
                xQueueReset(esp_gps->event_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(GPS_TAG, "Ring Buffer Full");
                uart_flush(esp_gps->uart_port);
                xQueueReset(esp_gps->event_queue);
                break;
            case UART_BREAK: ESP_LOGW(GPS_TAG, "Rx Break"); break;
            case UART_PARITY_ERR: ESP_LOGE(GPS_TAG, "Parity Error"); break;
            case UART_FRAME_ERR: ESP_LOGE(GPS_TAG, "Frame Error"); break;
            case UART_PATTERN_DET: esp_handle_uart_pattern(esp_gps); break;
            default: ESP_LOGW(GPS_TAG, "unknown uart event type: %d", event.type); break;
            }
        }
        esp_event_loop_run(esp_gps->event_loop_hdl, pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config)
{
    esp_gps_t *esp_gps = calloc(1, sizeof(esp_gps_t));
    if (!esp_gps) {
        ESP_LOGE(GPS_TAG, "calloc memory for esp_gps failed");
        goto err_gps;
    }
    esp_gps->buffer = calloc(1, NMEA_PARSER_RUNTIME_BUFFER_SIZE);
    if (!esp_gps->buffer) {
        ESP_LOGE(GPS_TAG, "calloc memory for runtime buffer failed");
        goto err_buffer;
    }
    esp_gps->all_statements |= (1 << STATEMENT_GSA);
    esp_gps->all_statements |= (1 << STATEMENT_GSV);
    esp_gps->all_statements |= (1 << STATEMENT_GGA);
    esp_gps->all_statements |= (1 << STATEMENT_RMC);
    esp_gps->all_statements |= (1 << STATEMENT_GLL);
    esp_gps->all_statements |= (1 << STATEMENT_VTG);

    esp_gps->uart_port = config->uart.uart_port;
    esp_gps->all_statements &= 0xFE;

    uart_config_t uart_config = {
        .baud_rate = config->uart.baud_rate,
        .data_bits = config->uart.data_bits,
        .parity = config->uart.parity,
        .stop_bits = config->uart.stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(esp_gps->uart_port, NMEA_PARSER_RING_BUFFER_SIZE, 0,
                            config->uart.event_queue_size, &esp_gps->event_queue, 0) != ESP_OK) {
        ESP_LOGE(GPS_TAG, "install uart driver failed");
        goto err_uart_install;
    }
    if (uart_param_config(esp_gps->uart_port, &uart_config) != ESP_OK) {
        ESP_LOGE(GPS_TAG, "config uart parameter failed");
        goto err_uart_config;
    }
    if (uart_set_pin(esp_gps->uart_port, UART_PIN_NO_CHANGE, config->uart.rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(GPS_TAG, "config uart gpio failed");
        goto err_uart_config;
    }
    uart_enable_pattern_det_baud_intr(esp_gps->uart_port, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(esp_gps->uart_port, config->uart.event_queue_size);
    uart_flush(esp_gps->uart_port);

    esp_event_loop_args_t loop_args = {
        .queue_size = NMEA_EVENT_LOOP_QUEUE_SIZE,
        .task_name = NULL
    };
    if (esp_event_loop_create(&loop_args, &esp_gps->event_loop_hdl) != ESP_OK) {
        ESP_LOGE(GPS_TAG, "create event loop failed");
        goto err_eloop;
    }
    BaseType_t err = xTaskCreate(
                         nmea_parser_task_entry,
                         "nmea_parser",
                         NMEA_PARSER_TASK_STACK_SIZE,
                         esp_gps,
                         NMEA_PARSER_TASK_PRIORITY,
                         &esp_gps->tsk_hdl);
    if (err != pdTRUE) {
        ESP_LOGE(GPS_TAG, "create NMEA Parser task failed");
        goto err_task_create;
    }
    ESP_LOGI(GPS_TAG, "NMEA Parser init OK");
    return esp_gps;

err_task_create:
    esp_event_loop_delete(esp_gps->event_loop_hdl);
err_eloop:
err_uart_install:
    uart_driver_delete(esp_gps->uart_port);
err_uart_config:
err_buffer:
    free(esp_gps->buffer);
err_gps:
    free(esp_gps);
    return NULL;
}

esp_err_t nmea_parser_deinit(nmea_parser_handle_t nmea_hdl)
{
    esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
    vTaskDelete(esp_gps->tsk_hdl);
    esp_event_loop_delete(esp_gps->event_loop_hdl);
    esp_err_t err = uart_driver_delete(esp_gps->uart_port);
    free(esp_gps->buffer);
    free(esp_gps);
    return err;
}

esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler, void *handler_args)
{
    esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
    return esp_event_handler_register_with(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

esp_err_t nmea_parser_remove_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler)
{
    esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
    return esp_event_handler_unregister_with(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, ESP_EVENT_ANY_ID, event_handler);
}
