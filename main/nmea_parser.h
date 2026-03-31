/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/uart.h"

#define GPS_MAX_SATELLITES_IN_USE (12)
#define GPS_MAX_SATELLITES_IN_VIEW (16)

ESP_EVENT_DECLARE_BASE(ESP_NMEA_EVENT);

typedef enum {
    GPS_FIX_INVALID,
    GPS_FIX_GPS,
    GPS_FIX_DGPS,
} gps_fix_t;

typedef enum {
    GPS_MODE_INVALID = 1,
    GPS_MODE_2D,
    GPS_MODE_3D
} gps_fix_mode_t;

typedef struct {
    uint8_t num;
    uint8_t elevation;
    uint16_t azimuth;
    uint8_t snr;
} gps_satellite_t;

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t thousand;
} gps_time_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
} gps_date_t;

typedef enum {
    STATEMENT_UNKNOWN = 0,
    STATEMENT_GGA,
    STATEMENT_GSA,
    STATEMENT_RMC,
    STATEMENT_GSV,
    STATEMENT_GLL,
    STATEMENT_VTG
} nmea_statement_t;

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    gps_fix_t fix;
    uint8_t sats_in_use;
    gps_time_t tim;
    gps_fix_mode_t fix_mode;
    uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];
    float dop_h;
    float dop_p;
    float dop_v;
    uint8_t sats_in_view;
    gps_satellite_t sats_desc_in_view[GPS_MAX_SATELLITES_IN_VIEW];
    gps_date_t date;
    bool valid;
    float speed;
    float cog;
    float variation;
} gps_t;

typedef struct {
    struct {
        uart_port_t uart_port;
        uint32_t rx_pin;
        uint32_t baud_rate;
        uart_word_length_t data_bits;
        uart_parity_t parity;
        uart_stop_bits_t stop_bits;
        uint32_t event_queue_size;
    } uart;
} nmea_parser_config_t;

typedef void *nmea_parser_handle_t;

typedef enum {
    GPS_UPDATE,
    GPS_UNKNOWN
} nmea_event_id_t;

nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config);
esp_err_t nmea_parser_deinit(nmea_parser_handle_t nmea_hdl);
esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler, void *handler_args);
esp_err_t nmea_parser_remove_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif
