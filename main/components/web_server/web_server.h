#ifndef WEB_SERVER
#define WEB_SERVER

#pragma once

#include "esp_http_server.h"

#define WEB_SERVER_MAJOR 1
#define WEB_SERVER_MINOR 1
#define WEB_SERVER_PATCH 0

#define WIFI_SSID            "DOOR_SENSOR"
#define TAG_WEBSERVER        "WEB_SERVER"
#define DEVICE_NUMBER_SIZE   4
#define MAX_STA_CONN         1
#define DEVICE_NAME_SIZE     16
#define BUFFER_PARAMETR_SIZE 32

void get_device_name(char *device_name, uint8_t len);

uint8_t get_device_id();

uint8_t get_status_registration();

void set_status_registration(uint8_t registration);

esp_err_t wifi_init_softap();

esp_err_t start_webserver(httpd_handle_t server);

__attribute__((unused)) void stop_webserver(httpd_handle_t server);

#endif