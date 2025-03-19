#ifndef WEB_SERVER
#define WEB_SERVER

#pragma once

#include "esp_http_server.h"

#define WEB_SERVER_MAJOR 0
#define WEB_SERVER_MINOR 1
#define WEB_SERVER_PATCH 0

void get_device_name(char *device_name, uint8_t len);

uint8_t get_device_id();

uint8_t is_device_configured();

esp_err_t init_webserver();

esp_err_t start_webserver();

esp_err_t stop_webserver();

#endif
