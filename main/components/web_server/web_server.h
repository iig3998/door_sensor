#ifndef WEB_SERVER
#define WEB_SERVER

#pragma once

#include "esp_http_server.h"

#define WEB_SERVER_MAJOR 1
#define WEB_SERVER_MINOR 0
#define WEB_SERVER_PATCH 0

void get_device_name(char *device_name, uint8_t len);

uint8_t get_device_id();

bool get_conf();

void wifi_init_softap();

esp_err_t start_webserver(httpd_handle_t server);

void stop_webserver(httpd_handle_t server);

#endif
