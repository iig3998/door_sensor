#ifndef WIFI_H
#define WIFI_H

#pragma once

#include "esp_err.h"

#define MAJOR_WIFI_VER 0
#define MINOR_WIFI_VER 1
#define PATCH_WIFI_VER 0

/* Print wifi version */
void print_wifi_version();

/* Init WiFi station */
esp_err_t init_wifi_sta(uint8_t wifi_channel);

/* De init wifi station */
esp_err_t deinit_wifi_sta();

#endif