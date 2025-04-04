#ifndef WIFI_H
#define WIFI_H

#pragma once

#define MAJOR_WIFI_VER 0
#define MINOR_WIFI_VER 1
#define PATCH_WIFI_VER 0

#define TAG_WIFI "WIFI"

#define ESPNOW_CHANNEL 7

/* Print wifi version */
void print_wifi_version();

/* Init WiFi station */
esp_err_t init_wifi_sta();

void deinit_wifi_sta();

#endif