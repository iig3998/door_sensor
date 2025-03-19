#ifndef LED_H
#define LED_H

#pragma once

#include "driver/rmt_tx.h"

#define NVS_MGMT_MAJOR 0
#define NVS_MGMT_MINOR 1
#define NVS_MGMT_PATCH 0
#define GREEN          (uint8_t []){0xFF, 0x00, 0x00}
#define RED            (uint8_t []){0x00, 0xFF, 0x00}
#define BLUE           (uint8_t []){0x00, 0x00, 0xFF}
#define WHITE          (uint8_t []){0xFF, 0xFF, 0xFF}

esp_err_t init_led_driver();

void set_color_led(uint8_t color[]);

void power_off_led();

void deinit_led_driver();

#endif