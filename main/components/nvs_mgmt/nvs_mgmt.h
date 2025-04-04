#ifndef NVS_MGMT_H
#define NVS_MGMT_H

#pragma once

#include "esp_err.h"

#define MAJOR_NVS_MGMT_VER 0
#define MINOR_NVS_MGMT_VER 1
#define PATCH_NVS_MGMT_VER 0

void print_nvs_mgmt_version();

esp_err_t init_nvs();

esp_err_t save_uint32_to_nvs(const char *namespace, const char *key, uint32_t value);

esp_err_t save_uint16_to_nvs(const char *namespace, const char *key, uint16_t value);

esp_err_t save_uint8_to_nvs(const char *namespace, const char *key, uint8_t value);

esp_err_t save_string_to_nvs(const char *namespace, const char *key, char* str);

esp_err_t read_uint32_from_nvs(const char *namespace, const char *key, uint32_t *value);

esp_err_t read_uint16_from_nvs(const char *namespace, const char *key, uint16_t *value);

esp_err_t read_uint8_from_nvs(const char *namespace, const char *key, uint8_t *value);

esp_err_t read_string_from_nvs(const char *namespace, const char *key, char *str, size_t len);

esp_err_t delete_key_from_nvs(const char *namespace, const char *key);

#endif