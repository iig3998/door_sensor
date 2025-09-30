#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs_mgmt.h"

#include "conf.h"

#define DEVICE_NAME_SIZE 15

#define TAG_CLI "CLI"

/* Init conf */
void init_conf() {
    
    init_nvs();

    return;
}

/* Set device name */
int8_t set_device_name(char *device_name) {

    esp_err_t err = ESP_FAIL;

    if (!device_name)
        return -1;

    err = save_string_to_nvs("storage", "device_name", device_name);
    if (err != ESP_OK) {
        return -1;
    }

    return 0;
}

/* Get device name */
char *get_device_name() {

    esp_err_t err = ESP_FAIL;

    static char device_name[DEVICE_NAME_SIZE] = {'\0'};

    memset(device_name, '\0', sizeof(device_name));
    err = read_string_from_nvs("storage", "device_name", device_name, DEVICE_NAME_SIZE);
    if (err != ESP_OK) {
        return NULL;
    }

    return device_name;
}

/* Del device name */
int8_t del_device_name() {

    esp_err_t err = ESP_FAIL;
    err = delete_key_from_nvs("storage", "device_name");
    if (err != ESP_OK) {
        return -1;
    }

    return 0;
}

/* Set device id */
int8_t set_device_id(uint8_t device_id) {

    esp_err_t err = ESP_FAIL;
    
    err = save_uint8_to_nvs("storage", "device_id", device_id);
    if (err != ESP_OK) {
        return -1;
    }

    return 0;
}

/* Get device id */
uint8_t get_device_id() {

    esp_err_t err = ESP_FAIL;
    uint8_t device_id;

    err = read_uint8_from_nvs("storage", "device_id", &device_id);
    if (err != ESP_OK) {
        return 0;
    }

    return device_id;
}

/* Del device id */
int8_t del_device_id() {

    esp_err_t err = ESP_FAIL;
    err = delete_key_from_nvs("storage", "device_id");
    if (err != ESP_OK) {
        return -1;
    }

    return 0;

}

/* Set status registartion */
int8_t set_status_registration(uint8_t registration) {

    esp_err_t err = ESP_FAIL;
    
    err = save_uint8_to_nvs("storage", "registration", registration);
    if (err != ESP_OK) {
        ESP_LOGD(TAG_CLI, "Error, impossible save registration value");
        return -1;
    }

    return 0;
}

/* Get status register */
uint8_t get_status_registration() {

    esp_err_t err = ESP_FAIL;
    uint8_t registration;

    err = read_uint8_from_nvs("storage", "registration", &registration);
    if (err != ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG_CLI, "Error, impossible read registration value");
        set_status_registration(UNREGISTRATION_DOOR_SENSOR);
        return UNREGISTRATION_DOOR_SENSOR;
    }

    return registration;
}

/* Del status registration */
int8_t del_status_registration() {

    esp_err_t err = ESP_FAIL;
    err = delete_key_from_nvs("storage", "registration");
    if (err != ESP_OK) {
        return -1;
    }

    return 0;
}