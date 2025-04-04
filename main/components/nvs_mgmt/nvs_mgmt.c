#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#include "nvs_mgmt.h"

#define TAG_NVS_MGMT "NVS_MGMT"

/* Print version library */
void print_nvs_mgmt_version() {

    ESP_LOGI(TAG_NVS_MGMT, "Sensor version: %u.%u.%u", MAJOR_NVS_MGMT_VER, MINOR_NVS_MGMT_VER, PATCH_NVS_MGMT_VER);

    return;
}

/* Init nvs flash */
esp_err_t init_nvs() {

    esp_err_t err = ESP_FAIL;
    nvs_handle handle;
    
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Create storage section inside nvs */
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_NVS_MGMT, "Nvs storage not initied");
    }

    nvs_close(handle);

    return err;
}

/* Save uint32 variable */
esp_err_t save_uint32_to_nvs(const char *namespace, const char *key, uint32_t value) {

    esp_err_t err = ESP_FAIL;    
    nvs_handle_t handle = 0;
    
    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs memory not open");
        return err;
    }

    err = nvs_set_u32(handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not saved successfully", key);
        nvs_close(handle);
        return err;
    }

    nvs_commit(handle);
    ESP_LOGI(TAG_NVS_MGMT, "Varible %s saved successfully", key);

    nvs_close(handle);

    return err;
}

/* Save uint16 variable */
esp_err_t save_uint16_to_nvs(const char *namespace, const char *key, uint16_t value) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle = 0;
    
    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs memory not open");
        return err;
    }

    err = nvs_set_u16(handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not saved successfully", key);
        nvs_close(handle);
        return err;
    }

    nvs_commit(handle);
    ESP_LOGI(TAG_NVS_MGMT, "Varible %s saved successfully", key);

    nvs_close(handle);

    return err;
}

/* Save uint8 variable */
esp_err_t save_uint8_to_nvs(const char *namespace, const char *key, uint8_t value) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle = 0;
    
    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs memory not open");
        return err;
    }

    err = nvs_set_u8(handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not saved successfully", key);
        nvs_close(handle);
        return err;
    }

    nvs_commit(handle);
    ESP_LOGI(TAG_NVS_MGMT, "Varible %s saved successfully", key);

    nvs_close(handle);

    return err;
}

/* Save string */
esp_err_t save_string_to_nvs(const char *namespace, const char *key, char* str) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle = 0;
    
    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs memory not open");
        return err;
    }

    err = nvs_set_str(handle, key, str);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, string %s not saved successfully", key);
        nvs_close(handle);
        return err;
    }

    nvs_commit(handle);
    ESP_LOGI(TAG_NVS_MGMT, "Varible %s saved successfully", key);

    nvs_close(handle);

    return err;
}

/* Read uint32 variable */
esp_err_t read_uint32_from_nvs(const char *namespace, const char *key, uint32_t *value) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle = 0;

    err = nvs_open(namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs not opened");
        return err;
    }

    err = nvs_get_u32(handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not read successfully", key);
    }

    nvs_close(handle);

    return err;
}

/* Read uint16 variable */
esp_err_t read_uint16_from_nvs(const char *namespace, const char *key, uint16_t *value) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle;

    err = nvs_open(namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs not opened");
        return err;
    }

    err = nvs_get_u16(handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not read successfully", key);
    }

    nvs_close(handle);

    return err;
}

/* Read uint8 variable */
esp_err_t read_uint8_from_nvs(const char *namespace, const char *key, uint8_t *value) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle;

    err = nvs_open(namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs not opened");
        return err;
    }

    err = nvs_get_u8(handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not read successfully", key);
    }

    nvs_close(handle);

    return err;
}

/* Read string variable */
esp_err_t read_string_from_nvs(const char *namespace, const char *key, char *str, size_t len) {

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle;

    if (!str) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, destination string in null");
        return err;
    }

    err = nvs_open(namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs not opened");
        return err;
    }

    err = nvs_get_str(handle, key, str, &len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, string %s not read successfully", key);
    }

    nvs_close(handle);

    return err;
}

/* Delete variable from nvs */
esp_err_t delete_key_from_nvs(const char *namespace, const char *key) {

    esp_err_t err = ESP_FAIL;    
    nvs_handle_t handle;

    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, nvs not opened");
        return err;
    }

    err = nvs_erase_key(handle, key);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS_MGMT, "Error, variable %s not deleted successfully", key);
        nvs_close(handle);
        return err;
    }

    nvs_commit(handle);

    nvs_close(handle);

    return err;
}