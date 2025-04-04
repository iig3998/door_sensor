
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "adc.h"

#define TAG_ADC            "ADC"
#define DEFAULT_VREF       1100
#define NUM_SAMPLES        64
#define VREF_STATE_BATTERY 3.5

/* Read voltage from adc */
static float read_adc_voltage(uint8_t adc, uint8_t adc_channel, float coeff) {

    int32_t adc_raw;
    float v_batt;

    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    if(!adc_chars) {
        ESP_LOGE(TAG_ADC, "Error, memory not allocated for adc");
        return -1;
    }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_12);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    adc_raw = 0;
    for(uint8_t i = 0; i < NUM_SAMPLES; i++) {
        adc_raw += adc1_get_raw(adc);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    adc_raw = adc_raw / NUM_SAMPLES;

    v_batt = esp_adc_cal_raw_to_voltage(adc_raw, adc_chars) / coeff;

    ESP_LOGI(TAG_ADC, "Voltage read: %.2f V", v_batt);

    return v_batt;
}

/* Read voltage battery  */
bool read_status_battery() {

    float v_batt;

    v_batt = read_adc_voltage( ADC_UNIT_1,  ADC1_CHANNEL_7, 500.0);

    if (v_batt > VREF_STATE_BATTERY)
        return true;

    return false;
}

/* Check usb connection */
bool check_usb_connection() {

    float v_batt;

    v_batt = read_adc_voltage(ADC_UNIT_2, ADC2_CHANNEL_0, 0.6875);

    ESP_LOGI(TAG_ADC, "USB voltage: %.2f V", v_batt);

    if (v_batt > VREF_STATE_BATTERY)
        return true;

    return false;
}



