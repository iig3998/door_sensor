
#include <string.h>

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

/* Read voltage from adc1 channel */
static float read_adc_voltage(adc1_channel_t adc_channel) {

    uint32_t voltage_mv = 0;
    uint32_t one_voltage_read = 0;

    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_12);

    esp_adc_cal_characteristics_t adc_cal_characteristics;
    memset(&adc_cal_characteristics, 0, sizeof(adc_cal_characteristics));

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_10, DEFAULT_VREF, &adc_cal_characteristics);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        esp_adc_cal_get_voltage(adc_channel, &adc_cal_characteristics, &one_voltage_read);
        voltage_mv += one_voltage_read;
    }
    voltage_mv = (voltage_mv / NUM_SAMPLES);

    ESP_LOGI(TAG_ADC, "Voltage: %ld", voltage_mv);

    return voltage_mv;
}

/* Read voltage battery  */
bool check_status_battery() {

    float voltage = 0;

    voltage = read_adc_voltage(ADC_CHANNEL_7) / 1000.0 * 2;

    if (voltage <= VREF_STATE_BATTERY)
        return false;

    return true;
}

/* Check usb connection */
bool check_usb_connection() {

    float voltage = 0;

    voltage = read_adc_voltage(ADC_CHANNEL_4) / 1000.0;

    if (voltage <= 3.0)
        return false;

    return true;
}



