#include <string.h>

#include "esp_log.h"

#include "sensor.h"

#define TAG_SENSOR "SENSOR"

/* Calculate crc16 */
static uint16_t calc_crc16(uint8_t *data, uint32_t length) {

    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++) {

        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0; bit < 8; bit++)  {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8005;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* Build command to send sensor */
node_id_cmd build_command(uint8_t id, enum node_type st, enum cmd_type ct, int64_t time) {

    node_id_cmd pkt;

    memset(&pkt, 0, sizeof(pkt));
    pkt.id = id;

    pkt.node = st;
    pkt.cmd = ct;
    pkt.time = time;
    pkt.crc = calc_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

    return pkt;
}

/* Alarm sensor packet response */
node_id_alarm set_alarm_sensor(uint8_t id, bool state, bool battery_low_detect, int64_t time) {

    node_id_alarm pkt;

    memset(&pkt, 0, sizeof(pkt));
    pkt.id = id;

	pkt.node = ALARM_SENSOR;

    pkt.state = state;
    pkt.battery_low_detect = battery_low_detect;
    memset(pkt.code, '\0', sizeof(pkt.code));
	pkt.time = time;
	pkt.crc = calc_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

    ESP_LOGI(TAG_SENSOR, "ID: %u", pkt.id);
    ESP_LOGI(TAG_SENSOR, "Node type: %u", pkt.node);
    ESP_LOGI(TAG_SENSOR, "Sensor state: %u", pkt.state);
    ESP_LOGI(TAG_SENSOR, "Battery low detect: %u", pkt.battery_low_detect);
    ESP_LOGI(TAG_SENSOR, "Code: %s", pkt.code);
    ESP_LOGI(TAG_SENSOR, "Time: %lld", pkt.time);
    ESP_LOGI(TAG_SENSOR, "CRC16: %u", pkt.crc);

    return pkt;
}
