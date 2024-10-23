#include <stdint.h>
#include <string.h>
#include "sensor.h"

/* Set alarm sensor structure */
node_id set_alarm_sensor(uint8_t id, bool state) {

    node_id n;

    memset(&n, 0, sizeof(n));

    n.id = id;
    n.type = ALARM_SENSOR;
    n.select.as.state = state;

    return n;
}

/* Set metric sensor structure */
node_id set_metric_sensor(uint8_t id, float temp, uint8_t hum) {

    node_id n;

    memset(&n, 0, sizeof(n));

    n.id = id;
    n.type = TEMPERATURE_HUMIDITY_SENSOR;
    n.select.ms.temp = temp;
    n.select.ms.hum = hum;

    return n;
}