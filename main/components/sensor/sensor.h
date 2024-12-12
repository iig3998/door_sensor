#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

enum sensor_type {
    TEMPERATURE_HUMIDITY_SENSOR = 0,
    ALARM_SENSOR
};

enum cmd_type {
	GET = 0,
	SET,
	DEL
};

/* Struct command sensor */
typedef struct {
	uint8_t id;
	enum cmd_type cmd;
	enum sensor_type sensor;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_cmd;

/* Struct alarm sensor */
typedef struct {
	uint8_t id;
	enum sensor_type sensor;
	struct {
		bool state;
		bool battery_low_detect;
	} alarm_sensor;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_alarm;

/* Struct metric sensor */
typedef struct {
	uint8_t id;
	enum sensor_type sensor;
	struct {
		float temp;
		uint8_t hum;
	} metric_sensor;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_weather_metric;

node_id_cmd build_command(uint8_t id, enum sensor_type st, enum cmd_type ct, int64_t time);

node_id_alarm set_alarm_sensor(uint8_t id, bool state, int64_t time);

node_id_weather_metric set_metric_sensor(uint8_t id, float temp, uint8_t hum, int64_t time);

#endif
