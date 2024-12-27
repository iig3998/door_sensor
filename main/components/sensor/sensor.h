#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define SIZE_CODE 6

enum node_type {
    ALARM_SENSOR = 0,
    SIREN_SENSOR,
	KEYBOARD
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
	enum node_type node;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_cmd;

/* Struct alarm sensor */
typedef struct {
	uint8_t id;
	enum node_type node;
	bool state;
	bool battery_low_detect;
	char code[SIZE_CODE];
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_alarm;

node_id_cmd build_command(uint8_t id, enum node_type st, enum cmd_type ct, int64_t time);

node_id_alarm set_alarm_sensor(uint8_t id, bool state, bool battery_low_detect, int64_t time);

#endif
