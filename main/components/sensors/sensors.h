#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

enum node_type {
    ALARM_SENSOR = 0,
    SIREN,
	KEYBOARD,
	GATEWAY
};

enum cmd_type {
	ADD = 0,
	DEL,
	UPDATE,
	ACTIVE,
	DEACTIVE
};

/* Strcu header message */
typedef struct {
	enum node_type node;
	uint8_t id;
	uint8_t mac[6];
	enum cmd_type cmd;
} __attribute__((__packed__)) node_id_header;

/* Struct alarm sensor message */
typedef struct {
	node_id_header header;
	bool state;
	bool battery_low_detect;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_alarm;

/* Struct gateway message */
typedef struct {
	node_id_header header;
	bool ack;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_gateway;

/* Struct response message */
typedef struct {
	node_id_header header;
	bool ack;
	int64_t time;
	uint16_t crc;
} __attribute__((__packed__)) node_id_response;

node_id_alarm alarm_sensor_update_pkt(uint8_t id, bool state, bool battery_low_detect, int64_t time);

node_id_alarm alarm_sensor_add_pkt(uint8_t id, int64_t time);

#endif
