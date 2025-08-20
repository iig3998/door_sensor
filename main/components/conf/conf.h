#ifndef CONF_H
#define CONF_H

#include <stdio.h>
#include <stdint.h>

#define UNREGISTRATION_DOOR_SENSOR 1
#define REGISTRATION_DOOR_SENSOR   2

void init_conf();

int8_t set_device_id(uint8_t device_id);

uint8_t get_device_id();

int8_t del_device_id();

int8_t set_device_name(char *device_name);

char *get_device_name();

int8_t del_device_name();

int8_t set_status_registration(uint8_t registration);

int8_t get_status_registration();

int8_t del_status_registration();

#endif