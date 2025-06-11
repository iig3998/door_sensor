#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define MAJOR_COMMON_VER 0
#define MINOR_COMMON_VER 1
#define PATCH_COMMON_VER 0

uint16_t calc_crc16_msg(uint8_t *data, uint32_t length);

#endif