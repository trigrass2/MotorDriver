#pragma once

#include <stdint.h>

typedef struct _CAN_MSG_ {
	uint32_t id;
	uint8_t len;
	uint8_t data[8];
} CAN_MSG;