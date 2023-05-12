#pragma once

#include "ipccore.h"

#define PROTOCOL_PARAM_VALUE_EMPTY           0
#define PROTOCOL_PARAM_VALUE_INTEGER_VALUE   1
#define PROTOCOL_PARAM_VALUE_REAL_VALUE      2
#define PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE   3
#define PROTOCOL_PARAM_VALUE_STRING_VALUE    4

struct uavcan_parameter_value_s {
	uint64_t timestamp; // required for logger
	int32_t int_value;
	float real_value;
	int16_t param_index;
	uint8_t node_id;
	char param_id[17];
	uint8_t param_type;
	int     param_count;
};

/* register this as object request broker structure */
IPC_DECLARE(uavcan_parameter_value);

