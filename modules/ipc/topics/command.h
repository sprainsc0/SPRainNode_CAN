#pragma once


#include "ipccore.h"

struct command_s {
	uint64_t timestamp;
    uint16_t command;
	uint16_t sub_cmd;
	float param1;
	float param2;
	float param3;
	float param4;
	float param5;
	bool from_external;
};

/* register this as object request broker structure */
IPC_DECLARE(command);

