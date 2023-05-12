#pragma once


#include <ipccore.h>

struct battery_status_s {
	uint64_t timestamp;
	float voltage_v;
	float current_a;
	uint16_t remaining;
	uint16_t raltive_soc;
	uint16_t capacity;
    uint8_t cell_count;
};

/* register this as object request broker structure */
IPC_DECLARE(battery_status);

