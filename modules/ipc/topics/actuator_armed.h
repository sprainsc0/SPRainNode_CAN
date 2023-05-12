#pragma once


#include <ipccore.h>


#ifndef __cplusplus

#endif

struct actuator_armed_s {

	uint64_t timestamp; // required for logger
	uint32_t armed_time_ms;
	bool armed;
	bool prearmed;
	bool ready_to_arm;
};

/* register this as object request broker structure */
IPC_DECLARE(actuator_armed);


