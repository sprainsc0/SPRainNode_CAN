
#pragma once


#include <ipccore.h>


#ifndef __cplusplus

#endif


struct sensor_mag_s {
	uint64_t timestamp; // required for logger
	uint32_t lag_ms;
	bool healthy;
	float field_raw[3];
	float field[3];
    float offsets[3];
	float temperature;
	uint8_t primary;
	bool is_external;
	bool learn_offsets;
	float covariance;

#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_mag);

