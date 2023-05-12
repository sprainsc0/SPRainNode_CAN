#pragma once


#include <ipccore.h>

#ifndef __cplusplus

#endif


struct sensor_airspeed_s {
	uint64_t timestamp;
	uint32_t lag_ms;
	bool healthy;
    uint8_t calibrated;
	float pressure;
	float raw_airspeed;
	float covariance;
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_airspeed);

