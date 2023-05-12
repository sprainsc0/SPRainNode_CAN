#pragma once


#include <ipccore.h>

#ifndef __cplusplus

#endif


struct sensor_vio_s {
	uint64_t timestamp; // required for logger
    float delTime;
	uint32_t lag_ms;
	float velVariance[3];
	float posVariance[3];
	float attVariance[3];
	float angVariance[3];

	float position[3];
	float velocity[3];
	float quat[4];
	float delAng[3];
	float posOffset[3];

};

/* register this as object request broker structure */
IPC_DECLARE(sensor_vio);

