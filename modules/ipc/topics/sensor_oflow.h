#pragma once


#include <ipccore.h>


#ifndef __cplusplus

#endif


struct sensor_oflow_s {
	uint64_t timestamp; // required for logger
	uint32_t lag_ms;
	float rawFlowRates[2];
	float rawGyroRates[3];
	uint8_t quality;
	float posOffset[3];
	float covariance;
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_oflow);

