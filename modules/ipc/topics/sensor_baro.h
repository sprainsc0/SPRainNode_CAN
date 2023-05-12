
#pragma once

#include <ipccore.h>

struct sensor_baro_s {
	uint64_t timestamp;
	uint32_t lag_ms;
    bool healthy;
    uint8_t calibrated;
	float pressure;
	float altitude;
	float temperature;
    float baro_climb_rate;
	float EAS2TAS;
	float covariance;

#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_baro);

