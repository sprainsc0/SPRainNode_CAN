#pragma once

#include <ipccore.h>

#ifndef __cplusplus

#endif

struct sensor_imu_s {
	uint64_t timestamp;
	float gyro[3];
    float gyro_filter[3];
    float gyro_raw[3];
    float accel[3];
    float accel_filter[3];
    float accel_raw[3];
    
	float temperature;
    uint8_t instance;
    bool calibrated;
    bool healthy;
#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_imu);

