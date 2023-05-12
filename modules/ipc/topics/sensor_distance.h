#pragma once


#include <ipccore.h>

#define MAV_DISTANCE_SENSOR_LASER 0
#define MAV_DISTANCE_SENSOR_ULTRASOUND 1
#define MAV_DISTANCE_SENSOR_INFRARED 2
#define MAV_DISTANCE_SENSOR_RADAR 3
#define ROTATION_DOWNWARD_FACING 25
#define ROTATION_UPWARD_FACING 24
#define ROTATION_BACKWARD_FACING 12
#define ROTATION_FORWARD_FACING 0
#define ROTATION_LEFT_FACING 6
#define ROTATION_RIGHT_FACING 2


struct sensor_distance_s {
	uint64_t timestamp; // required for logger
	uint32_t lag_ms;
	float min_distance;
	float max_distance;
	float current_distance;
	uint16_t quality;
	float temp;
	float posOffset[3];
	float covariance;
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_distance);

