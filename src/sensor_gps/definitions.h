#pragma once

#include <hrt_timer.h>
#include "debug.h"
#include "cmsis_os.h"
#include <time64.h>
#include <datatypes.h>

#include <topics/satellite_info.h>
#include <topics/sensor_gps.h>

#define GPS_INFO(...) Info_Debug(__VA_ARGS__)
#define GPS_WARN(...) Info_Debug(__VA_ARGS__)
#define GPS_ERR(...)  Info_Debug(__VA_ARGS__)

#define M_DEG_TO_RAD_F	    0.0174532925f
#define M_RAD_TO_DEG_F	    57.2957795f
#define M_DEG_TO_RAD 		0.017453292519943295
#define M_RAD_TO_DEG 		57.295779513082323


#define gps_usleep osDelay

/**
 * Get the current time in us. Function signature:
 * uint64_t hrt_absolute_time()
 */
#define gps_absolute_time micros_sync
typedef uint64_t gps_abstime;

