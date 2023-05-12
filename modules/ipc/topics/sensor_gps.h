#pragma once


#include <ipccore.h>

#define GPS_FIX_TYPE_NO_GPS    0u   ///< No GPS connected/detected
#define GPS_FIX_TYPE_NO_FIX    1u   ///< Receiving valid GPS messages but no lock
#define GPS_FIX_TYPE_2D_FIX    2u   ///< Receiving valid messages and 2D lock
#define GPS_FIX_TYPE_3D_FIX    3u   ///< Receiving valid messages and 3D lock
#define GPS_FIX_TYPE_DGPS      4u   ///< Receiving valid messages and 3D lock with differential improvements
#define GPS_FIX_TYPE_RTK_FLOAT 5u   ///< Receiving valid messages and 3D RTK Float
#define GPS_FIX_TYPE_RTK_FIXED 6u   ///< Receiving valid messages and 3D RTK Fixed

struct sensor_gps_s {

    // GPS position in WGS84 coordinates.
    // the field 'timestamp' is for the position & velocity (microseconds)
    uint64_t timestamp;		// time since system start (microseconds)
    uint8_t instance;
    uint32_t last_message_time_ms;

    int32_t lat;			// Latitude in 1E-7 degrees
    int32_t lon;			// Longitude in 1E-7 degrees
    int32_t alt;			// Altitude in 1E-3 meters above MSL, (millimetres)
    int32_t alt_ellipsoid; 		// Altitude in 1E-3 meters bove Ellipsoid, (millimetres)

    float s_variance_m_s;		// GPS speed accuracy estimate, (metres/sec)
    float c_variance_rad;		// GPS course accuracy estimate, (radians)
    uint8_t fix_type; // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

    float eph;			// GPS horizontal position accuracy (metres)
    float epv;			// GPS vertical position accuracy (metres)

    float hdop;			// Horizontal dilution of precision
    float vdop;			// Vertical dilution of precision

    int32_t noise_per_ms;		// GPS noise per millisecond
    int32_t jamming_indicator;		// indicates jamming is occurring

    float vel_m_s;			// GPS ground speed, (metres/sec)
    float vel_n_m_s;		// GPS North velocity, (metres/sec)
    float vel_e_m_s;		// GPS East velocity, (metres/sec)
    float vel_d_m_s; 	// GPS Down velocity, (metres/sec)
    float cog_rad;			// Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
    bool vel_ned_valid;		// True if NED velocity is valid

    int32_t timestamp_time_relative;	// timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
    uint64_t time_utc_usec;		// Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

    uint8_t satellites_used;		// Number of satellites used

    float heading;			// heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
    float heading_offset;		// heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])

    float posOffset[3];
    uint32_t gps_lag_ms;
    bool healthy;
};

/* register this as object request broker structure */
IPC_DECLARE(sensor_gps);

