#pragma once

#include <ipccore.h>

#define SAT_INFO_MAX_SATELLITES   20u

struct satellite_info_s {

    uint64_t timestamp;		// time since system start (microseconds)
    
    uint8_t count;			// Number of satellites visible to the receiver
    uint8_t svid[20];	 		// Space vehicle ID [1..255], see scheme below
    uint8_t used[20];			// 0: Satellite not used, 1: used for navigation
    uint8_t elevation[20];		// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
    uint8_t azimuth[20];		// Direction of satellite, 0: 0 deg, 255: 360 deg.
    uint8_t snr[20];			// dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite.
    uint8_t prn[20];                   // Satellite PRN code assignment, (psuedorandom number SBAS, valid codes are 120-144)
};

/* register this as object request broker structure */
IPC_DECLARE(satellite_info);

