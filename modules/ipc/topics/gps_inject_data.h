#pragma once

#include <ipccore.h>

struct gps_inject_data_s {

    uint64_t timestamp;		// time since system start (microseconds)
    
    uint8_t len;			// length of data
    uint8_t flags;         		// LSB: 1=fragmented
    uint8_t data[182];		// data to write to GPS device (RTCM message)
};

/* register this as object request broker structure */
IPC_DECLARE(gps_inject_data);

