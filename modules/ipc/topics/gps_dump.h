#pragma once

#include <ipccore.h>

struct gps_dump_s {

    uint64_t timestamp;		// time since system start (microseconds)
    
    uint8_t len;			// length of data, MSB bit set = message to the gps device,
				            // clear = message from the device
    uint8_t data[79];		// data to write to the log
};

/* register this as object request broker structure */
IPC_DECLARE(gps_dump);

