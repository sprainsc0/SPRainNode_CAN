#pragma once


#include <ipccore.h>


#ifndef __cplusplus

#endif


struct mavlink_log_s {
    uint64_t timestamp;
    uint8_t text[50];
    uint8_t severity; // log level (same as in the linux kernel, starting with 0)
#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
IPC_DECLARE(mavlink_log);

