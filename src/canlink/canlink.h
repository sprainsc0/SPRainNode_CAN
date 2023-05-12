#ifndef __CANLINK_H__
#define __CANLINK_H__

#include <stdint.h>
#include "cmsis_os.h"
#include <ipccore.h>
#include <ipcpush.h>
#include <ipcpull.h>
#include "param.h"
#include "uPerf.h"
#include "can.h"
#include "canlink_stream.h"
#include "canlink_parameter.h"

#include <topics/parameter_update.h>
#include <topics/actuator_notify.h>
#include <topics/uavcan_parameter_request.h>
#include <topics/uavcan_parameter_value.h>

class CanLink
{
public:
    CanLink(void);

    void send(void);
    void recv(void);

    bool init(void);

    uint64_t fcu_vailed;
    uint64_t param_send_ts;

    bool connected(void) const { return connect; }

    CanardInstance *canIns(void) { return &canard; }

protected:
    osThreadId_t _send_handle;
    osThreadId_t _recv_handle;
private:
    CanardInstance canard;

    bool connect;
    bool per_connect;

    CanardSTM32AcceptanceFilterConfiguration filter[14];

    CanStream *_can_streams;
    CanStream *_sub_streams;
    CanParam  *_can_param;

    IPCPush _led_pub;

    int interval_from_rate(float rate);
    int configure_link(const char *stream_name, const float rate);
    uint32_t configure_subscription(const char *stream_name, const float rate);
    void processReceivedTransfer(uint64_t ts, uint8_t index, CanardTransfer *transfer);
    
    perf_counter_t pref_can_interval;
    perf_counter_t pref_can_elapsed;
};

#endif
