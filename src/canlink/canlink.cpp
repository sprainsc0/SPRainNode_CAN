#include "canlink.h"
#include <string>
#include "hrt_timer.h"
#include "buildin_function.h"
#include "debug.h"
#include <utlist.h>

#include "canlink_messages.h"
#include "canlink_subscription.h"

const osThreadAttr_t cans_attributes = {
    .name = "cans",
    .priority = (osPriority_t)osPriorityRealtime1,
    .stack_size = 1024};

const osThreadAttr_t canr_attributes = {
    .name = "canr",
    .priority = (osPriority_t)osPriorityAboveNormal7,
    .stack_size = 1024};

static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    void *vailed = pvPortMalloc(amount);
    if(vailed == nullptr) {
        Info_Debug("No enought memory\n");
    }
    return vailed;
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    vPortFree(pointer);
}

namespace CAN
{
static CanLink	*gCan;
}

static void cans_func(CanLink *pThis)
{
    pThis->send();
}

static void canr_func(CanLink *pThis)
{
    pThis->recv();
}

CanLink::CanLink(void):
    _led_pub(IPC_ID(actuator_notify)),
    fcu_vailed(0),
    param_send_ts(0),
    connect(false),
    per_connect(false)
{
    
}

bool CanLink::init(void)
{
    can_init();

    canard = canardInit(&memAllocate, &memFree);
#ifdef UAVCAN_NODE_PMU
    canard.node_id   = CANLINK_ID_PMU;
#endif
#ifdef UAVCAN_NODE_GPS1
    canard.node_id   = CANLINK_ID_GPS1;
#endif
#ifdef UAVCAN_NODE_GPS2
    canard.node_id   = CANLINK_ID_GPS2;
#endif
#ifdef UAVCAN_NODE_ROS
    canard.node_id   = CANLINK_ID_ROS;
#endif
    canard.mtu_bytes = CANARD_MTU_CAN_FD;

    _can_param = new CanParam(this);

    pref_can_interval = perf_alloc(PC_INTERVAL, "can_int");
    pref_can_elapsed = perf_alloc(PC_ELAPSED,   "can_ela");

    _send_handle = osThreadNew((osThreadFunc_t)cans_func, this, &cans_attributes);
    _recv_handle = osThreadNew((osThreadFunc_t)canr_func, this, &canr_attributes);

    if ((_send_handle == nullptr) || (_recv_handle == nullptr)) {
        return false;
    }

	return true;
}

int CanLink::interval_from_rate(float rate)
{
	if (rate > 0.000001f) {
		return (int)(1000000.0f / rate);

	} else if (rate < 0.0f) {
		return -1;

	} else {
		return 0;
	}
}

int CanLink::configure_link(const char *stream_name, const float rate)
{
	/* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = interval_from_rate(rate);

	/* search if stream exists */
	CanStream *stream;
	LL_FOREACH(_can_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				LL_DELETE(_can_streams, stream);
				delete stream;
			}

			return 1;
		}
	}

	if (interval == 0) {
		/* stream was not active and is requested to be disabled, do nothing */
		return 1;
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	stream = create_message_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
		LL_APPEND(_can_streams, stream);

		return 1;
	}

	return 0;
}

uint32_t CanLink::configure_subscription(const char *stream_name, const float rate)
{
    /* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = interval_from_rate(rate);
    uint32_t sub_id = 0;

	/* search if stream exists */
	CanStream *stream;
	LL_FOREACH(_sub_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);
                sub_id = stream->get_id();
                return sub_id;
			} else {
				/* delete stream */
				LL_DELETE(_sub_streams, stream);
				delete stream;
			}

			return 0;
		}
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	stream = create_subscription_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
        sub_id = stream->Subscription();
		LL_APPEND(_sub_streams, stream);

		return sub_id;
	}

	return 0;
}

void CanLink::processReceivedTransfer(uint64_t ts, uint8_t index, CanardTransfer *transfer)
{
    /* update streams */
    CanStream *can;
    LL_FOREACH(_sub_streams, can) {
        can->update(ts, index, (const void *)transfer);
    }
}

void CanLink::send(void)
{
    configure_link("NODE",    1.0f);
    configure_link("SYNC_R",  10.0f);
#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)
    configure_link("GPS",   500.0f);
    configure_link("MAG",   500.0f);
#endif
    
#ifdef UAVCAN_NODE_PMU
    configure_link("BATT",  10.0f);
    configure_link("RNG",   500.0f);
#endif

#ifdef UAVCAN_NODE_ROS
    configure_link("ODOM",   500.0f);
#endif

    while (1)
    {
        const uint64_t ts = micros();

        perf_count(pref_can_interval);

        perf_begin(pref_can_elapsed);

        connect = (ts - fcu_vailed) < 3000000;
        if(per_connect != connect) {
            actuator_notify_s led_info;
            led_info.timestamp  = ts;
            led_info.lock       = NOTIFY_UNLOCK;
            led_info.group      = NOTIFY_GROUP_CHECK;
            led_info.priority   = NOTIFY_PRIORITY_VERY_HIGH;
            led_info.led_status = LED_PATTERN_BGC_FAST;
            led_info.rgb        = NOTIFY_RGB_RED;
            _led_pub.push(&led_info);

            per_connect = connect;
        }

        CanStream *can;
		LL_FOREACH(_can_streams, can) {
			can->update(ts);
		}

        _can_param->uavcan_param();
        if((ts - param_send_ts) > 200000) {
            _can_param->send_one();
            param_send_ts = ts;
        }

        for (const CanardFrame* txf = nullptr; (txf = canardTxPeek(&canard)) != nullptr;) {
            if ((txf->timestamp_usec == 0) || (txf->timestamp_usec > micros()))  {
                CAN_Transmit(txf);
            }
            canardTxPop(&canard);
            canard.memory_free(&canard, (CanardFrame*)txf);
        }
        perf_end(pref_can_elapsed);
        
        osDelay(2);
    }
}

void CanLink::recv(void)
{
    const uint32_t sync_id = configure_subscription("SYNC", 0.0f);
    filter[0].id   = sync_id;
    filter[0].mask = sync_id;

    const uint32_t node_id = configure_subscription("NODE", 0.0f);
    filter[1].id   = node_id;
    filter[1].mask = node_id;

    const uint32_t param_id = configure_subscription("PARAM", 0.0f);
    filter[2].id   = param_id;
    filter[2].mask = param_id;
#if defined(UAVCAN_NODE_GPS1)

    const uint32_t notify_id = configure_subscription("NOTIFY", 0.0f);
    filter[3].id   = notify_id;
    filter[3].mask = notify_id;

    const uint32_t inject_id = configure_subscription("INJECT", 0.0f);
    filter[4].id   = inject_id;
    filter[4].mask = inject_id;

    STM32_CAN_Filters(&filter[0], 5);
#elif defined(UAVCAN_NODE_ROS)
    const uint32_t mag_id = configure_subscription("MAG", 0.0f);
    filter[3].id   = mag_id;
    filter[3].mask = mag_id;

    const uint32_t gnss_id = configure_subscription("GNSS", 0.0f);
    filter[4].id   = gnss_id;
    filter[4].mask = gnss_id;

    const uint32_t imu_id = configure_subscription("IMU", 0.0f);
    filter[5].id   = imu_id;
    filter[5].mask = imu_id;

    STM32_CAN_Filters(&filter[0], 6);
#else

    const uint32_t notify_id = configure_subscription("NOTIFY", 0.0f);
    filter[3].id   = notify_id;
    filter[3].mask = notify_id;

    STM32_CAN_Filters(&filter[0], 4);
#endif

    while (1)
    {
        CanardTransfer transfer;
        CanardFrame received_frame;

        const uint64_t ts = micros();

        for (int16_t rx_res=0; (rx_res = CAN_Receive(&received_frame)) > 0;) {
            if(rx_res > 0) {
                const int8_t result = canardRxAccept(&canard,
                                     &received_frame,            // The CAN frame received from the bus.
                                     0,  // If the transport is not redundant, use 0.
                                     &transfer);
                 if (result < 0) {
                    // An error has occurred: either an argument is invalid or we've ran out of memory.
                    // It is possible to statically prove that an out-of-memory will never occur for a given application if
                    // the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
                    // Reception of an invalid frame is NOT an error.
                    Info_Debug("canard receive error");
                }
                else if (result == 1) {
                    processReceivedTransfer(ts, 0, &transfer);  // A transfer has been received, process it.
                    canard.memory_free(&canard, (void*)transfer.payload);  // Deallocate the dynamic memory afterwards.
                }
                else {
                    // Nothing to do.
                    // The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
                    // Reception of an invalid frame is NOT reported as an error because it is not an error.
                }
            }
        }

        osDelay(2);
    }
}


int canlink_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error\n");
		return 1;
	}
    for(int i=0; i<argc; i++) {
        if (!strcmp(argv[i], "start")) {

            if (CAN::gCan != nullptr) {
                Info_Debug("already running\n");
                return 0;
            }

            CAN::gCan = new CanLink();
            

            if (CAN::gCan == NULL) {
                Info_Debug("alloc failed\n");
                return 0;
            }
            CAN::gCan->init();
        }
    }
    return 1;
}
