
/* XXX trim includes */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include <errno.h>
#include <stdlib.h>
#include "param.h"
#include "buildin_function.h"
#include "serial.h"
#include "debug.h"

#include "mavlink_messages.h"
#include "mavlink_bridge_header.h"
#include "mavlink_receiver.h"
#include "mavlink_main.h"

#ifdef UAVCAN_NODE_ROS
const osThreadAttr_t telr_attributes = {
		.name = "telr",
		.priority = (osPriority_t)osPriorityNormal6,
		.stack_size = 2048};

namespace MavR {
	static MavlinkReceiver *gRecv;
}

static void mavrecv_func(MavlinkReceiver *pThis)
{
    pThis->run(pThis->_param);
}

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	_mavlink(parent),
	_status{},
	_telemetry_status_pub(nullptr),
	_odometry_pub(nullptr),
	_param_sub(IPC_ID(parameter_update)),
	_param(nullptr)
{
	_last_vio_sample = 0;
}

MavlinkReceiver::~MavlinkReceiver()
{

}


void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;
	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;
	default:
		break;
	}

	/* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

void MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);

	struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

	/* set heartbeat time and topic time and publish -
		* the telem status also gets updated on telemetry events
		*/
	tstatus.timestamp = micros();
	tstatus.heartbeat_time = tstatus.timestamp;

	if (_telemetry_status_pub == nullptr) {
		_telemetry_status_pub = ipc_active(IPC_ID(telemetry_status), &tstatus);

	} else {
		ipc_push(IPC_ID(telemetry_status), _telemetry_status_pub, &tstatus);
	}

	// Info_Debug("Receive heartbeat\n");
}

void MavlinkReceiver::handle_message_odometry(mavlink_message_t *msg)
{
	mavlink_odometry_t odom;
	mavlink_msg_odometry_decode(msg, &odom);

	perf_count(pref_odometry_interval);

	// sensor_vio_s odom_data;
    const uint64_t ts = micros();

	odom_data.timestamp = ts;
    if(_last_vio_sample != 0) {
        odom_data.delTime = (ts - _last_vio_sample)/1000000.0f;
    } else {
        odom_data.delTime = 0.0333f;
    }
	odom_data.position[0] = odom.x;
	odom_data.position[1] = odom.y;
	odom_data.position[2] = odom.z;

	odom_data.velocity[0] = odom.vx;
	odom_data.velocity[1] = odom.vy;
	odom_data.velocity[2] = odom.vz;

	odom_data.quat[0] = odom.q[0];
	odom_data.quat[1] = odom.q[1];
	odom_data.quat[2] = odom.q[2];
	odom_data.quat[3] = odom.q[3];

	odom_data.delAng[0] = odom.rollspeed;
	odom_data.delAng[1] = odom.pitchspeed;
	odom_data.delAng[2] = odom.yawspeed;

	odom_data.posVariance[0] = odom.pose_covariance[0];
	odom_data.posVariance[1] = odom.pose_covariance[6];
	odom_data.posVariance[2] = odom.pose_covariance[11];

	odom_data.attVariance[0] = odom.pose_covariance[15];
	odom_data.attVariance[1] = odom.pose_covariance[18];
	odom_data.attVariance[2] = odom.pose_covariance[20];

	odom_data.velVariance[0] = odom.velocity_covariance[0];
	odom_data.velVariance[1] = odom.velocity_covariance[6];
	odom_data.velVariance[2] = odom.velocity_covariance[11];

	odom_data.angVariance[0] = odom.velocity_covariance[15];
	odom_data.angVariance[1] = odom.velocity_covariance[18];
	odom_data.angVariance[2] = odom.velocity_covariance[20];

	odom_data.posOffset[0] = _config._pos_offset_x;
	odom_data.posOffset[1] = _config._pos_offset_y;
	odom_data.posOffset[2] = _config._pos_offset_z;

	if (_odometry_pub == nullptr) {
		_odometry_pub = ipc_active(IPC_ID(sensor_vio), &odom_data);
	} else {
		ipc_push(IPC_ID(sensor_vio), _odometry_pub, &odom_data);
	}

	_last_vio_sample = ts;
}

void MavlinkReceiver::parameter_update(bool force)
{
    parameter_update_s data;
    if(force || _param_sub.update_if_changed(&data)) {
        param_get(_params_handles.pos_offset_x, &_config._pos_offset_x);
        param_get(_params_handles.pos_offset_y, &_config._pos_offset_y);
        param_get(_params_handles.pos_offset_z, &_config._pos_offset_z);
		param_get(_params_handles.rotation,     &_config._rotation);
    }
}

/**
 * Receive data from UART.
 */
void MavlinkReceiver::run(void *parameter)
{
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[256];

	mavlink_message_t msg;

	uint16_t nread = 0;

	parameter_update(true);

	while (1) {

		perf_count(pref_receive_interval);
		/*
			* to avoid reading very small chunks wait for data before reading
			* this is designed to target one message, so >20 bytes at a time
			*/
		const unsigned character_count = 20;

		parameter_update(false);

		nread = hal_uart1_read(buf, sizeof(buf));

		/* non-blocking read. read may return negative values */
		if (nread < (uint16_t)character_count) {
			unsigned sleeptime = (unsigned)((1.0f / (_mavlink->get_baudrate() / 10)) * character_count * 1000);
			osDelay(sleeptime);
		}
		// only start accepting messages once we're sure who we talk to

		/* if read failed, this loop won't execute */
		for (uint16_t i = 0; i < nread; i++) {
			if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &_status)) {

				/* handle generic messages and commands */
				handle_message(&msg);
			}
		}

		/* count received bytes (nread will be -1 on read error) */
		if (nread > 0) {
			_mavlink->count_rxbytes(nread);
		}

		// 500Hz loop
        osDelay(2);
	}
}

bool MavlinkReceiver::init()
{
	_params_handles.pos_offset_x = param_find("POS_OFFX");
    _params_handles.pos_offset_y = param_find("POS_OFFY");
    _params_handles.pos_offset_z = param_find("POS_OFFZ");
	_params_handles.rotation     = param_find("EXT_ROTAT");

	pref_receive_interval = perf_alloc(PC_INTERVAL, "recv_int");
	pref_odometry_interval = perf_alloc(PC_INTERVAL, "vio_int");

	_handle = osThreadNew((osThreadFunc_t)mavrecv_func, this, &telr_attributes);

    if (_handle == nullptr) {
		Info_Debug("telemetry task start error!\n");
        return false;
    }

	return true;
}

bool
MavlinkReceiver::receive_start(Mavlink *parent)
{
	MavR::gRecv = new MavlinkReceiver(parent);
	if (MavR::gRecv == nullptr) {
		Info_Debug("alloc failed");
		return false;
	}

	if(!MavR::gRecv->init()) {
		delete MavR::gRecv;
		return false;
	}
	return true;
}
#endif