#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <ipcpush.h>
#include <hrt_timer.h>
#include "debug.h"

#include "canlink_subscription.h"
#include "canlink.h"

#include <topics/actuator_armed.h>
#include <topics/actuator_notify.h>
#include <topics/uavcan_parameter_request.h>
#include <topics/uavcan_parameter_value.h>
#include <topics/uavcan_command_response.h>
#include <topics/gps_inject_data.h>
#include <topics/sensor_gps.h>
#include <topics/sensor_mag.h>
#include <topics/sensor_imu.h>

#include <uavcan/protocol/GlobalTimeSync.h>
#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/param/GetSet.h>
#include <uavcan/equipment/actuator/Command.h>
#include <uavcan/equipment/notify/Notify.h>
#include <uavcan/equipment/gnss/Inject.h>
#include <uavcan/equipment/gnss/Fix2.h>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.h>
#include <uavcan/equipment/ahrs/RawIMU.h>

#define TOPICS_OFFSET_PRIORITY        26U
#define TOPICS_OFFSET_SUBJECT_ID      8U
#define TOPICS_OFFSET_SERVICE_ID      14U
#define TOPICS_OFFSET_DST_NODE_ID     7U

#define SERVICE_NOT_MESSAGE  (UINT32_C(1) << 25U)
#define ANONYMOUS_MESSAGE    (UINT32_C(1) << 24U)
#define REQUEST_NOT_RESPONSE (UINT32_C(1) << 24U)

static uint32_t take_message_id(const CanardPortID subject_id, const CanardNodeID src_node_id, const CanardPriority prio)
{
    const uint32_t tmp = subject_id | (CANARD_SUBJECT_ID_MAX + 1) | ((CANARD_SUBJECT_ID_MAX + 1) * 2);
    return src_node_id | (tmp << TOPICS_OFFSET_SUBJECT_ID) | (prio << TOPICS_OFFSET_PRIORITY);
}

static uint32_t take_service_id(const CanardPortID service_id,
                                const bool request_not_response,
                                const CanardNodeID src_node_id,
                                const CanardNodeID dst_node_id,
                                const CanardPriority prio)
{
    return src_node_id | (((uint32_t) dst_node_id) << TOPICS_OFFSET_DST_NODE_ID) |
           (((uint32_t) service_id) << TOPICS_OFFSET_SERVICE_ID) |
           (request_not_response ? REQUEST_NOT_RESPONSE : 0U) | SERVICE_NOT_MESSAGE | (prio << TOPICS_OFFSET_PRIORITY);
}

class CanStreamSync : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamSync::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYNC";
	}

	static uint32_t get_id_static()
	{
#if defined(UAVCAN_NODE_GPS1)
		return take_service_id(UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID, false, CANLINK_ID_FCU, CANLINK_ID_GPS1, CanardPrioritySlow);
#elif defined(UAVCAN_NODE_GPS2)
		return take_service_id(UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID, false, CANLINK_ID_FCU, CANLINK_ID_GPS2, CanardPrioritySlow);
#elif defined(UAVCAN_NODE_PMU)
		return take_service_id(UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID, false, CANLINK_ID_FCU, CANLINK_ID_PMU, CanardPrioritySlow);
#elif defined(UAVCAN_NODE_ROS)
		return take_service_id(UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID, false, CANLINK_ID_FCU, CANLINK_ID_ROS, CanardPrioritySlow);
#else
		return 0;
#endif
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamSync(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindResponse,
								UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID,
								UAVCAN_PROTOCOL_GLOBALTIMESYNC_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:

	CanardRxSubscription subscription_info;
	// Filter gains
	//
	// Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead
	// to a smoother estimate, but track time drift more slowly, introducing a bias
	// in the estimate. Larger values will cause low-amplitude oscillations.
	//
	// Beta : Used to smooth the clock skew estimate. Smaller values will lead to a
	// tighter estimation of the skew (derivative), but will negatively affect how fast the
	// filter reacts to clock skewing (e.g cause by temperature changes to the oscillator).
	// Larger values will cause large-amplitude oscillations.
	static constexpr double ALPHA_GAIN_INITIAL = 0.2;
	static constexpr double BETA_GAIN_INITIAL = 0.2;
	static constexpr double ALPHA_GAIN_FINAL = 0.01;
	static constexpr double BETA_GAIN_FINAL = 0.01;

	// Filter gain scheduling
	//
	// The filter interpolates between the INITIAL and FINAL gains while the number of
	// exhanged timesync packets is less than CONVERGENCE_WINDOW. A lower value will
	// allow the timesync to converge faster, but with potentially less accurate initial
	// offset and skew estimates.
	static constexpr uint32_t CONVERGENCE_WINDOW = 50;

	// Outlier rejection and filter reset
	//
	// Samples with round-trip time higher than MAX_RTT_SAMPLE are not used to update the filter.
	// More than MAX_CONSECUTIVE_HIGH_RTT number of such events in a row will throw a warning
	// but not reset the filter.
	// Samples whose calculated clock offset is more than MAX_DEVIATION_SAMPLE off from the current
	// estimate are not used to update the filter. More than MAX_CONSECUTIVE_HIGH_DEVIATION number
	// of such events in a row will reset the filter. This usually happens only due to a time jump
	// on the remote system.
	// TODO : automatically determine these using ping statistics?
	static constexpr uint64_t MAX_RTT_SAMPLE = 10000;
	static constexpr uint64_t MAX_DEVIATION_SAMPLE = 100000;
	static constexpr uint32_t MAX_CONSECUTIVE_HIGH_RTT = 5;
	static constexpr uint32_t MAX_CONSECUTIVE_HIGH_DEVIATION = 5;

	uint32_t _sequence{0};
	double _time_offset{0};
	double _time_skew{0};

	// Filter parameters
	double _filter_alpha{ALPHA_GAIN_INITIAL};
	double _filter_beta{BETA_GAIN_INITIAL};

	uint32_t _high_deviation_count{0};
	uint32_t _high_rtt_count{0};

	/* do not allow top copying this class */
	CanStreamSync(CanStreamSync &);
	CanStreamSync &operator = (const CanStreamSync &);

protected:
	explicit CanStreamSync(CanLink *can) : CanStream(can)
	{}

	bool sync_converged()
	{
		return _sequence >= CONVERGENCE_WINDOW;
	}

	void add_sample(int64_t offset_us)
	{
		/* Online exponential smoothing filter. The derivative of the estimate is also
		* estimated in order to produce an estimate without steady state lag:
		* https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
		*/

		double time_offset_prev = _time_offset;

		if (_sequence == 0) {			// First offset sample
			_time_offset = offset_us;

		} else {
			// Update the clock offset estimate
			_time_offset = _filter_alpha * offset_us + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

			// Update the clock skew estimate
			_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
		}
	}

	void reset_filter()
	{
		// Do a full reset of all statistics and parameters
		_sequence = 0;
		_time_offset = 0.0;
		_time_skew = 0.0;
		_filter_alpha = ALPHA_GAIN_INITIAL;
		_filter_beta = BETA_GAIN_INITIAL;
		_high_deviation_count = 0;
		_high_rtt_count = 0;
	}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID) || 
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindResponse)) {
			return false;
		}

		uavcan_protocol_GlobalTimeSync tsync;

		if(uavcan_protocol_GlobalTimeSync_decode(transfer, transfer->payload_size, &tsync, nullptr) < 0) {
			return false;
		}

		const uint64_t now = micros();

		if(tsync.tc1 > 0) {
			int64_t offset_us = (int64_t)(tsync.ts1 + now - tsync.tc1 * 2) / 2;

			// Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
			uint64_t rtt_us = now - tsync.ts1;

			// Calculate the difference of this sample from the current estimate
			uint64_t deviation = llabs((int64_t)_time_offset - offset_us);

			if (rtt_us < MAX_RTT_SAMPLE) {	// Only use samples with low RTT
				if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE)) {

					// Increment the counter if we have a good estimate and are getting samples far from the estimate
					_high_deviation_count++;

					// We reset the filter if we received 5 consecutive samples which violate our present estimate.
					// This is most likely due to a time jump on the offboard system.
					if (_high_deviation_count > MAX_CONSECUTIVE_HIGH_DEVIATION) {
						Info_Debug("[timesync] Time jump detected. Resetting time synchroniser.\n");
						// Reset the filter
						reset_filter();
					}
				} else {

					// Filter gain scheduling
					if (!sync_converged()) {
						// Interpolate with a sigmoid function
						double progress = (double)_sequence / (double)CONVERGENCE_WINDOW;
						double p = 1.0 - exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
						_filter_alpha = p * ALPHA_GAIN_FINAL + (1.0 - p) * ALPHA_GAIN_INITIAL;
						_filter_beta = p * BETA_GAIN_FINAL + (1.0 - p) * BETA_GAIN_INITIAL;
					} else {
						_filter_alpha = ALPHA_GAIN_FINAL;
						_filter_beta = BETA_GAIN_FINAL;
						sync_offset((int64_t)_time_offset);
					}

					// Perform filter update
					add_sample(offset_us);

					// Increment sequence counter after filter update
					_sequence++;

					// Reset high deviation count after filter update
					_high_deviation_count = 0;

					// Reset high RTT count after filter update
					_high_rtt_count = 0;
				}
			} else {
				// Increment counter if round trip time is too high for accurate timesync
				_high_rtt_count++;

				if (_high_rtt_count > MAX_CONSECUTIVE_HIGH_RTT) {
					Info_Debug("[timesync] RTT too high for timesync: %d ms\n", rtt_us/1000);
					// Reset counter to rate-limit warnings
					_high_rtt_count = 0;
				}

			}
		}

		return true;
	}
};

class CanStreamNode : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamNode::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NODE";
	}

	static uint32_t get_id_static()
	{
		return take_message_id(UAVCAN_PROTOCOL_NODESTATUS_ID, CANLINK_ID_FCU, CanardPrioritySlow);
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamNode(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindMessage,
								UAVCAN_PROTOCOL_NODESTATUS_ID,
								UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
	IPCPush  arm_pub;
	CanardRxSubscription subscription_info;

	/* do not allow top copying this class */
	CanStreamNode(CanStreamNode &);
	CanStreamNode &operator = (const CanStreamNode &);

protected:
	explicit CanStreamNode(CanLink *can) : CanStream(can),
		arm_pub(IPC_ID(actuator_armed))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_PROTOCOL_NODESTATUS_ID) ||
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindMessage)) {
			return false;
		}

		uavcan_protocol_NodeStatus node;

		if(uavcan_protocol_NodeStatus_decode(transfer, transfer->payload_size, &node, nullptr) < 0) {
			return false;
		}
		actuator_armed_s armed_data;

		armed_data.timestamp = t;
		armed_data.armed = node.sub_mode;

		arm_pub.push(&armed_data);

		_can->fcu_vailed = transfer->timestamp_usec;
		
		return true;
	}
};

#if !defined(UAVCAN_NODE_ROS)
class CanStreamNotify : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamNotify::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NOTIFY";
	}

	static uint32_t get_id_static()
	{
		return take_message_id(UAVCAN_EQUIPMENT_NOTIFY_NOTIFY_ID, CANLINK_ID_FCU, CanardPrioritySlow);
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamNotify(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindMessage,
								UAVCAN_EQUIPMENT_NOTIFY_NOTIFY_ID,
								UAVCAN_EQUIPMENT_NOTIFY_NOTIFY_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
	CanardRxSubscription subscription_info;

	IPCPush  led_pub;

	/* do not allow top copying this class */
	CanStreamNotify(CanStreamNotify &);
	CanStreamNotify &operator = (const CanStreamNotify &);

protected:
	explicit CanStreamNotify(CanLink *can) : CanStream(can),
		led_pub(IPC_ID(actuator_notify))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_EQUIPMENT_NOTIFY_NOTIFY_ID) || 
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindMessage)) {
			return false;
		}

		uavcan_equipment_notify_Notify notify_led;

		if(uavcan_equipment_notify_Notify_decode(transfer, transfer->payload_size, &notify_led, nullptr) < 0) {
			return false;
		}

		actuator_notify_s led_info;
		led_info.timestamp  = t;
		led_info.lock       = notify_led.lock;
		led_info.group      = notify_led.group;
		led_info.priority   = notify_led.priority;
		led_info.led_status = notify_led.led_status;
		led_info.rgb        = notify_led.rgb;
		led_pub.push(&led_info);

		return true;
	}
};
#endif

class CanStreamParam : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamParam::get_name_static();
	}

	static const char *get_name_static()
	{
		return "PARAM";
	}

	static uint32_t get_id_static()
	{
#if defined(UAVCAN_NODE_GPS1)
		return take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_FCU, CANLINK_ID_GPS1, CanardPrioritySlow);
#elif defined(UAVCAN_NODE_GPS2)
		return take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_FCU, CANLINK_ID_GPS2, CanardPrioritySlow);
#elif defined(UAVCAN_NODE_PMU)
		return take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_FCU, CANLINK_ID_PMU, CanardPrioritySlow);
#elif defined(UAVCAN_NODE_ROS)
		return take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_FCU, CANLINK_ID_ROS, CanardPrioritySlow);
#else
		return 0;
#endif
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamParam(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindRequest,
								UAVCAN_PROTOCOL_PARAM_GETSET_ID,
								UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
	CanardRxSubscription subscription_info;

	IPCPush param_req;


	/* do not allow top copying this class */
	CanStreamParam(CanStreamParam &);
	CanStreamParam &operator = (const CanStreamParam &);

protected:
	explicit CanStreamParam(CanLink *can) : CanStream(can),
		param_req(IPC_ID(uavcan_parameter_request))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_PROTOCOL_PARAM_GETSET_ID) ||
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindRequest)) {
			return false;
		}

		uint8_t data_buf[256];
		uint8_t *dy_buf = &data_buf[0];
		uavcan_protocol_param_GetSetRequest can_param_data;

		if(uavcan_protocol_param_GetSetRequest_decode(transfer, 0, &can_param_data, &dy_buf) < 0) {
			return true;
		}
		uavcan_parameter_request_s req;
		
		req.timestamp = t;
		req.param_type = (uint8_t)can_param_data.value.union_tag;
		req.param_index = can_param_data.index;
		req.message_type = can_param_data.operate;
		strncpy(req.param_id, (const char *)can_param_data.name.data,  can_param_data.name.len + 1);
		req.param_id[can_param_data.name.len + 1] = '\0';
		
		if(can_param_data.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
			req.int_value = can_param_data.value.integer_value;
		} else if(can_param_data.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
			req.real_value = can_param_data.value.real_value;
		}

		// Info_Debug("PARAM1");

		param_req.push(&req);
		
		return true;
	}
};

#if defined(UAVCAN_NODE_GPS1)
class CanStreamInject : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamInject::get_name_static();
	}

	static const char *get_name_static()
	{
		return "INJECT";
	}

	static uint32_t get_id_static()
	{
		return take_message_id(UAVCAN_EQUIPMENT_GNSS_INJECT_ID, CANLINK_ID_FCU, CanardPriorityFast);
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamInject(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindMessage,
								UAVCAN_EQUIPMENT_GNSS_INJECT_ID,
								UAVCAN_EQUIPMENT_GNSS_INJECT_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
    uavcan_equipment_gnss_Inject ingect_data;

	CanardRxSubscription subscription_info;

	IPCPush  inject_pub;
	uint8_t data_buf[182];

	/* do not allow top copying this class */
	CanStreamInject(CanStreamInject &);
	CanStreamInject &operator = (const CanStreamInject &);

protected:
	explicit CanStreamInject(CanLink *can) : CanStream(can),
		inject_pub(IPC_ID(gps_inject_data))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
        uint8_t *dy_buf = &data_buf[0];
		
		if((transfer->port_id != UAVCAN_EQUIPMENT_GNSS_INJECT_ID) || 
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindMessage)) {
			return false;
		}

		if(uavcan_equipment_gnss_Inject_decode(transfer, 0, &ingect_data, &dy_buf) < 0) {
			return false;
		}

		gps_inject_data_s inject_info;
		inject_info.timestamp  = t;
		inject_info.flags = ingect_data.flags;
		inject_info.len   = ingect_data.len;

		memcpy(inject_info.data, ingect_data.data.data, ingect_data.len);

		inject_pub.push(&inject_info);

		return true;
	}
};
#endif

#if defined(UAVCAN_NODE_ROS)
class CanStreamGNSS : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamGNSS::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GNSS";
	}

	static uint32_t get_id_static()
	{
		return take_message_id(UAVCAN_EQUIPMENT_GNSS_FIX2_ID, CANLINK_ID_GPS1, CanardPriorityHigh);
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamGNSS(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindMessage,
								UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
								UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
	CanardRxSubscription subscription_info;

	IPCPush  gps_pub;

	/* do not allow top copying this class */
	CanStreamGNSS(CanStreamGNSS &);
	CanStreamGNSS &operator = (const CanStreamGNSS &);

protected:
	explicit CanStreamGNSS(CanLink *can) : CanStream(can),
		gps_pub(IPC_ID(sensor_gps))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_EQUIPMENT_GNSS_FIX2_ID) || 
        (transfer->remote_node_id != CANLINK_ID_GPS1) ||
        (transfer->transfer_kind != CanardTransferKindMessage)) {
			return false;
		}

		uavcan_equipment_gnss_Fix2 gps_filed;

		if(uavcan_equipment_gnss_Fix2_decode(transfer, transfer->payload_size, &gps_filed, nullptr) < 0) {
			return false;
		}

		sensor_gps_s gps_info;
		gps_info.timestamp = transfer->timestamp_usec;
		gps_info.last_message_time_ms = gps_info.timestamp / 1000;
		gps_info.instance = 0;
		gps_info.healthy  = true;

		gps_info.lat = gps_filed.latitude_deg_1e8;			     // Latitude in 1E-7 degrees
		gps_info.lon = gps_filed.longitude_deg_1e8;			 // Longitude in 1E-7 degrees
		gps_info.alt = gps_filed.height_msl_mm;			     // Altitude in 1E-3 meters above MSL, (millimetres)
		gps_info.alt_ellipsoid = 0; 		                         // Altitude in 1E-3 meters bove Ellipsoid, (millimetres)

		gps_info.s_variance_m_s = gps_filed.vel_covariance;     // GPS speed accuracy estimate, (metres/sec)
		gps_info.c_variance_rad = 0.0f;		                     // GPS course accuracy estimate, (radians)
		gps_info.fix_type = gps_filed.status;                   // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

		gps_info.eph = gps_filed.pos_covariance;			     // GPS horizontal position accuracy (metres)
		gps_info.epv = gps_filed.hgt_covariance;			     // GPS vertical position accuracy (metres)

		gps_info.hdop = gps_filed.hdop;			             // Horizontal dilution of precision
		gps_info.vdop = gps_filed.vdop;			             // Vertical dilution of precision

		gps_info.noise_per_ms = 0;		                         // GPS noise per millisecond
		gps_info.jamming_indicator = 0;		                     // indicates jamming is occurring

		gps_info.vel_m_s = sqrt(gps_filed.ned_velocity[0]*gps_filed.ned_velocity[0] * gps_filed.ned_velocity[1]*gps_filed.ned_velocity[1]); // GPS ground speed, (metres/sec)
		gps_info.vel_n_m_s = gps_filed.ned_velocity[0];		 // GPS North velocity, (metres/sec)
		gps_info.vel_e_m_s = gps_filed.ned_velocity[1];		 // GPS East velocity, (metres/sec)
		gps_info.vel_d_m_s = gps_filed.ned_velocity[2]; 	     // GPS Down velocity, (metres/sec)
		gps_info.cog_rad = 0.0f;			                         // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
		gps_info.vel_ned_valid = true;		                     // True if NED velocity is valid

		gps_info.timestamp_time_relative = 0;	                     // timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
		gps_info.time_utc_usec = gps_filed.gnss_timestamp.usec; // Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

		gps_info.satellites_used = gps_filed.sats_used;		 // Number of satellites used

		gps_info.heading = 0.0f;			                         // heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
		gps_info.heading_offset = 0.0f;		                     // heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])

		gps_info.posOffset[0] = gps_filed.pos_offset[0];
		gps_info.posOffset[1] = gps_filed.pos_offset[1];
		gps_info.posOffset[2] = gps_filed.pos_offset[2];
		gps_info.gps_lag_ms = gps_filed.lag_ms;

		gps_pub.push(&gps_info);

		return true;
	}
};

class CanStreamMag : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamMag::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MAG";
	}

	static uint32_t get_id_static()
	{
		return take_message_id(UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID, CANLINK_ID_FCU, CanardPriorityNominal);
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamMag(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindMessage,
								UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID,
								UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
	CanardRxSubscription subscription_info;

	IPCPush  mag_pub;

	/* do not allow top copying this class */
	CanStreamMag(CanStreamMag &);
	CanStreamMag &operator = (const CanStreamMag &);

protected:
	explicit CanStreamMag(CanLink *can) : CanStream(can),
		mag_pub(IPC_ID(sensor_mag))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID) || 
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindMessage)) {
			return false;
		}

		uavcan_equipment_ahrs_MagneticFieldStrength2 mag_data;

		if(uavcan_equipment_ahrs_MagneticFieldStrength2_decode(transfer, transfer->payload_size, &mag_data, nullptr) < 0) {
			return false;
		}

		sensor_mag_s mag_info;
		mag_info.timestamp = transfer->timestamp_usec;
		mag_info.healthy       = true;
		mag_info.is_external   = true;
		mag_info.learn_offsets = false;
		mag_info.lag_ms        = 45;

		mag_info.field_raw[0] = 0.0f;
		mag_info.field_raw[1] = 0.0f;
		mag_info.field_raw[2] = 0.0f;

		mag_info.field[0] = mag_data.magnetic_field_ga[0];
		mag_info.field[1] = mag_data.magnetic_field_ga[1];
		mag_info.field[2] = mag_data.magnetic_field_ga[2];

		mag_info.offsets[0] = 0.0f;
		mag_info.offsets[1] = 0.0f;
		mag_info.offsets[2] = 0.0f;

		mag_pub.push(&mag_info);

		return true;
	}
};

class CanStreamIMU : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "IMU";
	}

	static uint32_t get_id_static()
	{
		return take_message_id(UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID, CANLINK_ID_FCU, CanardPriorityNominal);
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamIMU(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(_can->canIns(),
								CanardTransferKindMessage,
								UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID,
								UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		return get_id_static();
	}

private:
    uavcan_equipment_ahrs_RawIMU imu_filed;

	CanardRxSubscription subscription_info;

	IPCPush  imu_pub;

	/* do not allow top copying this class */
	CanStreamIMU(CanStreamIMU &);
	CanStreamIMU &operator = (const CanStreamIMU &);

protected:
	explicit CanStreamIMU(CanLink *can) : CanStream(can),
		imu_pub(IPC_ID(sensor_imu))
	{}

	bool link_proc(const uint64_t t, const uint8_t index, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id != UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID) || 
        (transfer->remote_node_id != CANLINK_ID_FCU) ||
        (transfer->transfer_kind != CanardTransferKindMessage)) {
			return false;
		}

		if(uavcan_equipment_ahrs_RawIMU_decode(transfer, transfer->payload_size, &imu_filed, nullptr) < 0) {
			return false;
		}

		sensor_imu_s imu_info;
		imu_info.timestamp       = transfer->timestamp_usec;
		imu_info.gyro_filter[0]  = imu_filed.rate_gyro_latest[0];
		imu_info.gyro_filter[1]  = imu_filed.rate_gyro_latest[1];
		imu_info.gyro_filter[2]  = imu_filed.rate_gyro_latest[2];

		imu_info.accel_filter[0] = imu_filed.accelerometer_latest[0];
		imu_info.accel_filter[1] = imu_filed.accelerometer_latest[1];
		imu_info.accel_filter[2] = imu_filed.accelerometer_latest[2];
		
		imu_pub.push(&imu_info);

		return true;
	}
};
#endif

static const CANStreamSubscriptionItem subscription_list[] = {
	CANStreamSubscriptionItem(&CanStreamSync::new_instance,          &CanStreamSync::get_name_static,          &CanStreamSync::get_id_static),
#if !defined(UAVCAN_NODE_ROS)
	CANStreamSubscriptionItem(&CanStreamNotify::new_instance,        &CanStreamNotify::get_name_static,        &CanStreamNotify::get_id_static),
#endif
	CANStreamSubscriptionItem(&CanStreamNode::new_instance,          &CanStreamNode::get_name_static,          &CanStreamNode::get_id_static),
	CANStreamSubscriptionItem(&CanStreamParam::new_instance,         &CanStreamParam::get_name_static,         &CanStreamParam::get_id_static),
#if defined(UAVCAN_NODE_GPS1)
	CANStreamSubscriptionItem(&CanStreamInject::new_instance,        &CanStreamInject::get_name_static,        &CanStreamInject::get_id_static),
#endif
#if defined(UAVCAN_NODE_ROS)
	CANStreamSubscriptionItem(&CanStreamGNSS::new_instance,          &CanStreamGNSS::get_name_static,          &CanStreamGNSS::get_id_static),
	CANStreamSubscriptionItem(&CanStreamMag::new_instance,           &CanStreamMag::get_name_static,           &CanStreamMag::get_id_static),
	CANStreamSubscriptionItem(&CanStreamIMU::new_instance,           &CanStreamIMU::get_name_static,           &CanStreamIMU::get_id_static),
#endif
};

const char *get_subscription_name(const uint32_t link_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &link : subscription_list) {
		if (link_id == link.get_id()) {
			return link.get_name();
		}
	}

	return nullptr;
}

CanStream *create_subscription_stream(const char *link_name, CanLink *can)
{
	// search for stream with specified name in supported streams list
	if (link_name != nullptr) {
		for (const auto &link : subscription_list) {
			if (strcmp(link_name, link.get_name()) == 0) {
				return link.new_instance(can);
			}
		}
	}

	return nullptr;
}
