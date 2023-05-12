#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <ipcpull.h>

#include "canlink_messages.h"
#include "canlink.h"

#include <topics/sensor_mag.h>
#include <topics/sensor_gps.h>
#include <topics/sensor_vio.h>
#include <topics/battery_status.h>
#include <topics/sensor_distance.h>
#include <topics/sensor_oflow.h>
#include <topics/uavcan_parameter_request.h>
#include <topics/uavcan_command_request.h>

#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/GlobalTimeSync.h>
#include <uavcan/protocol/param/GetSet.h>
#include <uavcan/equipment/actuator/Command.h>
#include <uavcan/equipment/optical_flow/OpticalFlow.h>
#include <uavcan/equipment/range_sensor/Measurement.h>
#include <uavcan/equipment/power/BatteryInfo.h>
#include <uavcan/equipment/gnss/Fix2.h>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.h>
#include <uavcan/equipment/ros/Odometry.h>

class CanStreamStatus : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NODE";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_PROTOCOL_NODESTATUS_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamStatus(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamStatus(CanStreamStatus &);
	CanStreamStatus &operator = (const CanStreamStatus &);


protected:
	explicit CanStreamStatus(CanLink *can) : CanStream(can),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
        uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
        
        memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);

        uavcan_protocol_NodeStatus nodestatus;
        nodestatus.uptime_sec = millis();
        nodestatus.health     = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
        nodestatus.mode       = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
        nodestatus.sub_mode   = 0;
        nodestatus.vendor_specific_status_code = 0;

        const uint32_t total_size = uavcan_protocol_NodeStatus_encode(&nodestatus, buffer);

        const CanardTransfer transfer = {
            /*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
            /*priority       */ CanardPrioritySlow,
            /*transfer_kind  */ CanardTransferKindMessage,
            /*port_id        */ UAVCAN_PROTOCOL_NODESTATUS_ID, // This is the subject-ID.
            /*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
            /*transfer_id    */ transfer_id,
            /*payload_size   */ total_size,
            /*payload        */ buffer,
        };

		++transfer_id;

        const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

        if (resp_res <= 0) {
            return false;
        }

		return true;
	}
};

class CanStreamSyncReq : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamSyncReq::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYNC_R";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamSyncReq(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamSyncReq(CanStreamSyncReq &);
	CanStreamSyncReq &operator = (const CanStreamSyncReq &);


protected:
	explicit CanStreamSyncReq(CanLink *can) : CanStream(can),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
        uint8_t buffer[UAVCAN_PROTOCOL_GLOBALTIMESYNC_MAX_SIZE];
        
        memset(buffer, 0, UAVCAN_PROTOCOL_GLOBALTIMESYNC_MAX_SIZE);

        uavcan_protocol_GlobalTimeSync sync_data;

		sync_data.tc1 = 0;
		sync_data.ts1 = micros();

        const uint32_t total_size = uavcan_protocol_GlobalTimeSync_encode(&sync_data, buffer);

        const CanardTransfer transfer = {
            /*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
            /*priority       */ CanardPrioritySlow,
            /*transfer_kind  */ CanardTransferKindRequest,
            /*port_id        */ UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID, // This is the subject-ID.
            /*remote_node_id */ CANLINK_ID_FCU,            // Messages cannot be unicast, so use UNSET.
            /*transfer_id    */ transfer_id,
            /*payload_size   */ total_size,
            /*payload        */ buffer,
        };

		++transfer_id;

        const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

        if (resp_res <= 0) {
            return false;
        }

		return true;
	}
};

#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)

class CanStreamGPS : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamGPS::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_EQUIPMENT_GNSS_FIX2_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamGPS(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	IPCPull gps_sub;
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamGPS(CanStreamGPS &);
	CanStreamGPS &operator = (const CanStreamGPS &);


protected:
	explicit CanStreamGPS(CanLink *can) : CanStream(can),
		gps_sub(IPC_ID(sensor_gps)),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
		sensor_gps_s gps_info;
		if(gps_sub.update_if_changed(&gps_info)) {
			uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE];
        
			memset(buffer, 0, UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE);

			uavcan_equipment_gnss_Fix2 gps_fix;
			gps_fix.timestamp.usec      = gps_info.timestamp;
			gps_fix.gnss_timestamp.usec = gps_info.time_utc_usec;
			gps_fix.gnss_time_standard  = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC;
			gps_fix.gnss_id             = 0;
			gps_fix.lag_ms              = gps_info.gps_lag_ms;
			gps_fix.longitude_deg_1e8   = gps_info.lon;
			gps_fix.latitude_deg_1e8    = gps_info.lat;
			gps_fix.height_msl_mm       = gps_info.alt;
			gps_fix.ned_velocity[0]     = gps_info.vel_n_m_s;
			gps_fix.ned_velocity[1]     = gps_info.vel_e_m_s;
			gps_fix.ned_velocity[2]     = gps_info.vel_d_m_s;
			gps_fix.sats_used           = gps_info.satellites_used;
			gps_fix.status              = gps_info.fix_type;
			gps_fix.pos_covariance      = gps_info.eph;
			gps_fix.hgt_covariance      = gps_info.epv;
			gps_fix.vel_covariance      = gps_info.s_variance_m_s;
			gps_fix.hdop                = gps_info.hdop;
			gps_fix.vdop                = gps_info.vdop;
			gps_fix.pos_offset[0]       = gps_info.posOffset[0];
			gps_fix.pos_offset[1]       = gps_info.posOffset[1];
			gps_fix.pos_offset[2]       = gps_info.posOffset[2];

			const uint32_t total_size = uavcan_equipment_gnss_Fix2_encode(&gps_fix, buffer);

			const CanardTransfer transfer = {
				/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
				/*priority       */ CanardPriorityHigh,
				/*transfer_kind  */ CanardTransferKindMessage,
				/*port_id        */ UAVCAN_EQUIPMENT_GNSS_FIX2_ID, // This is the subject-ID.
				/*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
				/*transfer_id    */ transfer_id,
				/*payload_size   */ total_size,
				/*payload        */ buffer,
			};

			++transfer_id;

			const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

			if (resp_res <= 0) {
				return false;
			}
		}
		return true;
	}
};

class CanStreamMAG : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamMAG::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MAG";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamMAG(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	IPCPull mag_sub;
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamMAG(CanStreamMAG &);
	CanStreamMAG &operator = (const CanStreamMAG &);


protected:
	explicit CanStreamMAG(CanLink *can) : CanStream(can),
		mag_sub(IPC_ID(sensor_mag)),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
		sensor_mag_s mag_info;
		if(mag_sub.update_if_changed(&mag_info)) {
			uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_MAX_SIZE];
        
			memset(buffer, 0, UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_MAX_SIZE);

			uavcan_equipment_ahrs_MagneticFieldStrength2 mag_data;
			mag_data.timestamp.usec       = mag_info.timestamp;
			mag_data.sensor_id            = 0;
			mag_data.magnetic_field_ga[0] = mag_info.field_raw[0];
			mag_data.magnetic_field_ga[1] = mag_info.field_raw[1];
			mag_data.magnetic_field_ga[2] = mag_info.field_raw[2];
			mag_data.magnetic_field_covariance = mag_info.covariance;

			const uint32_t total_size = uavcan_equipment_ahrs_MagneticFieldStrength2_encode(&mag_data, buffer);

			const CanardTransfer transfer = {
				/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
				/*priority       */ CanardPriorityHigh,
				/*transfer_kind  */ CanardTransferKindMessage,
				/*port_id        */ UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID, // This is the subject-ID.
				/*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
				/*transfer_id    */ transfer_id,
				/*payload_size   */ total_size,
				/*payload        */ buffer,
			};

			++transfer_id;

			const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

			if (resp_res <= 0) {
				return false;
			}
		}
		return true;
	}
};

#endif

#ifdef UAVCAN_NODE_PMU
class CanStreamPower : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamPower::get_name_static();
	}

	static const char *get_name_static()
	{
		return "BATT";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamPower(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	IPCPull bat_sub;
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamPower(CanStreamPower &);
	CanStreamPower &operator = (const CanStreamPower &);


protected:
	explicit CanStreamPower(CanLink *can) : CanStream(can),
		bat_sub(IPC_ID(battery_status)),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
		battery_status_s bat_info;
		if(bat_sub.update_if_changed(&bat_info)) {
			uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
        
			memset(buffer, 0, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE);

			uavcan_equipment_power_BatteryInfo bat_data;
			bat_data.temperature             = 0;
			bat_data.voltage                 = bat_info.voltage_v;
			bat_data.current                 = bat_info.current_a;
			bat_data.average_power_10sec     = 0;
			bat_data.remaining_capacity_wh   = 0;
			bat_data.full_charge_capacity_wh = 0;

			const uint32_t total_size = uavcan_equipment_power_BatteryInfo_encode(&bat_data, buffer);

			const CanardTransfer transfer = {
				/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
				/*priority       */ CanardPriorityNominal,
				/*transfer_kind  */ CanardTransferKindMessage,
				/*port_id        */ UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, // This is the subject-ID.
				/*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
				/*transfer_id    */ transfer_id,
				/*payload_size   */ total_size,
				/*payload        */ buffer,
			};

			++transfer_id;

			const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

			if (resp_res <= 0) {
				return false;
			}
		}
		return true;
	}
};

class CanStreamRng : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamRng::get_name_static();
	}

	static const char *get_name_static()
	{
		return "RNG";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamRng(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	IPCPull rng_sub;
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamRng(CanStreamRng &);
	CanStreamRng &operator = (const CanStreamRng &);


protected:
	explicit CanStreamRng(CanLink *can) : CanStream(can),
		rng_sub(IPC_ID(sensor_distance)),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
		sensor_distance_s rng_info;
		if(rng_sub.update_if_changed(&rng_info)) {
			uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
        
			memset(buffer, 0, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE);

			uavcan_equipment_range_sensor_Measurement rng_data;

			rng_data.timestamp.usec         = rng_info.timestamp;
			rng_data.sensor_type            = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
			if(rng_info.current_distance > rng_info.max_distance) {
				rng_data.reading_type       = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
			} else if(rng_info.current_distance < rng_info.min_distance) {
				rng_data.reading_type       = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
			} else {
				rng_data.reading_type       = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
			}
			rng_data.range                  = rng_info.current_distance;
			rng_data.qualty                 = rng_info.quality;
			rng_data.covariance             = 0.25f;
			rng_data.rng_max                = rng_info.max_distance;
			rng_data.rng_min                = rng_info.min_distance;

			rng_data.pos_offset[0]       = rng_info.posOffset[0];
			rng_data.pos_offset[1]       = rng_info.posOffset[1];
			rng_data.pos_offset[2]       = rng_info.posOffset[2];

			const uint32_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&rng_data, buffer);

			const CanardTransfer transfer = {
				/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
				/*priority       */ CanardPriorityHigh,
				/*transfer_kind  */ CanardTransferKindMessage,
				/*port_id        */ UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID, // This is the subject-ID.
				/*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
				/*transfer_id    */ transfer_id,
				/*payload_size   */ total_size,
				/*payload        */ buffer,
			};

			++transfer_id;

			const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

			if (resp_res <= 0) {
				return false;
			}
		}
		return true;
	}
};

#endif

#ifdef UAVCAN_NODE_ROS
class CanStreamOdom : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamOdom::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ODOM";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_EQUIPMENT_ROS_ODOMETRY_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamOdom(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	IPCPull vio_sub;
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamOdom(CanStreamOdom &);
	CanStreamOdom &operator = (const CanStreamOdom &);


protected:
	explicit CanStreamOdom(CanLink *can) : CanStream(can),
		vio_sub(IPC_ID(sensor_vio)),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
		sensor_vio_s vio_info;
		if(vio_sub.update_if_changed(&vio_info)) {
			uint8_t buffer[UAVCAN_EQUIPMENT_ROS_ODOMETRY_MAX_SIZE];
        
			memset(buffer, 0, UAVCAN_EQUIPMENT_ROS_ODOMETRY_MAX_SIZE);

			uavcan_equipment_ros_Odometry vio_data;

			vio_data.timestamp.usec = vio_info.timestamp;
			
			vio_data.position[0] = vio_info.position[0];
			vio_data.position[1] = vio_info.position[1];
			vio_data.position[2] = vio_info.position[2];

			vio_data.quaternion[0] = vio_info.quat[0];
			vio_data.quaternion[1] = vio_info.quat[1];
			vio_data.quaternion[2] = vio_info.quat[2];
			vio_data.quaternion[3] = vio_info.quat[3];

			vio_data.velocity[0] = vio_info.velocity[0];
			vio_data.velocity[1] = vio_info.velocity[1];
			vio_data.velocity[2] = vio_info.velocity[2];
			
			vio_data.angular[0] = vio_info.delAng[0];
			vio_data.angular[1] = vio_info.delAng[1];
			vio_data.angular[2] = vio_info.delAng[2];

			vio_data.pos_variance[0] = vio_info.posVariance[0];
			vio_data.pos_variance[0] = vio_info.posVariance[1];
			vio_data.pos_variance[0] = vio_info.posVariance[2];

			vio_data.att_variance[0] = vio_info.attVariance[0];
			vio_data.att_variance[1] = vio_info.attVariance[1];
			vio_data.att_variance[2] = vio_info.attVariance[2];

			vio_data.vel_variance[0] = vio_info.velVariance[0];
			vio_data.vel_variance[1] = vio_info.velVariance[1];
			vio_data.vel_variance[2] = vio_info.velVariance[2];

			vio_data.ang_variance[0] = vio_info.angVariance[0];
			vio_data.ang_variance[1] = vio_info.angVariance[1];
			vio_data.ang_variance[2] = vio_info.angVariance[2];

			vio_data.pos_offset[0] = vio_info.posOffset[0];
			vio_data.pos_offset[1] = vio_info.posOffset[1];
			vio_data.pos_offset[2] = vio_info.posOffset[2];

			vio_data.deltTime = vio_info.delTime;

			const uint32_t total_size = uavcan_equipment_ros_Odometry_encode(&vio_data, buffer);

			const CanardTransfer transfer = {
				/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
				/*priority       */ CanardPriorityHigh,
				/*transfer_kind  */ CanardTransferKindMessage,
				/*port_id        */ UAVCAN_EQUIPMENT_ROS_ODOMETRY_ID, // This is the subject-ID.
				/*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
				/*transfer_id    */ transfer_id,
				/*payload_size   */ total_size,
				/*payload        */ buffer,
			};

			++transfer_id;

			const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

			if (resp_res <= 0) {
				return false;
			}
		}
		return true;
	}
};

#endif

static const CANStreamListItem message_list[] = {
	CANStreamListItem(&CanStreamStatus::new_instance,         &CanStreamStatus::get_name_static,           &CanStreamStatus::get_id_static),
	CANStreamListItem(&CanStreamSyncReq::new_instance,        &CanStreamSyncReq::get_name_static,          &CanStreamSyncReq::get_id_static),
#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)
	CANStreamListItem(&CanStreamGPS::new_instance,            &CanStreamGPS::get_name_static,              &CanStreamGPS::get_id_static),
	CANStreamListItem(&CanStreamMAG::new_instance,            &CanStreamMAG::get_name_static,              &CanStreamMAG::get_id_static),
#endif
#ifdef UAVCAN_NODE_PMU
	CANStreamListItem(&CanStreamPower::new_instance,          &CanStreamPower::get_name_static,            &CanStreamPower::get_id_static),
	CANStreamListItem(&CanStreamRng::new_instance,            &CanStreamRng::get_name_static,              &CanStreamRng::get_id_static),
#endif
#ifdef UAVCAN_NODE_ROS
	CANStreamListItem(&CanStreamOdom::new_instance,           &CanStreamOdom::get_name_static,             &CanStreamOdom::get_id_static),
#endif
};

const char *get_message_name(const uint32_t link_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &link : message_list) {
		if (link_id == link.get_id()) {
			return link.get_name();
		}
	}

	return nullptr;
}

CanStream *create_message_stream(const char *link_name, CanLink *can)
{
	// search for stream with specified name in supported streams list
	if (link_name != nullptr) {
		for (const auto &link : message_list) {
			if (strcmp(link_name, link.get_name()) == 0) {
				return link.new_instance(can);
			}
		}
	}

	return nullptr;
}
