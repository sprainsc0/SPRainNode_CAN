#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include "mavlink_main.h"
#include "mavlink_messages.h"

#include "ipcpull.h"

#include <topics/sensor_baro.h>
#include <topics/sensor_mag.h>
#include <topics/sensor_gps.h>
#include <topics/sensor_imu.h>
#include <topics/sensor_distance.h>
#include <topics/sensor_oflow.h>
#include <topics/battery_status.h>
#include <topics/actuator_armed.h>

#ifdef UAVCAN_NODE_ROS

class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	IPCPull  *arm_sub;
	uint32_t custom_mode;
	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &);
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &);

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink),
		arm_sub(_mavlink->add_orb_subscription(IPC_ID(actuator_armed))),
		custom_mode(0)
	{}

	bool send(const uint64_t t)
	{
		uint8_t base_mode = 0;
		uint8_t system_status = 0;

		actuator_armed_s arm_data;

		if (arm_sub->update_if_changed(&arm_data)) {
			custom_mode = arm_data.armed;
		}

		mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), 0,
					   base_mode, custom_mode, system_status);

		return true;
	}
};

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	IPCPull *_gps_sub;

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &) = delete;
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &) = delete;

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(IPC_ID(sensor_gps)))
	{}

	bool send(const uint64_t t)
	{
		sensor_gps_s gps;

		if (_gps_sub->update_if_changed(&gps) && _mavlink->is_connected()) {
			mavlink_gps_raw_int_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = (int)(gps.hdop * 100);
			msg.epv = (int)(gps.vdop * 100);
			msg.h_acc = (uint32_t)(gps.eph * 1000);
			msg.v_acc = (uint32_t)(gps.epv * 1000);
			msg.vel_acc = (uint32_t)(gps.s_variance_m_s * 1000);

			msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamScaledIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SENSOR_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SCALED_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	IPCPull *_raw_mag_sub;
	IPCPull *_raw_imu_sub;

	// do not allow top copy this class
	MavlinkStreamScaledIMU(MavlinkStreamScaledIMU &) = delete;
	MavlinkStreamScaledIMU &operator = (const MavlinkStreamScaledIMU &) = delete;

protected:
	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_mag_sub(_mavlink->add_orb_subscription(IPC_ID(sensor_mag))),
		_raw_imu_sub(_mavlink->add_orb_subscription(IPC_ID(sensor_imu)))
	{}

	bool send(const uint64_t t)
	{
		struct sensor_mag_s sensor_mag = {};
		struct sensor_imu_s sensor_imu = {};

		bool updated = true;
		updated &= _raw_mag_sub->update_if_changed(&sensor_mag);
		updated &= _raw_imu_sub->update_if_changed(&sensor_imu);

		if (updated && _mavlink->is_connected()) {

			mavlink_scaled_imu_t msg = {};

			msg.time_boot_ms = sensor_mag.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_imu.accel_filter[0]*1000); 	// [milli g]
			msg.yacc = (int16_t)(sensor_imu.accel_filter[1]*1000); 	// [milli g]
			msg.zacc = (int16_t)(sensor_imu.accel_filter[2]*1000); 	// [milli g]
			msg.xgyro = (int16_t)(sensor_imu.gyro_filter[0]*1000);					// [milli rad/s]
			msg.ygyro = (int16_t)(sensor_imu.gyro_filter[1]*1000);					// [milli rad/s]
			msg.zgyro = (int16_t)(sensor_imu.gyro_filter[2]*1000);					// [milli rad/s]
			msg.xmag = (int16_t)(sensor_mag.field[0]*1000);					// [milli tesla]
			msg.ymag = (int16_t)(sensor_mag.field[1]*1000);					// [milli tesla]
			msg.zmag = (int16_t)(sensor_mag.field[2]*1000);					// [milli tesla]

			mavlink_msg_scaled_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	IPCPull *_battery_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &);
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &);

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_battery_status_sub(_mavlink->add_orb_subscription(IPC_ID(battery_status)))
	{}

	bool send(const uint64_t t)
	{
		struct battery_status_s battery_status;

		if (_battery_status_sub->update(&battery_status) && _mavlink->is_connected()) {
			mavlink_sys_status_t msg = {};

			msg.onboard_control_sensors_present = 1;
			msg.onboard_control_sensors_enabled = 1;
			msg.onboard_control_sensors_health = 1;
			msg.load = 0;
			msg.voltage_battery = (uint16_t)(battery_status.voltage_v * 1000);
			msg.current_battery = (int16_t)(battery_status.current_a * 100);
			msg.battery_remaining = battery_status.raltive_soc;
			
			msg.drop_rate_comm = 0;
			msg.errors_comm = 0;
			msg.errors_count1 = 0;
			msg.errors_count2 = 0;
			msg.errors_count3 = 0;
			msg.errors_count4 = 0;

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);
            
            /* battery status message with higher resolution */
			mavlink_battery_status_t bat_msg = {};
			bat_msg.id = 0;
			bat_msg.battery_function = 0;
			bat_msg.type = 0;
			bat_msg.current_consumed = battery_status.remaining;
			bat_msg.energy_consumed = -1;
			bat_msg.current_battery = (int16_t)(battery_status.current_a * 100);
			bat_msg.battery_remaining = battery_status.remaining;
			bat_msg.temperature = 25;

			for (unsigned int i = 0; i < (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0])); i++) {
				if ((int)i < battery_status.cell_count) {
					bat_msg.voltages[i] = (uint16_t)((battery_status.voltage_v / battery_status.cell_count) * 1000);
				} else {
					bat_msg.voltages[i] = UINT16_MAX;
				}
			}

			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

			return true;
		}

		return false;
	}
};


static const StreamListItem streams_list[] = {
	StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static, &MavlinkStreamHeartbeat::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU::new_instance, &MavlinkStreamScaledIMU::get_name_static, &MavlinkStreamScaledIMU::get_id_static),
	StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static, &MavlinkStreamSysStatus::get_id_static),
	StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static, &MavlinkStreamGPSRawInt::get_id_static)
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}
#endif