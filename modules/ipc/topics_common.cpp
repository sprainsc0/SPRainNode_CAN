#include "ipccore.h"
#include "topics/actuator_armed.h"
#include "topics/actuator_notify.h"
#include "topics/parameter_update.h"
#include "topics/command.h"
#include "topics/telemetry_status.h"
#include "topics/sensor_gps.h"
#include "topics/sensor_mag.h"
#include "topics/sensor_baro.h"
#include "topics/sensor_imu.h"
#include "topics/sensor_vio.h"
#include "topics/sensor_airspeed.h"
#include "topics/sensor_distance.h"
#include "topics/sensor_oflow.h"
#include "topics/gps_dump.h"
#include "topics/gps_inject_data.h"
#include "topics/satellite_info.h"
#include "topics/uavcan_parameter_request.h"
#include "topics/uavcan_parameter_value.h"
#include "topics/uavcan_command_request.h"
#include "topics/uavcan_command_response.h"
#include "topics/calibrate_status.h"
#include "topics/battery_status.h"
#include "topics/mavlink_log.h"

//         name                       uORB struct                      serial id   buffer   buffer
IPC_DEFINE(actuator_notify,           struct actuator_notify_s,           0,        false,    0);
IPC_DEFINE(parameter_update,          struct parameter_update_s,          1,        false,    0);
IPC_DEFINE(command,                   struct command_s,                   2,        false,    0);
IPC_DEFINE(telemetry_status,          struct telemetry_status_s,          3,        false,    0);
IPC_DEFINE(calibrate_status,          struct calibrate_status_s,          4,        false,    0);

IPC_DEFINE(sensor_gps,                struct sensor_gps_s,                5,        false,    0);
IPC_DEFINE(sensor_mag,                struct sensor_mag_s,                6,        false,    0);
IPC_DEFINE(sensor_baro,               struct sensor_baro_s,               7,        false,    0);
IPC_DEFINE(sensor_airspeed,           struct sensor_airspeed_s,           8,        false,    0);
IPC_DEFINE(sensor_distance,           struct sensor_distance_s,           9,        false,    0);
IPC_DEFINE(sensor_oflow,              struct sensor_oflow_s,              10,       false,    0);
IPC_DEFINE(gps_dump,                  struct gps_dump_s,                  11,       false,    0);
IPC_DEFINE(gps_inject_data,           struct gps_inject_data_s,           12,       false,    0);
IPC_DEFINE(satellite_info,            struct satellite_info_s,            13,       false,    0);
IPC_DEFINE(battery_status,            struct battery_status_s,            18,       false,    0);
IPC_DEFINE(mavlink_log,               struct mavlink_log_s,               19,       false,    0);
IPC_DEFINE(sensor_vio,                struct sensor_vio_s,                20,       false,    0);
IPC_DEFINE(sensor_imu,                struct sensor_imu_s,                21,       false,    0);
IPC_DEFINE(actuator_armed,            struct actuator_armed_s,            22,       false,    0);

// queue buffer for can
IPC_DEFINE(uavcan_parameter_request,  struct uavcan_parameter_request_s,  14,        true,     3);
IPC_DEFINE(uavcan_parameter_value,    struct uavcan_parameter_value_s,    15,        true,     3);
IPC_DEFINE(uavcan_command_request,    struct uavcan_command_request_s,    16,        true,     3);
IPC_DEFINE(uavcan_command_response,   struct uavcan_command_response_s,   17,        true,     3);