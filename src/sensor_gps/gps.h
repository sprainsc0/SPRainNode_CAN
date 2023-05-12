#pragma once

#include <uPerf.h>
#include <inttypes.h>
#include "cmsis_os.h"
#include <ipcpush.h>
#include <ipcpull.h>
#include <param.h>
#include <buildin_function.h>

#include <topics/sensor_gps.h>
#include <topics/satellite_info.h>
#include <topics/gps_dump.h>
#include <topics/gps_inject_data.h>
#include <topics/actuator_notify.h>
#include <topics/parameter_update.h>

#include "device/gps_helper.h"
#include "device/ubx.h"

class GPS
{
public:
    GPS(void);
    ~GPS(void);

    /// Startup initialisation.
    bool init(void);
    void run(void);

    int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);
    void parameter_update(bool force);

protected:
    osThreadId_t _gps_handle;

    static int callback(GPSCallbackType type, void *data1, int data2, void *user);

private:
    const uint32_t               _gps_lag_ms = 120;
    const bool                   _push_satellite_info = false;
    const unsigned	             _configured_baudrate = 0;
	const GPSHelper::Interface	 _interface = (GPSHelper::Interface)0;
    const bool                   _should_dump_communication = false;
    const int32_t                _gps_ubx_dynmodel = 7;  // value 2 stationary
                                                         // value 4 automotive
                                                         // value 6 airborne with <1g acceleration
                                                         // value 7 airborne with <2g acceleration
                                                         // value 8 airborne with <4g acceleration

    // parameter 
    struct {
        param_t pos_offset_x;
        param_t pos_offset_y;
        param_t pos_offset_z;
    } _params_handles;

    struct {
        float _pos_offset_x;
        float _pos_offset_y;
        float _pos_offset_z;
    } _config;

    unsigned			    _baudrate = 0;
    bool                    _gps_detected_ok;

	sensor_gps_s			_report_gps_pos{};				///< uORB topic for gps position
	satellite_info_s		_p_report_sat_info{};			///< pointer to uORB topic for satellite info
    gps_dump_s              _dump_to_device;
    gps_dump_s              _dump_from_device;

    IPCPush                _gps_pub;
    IPCPush                _sat_pub;
    IPCPull                _inject_sub;
    IPCPush                _inject_pub;
    IPCPush                _led_notify;
    IPCPull                _param_sub;

    float				_rate{0.0f};					        ///< position update rate
	float				_rate_rtcm_injection{0.0f};			    ///< RTCM message injection rate
	unsigned			_last_rate_rtcm_injection_count{0};		///< counter for number of RTCM messages

    void handleInjectDataTopic();
    bool injectData(uint8_t *data, size_t len);
    void dumpGpsData(uint8_t *data, size_t len, bool msg_to_gps_device);
    int setBaudrate(unsigned baud);
    void gpc_configure(void);

    perf_counter_t pref_gps_interval;
    perf_counter_t pref_gps_elapsed;
};
