#include <climits>
#include <buildin_function.h>
#include <hrt_timer.h>
#include <stdio.h>
#include <stdlib.h>
#include "gps.h"
extern "C" {
#include <serial.h>
}

#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)

#define MIN(a, b)        (((a) < (b)) ? (a) : (b))
#define MAX(a, b)        (((a) > (b)) ? (a) : (b))

const osThreadAttr_t gps_attributes = {
    .name = "gps",
    .priority = (osPriority_t)osPriorityRealtime3,
    .stack_size = 2048};

namespace Sensors_GPS
{
static GPS	     *gGPS;
static GPSHelper *helper;				///< instance of GPS parser
}

static void gps_func(GPS *pThis)
{
    pThis->run();
}

GPS::GPS():
    _gps_pub(IPC_ID(sensor_gps)),
    _sat_pub(IPC_ID(satellite_info)),
    _inject_sub(IPC_ID(gps_inject_data)),
    _inject_pub(IPC_ID(gps_inject_data)),
    _led_notify(IPC_ID(actuator_notify)),
    _param_sub(IPC_ID(parameter_update)),
    _gps_detected_ok(false)
{
    
}

GPS::~GPS()
{
    perf_free(pref_gps_interval);
    perf_free(pref_gps_elapsed);
}

// Startup initialisation.
bool GPS::init()
{
    _params_handles.pos_offset_x = param_find("POS_OFFX");
    _params_handles.pos_offset_y = param_find("POS_OFFY");
    _params_handles.pos_offset_z = param_find("POS_OFFZ");

    pref_gps_interval = perf_alloc(PC_INTERVAL, "gps_int");
    pref_gps_elapsed = perf_alloc(PC_ELAPSED, "gps_ela");
    
    _gps_handle = osThreadNew((osThreadFunc_t)gps_func, this, &gps_attributes);

    if(_gps_handle == nullptr) {
        return false;
    }

    return true;
}

void GPS::parameter_update(bool force)
{
    parameter_update_s data;
    if(force || _param_sub.update_if_changed(&data)) {
        param_get(_params_handles.pos_offset_x, &_config._pos_offset_x);
        param_get(_params_handles.pos_offset_y, &_config._pos_offset_y);
        param_get(_params_handles.pos_offset_z, &_config._pos_offset_z);
    }
}

void GPS::handleInjectDataTopic()
{
	bool updated = false;

	// Limit maximum number of GPS injections to 6 since usually
	// GPS injections should consist of 1-4 packets (GPS, Glonass, BeiDou, Galileo).
	// Looking at 6 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	const size_t max_num_injections = 6;
	size_t num_injections = 0;

	do {
        gps_inject_data_s msg;
		num_injections++;
        updated = _inject_sub.update_if_changed(&msg);
		if(updated){
            /* Write the message to the gps device. Note that the message could be fragmented.
                * But as we don't write anywhere else to the device during operation, we don't
                * need to assemble the message first.
                */
			injectData(msg.data, msg.len);

			++_last_rate_rtcm_injection_count;
		}
	} while (updated && num_injections < max_num_injections);
}

bool GPS::injectData(uint8_t *data, size_t len)
{
	dumpGpsData(data, len, true);

    size_t written = hal_uart1_write(data, len);

	return written == len;
}

void GPS::dumpGpsData(uint8_t *data, size_t len, bool msg_to_gps_device)
{
	if (!_should_dump_communication) {
		return;
	}

	gps_dump_s *dump_data = msg_to_gps_device ? &_dump_to_device : &_dump_from_device;

	while (len > 0) {
		size_t write_len = len;

		if (write_len > sizeof(dump_data->data) - dump_data->len) {
			write_len = sizeof(dump_data->data) - dump_data->len;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= sizeof(dump_data->data)) {
			if (msg_to_gps_device) {
				dump_data->len |= 1 << 7;
			}

			dump_data->timestamp = micros();
            _inject_pub.push(dump_data);
			dump_data->len = 0;
		}
	}
}

int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user)
{
	GPS *gps = (GPS *)user;

	switch (type) {
	case GPSCallbackType::readDeviceData: {
			int num_read = gps->pollOrRead((uint8_t *)data1, data2, *((int *)data1));

			if (num_read > 0) {
				gps->dumpGpsData((uint8_t *)data1, (size_t)num_read, false);
			}

			return num_read;
		}

	case GPSCallbackType::writeDeviceData:
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, true);

		return hal_uart1_write(data1, (size_t)data2);

	case GPSCallbackType::setBaudrate:
		return gps->setBaudrate(data2);

	case GPSCallbackType::gotRTCMMessage:
		/* not used */
		break;

	case GPSCallbackType::surveyInStatus:
		/* not used */
		break;

	case GPSCallbackType::setClock:
        /* not used */
		// clock_settime(CLOCK_REALTIME, (timespec *)data1);
		break;
	}

	return 0;
}

int GPS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	handleInjectDataTopic();

    uint32_t ret = hal_uart1_poll(timeout);
	osDelay(12);
	return hal_uart1_read(buf, buf_length);

}

int GPS::setBaudrate(unsigned baud)
{
    hal_uart1_setbaud(baud);
	return 0;
}

void GPS::gpc_configure(void)
{
    uint8_t retry = 10;
    _baudrate = _configured_baudrate;

    if(Sensors_GPS::helper == nullptr) {
        return;
    }

    while(retry--) {
        if (!_gps_detected_ok && Sensors_GPS::helper->configure(_baudrate, GPSHelper::OutputMode::GPS) == 0) {
            memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));
            _report_gps_pos.heading = NAN;
            _report_gps_pos.heading_offset = 0.0f;

            Sensors_GPS::helper->resetUpdateRates();

            _gps_detected_ok = true;
            return;
        }
        Info_Debug("GPS Dected retry %d \n", retry);
    }
}

void GPS::run(void)
{
    struct actuator_notify_s led;

    Sensors_GPS::helper = new GPSDriverUBX(_interface, &GPS::callback, this, &_report_gps_pos, &_p_report_sat_info, _gps_ubx_dynmodel);
    
    gpc_configure();

    parameter_update(true);

    if(_gps_detected_ok) {
        Info_Debug("GPS Dected success \n");
    } else {
        led.timestamp  = micros();
        led.lock       = NOTIFY_LOCK;
        led.led_status = LED_PATTERN_BGC_FAST;
        led.group      = NOTIFY_GROUP_CHECK;
	    led.priority   = NOTIFY_PRIORITY_VERY_HIGH;
	    led.rgb        = NOTIFY_RGB_BLUE;
        _led_notify.push(&led);
        Info_Debug("GPS Dected error \n");
    }

    uint64_t last_rate_measurement = micros();
	unsigned last_rate_count = 0;

    while (true)
    {
        const uint64_t ts = micros();

        perf_begin(pref_gps_elapsed);

        parameter_update(false);

        int helper_ret = Sensors_GPS::helper->receive(500); // 5Hz

        if (helper_ret > 0) {
            perf_count(pref_gps_interval);
            //hal_uart1_flush();
            if (helper_ret & 1) {
                _report_gps_pos.instance = 0;
                _report_gps_pos.last_message_time_ms = _report_gps_pos.timestamp / 1000;
                _report_gps_pos.healthy = true;
                _report_gps_pos.gps_lag_ms = _gps_lag_ms;
                _report_gps_pos.posOffset[0] = _config._pos_offset_x;
                _report_gps_pos.posOffset[1] = _config._pos_offset_y;
                _report_gps_pos.posOffset[2] = _config._pos_offset_z;
                _gps_pub.push(&_report_gps_pos);
                last_rate_count++;
            }

            if (_push_satellite_info && (helper_ret & 2)) {
                _sat_pub.push(&_p_report_sat_info);
            }
        }

        /* measure update rate every 5 seconds */
        if (ts - last_rate_measurement > 5000000) {
            float dt = (float)((ts - last_rate_measurement)) / 1000000.0f;
            _rate = last_rate_count / dt;
            _rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
            last_rate_measurement = ts;
            last_rate_count = 0;
            _last_rate_rtcm_injection_count = 0;
            Sensors_GPS::helper->storeUpdateRates();
            Sensors_GPS::helper->resetUpdateRates();
//            Info_Debug("%d.%d \n", (int)(_rate*10)/10, (int)(_rate*10)%10);
        }

        perf_end(pref_gps_elapsed);
    }
}

/* ---------------------------------------namespace define----------------------------------------- */
int gps_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error");
		return 0;
	}

    for(int i=0; i<argc; i++) {
        const char *operate = argv[i];

        if (!strcmp(operate, "start")) {
            if (Sensors_GPS::gGPS != nullptr) {
                Info_Debug("already running");
                return 0;
            }

            Sensors_GPS::gGPS = new GPS();

            if (Sensors_GPS::gGPS == nullptr) {
                Info_Debug("alloc failed");
                return 0;
            }
            if(!Sensors_GPS::gGPS->init()) {
                delete Sensors_GPS::gGPS;
                Sensors_GPS::gGPS = nullptr;
                Info_Debug("gps init failed \n");
                return 0;
            }
        }
    }
    return 1;
}

#endif