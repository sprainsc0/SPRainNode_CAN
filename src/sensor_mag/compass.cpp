#include "compass.h"
#include "debug.h"
#include <ist8310.h>
#include <string>
#include <hrt_timer.h>
#include <rotation.h>
#include <buildin_function.h>

#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)

const osThreadAttr_t mag_attributes = {
    .name = "mag",
    .priority = (osPriority_t)osPriorityHigh1,
    .stack_size = 512};

namespace Sensors_Mag
{
static Compass	*gMag;
}

static void mag_func(Compass *pThis)
{
    pThis->run();
}

Compass::Compass():
    _mag_pub(IPC_ID(sensor_mag)),
    _param_sub(IPC_ID(parameter_update)),
    _initialised(false),
    _healthy(true),
    _mag_x_accum(0.0f),
    _mag_y_accum(0.0f),
    _mag_z_accum(0.0f),
    _accum_count(0)
{
    
}

Compass::~Compass()
{
    perf_free(pref_mag_elapsed);
    perf_free(pref_mag_interval);
    perf_free(pref_mag_error);
}

void Compass::caculate_publish()
{
    if (_accum_count == 0) {
        return;
    }
    float field[3];

    field[0] = _mag_x_accum / _accum_count;
    field[1] = _mag_y_accum / _accum_count;
    field[2] = _mag_z_accum / _accum_count;

    _accum_count = 0;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    rotate_3f((enum Rotation)_config._ext_rotate, &field[0], &field[1], &field[2]);
    
    mag_raw.timestamp = micros_sync();
    mag_raw.lag_ms  = _magDelay_ms;
    mag_raw.healthy = _healthy;
    
    mag_raw.learn_offsets = false;
    mag_raw.field_raw[0] = field[0];
    mag_raw.field_raw[1] = field[1];
    mag_raw.field_raw[2] = field[2];

    mag_raw.covariance = 0.02f;
    
    _mag_pub.push(&mag_raw);
}

bool Compass::init(void)
{
    if(ist8310_init() == 0) {
        return false;
    }

    _params_handles.ext_rotate = param_find("EXT_ROTAT");

    pref_mag_elapsed  = perf_alloc(PC_ELAPSED, "mag_ela");
    pref_mag_interval = perf_alloc(PC_INTERVAL, "mag_int");
    pref_mag_error    = perf_alloc(PC_COUNT, "mag_err");

    _mag_handle = osThreadNew((osThreadFunc_t)mag_func, this, &mag_attributes);

    if(_mag_handle == nullptr) {
        return false;
    }

    return true;
}

void Compass::parameter_update(bool force)
{
    parameter_update_s data;
    if(force || _param_sub.update_if_changed(&data)) {
        param_get(_params_handles.ext_rotate, &_config._ext_rotate);
    }
}

void Compass::run(void)
{
    static uint64_t _last_publish_us = micros();

    parameter_update(true);

    while (true)
    {
        const uint64_t ts = micros();

        struct ist8310_data data;

        perf_begin(pref_mag_elapsed);

        if(!_healthy) {
            continue;
        }

        parameter_update(false);

        uint8_t ret = ist8310_read(&data);
        ist8310_collect();

        if (ret == 0) {
            perf_count(pref_mag_error);
            if(perf_event_count(pref_mag_error) > 20) {
                _healthy = false;
            }
            osDelay(100);
            continue;
        } else {
            _mag_x_accum += data.magx;
            _mag_y_accum += data.magy;
            _mag_z_accum += data.magz;
            _accum_count++;
            perf_count(pref_mag_interval);
        }
        
        if((ts - _last_publish_us) > 95000) {
            caculate_publish();
            _last_publish_us = ts;
        }
        perf_end(pref_mag_elapsed);
        // 100Hz loop
        osDelay(10);
    }
}


/* ---------------------------------------namespace define----------------------------------------- */
int mag_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error");
		return 0;
	}
    for(int i=0; i<argc; i++) {
        const char *operate = argv[i];

        if (!strcmp(operate, "start")) {
            if (Sensors_Mag::gMag != nullptr) {
                Info_Debug("already running");
                return 0;
            }

            Sensors_Mag::gMag = new Compass();

            if (Sensors_Mag::gMag == nullptr) {
                Info_Debug("alloc failed");
                return 0;
            }
            if(!Sensors_Mag::gMag->init()) {
                delete Sensors_Mag::gMag;
                Sensors_Mag::gMag = nullptr;
                Info_Debug("mag init failed \n");
                return 0;
            }
        }
    }
    return 1;
}

#endif
