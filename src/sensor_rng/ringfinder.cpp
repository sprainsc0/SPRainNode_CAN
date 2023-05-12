#include "ringfinder.h"
#include "debug.h"
#include <tfmini.h>
#include <string>
#include <hrt_timer.h>
#include <buildin_function.h>

#ifdef UAVCAN_NODE_PMU

const osThreadAttr_t rng_attributes = {
    .name = "rng",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 512};

namespace Sensors_Rng
{
static RingFinder	*gRng;
}

static void rng_func(RingFinder *pThis)
{
    pThis->run();
}

RingFinder::RingFinder():
    _rng_pub(IPC_ID(sensor_distance)),
    _param_sub(IPC_ID(parameter_update)),
    _take_sample(false)
{
    
}

RingFinder::~RingFinder()
{
    perf_free(pref_rng_elapsed);
    perf_free(pref_rng_interval);
}


bool RingFinder::init(void)
{
    if(tfmini_init() == 0) {
        return false;
    }

    _params_handles.pos_offset_x = param_find("POS_OFFX");
    _params_handles.pos_offset_y = param_find("POS_OFFY");
    _params_handles.pos_offset_z = param_find("POS_OFFZ");

    _params_handles.rng_max = param_find("RNG_MAX");
    _params_handles.rng_min = param_find("RNG_MIN");

    pref_rng_elapsed  = perf_alloc(PC_ELAPSED,  "rng_ela");
    pref_rng_interval = perf_alloc(PC_INTERVAL, "rng_int");

    _rng_handle = osThreadNew((osThreadFunc_t)rng_func, this, &rng_attributes);

    if(_rng_handle == nullptr) {
        return false;
    }

    return true;
}

void RingFinder::parameter_update(bool force)
{
    parameter_update_s data;
    if(force || _param_sub.update_if_changed(&data)) {
        param_get(_params_handles.pos_offset_x, &_config._pos_offset_x);
        param_get(_params_handles.pos_offset_y, &_config._pos_offset_y);
        param_get(_params_handles.pos_offset_z, &_config._pos_offset_z);
        param_get(_params_handles.rng_max, &_config._rng_max);
        param_get(_params_handles.rng_min, &_config._rng_min);
    }
}

void RingFinder::run(void)
{
    parameter_update(true);
    while (true)
    {
        const uint64_t ts = micros();

        struct tfmini_data data;

        perf_count(pref_rng_interval);
        perf_begin(pref_rng_elapsed);

        parameter_update(false);

        if(_take_sample) {
            tfmini_collect();
        } else {
            uint8_t ret = tfmini_read(&data);
            if (ret != 0) {
                rng_raw.timestamp        = micros_sync(); // required for logger
                rng_raw.lag_ms           = _rngDelay_ms;
                rng_raw.min_distance     = _config._rng_min;
                rng_raw.max_distance     = _config._rng_max;
                rng_raw.current_distance = data.distance/100.0f;
                rng_raw.quality          = data.quality;
                rng_raw.temp             = 25.0f;
                rng_raw.posOffset[0]     = _config._pos_offset_x;
                rng_raw.posOffset[1]     = _config._pos_offset_y;
                rng_raw.posOffset[2]     = _config._pos_offset_z;
                _rng_pub.push(&rng_raw);
            } 
        }
        
        _take_sample = !_take_sample;
        
        perf_end(pref_rng_elapsed);
        // 40Hz loop
        osDelay(25);
    }
}


/* ---------------------------------------namespace define----------------------------------------- */
int rng_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error");
		return 0;
	}
    for(int i=0; i<argc; i++) {
        const char *operate = argv[i];

        if (!strcmp(operate, "start")) {
            if (Sensors_Rng::gRng != nullptr) {
                Info_Debug("already running");
                return 0;
            }

            Sensors_Rng::gRng = new RingFinder();

            if (Sensors_Rng::gRng == nullptr) {
                Info_Debug("alloc failed");
                return 0;
            }
            if(!Sensors_Rng::gRng->init()) {
                delete Sensors_Rng::gRng;
                Sensors_Rng::gRng = nullptr;
                Info_Debug("rng init failed \n");
                return 0;
            }
        }
    }
    return 1;
}

#endif
