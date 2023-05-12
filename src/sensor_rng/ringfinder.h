#ifndef __SENSOR_RNG_H__
#define __SENSOR_RNG_H__

#include <uPerf.h>
#include <inttypes.h>
#include "cmsis_os.h"
#include <ipcpush.h>
#include <ipcpull.h>
#include <param.h>
#include <buildin_function.h>

#include <topics/sensor_distance.h>
#include <topics/parameter_update.h>

class RingFinder
{
public:
    RingFinder(void);
    ~RingFinder(void);
    
    bool init(void);
    void run(void);
    void parameter_update(bool force);

protected:
    osThreadId_t _rng_handle;

private:
    static constexpr uint32_t _rngDelay_ms = 60;

    // parameter 
    struct {
        param_t pos_offset_x;
        param_t pos_offset_y;
        param_t pos_offset_z;
        param_t rng_max;
        param_t rng_min;
    } _params_handles;

    struct {
        float _pos_offset_x;
        float _pos_offset_y;
        float _pos_offset_z;
        float _rng_max;
        float _rng_min;
    } _config;

    bool _take_sample;
    
    IPCPush _rng_pub;
    IPCPull _param_sub;
    struct sensor_distance_s rng_raw;
    
    perf_counter_t pref_rng_elapsed;
    perf_counter_t pref_rng_interval;
};

#endif