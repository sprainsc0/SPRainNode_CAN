#ifndef __SENSOR_MAG_H__
#define __SENSOR_MAG_H__

#include <uPerf.h>
#include <inttypes.h>
#include "cmsis_os.h"
#include <ipcpush.h>
#include <ipcpull.h>
#include <param.h>
#include <buildin_function.h>

#include <topics/sensor_mag.h>
#include <topics/parameter_update.h>

class Compass
{
public:
    Compass(void);
    ~Compass(void);
    
    bool init(void);
    void run(void);
    void parameter_update(bool force);

    void caculate_publish();

protected:
    osThreadId_t _mag_handle;

private:
    static constexpr uint32_t _magDelay_ms = 60;

    // parameter 
    struct {
        param_t ext_rotate;
    } _params_handles;

    struct {
        int _ext_rotate;
    } _config;
    
    float _mag_x_accum;
    float _mag_y_accum;
    float _mag_z_accum;
    uint16_t _accum_count;
    
    bool _initialised;
    bool _healthy;
    
    IPCPull _param_sub;
    IPCPush _mag_pub;
    struct sensor_mag_s mag_raw;
    
    perf_counter_t pref_mag_elapsed;
    perf_counter_t pref_mag_interval;
    perf_counter_t pref_mag_error;
};

#endif