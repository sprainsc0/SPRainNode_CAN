#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <stdint.h>
#include "cmsis_os.h"
#include "ipcpush.h"
#include "ipcpull.h"
#include "uPerf.h"
#include <param.h>


#include <topics/parameter_update.h>
#include <topics/battery_status.h>
#include <topics/actuator_notify.h>

#define ADC_CURRENT_OHM		                  0.0005f
#define ADC_CURRENT_AMP		                  20.0f
#define VREFINT                               1.2f
#define VREF                                  3.3f

#define RESISTANCE1                           30.0f 
#define RESISTANCE2                           3.3f

class Battery
{
public:
    Battery(void);

    void run(void);

    bool init(void);

    void parameter_update(bool force);

    void current_calibration();
    
protected:
    osThreadId_t _handle;

private:
    uint16_t _refint;
    float _voltage;
    float _current;

    // parameter 
    struct {
        param_t res_ohm;
    } _params_handles;

    struct {
        float _res_ohm;
    } _config;

    float current_offset;
    bool _calibration_ok;
    // command subscribe
    IPCPull _param_sub;
    IPCPush _bat_pub;
    struct battery_status_s _bat_data;

    uint16_t get_verf(void);
    uint16_t get_current(void);
    uint16_t get_voltage(void);
};

#endif
