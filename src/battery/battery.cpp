#include "battery.h"
#include <string>
#include "hrt_timer.h"
#include "buildin_function.h"
#include "debug.h"
#include <utils.h>

#ifdef UAVCAN_NODE_PMU

static uint16_t adc_raw[3];

extern ADC_HandleTypeDef hadc1;

const osThreadAttr_t bat_attributes = {
		.name = "bat",
		.priority = (osPriority_t)osPriorityAboveNormal,
		.stack_size = 1024};

namespace Power
{
static Battery	*gBat;
}

static void bat_func(Battery *pThis)
{
    pThis->run();
}

Battery::Battery(void):
    _refint(0),
    _calibration_ok(false),
    _bat_pub(IPC_ID(battery_status)),
    _param_sub(IPC_ID(parameter_update))
{

}

bool Battery::init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 3);

    _params_handles.res_ohm = param_find("RES_OHM");

    _handle = osThreadNew((osThreadFunc_t)bat_func, this, &bat_attributes);

    if (_handle != nullptr) {
        return true;
    }

	return false;
}

void Battery::parameter_update(bool force)
{
    parameter_update_s data;
    if(force || _param_sub.update_if_changed(&data)) {
        param_get(_params_handles.res_ohm, &_config._res_ohm);
    }
}

void Battery::current_calibration()
{
    volatile float phase_temp = 0;
    uint16_t sample_cnt = 0;

    osDelay(1000);

    while(sample_cnt < 1000) {
		if(_refint == 0) {
			osDelay(1);
			continue;
		}
        
        phase_temp += VREFINT * ((adc_raw[1]*(VREF/4096))/(_refint*(VREF/4096)));
		sample_cnt++;
        
        osDelay(1);
    }

    current_offset = phase_temp / sample_cnt;
    _calibration_ok = true;
    Info_Debug("Current calibration success\n");
}

uint16_t Battery::get_voltage(void)
{
    return adc_raw[0];
}

uint16_t Battery::get_current(void)
{
    return adc_raw[1];
}

uint16_t Battery::get_verf(void)
{
    return adc_raw[2];
}

void Battery::run(void)
{
    parameter_update(true);
    while (1)
    {
        _refint = adc_raw[2];

        parameter_update(false);

        if(_calibration_ok) {
            const float voltage = (VREFINT * ((adc_raw[0]*(VREF/4096))/(_refint*(VREF/4096)))) * ((RESISTANCE1+RESISTANCE2) / RESISTANCE2);
            const float current = ((VREFINT * ((adc_raw[1]*(VREF/4096))/(_refint*(VREF/4096)))) - current_offset) / (ADC_CURRENT_AMP*ADC_CURRENT_OHM);

            UTILS_LP_FAST(_voltage, voltage, 0.1f);
            UTILS_LP_FAST(_current, current, 0.1f);

            _bat_data.timestamp = micros_sync();
            _bat_data.voltage_v = _voltage + _config._res_ohm * _current;
            _bat_data.current_a = _current;

            _bat_pub.push(&_bat_data);
        }

        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, 3);

        // 1000Hz loop
        osDelay(1);
    }
}

int battery_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error\n");
		return 0;
	}
    for(int i=0; i<argc; i++) {
        if (!strcmp(argv[i], "start")) {

            if (Power::gBat != nullptr) {
                Info_Debug("already running\n");
                return 0;
            }

            Power::gBat = new Battery();

            if (Power::gBat == NULL) {
                Info_Debug("alloc failed\n");
                return 0;
            }
            Power::gBat->init();
        }

        if (!strcmp(argv[i], "cali")) {
            Power::gBat->current_calibration();
        }
    }
    return 1;
}

#endif
