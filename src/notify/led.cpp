#include "led.h"
#include <ncp5623c.h>
#include "main.h"
#include <string>
#include "hrt_timer.h"
#include "buildin_function.h"
#include "debug.h"

const osThreadAttr_t led_attributes = {
		.name = "led",
		.priority = (osPriority_t)osPriorityLow7,
		.stack_size = 512};

namespace Notify
{
static LedNotify	*gNotify;
}

static void led_func(LedNotify *pThis)
{
    pThis->run();
}

LedNotify::LedNotify(void):
    led_status(LED_PATTERN_BGC_FAST),
    current_group(0),
    current_priority(0),
    blink_count(0),
    lock_statu(0)
{
    _r_led = 0xFF;
    _g_led = 0;
    _b_led = 0;
}

bool LedNotify::init(void)
{
    led_notify_sub = ipc_subscibe(IPC_ID(actuator_notify));

    pref_led_elapsed  = perf_alloc(PC_ELAPSED,  "led_ela");
    pref_led_interval = perf_alloc(PC_INTERVAL, "led_int");
    
    _handle = osThreadNew((osThreadFunc_t)led_func, this, &led_attributes);

    if (_handle != nullptr) {
        return true;
    }

	return false;
}

void LedNotify::run(void)
{
    static uint64_t start_time = micros();

    while (1)
    {
        bool led_stat = !(led_status & (1 << blink_count));
        bool updated = false;

        const uint32_t ts_ms = millis_sync();

        perf_count(pref_led_interval);
        perf_begin(pref_led_elapsed);
        
        ipc_check(led_notify_sub, &updated);
        if(updated) {
            ipc_pull(IPC_ID(actuator_notify), led_notify_sub, &led_struct);
            if(!lock_statu || (led_struct.group == current_group)) {
                led_status       = led_struct.led_status;
                current_priority = led_struct.priority;
                current_group    = led_struct.group;
                lock_statu       = led_struct.lock;
                _r_led = (led_struct.rgb >> 16) & 0xFF;
                _g_led = (led_struct.rgb >> 8) & 0xFF;
                _b_led = led_struct.rgb & 0xFF;
            }
        }
        
        blink_count++;

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_stat ? GPIO_PIN_SET : GPIO_PIN_RESET);
#if defined(UAVCAN_NODE_GPS1)
        if(!rgb_detected && ((micros() - start_time) > 1000000) && ((micros() - start_time) < 3000000)) {
            rgb_detected = ncp5623_init();
            if(rgb_detected) {
                Info_Debug("RGB Led detected\n");
            }
            osDelay(100);
            continue;
        }
        
        if(rgb_detected && (last_led_stat != led_stat)) {
            struct ncp5623_data data;
            if(!led_stat) {
                data.r = _r_led;
                data.g = _g_led;
                data.b = _b_led;
                data.brightness = 0.8f;
                ncp5623_write(&data);
            } else {
                data.r = 0;
                data.g = 0;
                data.b = 0;
                data.brightness = 0.0f;
                ncp5623_write(&data);
            }
        }
        
        last_led_stat = led_stat;
#endif

        if (blink_count > 15) {
            blink_count = 0;
        }

        perf_end(pref_led_elapsed);
        // 10Hz loop
        osDelay(100);
    }
}

int notify_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error\n");
		return 1;
	}
    for(int i=0; i<argc; i++) {
        if (!strcmp(argv[i], "start")) {

            if (Notify::gNotify != nullptr) {
                Info_Debug("already running\n");
                return 0;
            }

            Notify::gNotify = new LedNotify();
            

            if (Notify::gNotify == NULL) {
                Info_Debug("alloc failed\n");
                return 0;
            }
            Notify::gNotify->init();
        }
    }
    return 1;
}
