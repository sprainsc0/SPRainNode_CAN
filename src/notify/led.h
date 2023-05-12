#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>
#include "cmsis_os.h"
#include "ipccore.h"
#include "uPerf.h"

#include "topics/actuator_notify.h"

class LedNotify
{
public:
    LedNotify(void);
    
    void run(void);

    bool init(void);

protected:
    osThreadId _handle;
private:
    
    int led_notify_sub;
    struct actuator_notify_s led_struct;

    uint8_t rgb_detected;

    bool last_led_stat;

    uint8_t _r_led;
    uint8_t _g_led;
    uint8_t _b_led;
    
    uint32_t led_status;
    uint8_t current_group;
    uint8_t current_priority;
    uint8_t lock_statu;
    uint8_t  blink_count;

    perf_counter_t pref_led_elapsed;
    perf_counter_t pref_led_interval;
};

#endif
