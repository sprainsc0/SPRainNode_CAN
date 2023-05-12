#include "hrt_timer.h"
#include "platform.h"
#include <math.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;

static uint64_t base_time = 0;
static uint32_t last_count = 0;
static int64_t  time_offset = 0;

uint64_t micros(void)
{
    __disable_irq();
    const uint32_t count = TIM2->CNT;

    if (count < last_count) {
        base_time += TIM2->ARR;
    }

    /* save the count for next time */
    last_count = count;

    const uint64_t abstime = (uint64_t)(base_time + count);
    __enable_irq();

    return abstime;
}

uint32_t millis(void)
{
    return micros()/1000;
}

uint32_t millis_sync(void)
{
    return micros_sync()/1000;
}

void sync_offset(int64_t ad)
{
    time_offset = ad;
}

uint64_t micros_sync(void)
{
    return (micros() - time_offset);
}

