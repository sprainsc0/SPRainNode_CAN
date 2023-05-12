#ifndef  __BSP_HRT_TIMER_H__
#define  __BSP_HRT_TIMER_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32g4xx_hal.h"

extern uint64_t micros(void);

extern uint32_t millis(void);

extern void sync_offset(int64_t ad);

extern uint64_t micros_sync(void);
extern uint32_t millis_sync(void);

#ifdef __cplusplus
}
#endif
#endif
