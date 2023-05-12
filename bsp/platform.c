#include "platform.h"
#include "serial.h"
#include "i2c.h"
#include "spi.h"

extern TIM_HandleTypeDef htim2;

void platform_init(void)
{
	// osDelay(1000);

	// micros timer startup
    HAL_TIM_Base_Start(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	hal_uart1_init();
	hal_uart2_init();
	hal_i2c1_init();
}
