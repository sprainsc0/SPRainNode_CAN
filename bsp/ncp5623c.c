#include "ncp5623c.h"
#include <stdint.h>
#include "delay.h"
#include "i2c.h"

static uint8_t reg_write(uint8_t reg, uint8_t data)
{
    uint8_t msg[1] = { 0x00 };
	msg[0] = ((reg & 0xe0) | (data & 0x1f));

	return hal_i2c1_write(NCP5623C_I2C_ADDR, &msg[0], 1);
}

static uint8_t ist8310_prob(void)
{
	for(uint8_t i=0; i<3; i++) {
        if(reg_write(NCP5623_LED_CURRENT, 0x03) != 0) {
            return 1;
        }
        osDelay(40);
    }

	return 0;
}

uint8_t ncp5623_init(void)
{
	if(ist8310_prob() == 0) {
		return 0;
	}
	return 1;
}


uint8_t ncp5623_write(struct ncp5623_data *raw)
{
	uint8_t msg[5] = {0x40, 0x70, 0x60, 0x70, 0x80};

	msg[0] = NCP5623_LED_PWM0 | ((uint8_t)((float)raw->r * raw->brightness) & 0x1f);
	msg[2] = NCP5623_LED_PWM1 | ((uint8_t)((float)raw->g * raw->brightness) & 0x1f);
	msg[4] = NCP5623_LED_PWM2 | ((uint8_t)((float)raw->b * raw->brightness) & 0x1f);

	return hal_i2c1_write(NCP5623C_I2C_ADDR, &msg[0], 5);
}
