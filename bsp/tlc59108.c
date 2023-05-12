#include "tlc59108.h"
#include <stdint.h>
#include "delay.h"
#include "i2c.h"

static uint8_t reg_write(uint8_t reg, uint8_t data)
{
    uint8_t msg[1] = { 0x00 };
	msg[0] = ((reg & 0xe0) | (data & 0x1f));

	return hal_i2c1_write(TLC59108_I2C_ADDR, &msg[0], 1);
}

static uint8_t ist8310_prob(void)
{
	for(uint8_t i=0; i<8; i++) {
        if(reg_write(NCP5623_LED_CURRENT, 0x00) != 0) {
            return 1;
        }
        osDelay(40);
    }

	return 0;
}

uint8_t tlc59108_init(void)
{
	if(ist8310_prob() == 0) {
		return 0;
	}
	return 1;
}


uint8_t tlc59108_write(struct tlc59108_data *raw)
{
	uint8_t msg[7] = {0x20, 0x70, 0x40, 0x70, 0x60, 0x70, 0x80};
	uint8_t brightness = 0x1f * 1.0f;

	msg[0] = NCP5623_LED_CURRENT | (brightness & 0x1f);
	msg[2] = NCP5623_LED_PWM0 | ((uint8_t)(raw->r * raw->brightness) & 0x1f);
	msg[4] = NCP5623_LED_PWM1 | ((uint8_t)(raw->g * raw->brightness) & 0x1f);
	msg[6] = NCP5623_LED_PWM2 | ((uint8_t)(raw->b * raw->brightness) & 0x1f);

	return hal_i2c1_write(NCP5623C_I2C_ADDR, &msg[0], 7);
}
