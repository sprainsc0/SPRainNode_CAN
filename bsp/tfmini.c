#include "tfmini.h"
#include <stdint.h>
#include "delay.h"
#include "i2c.h"

static uint8_t tfmini_prob(void)
{
	if(tfmini_collect() == 0) {
        return 0;
    }

	return 1;
}

uint8_t tfmini_init(void)
{
	if(tfmini_prob() == 0) {
		return 0;
	}

	return 1;
}

uint8_t tfmini_collect(void)
{
    uint8_t send_buf[5] = {0x5A, 0x05, 0x00, 0x07, 0x00};
    
	return hal_i2c1_write(TFMINI_I2C_ADDR, send_buf, 5);
}

uint8_t tfmini_read(struct tfmini_data *raw)
{
	uint8_t recv_buf[11];
    uint8_t sum = 0;

    if(hal_i2c1_read(TFMINI_I2C_ADDR, recv_buf, 11) == 0) {
		return 0;
	}

    for(uint8_t i=0;i<10;i++) {
        sum += recv_buf[i];
    }

    if(sum == 0) {
        return 0;
    }

    if(recv_buf[10] == sum) {
        raw->distance = (uint16_t)recv_buf[2] + (((uint16_t)recv_buf[3]) << 8);
        raw->quality  = (uint16_t)recv_buf[4] + (((uint16_t)recv_buf[5]) << 8);
    } else {
        return 0;
    }
    
    return 1;
}
