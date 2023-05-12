#include "ist8310.h"
#include <stdint.h>
#include "delay.h"
#include "i2c.h"

static uint8_t reg_read(uint8_t reg)
{
	uint8_t data = 0;
	hal_i2c1_mem_read(IST8310_I2C_ADDR, reg, &data, 1);

	return data;
}

static uint8_t reg_write(uint8_t reg, uint8_t data)
{
	return hal_i2c1_mem_write(IST8310_I2C_ADDR, reg, &data, 1);
}

static uint8_t ist8310_prob(void)
{
	uint8_t whoami;

	for(uint8_t i=0; i<8; i++) {
        whoami = reg_read(WAI_REG);
        if(whoami == DEVICE_ID) {
            return 1;
        }
        osDelay(40);
    }

	return 0;
}

uint8_t ist8310_init(void)
{
	if(ist8310_prob() == 0) {
		return 0;
	}

	uint8_t reset_count = 0;

    for (; reset_count < 5; reset_count++) {
        if (reg_write(CNTL2_REG, CNTL2_VAL_SRST) == 0) {
            osDelay(10);
            continue;
        }

        osDelay(10);

        uint8_t cntl2 = reg_read(CNTL2_REG);
        if ((cntl2 & 0x01) == 0) {
            break;
        }
    }

    if (reset_count == 5) {
        return 0;
    }

    if ((reg_write(AVGCNTL_REG, AVGCNTL_VAL_Y_16 | AVGCNTL_VAL_XZ_16) == 0) ||
        (reg_write(PDCNTL_REG, PDCNTL_VAL_PULSE_DURATION_NORMAL) == 0)) {
        return 0;
    }

    //start conversion
    reg_write(CNTL1_REG, CNTL1_VAL_SINGLE_MEASUREMENT_MODE);

	return 1;
}

uint8_t ist8310_collect(void)
{
	return reg_write(CNTL1_REG, CNTL1_VAL_SINGLE_MEASUREMENT_MODE);
}

uint8_t ist8310_read(struct ist8310_data *raw)
{
	uint8_t recv_data[6];

    if(hal_i2c1_mem_read(IST8310_I2C_ADDR, OUTPUT_X_L_REG, recv_data, 6) == 0) {
		return 0;
	}

    int16_t raw_magy = ((short)(recv_data[1] << 8) | recv_data[0]);
    int16_t raw_magx = ((short)(recv_data[3] << 8) | recv_data[2]);
    int16_t raw_magz = ((short)(recv_data[5] << 8) | recv_data[4]);

    if (raw_magx > IST8310_MAX_VAL_XY || raw_magx < IST8310_MIN_VAL_XY ||
	    raw_magy > IST8310_MAX_VAL_XY || raw_magy < IST8310_MIN_VAL_XY ||
	    raw_magz > IST8310_MAX_VAL_Z  || raw_magz < IST8310_MIN_VAL_Z) {
		return 0;
	}

    if (raw_magx == 0 && raw_magy == 0 && raw_magz == 0) {
		return 0;
	}

    raw->magy = (float)(raw_magy * IST8310_SCALE);
    raw->magx = (float)(raw_magx * IST8310_SCALE);
    raw->magz = (float)(raw_magz * IST8310_SCALE);
    
    return 1;
}
