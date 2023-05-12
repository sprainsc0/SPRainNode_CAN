#ifndef IST8310_H
#define IST8310_H

#ifdef __cplusplus
extern "C"{
#endif

#include "cmsis_os.h"
#include "i2c.h"
#include "datatypes.h"

#define IST8310_I2C_ADDR       0x0E

#define WAI_REG 0x0
#define DEVICE_ID 0x10

#define OUTPUT_X_L_REG 0x3
#define OUTPUT_X_H_REG 0x4
#define OUTPUT_Y_L_REG 0x5
#define OUTPUT_Y_H_REG 0x6
#define OUTPUT_Z_L_REG 0x7
#define OUTPUT_Z_H_REG 0x8

#define CNTL1_REG 0xA
#define CNTL1_VAL_SINGLE_MEASUREMENT_MODE 0x1

#define CNTL2_REG 0xB
#define CNTL2_VAL_SRST 1

#define AVGCNTL_REG 0x41
#define AVGCNTL_VAL_XZ_0  (0)
#define AVGCNTL_VAL_XZ_2  (1)
#define AVGCNTL_VAL_XZ_4  (2)
#define AVGCNTL_VAL_XZ_8  (3)
#define AVGCNTL_VAL_XZ_16 (4)
#define AVGCNTL_VAL_Y_0  (0 << 3)
#define AVGCNTL_VAL_Y_2  (1 << 3)
#define AVGCNTL_VAL_Y_4  (2 << 3)
#define AVGCNTL_VAL_Y_8  (3 << 3)
#define AVGCNTL_VAL_Y_16 (4 << 3)

#define PDCNTL_REG 0x42
#define PDCNTL_VAL_PULSE_DURATION_NORMAL 0xC0

/*
 * FSR:
 *   x, y: +- 1600 µT
 *   z:    +- 2500 µT
 *
 * Resolution according to datasheet is 0.3µT/LSB
 */
#define IST8310_RESOLUTION 0.3f
#define IST8310_SCALE      3.0f

static const int16_t IST8310_MAX_VAL_XY = (int16_t)((1600 / IST8310_RESOLUTION) + 1);
static const int16_t IST8310_MIN_VAL_XY = -(int16_t)((1600 / IST8310_RESOLUTION) + 1);
static const int16_t IST8310_MAX_VAL_Z  = (int16_t)((2500 / IST8310_RESOLUTION) + 1);
static const int16_t IST8310_MIN_VAL_Z  = -(int16_t)((2500 / IST8310_RESOLUTION) + 1);

struct ist8310_data {
      float		magx;
      float		magy;
      float		magz;
};

uint8_t ist8310_init(void);
uint8_t ist8310_read(struct ist8310_data *raw);
uint8_t ist8310_collect(void);

#ifdef __cplusplus
}
#endif

#endif
