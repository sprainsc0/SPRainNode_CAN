#ifndef TFMINI_H
#define TFMINI_H

#ifdef __cplusplus
extern "C"{
#endif

#include "cmsis_os.h"
#include "i2c.h"
#include "datatypes.h"

#define TFMINI_I2C_ADDR       0x10

struct tfmini_data {
      uint16_t		distance;
      uint16_t		quality;
};

uint8_t tfmini_init(void);
uint8_t tfmini_read(struct tfmini_data *raw);
uint8_t tfmini_collect(void);

#ifdef __cplusplus
}
#endif

#endif
