#ifndef I2C_EX_H
#define I2C_EX_H

#ifdef __cplusplus
extern "C"{
#endif

#include "datatypes.h"
#include "cmsis_os.h"

void hal_i2c1_init(void);
uint32_t hal_i2c1_mem_write(uint8_t dev_addres, uint8_t reg_addres, void *data, uint32_t size);
uint32_t hal_i2c1_mem_read(uint8_t dev_addres, uint8_t reg_addres, void *data, uint32_t size);
uint32_t hal_i2c1_write(uint8_t dev_addres, void *data, uint32_t size);
uint32_t hal_i2c1_read(uint8_t dev_addres, void *data, uint32_t size);

#ifdef __cplusplus
}
#endif
#endif

