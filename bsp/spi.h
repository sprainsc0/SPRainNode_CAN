#ifndef SPI_EX_H
#define SPI_EX_H

#ifdef __cplusplus
extern "C"{
#endif

#include "datatypes.h"
#include "cmsis_os.h"

void hal_spi1_init(void);
uint32_t hal_spi1_transmit(const void *tx_data, void * rx_data, uint32_t size);
uint32_t hal_spi1_write(const void *tx_data, uint32_t size);
uint32_t hal_spi1_read(void * rx_data, uint32_t size);

#ifdef __cplusplus
}
#endif
#endif

