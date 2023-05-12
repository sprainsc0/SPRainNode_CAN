#ifndef UART_EX_H
#define UART_EX_H

#ifdef __cplusplus
extern "C"{
#endif

#include "datatypes.h"
#include "cmsis_os.h"

void hal_uart1_init(void);
void hal_uart1_setbaud(uint32_t baud);
uint32_t hal_uart1_write(void *data, uint32_t size);
uint32_t hal_uart1_read(void *data, uint32_t size);
uint32_t hal_uart1_valid(void);
uint32_t hal_uart1_poll(uint8_t timeout);
void hal_uart1_flush(void);
void hal_uart1_irq();

void hal_uart2_init(void);
void hal_uart2_setbaud(uint32_t baud);
uint32_t hal_uart2_write(void *data, uint32_t size);
uint32_t hal_uart2_read(void *data, uint32_t size);
uint32_t hal_uart2_valid(void);
void hal_uart2_irq();
#ifdef __cplusplus
}
#endif
#endif

