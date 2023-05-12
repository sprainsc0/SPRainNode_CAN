#include "serial.h"
#include "ringbuffer.h"
#include <semphr.h>
#include "main.h"

extern SPI_HandleTypeDef hspi1;

static SemaphoreHandle_t spi1_sem;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE spiTaskWoken;
	if(spi1_sem != NULL) {
		xSemaphoreGiveFromISR(spi1_sem, &spiTaskWoken);
		portYIELD_FROM_ISR(spiTaskWoken);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE spiTaskWoken;
	if(spi1_sem != NULL) {
		xSemaphoreGiveFromISR(spi1_sem, &spiTaskWoken);
		portYIELD_FROM_ISR(spiTaskWoken);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE spiTaskWoken;
	if(spi1_sem != NULL) {
		xSemaphoreGiveFromISR(spi1_sem, &spiTaskWoken);
		portYIELD_FROM_ISR(spiTaskWoken);
	}
}

void hal_spi1_init(void)
{
	spi1_sem = xSemaphoreCreateBinary();
}

uint32_t hal_spi1_transmit(const void *tx_data, void * rx_data, uint32_t size)
{
    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_RESET);
    
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)tx_data, (uint8_t *)rx_data, size) != HAL_OK) {
        return 0;
    }
    
    xSemaphoreTake(spi1_sem, 1000);

    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET);

	return size;
}

uint32_t hal_spi1_write(const void *tx_data, uint32_t size)
{
    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_RESET);
    
    if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)tx_data, size) != HAL_OK) {
        return 0;
    }
    
    xSemaphoreTake(spi1_sem, 1000);

    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET);

	return size;
}

uint32_t hal_spi1_read(void * rx_data, uint32_t size)
{
    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_RESET);
    
    if (HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)rx_data, size) != HAL_OK) {
        return 0;
    }
    
    xSemaphoreTake(spi1_sem, 1000);

    HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET);

	return size;
}
