#include "i2c.h"
#include <semphr.h>

extern I2C_HandleTypeDef hi2c1;

static SemaphoreHandle_t i2c_sem;
static SemaphoreHandle_t i2c_comp;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	static portBASE_TYPE i2cTaskWoken;
	if(i2c_comp != NULL) {
		xSemaphoreGiveFromISR(i2c_comp, &i2cTaskWoken);
		portYIELD_FROM_ISR(i2cTaskWoken);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	static portBASE_TYPE i2cTaskWoken;
	if(i2c_comp != NULL) {
		xSemaphoreGiveFromISR(i2c_comp, &i2cTaskWoken);
		portYIELD_FROM_ISR(i2cTaskWoken);
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	static portBASE_TYPE i2cTaskWoken;
	if(i2c_comp != NULL) {
		xSemaphoreGiveFromISR(i2c_comp, &i2cTaskWoken);
		portYIELD_FROM_ISR(i2cTaskWoken);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	static portBASE_TYPE i2cTaskWoken;
	if(i2c_comp != NULL) {
		xSemaphoreGiveFromISR(i2c_comp, &i2cTaskWoken);
		portYIELD_FROM_ISR(i2cTaskWoken);
	}
}

void hal_i2c1_init(void)
{
	i2c_comp = xSemaphoreCreateCounting(1, 0);
	i2c_sem  = xSemaphoreCreateCounting(1, 1);
}

uint32_t hal_i2c1_mem_write(uint8_t dev_addres, uint8_t reg_addres, void *data, uint32_t size)
{
	if(i2c_sem == NULL) {
		return 0;
	}
	xSemaphoreTake(i2c_sem, 10000);

	uint32_t ret = size;

	dev_addres = (dev_addres << 1);

    if (HAL_I2C_Mem_Write_DMA(&hi2c1,
                            dev_addres,
                            reg_addres,
                            I2C_MEMADD_SIZE_8BIT,
                            (uint8_t *)data,
                            size) != HAL_OK)
    {
        ret = 0;
    }
    if(xSemaphoreTake(i2c_comp, 1000) == pdFALSE) {
		ret = 0;
	}

	xSemaphoreGive(i2c_sem);

	return ret;
}

uint32_t hal_i2c1_mem_read(uint8_t dev_addres, uint8_t reg_addres, void *data, uint32_t size)
{
	if(i2c_sem == NULL) {
		return 0;
	}
	xSemaphoreTake(i2c_sem, 10000);

	uint32_t ret = size;

	dev_addres = (dev_addres << 1) | 1;

	if (HAL_I2C_Mem_Read_DMA(&hi2c1,
							dev_addres,
							reg_addres, 
							I2C_MEMADD_SIZE_8BIT, 
							(uint8_t *)data, 
							size) != HAL_OK)
    {
        ret = 0;
    }

	if(xSemaphoreTake(i2c_comp, 1000) == pdFALSE) {
		ret = 0;
	}

	xSemaphoreGive(i2c_sem);

	return ret;
}

uint32_t hal_i2c1_write(uint8_t dev_addres, void *data, uint32_t size)
{
	if(i2c_sem == NULL) {
		return 0;
	}
	xSemaphoreTake(i2c_sem, 10000);

	uint32_t ret = size;

	dev_addres = (dev_addres << 1);

	if (HAL_I2C_Master_Transmit_DMA(&hi2c1,
                                dev_addres,
                                (uint8_t *)data,
                                size) != HAL_OK)
	{
		ret = 0;
	}
    if(xSemaphoreTake(i2c_comp, 1000) == pdFALSE) {
		ret = 0;
	}

	xSemaphoreGive(i2c_sem);

	return ret;
}

uint32_t hal_i2c1_read(uint8_t dev_addres, void *data, uint32_t size)
{
	if(i2c_sem == NULL) {
		return 0;
	}
	xSemaphoreTake(i2c_sem, 10000);

	uint32_t ret = size;

	dev_addres = (dev_addres << 1) | 1;

	if (HAL_I2C_Master_Receive_DMA(&hi2c1,
                                dev_addres,
                                (uint8_t *)data,
                                size) != HAL_OK)
	{
		ret = 0;
	}

	if(xSemaphoreTake(i2c_comp, 1000) == pdFALSE) {
		ret = 0;
	}

	xSemaphoreGive(i2c_sem);

	return ret;
}
