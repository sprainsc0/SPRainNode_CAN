#include "serial.h"
#include "ringbuffer.h"
#include <semphr.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static struct ringbuffer uart1_ring;
static SemaphoreHandle_t uart1_sem;
static SemaphoreHandle_t uart1_poll;
#define UART1RINGBUFFER_SIZE     1024
static uint8_t uart1_buffer[UART1RINGBUFFER_SIZE];

#define UART1DMABUFFERSIZE       1024
static uint8_t uart1_dma[UART1DMABUFFERSIZE];

static struct ringbuffer uart2_ring;
static SemaphoreHandle_t uart2_sem;
#define UART2RINGBUFFER_SIZE     100
static uint8_t uart2_buffer[UART2RINGBUFFER_SIZE];

#define UART2DMABUFFERSIZE       100
static uint8_t uart2_dma[UART2DMABUFFERSIZE];

static void HAL_UART1_TxCpltCallback(UART_HandleTypeDef *huart)
{
	static portBASE_TYPE uart1TaskWoken;
	if(uart1_sem != NULL) {
		xSemaphoreGiveFromISR(uart1_sem, &uart1TaskWoken);
		portYIELD_FROM_ISR(uart1TaskWoken);
	}
}

static void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static portBASE_TYPE uart1pollTaskWoken;

	HAL_UART_DMAStop(huart);
	uint16_t lenght = UART1DMABUFFERSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	ringbuffer_put(&uart1_ring, uart1_dma, lenght);
	HAL_UART_Receive_DMA(huart, uart1_dma, UART1DMABUFFERSIZE);
	
	if(uart1_poll != NULL) {
		xSemaphoreGiveFromISR(uart1_poll, &uart1pollTaskWoken);
		portYIELD_FROM_ISR(uart1pollTaskWoken);
	}
}

void hal_uart1_irq(void)
{
	static portBASE_TYPE uart1pollTaskWoken;

	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_FLUSH_DRREGISTER(&huart1);
        
        uint16_t lenght = UART1DMABUFFERSIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        // put data to ringbuffer
        ringbuffer_put(&uart1_ring, uart1_dma, lenght);
    
        HAL_UART_Receive_DMA(&huart1, uart1_dma, UART1DMABUFFERSIZE);

		if(uart1_poll != NULL) {
			xSemaphoreGiveFromISR(uart1_poll, &uart1pollTaskWoken);
			portYIELD_FROM_ISR(uart1pollTaskWoken);
		}
    }
}

void hal_uart1_init(void)
{
	ringbuffer_init(&uart1_ring, uart1_buffer, UART1RINGBUFFER_SIZE);

	uart1_sem = xSemaphoreCreateCounting(1, 0);
	uart1_poll = xSemaphoreCreateCounting(6, 0);

	HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART1_TxCpltCallback);
	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART1_RxCpltCallback);

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // __HAL_UART_ONE_BIT_SAMPLE_ENABLE(&huart1);

	HAL_UART_Receive_DMA(&huart1, uart1_dma, UART1DMABUFFERSIZE);
}

uint32_t hal_uart1_write(void *data, uint32_t size)
{
	while(huart1.gState != HAL_UART_STATE_READY);
    
    if(HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, size) != HAL_OK) {
        return 0;
    }
    xSemaphoreTake(uart1_sem, 1000);

	return size;
}

uint32_t hal_uart1_read(void *data, uint32_t size)
{
	return ringbuffer_get(&uart1_ring, (uint8_t *)data, size);
}

uint32_t hal_uart1_valid(void)
{
	return ringbuffer_data_len(&uart1_ring);
}

uint32_t hal_uart1_poll(uint8_t timeout)
{
	return xSemaphoreTake(uart1_poll, timeout);
}

void hal_uart1_flush()
{
	ringbuffer_reset(&uart1_ring);
}

void hal_uart1_setbaud(uint32_t baud)
{
	HAL_UART_DMAStop(&huart1);

	huart1.Init.BaudRate = baud;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	HAL_UART_Init(&huart1);

	HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART1_TxCpltCallback);
	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART1_RxCpltCallback);

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_ONE_BIT_SAMPLE_ENABLE(&huart1);

	HAL_UART_Receive_DMA(&huart1, uart1_dma, UART1DMABUFFERSIZE);

	ringbuffer_reset(&uart1_ring);
}

static void HAL_UART2_TxCpltCallback(UART_HandleTypeDef *huart)
{
	static portBASE_TYPE uart2TaskWoken;
	if(uart2_sem != NULL) {
		xSemaphoreGiveFromISR(uart2_sem, &uart2TaskWoken);
		portYIELD_FROM_ISR(uart2TaskWoken);
	}
}

static void HAL_UART2_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(huart);
	uint16_t lenght = UART2DMABUFFERSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	ringbuffer_put(&uart2_ring, uart2_dma, lenght);
	HAL_UART_Receive_DMA(huart, uart2_dma, UART2DMABUFFERSIZE);
}

void hal_uart2_irq(void)
{
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        
        HAL_UART_DMAStop(&huart2);
        __HAL_UART_FLUSH_DRREGISTER(&huart2);
        
        uint16_t lenght = UART2DMABUFFERSIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
        // put data to ringbuffer
        ringbuffer_put(&uart2_ring, uart2_dma, lenght);
    
        HAL_UART_Receive_DMA(&huart2, uart2_dma, UART2DMABUFFERSIZE);
    }
}

void hal_uart2_init(void)
{
	ringbuffer_init(&uart2_ring, uart2_buffer, UART2RINGBUFFER_SIZE);

	uart2_sem = xSemaphoreCreateCounting(1, 0);

	HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART2_TxCpltCallback);
	HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART2_RxCpltCallback);

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    //__HAL_UART_ONE_BIT_SAMPLE_ENABLE(&huart3);

	HAL_UART_Receive_DMA(&huart2, uart2_dma, UART2DMABUFFERSIZE);
}

uint32_t hal_uart2_write(void *data, uint32_t size)
{
	while(huart2.gState != HAL_UART_STATE_READY);
    
    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, size) != HAL_OK) {
        return 0;
    }
    xSemaphoreTake(uart2_sem, 1000);

	return size;
}

uint32_t hal_uart2_read(void *data, uint32_t size)
{
	return ringbuffer_get(&uart2_ring, (uint8_t *)data, size);
}

uint32_t hal_uart2_valid(void)
{
	return ringbuffer_data_len(&uart2_ring);
}

void hal_uart2_setbaud(uint32_t baud)
{
	HAL_UART_DMAStop(&huart2);

	huart2.Init.BaudRate = baud;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	HAL_UART_Init(&huart2);

	HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART2_TxCpltCallback);
	HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART2_RxCpltCallback);

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    //__HAL_UART_ONE_BIT_SAMPLE_ENABLE(&huart3);

	HAL_UART_Receive_DMA(&huart2, uart2_dma, UART2DMABUFFERSIZE);

	ringbuffer_reset(&uart2_ring);
}