#include "can.h"
#include "ringbuffer.h"

#include <stm32g4xx_hal.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "cmsis_os.h"
#include "debug.h"
#include "hrt_timer.h"

#define TOPICS_OFFSET_PRIORITY    26U
#define TOPICS_OFFSET_SUBJECT_ID  8U
#define TOPICS_OFFSET_SERVICE_ID  14U
#define TOPICS_OFFSET_DST_NODE_ID 7U

#define SERVICE_NOT_MESSAGE  (UINT32_C(1) << 25U)
#define ANONYMOUS_MESSAGE    (UINT32_C(1) << 24U)
#define REQUEST_NOT_RESPONSE (UINT32_C(1) << 24U)

extern FDCAN_HandleTypeDef hfdcan1;

static SemaphoreHandle_t can_sem;

static struct ringbuffer can1_ring = {0};
#define CAN1_RINGBUFFER_SIZE     512
static uint8_t can1_buffer[CAN1_RINGBUFFER_SIZE];
static struct can_msg recv_msg;

const uint8_t FDCANDLCToLength[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
const uint32_t FDCANLengthToDLC[65] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8, // 0-8
    FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12,                                                   // 9-12
    FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16,                                                  // 13-16
    FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20,                                                  // 17-20
    FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24,                                                  // 21-24
    FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,                                  // 25-32
    FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,  // 33-48
    FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,  // 49-64
};

uint32_t compare(uint32_t a, uint32_t b)
{
    uint32_t res = 0;
    for (uint32_t i=0; i<32; i++) {
        uint8_t temp = !((a >> i) & 1) ^ ((b >> i) & 1);
        res |= temp << i;
    }
    return res;
}

uint32_t caculate_filters(uint32_t *id, uint8_t len)
{
    uint32_t mask = 0x1FFFFFFF;
    for(uint8_t i=0; i<len; i++) {
        mask &= compare(id[i], id[0]);
    }
    return mask;
}

void can_init(void)
{
    FDCAN_FilterTypeDef  sFilterConfig;

    ringbuffer_init(&can1_ring, can1_buffer, CAN1_RINGBUFFER_SIZE);

    can_sem = xSemaphoreCreateCounting(1, 0);
    
    /*## Configure the CAN Filter ###########################################*/
   sFilterConfig.FilterIndex  = 0;
   sFilterConfig.FilterID1    = 0x1FFFFFFF;
   sFilterConfig.FilterID2    = 0x1FFFFFFF;
   sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
   sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
   sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
   if(HAL_FDCAN_ConfigFilter(&hfdcan1 , &sFilterConfig) != HAL_OK)
   {
       /* Filter configuration Error */
       Info_Debug("Can Filter init error \n");
   }

    // /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /*## Start the CAN peripheral ###########################################*/
    

    HAL_FDCAN_ConfigInterruptLines(&hfdcan1,  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
	HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1,  FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
	HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    HAL_FDCAN_ConfigInterruptLines(&hfdcan1,  FDCAN_IT_TX_COMPLETE, FDCAN_INTERRUPT_LINE1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_TX_FIFO_EMPTY, 0);

    HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_BUS_OFF, FDCAN_INTERRUPT_LINE1);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_ERROR_WARNING, FDCAN_INTERRUPT_LINE1);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_ERROR_PASSIVE, FDCAN_INTERRUPT_LINE1);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_ARB_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE1);

    HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_ERROR_WARNING, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_ERROR_PASSIVE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1,  FDCAN_IT_ARB_PROTOCOL_ERROR, 0);
    
    // HAL_FDCAN_DisableISOMode(&hfdcan1);

    /* Configure and enable Tx Delay Compensation : TdcOffset = DataTimeSeg1*DataPrescaler */
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 0xF, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
    
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Info_Debug("Can start error \n");
    }
}

void can_restart(void)
{
    if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
        Info_Debug("Can stop error \n");
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Info_Debug("Can start error \n");
    }
}

int STM32_CAN_Filters(void *config, uint32_t num_filter_configs)
{
    CanardSTM32AcceptanceFilterConfiguration* filter_configs = (CanardSTM32AcceptanceFilterConfiguration *)config;
    FDCAN_FilterTypeDef  sFilterConfig;

    if ((filter_configs == NULL) ||
        (num_filter_configs > 14))
    {
        return -1;
    }

    for (uint8_t i = 0; i < num_filter_configs; i++)
    {
        CanardSTM32AcceptanceFilterConfiguration* cfg = filter_configs + i;

        sFilterConfig.FilterIndex  = i;
        sFilterConfig.FilterID1    = cfg->id;
        sFilterConfig.FilterID2    = cfg->mask;
        sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
        sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
        if((i/2) == 0) {
            sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        } else {
            sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        }
        if(HAL_FDCAN_ConfigFilter(&hfdcan1 , &sFilterConfig) != HAL_OK)
        {
            /* Filter configuration Error */
            Info_Debug("Can Filter init error \n");
            return 0;
        }
    }
    return num_filter_configs;
}

static void fdcan_rx_fifo(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation) 
{
    FDCAN_RxHeaderTypeDef RxHeader;
    struct can_msg pmsg;
    if(HAL_FDCAN_GetRxMessage(hfdcan, RxLocation, &RxHeader, (uint8_t *)pmsg.data) == HAL_OK) {
        pmsg.ts = micros();
        if(RxHeader.IdType == FDCAN_EXTENDED_ID) {
            pmsg.ide = CAN_EXTID;
        } else {
            pmsg.ide = CAN_STDID;
        }
        if(RxHeader.RxFrameType == FDCAN_DATA_FRAME) {
            pmsg.rtr = CAN_DTR;
        } else {
            pmsg.rtr = CAN_RTR;
        }
        const uint8_t len = (RxHeader.DataLength>>16)&0x0f;
        pmsg.id = RxHeader.Identifier;
        pmsg.len = FDCANDLCToLength[len];
        pmsg.hdr = RxHeader.FilterIndex;
        ringbuffer_put(&can1_ring, (const uint8_t *)&pmsg, sizeof(pmsg));
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        fdcan_rx_fifo(hfdcan, FDCAN_RX_FIFO0);
    }

    uint32_t fifo_len = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
	while(fifo_len > 0) {
		fdcan_rx_fifo(hfdcan, FDCAN_RX_FIFO0);
		fifo_len--;
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        fdcan_rx_fifo(hfdcan, FDCAN_RX_FIFO1);
    }

    uint32_t fifo_len = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1);
	while(fifo_len > 0) {
		fdcan_rx_fifo(hfdcan, FDCAN_RX_FIFO1);
		fifo_len--;
	}
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    static portBASE_TYPE canTaskWoken;
	if(can_sem != NULL) {
		xSemaphoreGiveFromISR(can_sem, &canTaskWoken);
		portYIELD_FROM_ISR(canTaskWoken);
	}
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t ret = HAL_FDCAN_GetError(hfdcan);
}

int CAN_Receive(void* frame)
{
    CanardFrame* out_frame = (CanardFrame*)frame;
    
    if (out_frame == NULL) {
        return -1;
    }
    int ret = ringbuffer_get(&can1_ring, (uint8_t *)&recv_msg, sizeof(recv_msg));

    if(ret == sizeof(recv_msg)) {
        out_frame->timestamp_usec  = recv_msg.ts;
        out_frame->extended_can_id = recv_msg.id;
        out_frame->payload_size    = recv_msg.len;
        out_frame->payload         = (const void *)recv_msg.data;

        return ret;
    }

    return 0;
}

int CAN_Transmit(const void* txframe)
{
    CanardFrame* frame = (CanardFrame*)txframe;
    FDCAN_TxHeaderTypeDef p_txheader;
    
    if (frame == NULL) {
        return -1;
    }

    if(frame->payload_size > 64) {
        frame->payload_size = 64;
    }

    p_txheader.Identifier          = frame->extended_can_id;					
    p_txheader.IdType              = FDCAN_EXTENDED_ID;			
    p_txheader.TxFrameType         = FDCAN_DATA_FRAME;		
    p_txheader.DataLength          = FDCANLengthToDLC[frame->payload_size];		
    p_txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    p_txheader.BitRateSwitch       = FDCAN_BRS_ON;			
    p_txheader.FDFormat            = FDCAN_FD_CAN;			
    p_txheader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    p_txheader.MessageMarker       = 0;
    
    if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        xSemaphoreTake(can_sem, 1000);
    }

    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &p_txheader, (uint8_t *)frame->payload) != HAL_OK) {
        return 0;
    }
    
    return frame->payload_size;
}