//
// Created by Fan Jiang on 2017/5/1.
//
#include "bsp/sbus.h"
#include "cmsis_os.h"
#include "usart.h"
#include "usbd_cdc_if.h"

osThreadId sbusTaskHandle;

RC_Type hsbus1;

uint8_t sbus_buffer[SBUS_DMA_BUFFER_SIZE];

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
  if (tmp1 == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->ErrorCode  = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                  (uint32_t)pData, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

void SBUS_Reset_DMA_Rx(UART_HandleTypeDef* huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    // clear idle it flag
    // restart DMA
    uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(
        huart->hdmarx);

    __HAL_DMA_DISABLE(huart->hdmarx);
    __HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
    __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_DMA_BUFFER_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
}

void SBUS_Rx_Init(UART_HandleTypeDef* huart){
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  UART_Receive_DMA_No_IT(huart, sbus_buffer, SBUS_DMA_BUFFER_SIZE);
}
void StartSBUSTask(void const *argument){
  for(;;){
    if(HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY){
      SBUS_Rx_Init(&huart1);
    }
    osDelay(500);
  }
}