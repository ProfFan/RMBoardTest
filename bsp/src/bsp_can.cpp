//
// Created by Fan Jiang on 2017/5/4.
//

#include <stm32f4xx_hal_can.h>
#include <cmsis_os.h>
#include <climits>
#include "bsp/bsp_can.h"

osThreadId canTaskHandle;

CAN *can1;
CAN *can2;

CanTxMsgTypeDef Tx1Message;
CanRxMsgTypeDef Rx1Message;
CanTxMsgTypeDef Tx2Message;
CanRxMsgTypeDef Rx2Message;

BaseType_t xResult;
uint32_t ulNotifiedValue;

CAN::CAN(CAN_HandleTypeDef *hcan) {
  this->hcan = hcan;
  this->_mutex = new cpp_freertos::MutexStandard;
}

CAN::~CAN() {
  delete _mutex;
}

int CAN::registerCallback(uint32_t messageID, std::function<void(CanRxMsgTypeDef *)> callback) {

  _mutex->Lock();

  if (tableSize < MAX_CAN_CALLBACK) {
    uint32_t prim;

    /* Read PRIMASK register, check interrupt status before you disable them */
    /* Returns 0 if they are enabled, or non-zero if disabled */
    prim = __get_PRIMASK();

    __disable_irq();

    idTable[tableSize] = messageID;
    callbackTable[tableSize] = callback;
    tableSize = tableSize + 1;

    if (!prim) __enable_irq();

    _mutex->Unlock();
    return 0;
  } else {
    _mutex->Unlock();
    return 1;
  }
}

void CAN::processFrame(CanRxMsgTypeDef *message) {
  for (int i = 0; i < tableSize; i++) {
    if (idTable[i] == message->StdId) {
      callbackTable[i](message);
    }
  }
  // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
}

// extern int messageCount;

extern "C" {
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *_hcan) {

  if (_hcan == can1->hcan) {
    can1->processFrame(_hcan->pRxMsg);

    __HAL_CAN_ENABLE_IT(can1->hcan, CAN_IT_FMP0);
    //HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  }

  if (_hcan == can2->hcan) {
    can2->processFrame(_hcan->pRx1Msg);

    __HAL_CAN_ENABLE_IT(can2->hcan, CAN_IT_FMP1);
    //HAL_CAN_Receive_IT(&hcan2, CAN_FIFO1);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *_hcan) {
  if (_hcan == can1->hcan) {
    __HAL_CAN_CLEAR_FLAG(can1->hcan, CAN_FLAG_FOV0);
    //__HAL_CAN_ENABLE_IT(can1->hcan, CAN_IT_FMP0);
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  }

  if (_hcan == can2->hcan) {
    __HAL_CAN_CLEAR_FLAG(can2->hcan, CAN_FLAG_FOV1);
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO1);
  }
}

void StartCANBusTask(void const *argument) {
  can1 = new CAN(&hcan1);
  can2 = new CAN(&hcan2);

  CAN_FilterConfTypeDef CAN_FilterConfigStructure;

  CAN_FilterConfigStructure.FilterNumber = 0;
  CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterConfigStructure.BankNumber = 14; // Filter Bank 0-14 for CAN1
  CAN_FilterConfigStructure.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK) {
    while(1){

    }
  }

  osDelay(10);

  CAN_FilterConfigStructure.FilterNumber = 14;
  CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO1;
  if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure) != HAL_OK) {
    while(1){

    }
  }

  hcan1.pRxMsg = &Rx1Message;
  hcan1.pTxMsg = &Tx1Message;

  hcan2.pRx1Msg = &Rx2Message;
  hcan2.pTxMsg = &Tx2Message;

  osDelay(10);

  if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK) {
    while (1) {

    }
  }
  osDelay(10);

  if (HAL_CAN_Receive_IT(&hcan2, CAN_FIFO1) != HAL_OK){
    while(1){

    }
  }

  osDelay(10);

  while (true) {
//    xResult = xTaskNotifyWait(pdFALSE,    /* Don't clear bits on entry. */
//                              ULONG_MAX,        /* Clear all bits on exit. */
//                              &ulNotifiedValue, /* Stores the notified value. */
//                              1000);
//
//    if (xResult == pdPASS) {
//      /* A notification was received.  See which bits were set. */
//
//    } else {
//      /* Did not receive a notification within the expected time. */
//
//    }
    if(HAL_CAN_GetState(&hcan1)==HAL_CAN_STATE_READY){
      can1->errorCount += 1;
      HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    }
    if(HAL_CAN_GetState(&hcan2)==HAL_CAN_STATE_READY){
      HAL_CAN_Receive_IT(&hcan2, CAN_FIFO1);
    }
    osDelay(1);
  }
}

}