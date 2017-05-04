//
// Created by Fan Jiang on 2017/5/4.
//

#include <stm32f4xx_hal_can.h>
#include "bsp/bsp_can.h"

CAN::CAN(CAN_HandleTypeDef *hcan) {
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}

void CAN::registerCallback(uint32_t messageID, void *callback) {

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan){
  if(_hcan == can1->hcan){

  }

  if(_hcan == can2->hcan){

  }
}

void CAN_Initialize(){
  can1 = new CAN(&hcan1);
  can2 = new CAN(&hcan2);
}