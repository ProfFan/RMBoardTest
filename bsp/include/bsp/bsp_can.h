//
// Created by Fan Jiang on 2017/5/4.
//

#ifndef CHASSIS_BSP_CAN_H
#define CHASSIS_BSP_CAN_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CAN_CALLBACK 32

int CAN_Initialize();
// void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *_hcan);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <functional>

class CAN {
public:

  CAN(CAN_HandleTypeDef *hcan);

  int registerCallback(uint32_t messageID, std::function<void(CanRxMsgTypeDef *)> callback);

  void processFrame(CanRxMsgTypeDef *message);

  float current;
  float angle;
  CAN_HandleTypeDef *hcan;
private:
  int tableSize = 0;
  uint32_t idTable[MAX_CAN_CALLBACK];

  std::function<void(CanRxMsgTypeDef *)> callbackTable[MAX_CAN_CALLBACK];
};

extern CAN *can1;
extern CAN *can2;

#endif

#endif //CHASSIS_BSP_CAN_H
