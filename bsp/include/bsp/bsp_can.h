//
// Created by Fan Jiang on 2017/5/4.
//

#ifndef CHASSIS_BSP_CAN_H
#define CHASSIS_BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

void CAN_Initialize();
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "can.h"

class CAN {
public:

  CAN(CAN_HandleTypeDef *hcan);

  void registerCallback(uint32_t messageID, void* callback);

  float current;
  float angle;
  CAN_HandleTypeDef *hcan;
private:

  // integral error terms scaled by Ki
};

extern CAN* can1;
extern CAN* can2;

#endif

#endif //CHASSIS_BSP_CAN_H
