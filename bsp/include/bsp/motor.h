//
// Created by Fan Jiang on 2017/5/4.
//

#ifndef CHASSIS_MOTOR_H
#define CHASSIS_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "can.h"

class Motor {
public:

  Motor(CAN_HandleTypeDef *hcan);

  void UpdateSensors();

  void SetCurrent();

  float current;
  float angle;
private:

  CAN_HandleTypeDef *hcan;
  // integral error terms scaled by Ki
};

#endif

#endif //CHASSIS_MOTOR_H
