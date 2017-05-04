//
// Created by Fan Jiang on 2017/5/3.
//

#ifndef CHASSIS_GIMBAL_H
#define CHASSIS_GIMBAL_H

#include "bsp/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

void StartGimbalTask(void const *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "can.h"
#include "bsp/motor.h"

class Gimbal {
public:

  Gimbal(CAN_HandleTypeDef *hcan);

  float pitch_encoder = 0;
  float pitch_encoder_offset = 0;

  float yaw_encoder = 0;
  float yaw_encoder_offset = 0;

  void UpdateSensors();
private:

  PID pid_pitch;
  PID pid_yaw;

  PID pid_pitch_v;
  PID pid_yaw_v;

  CAN_HandleTypeDef *hcan;

  Motor pitch_motor;
  Motor yaw_motor;
};

extern Gimbal *gimbal;
#endif

#endif //CHASSIS_GIMBAL_H
