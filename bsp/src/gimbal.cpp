//
// Created by Fan Jiang on 2017/5/3.
//

#include "bsp/gimbal.h"
#include "bsp/param.h"
#include "bsp/imu.h"
#include "cmsis_os.h"
#include "can.h"

osThreadId gimbalTaskHandle;

Gimbal *gimbal;

Gimbal::Gimbal(CAN_HandleTypeDef *hcan) :
    pid_pitch(params->raw.PIDPitchKp, params->raw.PIDPitchKi, params->raw.PIDPitchKd, 0.001),
    pid_pitch_v(params->raw.PIDPitchKpV, 0, params->raw.PIDPitchKdV, 0.001),

    pid_yaw(params->raw.PIDYawKp, params->raw.PIDYawKi, params->raw.PIDYawKd, 0.001),
    pid_yaw_v(params->raw.PIDYawKp, 0, params->raw.PIDYawKdV, 0.001),
    pitch_motor(hcan),
    yaw_motor(hcan)
{
  this->hcan = hcan;
}

void Gimbal::UpdateSensors(){

}

void StartGimbalTask(void const *argument) {
  while(!ahrs->healthy) osDelay(100);

  for(;;){
    osDelay(1);
  }
}